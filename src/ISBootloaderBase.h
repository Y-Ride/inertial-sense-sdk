/**
 * @file ISBootloaderBase.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense base class for bootloader actions
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_BASE_H_
#define __IS_BOOTLOADER_BASE_H_

#include "ISConstants.h"
#include "ISSerialPort.h"
#include "libusb.h"
#include "ISUtilities.h"

#include <string>

namespace ISBootloader {

static constexpr int IS_DEVICE_LIST_LEN = 256;
static constexpr int IS_FIRMWARE_PATH_LENGTH = 256;

typedef enum {
    IS_LOG_LEVEL_NONE  = 0,
    IS_LOG_LEVEL_ERROR = 1,
    IS_LOG_LEVEL_WARN  = 2,
    IS_LOG_LEVEL_INFO  = 3,
    IS_LOG_LEVEL_DEBUG = 4,
    IS_LOG_LEVEL_SILLY = 5
} eLogLevel;

typedef enum {
    IS_DEV_TYPE_NONE = 0,
    IS_DEV_TYPE_SAMBA,
    IS_DEV_TYPE_ISB,
    IS_DEV_TYPE_APP,
    IS_DEV_TYPE_DFU,
} eDeviceType;

typedef enum {
    IS_PROCESSOR_SAMx70 = 0,        // uINS-5
    IS_PROCESSOR_STM32L4,           // uINS-3/4, EVB-2

    IS_PROCESSOR_NUM,               // Must be last
} eProcessorType;

typedef enum {
    // Bootloaders must be first because bootloaders may contain app signatures
    IS_IMAGE_SIGN_ISB_STM32L4 = 0x00000001,
    IS_IMAGE_SIGN_ISB_SAMx70_16K = 0x00000002,
    IS_IMAGE_SIGN_ISB_SAMx70_24K = 0x00000004,

    IS_IMAGE_SIGN_UINS_3_16K = 0x00000008,
    IS_IMAGE_SIGN_UINS_3_24K = 0x00000010,
    IS_IMAGE_SIGN_EVB_2_16K = 0x00000020,
    IS_IMAGE_SIGN_EVB_2_24K = 0x00000040,
    IS_IMAGE_SIGN_UINS_5 = 0x00000080,
    
    IS_IMAGE_SIGN_NUM_BITS_USED = 8,

    IS_IMAGE_SIGN_APP = IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K | IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K | IS_IMAGE_SIGN_UINS_5 | IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_STM32L4,
    IS_IMAGE_SIGN_ISB = IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K | IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K | IS_IMAGE_SIGN_UINS_5 | IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_STM32L4,
    IS_IMAGE_SIGN_SAMBA = IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K,
    IS_IMAGE_SIGN_DFU = IS_IMAGE_SIGN_ISB_STM32L4,

    IS_IMAGE_SIGN_EVB = IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K,

    IS_IMAGE_SIGN_NONE = 0,
} eImageSignature;

typedef struct 
{
    std::string path;
} firmware_t;

typedef struct 
{
    firmware_t fw_uINS_5;
    firmware_t fw_uINS_3;
    firmware_t bl_STM32L4;
    firmware_t bl_SAMx70;
    firmware_t fw_EVB_2;
} firmwares_t;

typedef is_operation_result (*pfnBootloadProgress)(void* obj, float percent);
typedef void (*pfnBootloadStatus)(void* obj, const char* infoString, eLogLevel level);

is_operation_result dummy_update_callback(void* obj, float percent);
is_operation_result dummy_verify_callback(void* obj, float percent);
static inline void dummy_info_callback(void* obj, const char* infoString, eLogLevel level) {}

class cISBootloaderBase
{
public:
    cISBootloaderBase(
        pfnBootloadProgress upload_cb,
        pfnBootloadProgress verify_cb,
        pfnBootloadStatus info_cb
    ) : 
        m_update_callback{upload_cb}, 
        m_verify_callback{verify_cb}, 
        m_info_callback{info_cb}
    {
        m_success = false;
        m_update_progress = 0.0;
        m_verify_progress = 0.0;
        m_use_progress = false;
        m_update_in_progress = true;
        m_retries_left = 3;
        m_start_time_ms = 0;
        m_finished_flash = false;

        if(m_update_callback == NULL) m_update_callback = dummy_update_callback;
        if(m_verify_callback == NULL) m_verify_callback = dummy_verify_callback;
        if(m_info_callback == NULL) m_info_callback = dummy_info_callback;
    }
        
    ~cISBootloaderBase() 
    {

    }

    static eImageSignature get_image_signature(std::string filename);

    virtual is_operation_result match_test(void* param) = 0;

    virtual eImageSignature check_is_compatible() = 0;

    /**
     * @brief Reboots into the same mode, giving the bootloader a fresh start
     */
    virtual is_operation_result reboot() = 0;
    
    /**
     * @brief Reboots into the level above, if available:
     *  - ISB to App
     *  - SAM-BA to ISB
     *  - DFU to ISB
     *  Make sure to tall the destructor after a successful call to this function
     */
    virtual is_operation_result reboot_up() = 0;

    /**
     * @brief Reboots into the level below, if available:
     *  - App to ISB
     *  - ISB to DFU
     *  - ISB to SAM-BA
     *  Make sure to call the destructor after a successful call to this function
     */
    virtual is_operation_result reboot_down() = 0;

    /**
     * @brief Get the serial number from the device, and fill out m_ctx with other info
     */
    virtual uint32_t get_device_info() = 0;

    /**
     * @brief Write an image to the device
     * 
     * @param image path to the image
     */
    virtual is_operation_result download_image(std::string image) = 0;

    /**
     * @brief Read an image from the device
     * 
     * @param image path to the image
     */
    virtual is_operation_result upload_image(std::string image) = 0;
    
    /**
     * @brief Verify an image against the device
     * 
     * @param image path to the image
     */
    virtual is_operation_result verify_image(std::string image) = 0;

    virtual bool is_serial_device() { return true; }
    
    bool m_update_in_progress;
    int m_retries_left;
    float m_update_progress;
    float m_verify_progress;
    bool m_success;

    // Callbacks
    pfnBootloadProgress m_update_callback;
    pfnBootloadProgress m_verify_callback;
    pfnBootloadStatus m_info_callback; 

    void* m_thread;
    bool m_finished_flash;
    int m_device_type;
    bool m_use_progress;
    int m_start_time_ms;

    serial_port_t* m_port;
    std::string m_port_name;
    int m_baud;

    uint32_t m_sn;                // Inertial Sense serial number, i.e. SN60000

    static is_operation_result update_device(
        firmwares_t filenames,
        serial_port_t* handle,
        cISBootloaderBase** obj,
        pfnBootloadStatus statusfn,
        pfnBootloadProgress updateprogress,
        pfnBootloadProgress verifyProgress
    );
    static is_operation_result update_device(
        firmwares_t filenames,
        libusb_device_handle* handle,
        cISBootloaderBase** obj,
        pfnBootloadStatus statusfn,
        pfnBootloadProgress updateprogress,
        pfnBootloadProgress verifyProgress
    );

    std::string m_filename;
    bool m_isISB;

protected:
    void status_update(const char* info, eLogLevel level) 
    { 
        if(m_info_callback) m_info_callback((void*)this, info, level); 
    }

    struct
    {
        uint8_t uins_version[4];
        uint8_t evb_version[4];

        char enable_command[5];         // "EBLE" (EVB) or "BLEN" (uINS) 
    } m_app;

    /**
     * @brief Get the file extension from a file name
     */
    static const char* get_file_ext(const char* filename);
    
    static eImageSignature get_hex_image_signature(std::string image);
    static eImageSignature get_bin_image_signature(std::string image);
};

}

#endif
﻿/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include "../../../src/ISUtilities.h"
#include "../../../src/ISLogger.h"
#include "../../../hw-libs/misc/bootloaderApp.h"
#include "../drivers/d_quadEnc.h"
#include "ISLogFileFatFs.h"
#include "xbee.h"
#include "wifi.h"
#include "sd_card_logger.h"
#include "communications.h"

StreamBufferHandle_t        g_xStreamBufferUINS;
StreamBufferHandle_t        g_xStreamBufferWiFiRx;
StreamBufferHandle_t        g_xStreamBufferWiFiTx;
bool                        g_usb_cdc_open=false;

int comWrite(int serialNum, const unsigned char *buf, int size, uint32_t ledPin )
{
    int len;
    if (serialNum == EVB2_PORT_UINS1 &&
    g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SPI_ENABLE)
    {
        len = spiTouINS_serWrite(buf, size);
    }
    else
    {
        len = serWrite(serialNum, buf, size, 0);
    }
    
    if(len)
    {
        LED_ON(ledPin);
    }
    return len;
}

int comRead(int serialNum, unsigned char *buf, int size, uint32_t ledPin)
{
    int len;
    if (serialNum == EVB2_PORT_UINS1 &&
    g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SPI_ENABLE)
    {
        len = spiTouINS_serRead(buf, size);
    }
    else
    {
        len = serRead(serialNum, buf, size, 0);
    }
    
    if(len)
    {
        LED_ON(ledPin);
    }
    return len;
}


void callback_cdc_set_config(uint8_t port, usb_cdc_line_coding_t * cfg)
{
    UNUSED(port);
#if 0
    uint32_t stopbits, parity, databits;
//     uint32_t imr;

    switch (cfg->bCharFormat)
    {
    case CDC_STOP_BITS_2:
        stopbits = US_MR_NBSTOP_2_BIT;
        break;
    case CDC_STOP_BITS_1_5:
        stopbits = US_MR_NBSTOP_1_5_BIT;
        break;
    case CDC_STOP_BITS_1:
        default:
        // Default stop bit = 1 stop bit
        stopbits = US_MR_NBSTOP_1_BIT;
        break;
    }

    switch (cfg->bParityType)
    {
    case CDC_PAR_EVEN:
        parity = US_MR_PAR_EVEN;
        break;
    case CDC_PAR_ODD:
        parity = US_MR_PAR_ODD;
        break;
    case CDC_PAR_MARK:
        parity = US_MR_PAR_MARK;
        break;
    case CDC_PAR_SPACE:
        parity = US_MR_PAR_SPACE;
        break;
        default:
    case CDC_PAR_NONE:
        parity = US_MR_PAR_NO;
        break;
    }
    
    switch(cfg->bDataBits)
    {
    case 5:
    case 6:
    case 7:
        databits = cfg->bDataBits - 5;
        break;
        default:
    case 8:
        databits = US_MR_CHRL_8_BIT;
        break;
    }

    // Options for USART.  This gets called when USB is first connected AND each time the USB CDC serial port is opened by the host.
    //  sam_usart_opt_t usart_options;
    // 	usart_options.baudrate = LE32_TO_CPU(cfg->dwDTERate);
    // 	usart_options.char_length = databits;
    // 	usart_options.parity_type = parity;
    // 	usart_options.stop_bits = stopbits;
    // 	usart_options.channel_mode = US_MR_CHMODE_NORMAL;
#endif
	
    uint32_t baudrate = LE32_TO_CPU(cfg->dwDTERate);
    if(comManagerValidateBaudRate(baudrate)==0)
    {
        // Set baudrate based on USB CDC baudrate
        if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_UINS0))
        {
            serSetBaudRate(EVB2_PORT_UINS0, baudrate);
        }
        if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_UINS1) &&
        !(g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SPI_ENABLE))
        {
            serSetBaudRate(EVB2_PORT_UINS1, baudrate);
        }
    }

    if(comManagerValidateBaudRate(baudrate)==0 || baudrate==9600)
    {
        // 	    if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_XBEE))
        // 	    {
        // 		    serSetBaudRate(EVB2_PORT_XBEE, baudrate);
        // 	    }
        if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_XRADIO))
        {
            serSetBaudRate(EVB2_PORT_XRADIO, baudrate);
        }
        if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_BLE))
        {
            serSetBaudRate(EVB2_PORT_BLE, baudrate);
        }
        if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_SP330))
        {
            serSetBaudRate(EVB2_PORT_SP330, baudrate);
        }
        if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_GPIO_H8))
        {
            serSetBaudRate(EVB2_PORT_GPIO_H8, baudrate);
        }
    }

}

void callback_cdc_disable(void)
{
    g_usb_cdc_open = false;
}

void callback_cdc_set_dtr(uint8_t port, bool b_enable)
{
    if (b_enable)
    {	// Host terminal has open COM
        g_usb_cdc_open = true;
    }
    else
    {	// Host terminal has close COM
        g_usb_cdc_open = false;
    }

    if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_XBEE))
    {
        if (b_enable)
        {	// Assert (LOW) DTR
            ioport_set_pin_level(UART_XBEE_N_DTR_PIN, IOPORT_PIN_LEVEL_LOW);      // Assert LOW
            ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_LOW);      // Assert LOW
        }
        else
        {	// De-assert (HIGH) DTR
            ioport_set_pin_level(UART_XBEE_N_DTR_PIN, IOPORT_PIN_LEVEL_HIGH);     // De-assert HIGH
            ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_HIGH);     // De-assert HIGH
        }
    }
}

// This function never gets called because of a bug in Atmel CDC driver.  Bummer.
// void callback_cdc_set_rts(uint8_t port, bool b_enable)
// {
// 	switch(g_msg.evb.comBridgeCfg)
// 	{
//     case EVB2_CBC_USBxXBEE:
//         if (b_enable)
//         {
//             ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_HIGH);
//         }
//         else
//         {
//             ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_LOW);
//         }
//         break;
//
//     case EVB2_CBC_RS232xUSB:
//     case EVB2_CBC_RS422xUSB:
//         break;
//     }
// }


void uINS0_stream_stop_all(is_comm_instance_t &comm)
{
    int len = is_comm_stop_broadcasts_all_ports(&comm);
    comWrite(EVB2_PORT_UINS0, comm.buffer, len, LED_INS_TXD_PIN);
}

void uINS0_stream_enable_std(is_comm_instance_t &comm)
{
    int len;
    
    len = is_comm_get_data(&comm, DID_INS_2, 0, 0, 10);       // 20 x 4ms = 40ms
    comWrite(EVB2_PORT_UINS0, comm.buffer, len, LED_INS_TXD_PIN);

    len = is_comm_get_data(&comm, DID_DEV_INFO, 0, 0, 500);   // 500ms
    comWrite(EVB2_PORT_UINS0, comm.buffer, len, LED_INS_TXD_PIN);
}

void uINS0_stream_enable_PPD(is_comm_instance_t &comm)
{
    rmc_t rmc;
    rmc.bits = RMC_PRESET_PPD_BITS;
    rmc.options = 0;
    int len = is_comm_set_data(&comm, DID_RMC, 0, sizeof(rmc_t), &rmc);
    comWrite(EVB2_PORT_UINS0, comm.buffer, len, LED_INS_TXD_PIN);

//     len = is_comm_get_data(&comm, DID_INS_2, 0, 0, 1);       // 1 x 4ms = 4ms
//     comWrite(EVB2_PORT_UINS0, comm.buffer, len, LED_INS_TXD_PIN);
}


extern void log_ublox_raw_to_SD(cISLogger& logger, is_comm_instance_t &comm); 

void parse_uINS_data(cISLogger &logger, is_comm_instance_t &comm)
{
    static uint8_t buf[STREAM_BUFFER_SIZE];
    uint32_t did;
    int len = xStreamBufferReceive(g_xStreamBufferUINS, (void*)buf, STREAM_BUFFER_SIZE, 0);
    static uDatasets d;    


	// Parse data
	for( int i=0; i<len; i++)
	{
    	if((did = is_comm_parse(&comm, buf[i])) != DID_NULL)
    	{
        	// Parse Data
        	switch(did)
        	{
            case DID_DEV_INFO:
                is_comm_copy_to_struct(&g_msg.uInsInfo, &comm, sizeof(dev_info_t));
                break;

            case DID_INS_1:
                is_comm_copy_to_struct(&d, &comm, sizeof(ins_1_t));
                g_status.week = d.ins1.week;
                g_status.timeOfWeekMs = (uint32_t)(d.ins1.timeOfWeek*1000);
                break;
                    
            case DID_INS_2:
                is_comm_copy_to_struct(&g_msg.ins2, &comm, sizeof(ins_2_t));
                g_status.week = g_msg.ins2.week;
                g_status.timeOfWeekMs = (uint32_t)(g_msg.ins2.timeOfWeek*1000);
                break;

            case DID_INS_3:
                is_comm_copy_to_struct(&d, &comm, sizeof(ins_3_t));
                g_status.week = d.ins1.week;
                g_status.timeOfWeekMs = (uint32_t)(d.ins3.timeOfWeek*1000);
                break;

            case DID_INS_4:
                is_comm_copy_to_struct(&d, &comm, sizeof(ins_4_t));
                g_status.week = d.ins4.week;
                g_status.timeOfWeekMs = (uint32_t)(d.ins4.timeOfWeek*1000);
                break;
                
            case _DID_EXTERNAL:
                switch(comm.externalDataIdentifier)
                {
                case EXTERNAL_DATA_ID_UBLOX:
                    
#if UBLOX_LOG_ENABLE
                    log_ublox_raw_to_SD(logger, comm);
#endif
                    break;
                    
                case EXTERNAL_DATA_ID_ASCII:
                    break;
                }
                break;
        	}
        	    
        	// Log data to SD Card
        	if(g_loggerEnabled && did!=_DID_EXTERNAL)
        	{
                p_data_hdr_t dataHdr;
                dataHdr.id = did;
                dataHdr.size = comm.dataSize;
                dataHdr.offset = comm.dataOffset;
                
                logger.LogData(0, &dataHdr, comm.buffer);
        	}
    	}
	}    
}


void update_flash_cfg(evb_flash_cfg_t &newCfg)
{
    if(error_check_config(&newCfg))
    {
        return;
    }
    
    // Data is identical, no changes.
	if( !memcmp(&newCfg, g_flashCfg, sizeof(evb_flash_cfg_t)) )
    {
		return;
    }        

    bool initCbPreset = false;
    bool initIOconfig = false;
    bool reinitXBee = false;
    bool reinitWiFi = false;

    // Detect changes
    if (newCfg.cbPreset != g_flashCfg->cbPreset)
    {
        initCbPreset = true;
    }
    if (newCfg.radioPID != g_flashCfg->radioPID ||
        newCfg.radioNID != g_flashCfg->radioNID ||
        newCfg.radioPowerLevel != g_flashCfg->radioPowerLevel)
    {
        reinitXBee = true;
    }    
    if (EVB_CFG_BITS_IDX_WIFI(newCfg.bits) != EVB_CFG_BITS_IDX_WIFI(g_flashCfg->bits) ||
        EVB_CFG_BITS_IDX_SERVER(newCfg.bits) != EVB_CFG_BITS_IDX_SERVER(g_flashCfg->bits))
    {   // WiFi or TCP server preset changed
        reinitWiFi = true;
    }  
    int i = EVB_CFG_BITS_IDX_WIFI(newCfg.bits);        
    if (strncmp((const char*)(newCfg.wifi[i].ssid), (const char*)(g_flashCfg->wifi[i].ssid), WIFI_SSID_PSK_SIZE)!=0 ||
        strncmp((const char*)(newCfg.wifi[i].psk),  (const char*)(g_flashCfg->wifi[i].psk),  WIFI_SSID_PSK_SIZE)!=0 ||
        newCfg.server[i].ipAddr != g_flashCfg->server[i].ipAddr ||
        newCfg.server[i].port   != g_flashCfg->server[i].port)
    {   // WiFi or TCP settings changed
        reinitWiFi = true;            
    }
    i = EVB_CFG_BITS_IDX_SERVER(newCfg.bits);
    if (newCfg.server[i].ipAddr != g_flashCfg->server[i].ipAddr ||
        newCfg.server[i].port   != g_flashCfg->server[i].port)
    {   // TCP settings changed
        reinitWiFi = true;
    }
    if ((newCfg.cbOptions&EVB2_CB_OPTIONS_XBEE_ENABLE) != (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_XBEE_ENABLE) ||
        (newCfg.cbOptions&EVB2_CB_OPTIONS_WIFI_ENABLE) != (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_WIFI_ENABLE))
    {
        initIOconfig = true;
    }    
    
    // Copy data from message to working location
    *g_flashCfg = newCfg;
    
    // Apply changes
    if(initCbPreset)
    {
        com_bridge_select_preset(g_flashCfg);
    }
    if(initIOconfig)
    {
        board_IO_config();
    }
    if(reinitXBee)
    {
        xbee_init();
    }
    if(reinitWiFi)
    {
        wifi_reinit();
    }
    refresh_CFG_LED();
    
	// Enable flash write
	g_nvr_manage_config.flash_write_needed = true;
	g_nvr_manage_config.flash_write_enable = true;
}



void parse_EVB_data(int len, is_comm_instance_t *comm, uint8_t *buf)
{
    int did;
    
	for( int i=0; i<len; i++)
	{
	    if((did = is_comm_parse(comm, buf[i])) != DID_NULL)
	    {                
	        // Parse Data
	        switch(did)
	        {
	        case DID_EVB_STATUS:
	            is_comm_copy_to_struct(&g_status, comm, sizeof(evb_status_t));
	            break;
	                    
	        case DID_EVB_FLASH_CFG:
	            evb_flash_cfg_t newCfg;
	            newCfg = *g_flashCfg;
	            is_comm_copy_to_struct(&newCfg, comm, sizeof(evb_flash_cfg_t));
	            update_flash_cfg(newCfg);                                        
	            break;
	                    
	        case DID_EVB_DEBUG_ARRAY:
	            is_comm_copy_to_struct(&g_debug, comm, sizeof(debug_array_t));
	            break;

	        case _DID_GET_DATA:
	            if(g_usb_cdc_open && udi_cdc_is_tx_ready())
	            {
	                p_data_hdr_t *dataHdr = (p_data_hdr_t*)(comm->buffer);
	                int n=0;
	                switch(dataHdr->id)
	                {
	                    case DID_EVB_STATUS:        n = is_comm_data(comm, DID_EVB_STATUS, 0, sizeof(evb_status_t), (void*)&(g_status));             break;
	                    case DID_EVB_FLASH_CFG:     n = is_comm_data(comm, DID_EVB_FLASH_CFG, 0, sizeof(evb_flash_cfg_t), (void*)(g_flashCfg));      break;
	                    case DID_EVB_DEBUG_ARRAY:   n = is_comm_data(comm, DID_EVB_DEBUG_ARRAY, 0, sizeof(debug_array_t), (void*)&(g_debug));        break;
	                    case DID_EVB_RTOS_INFO:     n = is_comm_data(comm, DID_EVB_RTOS_INFO, 0, sizeof(evb_rtos_info_t), (void*)&(g_rtos));         break;
	                }
	                if(n>0)
	                {
	                    udi_cdc_write_buf(comm->buffer, n);
	                }
	            }
	            break;

	        case _DID_EXTERNAL:
	            switch(comm->externalDataIdentifier)
	            {
	            case EXTERNAL_DATA_ID_ASCII:
	                {
	                    int messageIdUInt = ASCII_MESSAGEID_TO_UINT(&(comm->buffer[1]));
	                    switch (messageIdUInt)
	                    {
	                    case 0x424c454e: // "BLEN" - bootloader enable (uINS)
	                        break;
	                                
	                    case 0x45424c45: // "EBLE" - bootloader enable (EVB)
	                        enable_bootloader();
	                        break;
	                                
			            case 0x4e454c42: // "NELB" - SAM bootloader assistant (SAM-BA) enable
	    		            enable_bootloader_assistant();
	                        break;
	                    }                            
	                }                        
	                break;
	            }                        
	        }                
	    }
	}        
}


void com_bridge_forward(uint32_t srcPort, uint8_t *buf, int len)
{
    uint32_t dstCbf = g_flashCfg->cbf[srcPort];
    
    if(dstCbf == 0 || len==0)    // None
    {
        return;
    }        
    
    if(dstCbf & (1<<EVB2_PORT_UINS0))
    {
        comWrite(EVB2_PORT_UINS0, buf, len, LED_INS_TXD_PIN);
    }

    if(dstCbf & (1<<EVB2_PORT_UINS1))
    {
        comWrite(EVB2_PORT_UINS1, buf, len, LED_INS_TXD_PIN);
    }

    if(dstCbf & (1<<EVB2_PORT_USB))
    {
        if(g_usb_cdc_open && udi_cdc_is_tx_ready())
     	    udi_cdc_write_buf(buf, len);
    }

    if(dstCbf & (1<<EVB2_PORT_XBEE) && xbee_runtime_mode()) // Disable XBee communications when being configured
    {
		comWrite(EVB2_PORT_XBEE, buf, len, LED_XBEE_TXD_PIN);
    }

    if(dstCbf & (1<<EVB2_PORT_XRADIO))
    {
        comWrite(EVB2_PORT_XRADIO, buf, len, 0);
    }

    if(dstCbf & (1<<EVB2_PORT_BLE))
    {
        comWrite(EVB2_PORT_BLE, buf, len, LED_WIFI_TXD_PIN);
    }

    if(dstCbf & (1<<EVB2_PORT_SP330))
    {
        comWrite(EVB2_PORT_SP330, buf, len, 0);
    }

    if(dstCbf & (1<<EVB2_PORT_GPIO_H8))
    {
        comWrite(EVB2_PORT_GPIO_H8, buf, len, 0);
    }

#if 0   // Disabled when forwarding data directly in wifi task
    if(dstCbf & (1<<EVB2_PORT_WIFI))
    {
        xStreamBufferSend(g_xStreamBufferWiFiTx, (void*)buf, len, 0);
    }    
#endif
}


void step_com_bridge(is_comm_instance_t &comm)
{
#define BUF_SIZE 256    //USB CDC buffer size is 320
    uint8_t buf[BUF_SIZE];
    int len;
	
    // USB CDC Rx   =======================================================
	while(udi_cdc_is_rx_ready())
	{
		len = udi_cdc_read_no_polling(buf, BUF_SIZE);

        // Parse EVB data from USB
        parse_EVB_data(len, &comm, buf);

        // This follows data parsing to prevent uINS from receiving bootloader enabled command
        com_bridge_forward(EVB2_PORT_USB, buf, len);
	}

	// uINS Ser0 Rx   =======================================================
	while((len = comRead(EVB2_PORT_UINS0, buf, BUF_SIZE, LED_INS_RXD_PIN)) > 0)
	{
        com_bridge_forward(EVB2_PORT_UINS0, buf, len);
        
        // Send data to Maintenance task
        xStreamBufferSend(g_xStreamBufferUINS, (void*)buf, len, 0);
	}

	// uINS Ser1 (TTL or SPI) Rx   =======================================================
	while((len = comRead(EVB2_PORT_UINS1, buf, BUF_SIZE, 0)) > 0)
	{
        com_bridge_forward(EVB2_PORT_UINS1, buf, len);
	}
    
#ifdef CONF_BOARD_SERIAL_XBEE           // XBee Rx   =======================================================
    xbee_step(&comm);
#endif

#ifdef CONF_BOARD_SERIAL_EXT_RADIO      // External Radio Rx   =======================================================
	while((len = comRead(EVB2_PORT_XRADIO, buf, BUF_SIZE, 0)) > 0)
	{
        com_bridge_forward(EVB2_PORT_XRADIO, buf, len);
	}
#endif

#ifdef CONF_BOARD_SERIAL_ATWINC_BLE     // ATWINC BLE Rx   =======================================================
	while((len = comRead(EVB2_PORT_BLE, buf, BUF_SIZE, LED_WIFI_RXD_PIN)) > 0)
	{
        com_bridge_forward(EVB2_PORT_BLE, buf, len);
	}
#endif

#ifdef CONF_BOARD_SERIAL_SP330          // SP330 RS232/RS422 converter Rx   =======================================================
	while((len = comRead(EVB2_PORT_SP330, buf, BUF_SIZE, 0)) > 0)
	{
        com_bridge_forward(EVB2_PORT_SP330, buf, len);
	}
#endif

#ifdef CONF_BOARD_SERIAL_GPIO_H8        // H8 Header GPIO TTL Rx   =======================================================
	while((len = comRead(EVB2_PORT_GPIO_H8, buf, BUF_SIZE, 0)) > 0)
	{        
        com_bridge_forward(EVB2_PORT_GPIO_H8, buf, len);
	}
#endif

#ifdef CONF_BOARD_SPI_ATWINC_WIFI       // WiFi Rx   =======================================================
    if(len = xStreamBufferReceive(g_xStreamBufferWiFiRx, (void*)buf, STREAM_BUFFER_SIZE, 0))
    {
        com_bridge_forward(EVB2_PORT_WIFI, buf, len);
    }
#endif
}


void communications_init(void)
{
    const size_t xTriggerLevel = 1;
    g_xStreamBufferUINS = xStreamBufferCreate( STREAM_BUFFER_SIZE, xTriggerLevel );
    g_xStreamBufferWiFiRx = xStreamBufferCreate( STREAM_BUFFER_SIZE, xTriggerLevel );
    g_xStreamBufferWiFiTx = xStreamBufferCreate( STREAM_BUFFER_SIZE, xTriggerLevel );
}
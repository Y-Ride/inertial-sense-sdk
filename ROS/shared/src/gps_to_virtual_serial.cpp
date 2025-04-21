/*-----------------------------------------------------------
- This is a class used to extract the timestamp and GPS coordinates
- from the GPS message and write it to a virtual serial port
- in NMEA format. It is designed to be used with the
- Inertial Sense ROS2 driver.
- The class opens a virtual serial port (e.g., /dev/tnt1)
- and writes the NMEA GPRMC sentences to it.
- The class handles the conversion of GPS time to UTC
- and formats the latitude and longitude into the NMEA format.
- It also computes the checksum for the NMEA sentence.
- The virtual port is intended to be connected to gpsd for time sync.
*/

/**
 * \file gps_to_virtual_serial.hpp
 * \author Ben Boyack <bab272@byu.edu>
 * \group FRoSt Lab at Brigham Young University
*/

#include "gps_to_virtual_serial.hpp"
#include "data_sets.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

#if defined(ROS_DISTRO_JAZZY)
#include "rclcpp/rclcpp/rclcpp.hpp"
#elif defined(ROS_DISTRO_HUMBLE)
#include "rclcpp/rclcpp.hpp"
#endif


GPSToVirtSerial::GPSToVirtSerial(std::string port)
{
    // Open virtual serial port
    // serial_stream_.open("/dev/tnt1", std::ios::out | std::ios::binary);
    // if (!serial_stream_.is_open())
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/tnt1");
    // }
    // else
    // {
    //     RCLCPP_INFO(this->get_logger(), "Opened /dev/tnt1 successfully");
    // }
    // serial_fd_ = open(port.c_str(), O_WRONLY | O_NONBLOCK);
    serial_fd_ = open(port.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GPSToVirtSerial"), "Failed to open %s: %s", port.c_str(), strerror(errno));
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("GPSToVirtSerial"), "Opened %s successfully", port.c_str());
    }

}

GPSToVirtSerial::~GPSToVirtSerial()
{
    if (serial_fd_ >= 0)
    {
        close(serial_fd_);
    }
}

// (80) $GNGGA,195253.000,4014.78358,N,11138.83651,W,1,08,3.16,1417.95,M,-18.77,M,,*7D
// (74) $GNRMC,195253,A,4014.78358,N,11138.83651,W,000.1,131.6,140425,000.0,E*72
// (80) $GNGGA,195253.200,4014.78358,N,11138.83650,W,1,08,3.16,1417.94,M,-18.77,M,,*7F
// (74) $GNRMC,195253,A,4014.78358,N,11138.83650,W,000.0,-74.4,140425,000.0,E*6D

void GPSToVirtSerial::write_msg_to_nmea_to_serial(const inertial_sense_ros2::msg::GPS *msg)
{
    // Extract integer and fractional parts
    int64_t int_sec = msg->header.stamp.sec;
    int64_t int_nsec = msg->header.stamp.nanosec;
    double gps_time = int_sec + int_nsec * 1e-9;  // Full precision GPS time

    // Convert to UTC time struct
    std::time_t time_t_gps = static_cast<std::time_t>(int_sec);
    std::tm *utc_time = std::gmtime(&time_t_gps);

    // Log values for debugging
    RCLCPP_DEBUG(rclcpp::get_logger("GPSToVirtSerial"), "gps_time: %.9f", gps_time);
    // RCLCPP_INFO(this->get_logger(), "time_t_gps: %ld", time_t_gps);

    // Format as NMEA $GPZDA with millisecond precision
    // std::ostringstream nmea_sentence;
    // nmea_sentence << "$GPZDA,"
    //             << std::setw(2) << std::setfill('0') << utc_time->tm_hour
    //             << std::setw(2) << std::setfill('0') << utc_time->tm_min
    //             << std::setw(2) << std::setfill('0') << utc_time->tm_sec
    //             << "." << std::setw(3) << std::setfill('0') << (int_nsec / 1000000) // Milliseconds
    //             << ","
    //             << std::setw(2) << std::setfill('0') << utc_time->tm_mday << ","
    //             << std::setw(2) << std::setfill('0') << (utc_time->tm_mon + 1) << ","
    //             << utc_time->tm_year + 1900 << ",00,00*00\r\n";

    // std::string nmea_str = nmea_sentence.str();

    // Convert lat/lon to NMEA format
    auto convert_to_nmea = [](double deg, bool is_lat)
    {
        char hemi = (is_lat ? (deg >= 0 ? 'N' : 'S') : (deg >= 0 ? 'E' : 'W'));
        deg = std::abs(deg);
        int d = static_cast<int>(deg);
        double m = (deg - d) * 60;
        char buffer[16];
        if (is_lat)
            std::snprintf(buffer, sizeof(buffer), "%02d%07.4f", d, m);
        else
            std::snprintf(buffer, sizeof(buffer), "%03d%07.4f", d, m);
        return std::make_pair(std::string(buffer), hemi);
    };

    auto [lat_str, ns] = convert_to_nmea(msg->latitude, true);
    auto [lon_str, ew] = convert_to_nmea(msg->longitude, false);

    // Format time string hhmmss.ss
    char time_str[16];
    std::snprintf(time_str, sizeof(time_str), "%02d%02d%02d.%02d", utc_time->tm_hour, utc_time->tm_min, utc_time->tm_sec, (int)(int_nsec / 10000000));

    // Format date string ddmmyy
    char date_str[16];
    std::snprintf(date_str, sizeof(date_str), "%02d%02d%02d", utc_time->tm_mday, utc_time->tm_mon + 1, utc_time->tm_year % 100);

    // Assemble GPRMC (speed and track = 0.0)
    std::ostringstream gprmc;
    gprmc << "$GNRMC," << time_str << ",A," << lat_str << "," << ns << ","
            << lon_str << "," << ew << ",0.0,0.0," << date_str << ",,,";

    // Compute checksum
    std::string gprmc_body = gprmc.str().substr(1); // Excludes the initial '$'
    unsigned char checksum = 0;
    for (char c : gprmc_body)
        checksum ^= c;

    // Append checksum
    gprmc << "*" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (int)checksum << "\r\n";
    std::string nmea_str = gprmc.str();



    // Write to serial port
    // if (serial_stream_.is_open())
    // {
    //     serial_stream_ << nmea_str;
    //     serial_stream_.flush();
    // }
    // else
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("GPSToVirtSerial"), "Serial port not open!");
    // }
    if (serial_fd_ >= 0)
    {
        ssize_t result = write(serial_fd_, nmea_str.c_str(), nmea_str.length());
        if (result < 0)
        {
            if (errno == EAGAIN)
            {
                RCLCPP_WARN(rclcpp::get_logger("GPSToVirtSerial"), "Write skipped: no reader connected to /dev/tnt0");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("GPSToVirtSerial"), "Write error: %s", strerror(errno));
            }
        }
    }
}

void GPSToVirtSerial::write_msg_to_nmea_to_serial_GGA_RMC(const inertial_sense_ros2::msg::GPS *msg)
{
    // RCLCPP_INFO(rclcpp::get_logger("GPSToVirtSerial"), "Write GGA RMC start...");

    int64_t int_sec = msg->header.stamp.sec;
    int64_t int_nsec = msg->header.stamp.nanosec;
    double gps_time = int_sec + int_nsec * 1e-9;

    std::time_t time_t_gps = static_cast<std::time_t>(int_sec);
    std::tm *utc_time = std::gmtime(&time_t_gps);

    RCLCPP_DEBUG(rclcpp::get_logger("NMEA - GGA and RMC"), "gps_time: %.9f", gps_time);

    // Convert lat/lon to NMEA format
    auto convert_to_nmea = [](double deg, bool is_lat)
    {
        char hemi = (is_lat ? (deg >= 0 ? 'N' : 'S') : (deg >= 0 ? 'E' : 'W'));
        deg = std::abs(deg);
        int d = static_cast<int>(deg);
        double m = (deg - d) * 60;
        char buffer[16];
        if (is_lat)
            std::snprintf(buffer, sizeof(buffer), "%02d%07.4f", d, m);
        else
            std::snprintf(buffer, sizeof(buffer), "%03d%07.4f", d, m);
        return std::make_pair(std::string(buffer), hemi);
    };

    auto [lat_str, ns] = convert_to_nmea(msg->latitude, true);
    auto [lon_str, ew] = convert_to_nmea(msg->longitude, false);

    // Format time string
    char time_str[16];
    std::snprintf(time_str, sizeof(time_str), "%02d%02d%02d.%03d", utc_time->tm_hour, utc_time->tm_min, utc_time->tm_sec, (int)(int_nsec*1e-6));

    // Format date string for RMC
    char date_str[16];
    std::snprintf(date_str, sizeof(date_str), "%02d%02d%02d", utc_time->tm_mday, utc_time->tm_mon + 1, utc_time->tm_year % 100);

    // Gps status
    std::string status_gga = (msg->status & eGpsStatus::GPS_STATUS_FLAGS_FIX_OK) ? ",1," : ",0,";

    std::string status_rmc = ((msg->status & eGpsStatus::GPS_STATUS_FIX_MASK) != eGpsStatus::GPS_STATUS_FIX_NONE) ? ",A," : ",V,";

    // Assemble GNGGA sentence
    std::ostringstream gngga;
    gngga << "$GNGGA," << time_str << "," << lat_str << "," << ns << "," << lon_str << "," << ew << status_gga
            << std::setw(2) << std::setfill('0') << static_cast<int>(msg->num_sat) 
            << ",," << std::fixed << std::setprecision(2) << msg->hmsl << ",M,,M,,,";

    // Assemble GNRMC sentence
    std::ostringstream gnrmc;
    gnrmc << "$GNRMC," << time_str << status_rmc 
            << lat_str << "," << ns << "," << lon_str << "," << ew 
            << ",0.0,0.0," << date_str << ",,,";
    
    // Compute checksums
    auto compute_checksum = [](std::ostringstream &nmea_ostr)
    {
        std::string nmea_body = nmea_ostr.str().substr(1); // Exclude the initial '$'
        unsigned char checksum = 0;
        for (char c : nmea_body)
            checksum ^= c;

        // Append checksum
        nmea_ostr << "*" << std::uppercase << std:: hex << std::setfill('0') << (int)checksum << "\r\n";
        return nmea_ostr.str();
    };

    std::string gngga_str = compute_checksum(gngga);
    std::string gnrmc_str = compute_checksum(gnrmc);

    if (serial_fd_ >= 0)
    {
        ssize_t result = write(serial_fd_, gngga_str.c_str(), gngga_str.length());

        if (result < 0)
        {
            if (errno == EAGAIN)
            {
                RCLCPP_WARN(rclcpp::get_logger("GPSToVirtSerial - GGA"), "Write skipped: no reader connected to /dev/tnt0");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("GPSToVirtSerial - GGA"), "Write error: %s", strerror(errno));
            }
        }
        else 
        {
            // RCLCPP_INFO(rclcpp::get_logger("GPSToVirtSerial"), "Write of GGA sentence successfull");
            result = write(serial_fd_, gnrmc_str.c_str(), gnrmc_str.length());
            if (result < 0)
            {
                if (errno == EAGAIN)
                {
                    // RCLCPP_WARN(rclcpp::get_logger("GPSToVirtSerial - RMC"), "Write skipped: no reader connected to /dev/tnt0");
                }
                else
                {
                    // RCLCPP_ERROR(rclcpp::get_logger("GPSToVirtSerial - RMC"), "Write error: %s", strerror(errno));
                }
            }
        }
    }
    // RCLCPP_INFO(rclcpp::get_logger("GPSToVirtSerial"), "Write GGA RMC end.");

}

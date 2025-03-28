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

/**
 * \file gps_to_virtual_serial.hpp
 * \author Ben Boyack <bab272@byu.edu>
 * \group FRoSt Lab at Brigham Young University
*/

#pragma once
#ifndef GPS_TO_VIRT_SERIAL_HPP
#define GPS_TO_VIRT_SERIAL_HPP

#include "inertial_sense_ros2/msg/gps.hpp"

class GPSToVirtSerial
{
public:
    GPSToVirtSerial(std::string port);

    ~GPSToVirtSerial();

    void write_msg_to_nmea_to_serial(const inertial_sense_ros2::msg::GPS *msg);

private:
    
    // std::ofstream serial_stream_;
    int serial_fd_ = -1;

};

#endif // GPS_TO_VIRT_SERIAL_HPP
/* 
* Copyright (C) 2019, Smart Ocean System Lab, University of Rhode Island.
* 
* For Nortek DVL driver
*
* Author: Lin Zhao (linzhao@uri.edu)
* 
*/
#pragma once

#include <ros/ros.h>
#include <serial/serial.h>
#include <nortek_dvl/DVL.h>
#include <nortek_dvl/CurrentProfile.h>
#include <nortek_dvl/CellMeasure.h>
#include "configure.h"
#include <chrono>

class DVLDriver
{

public:
    DVLDriver(ros::NodeHandle node, ros::NodeHandle private_nh);

    ~DVLDriver();

    Result init();

    void readall();

    void readASCII();

    // Todos:
    // int read();
    // int write();
    // ...

private:
    serial::Serial *ser;

    bool isOpen = true;

    int sleep_t;

    struct
    {
        std::string frame_id;
        std::string port_name;
        int frequency;
    }config;
    

    int decode(std::string& stream);

    void decodeBottonTrack(const std::string& str);
    void decodeWaterTrack(const std::string& str);
    void decodeCurrentProfileI(const std::string& str);
    void decodeCurrentProfileS(const std::string& str);
    void decodeCurrentProfileC(const std::string& str);

    bool checkIntegrity(std::string& nmea_data);
    bool calcChecksum(const std::string& nmea_data);

    // ROS
    ros::Publisher button_track_pub;
    ros::Publisher currect_profile_pub;

    nortek_dvl::CurrentProfile cp_msg;

};
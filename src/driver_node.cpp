
/* 
* Copyright (C) 2019, Smart Ocean System Lab, University of Rhode Island.
* 
* For Nortek DVL driver
*
* Author: Lin Zhao (linzhao@uri.edu)
* 
*/

#include <ros/ros.h>
#include <nortek_dvl/dvl_driver.h>
#include <nortek_dvl/tic_toc.h>
#include <csignal>

int main (int argc, char** argv){
    ros::init(argc, argv, "driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    DVLDriver dvl(nh, private_nh);

    if(dvl.init()!= CORRECT){
        ROS_ERROR_STREAM("DVL Driver: Check the DVL serial port !");

        return -1;
    }
    else
    {
        ROS_INFO("DVL Driver: driver init good");
    }
    

   dvl.readall();

    return 0;
}
/* 
* Copyright (C) 2019, Smart Ocean System Lab, University of Rhode Island.
* 
* For Nortek DVL driver
*
* Author: Lin Zhao (linzhao@uri.edu)
* 
*/

#pragma once

/* serial communication*/
#define DEFAULT_PORT "/dev/ttyUSB2"    // Default port name
#define DEFAULT_BAUD 115200              // Default baudate
#define DEFAULT_PARITY 'N'               // Default parity
#define DEFAULT_DATABITS 8               // Default databits
#define DEFAULT_STOPBITS 1               // Default stopbits
#define DEFAULT_TIMEOUT 1000             // Default timeout

/* default ROS config */
#define FRAME_ID "nortek_dvl"

/* default dvl setup */
#define DEFAULT_FREQUENCY 1

/* default data formt */
#define DATA_NUM_BT 11                  // data number in Button track 
//#define DATA_NUM_WT 9999
#define DATA_NUM_CP_I 7                 //data number in current profile - configure
#define DATA_NUM_CP_S 15                  //data number in current profile - status
#define DATA_NUM_CP_C 16                   //data number in current profile - cell measure

enum Result
{
    CORRECT = 0,                        // No error
    ERR_SERIAL = -1                     // Serial error    
};


enum DataFormat
{
    BOTTON_TRACK = 'B',
    WATER_TRACK = 'W',
    CURRENT_PROFILE_I = 'I',
    CURRENT_PROFILE_S = 'S',
    CURRENT_PROFILE_C = 'C'
};

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <cstring>
#include <nortek_dvl/tic_toc.h>

serial::Serial ser;

// int nmea0183_checksum(char *nmea_data)
// {
//     int crc = 0;
//     int i;

//     // the first $ sign and the last two bytes of original CRC + the * sign
//     for (i = 1; i < strlen(nmea_data) - 3; i ++) {
//         crc ^= nmea_data[i];
//     }

//     return crc;
// }

// int checksum(const std::string& nmea_data)
// {
//     int crc = 0;
//     int i;

//     // the first $ sign and the last two bytes of original CRC + the * sign
//     for (i = 1; i < nmea_data.size() - 3; i ++) {
//         crc ^= nmea_data[i];
//     }

//     return crc;
// }

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_WARN_STREAM("Serial Port initialized");
    }else{
        ROS_WARN_STREAM("Serial Port  not initialized");
        return -1;
    }




    ros::Rate loop_rate(1);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            //ROS_WARN_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            // result.data = ser.readline(ser.available());
            // ROS_WARN_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}
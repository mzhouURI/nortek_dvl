# ROS driver for Nortek DVL 1Mhz and pressure 300m
This version tested in nVidia Jetson TX2 Ubuntu 18.04 with ROS melodic

## Licence: unknown

## Installation
- Install dependencies
    - Install ros serial, please check [here](http://wiki.ros.org/serial) for more information. For ubuntu 18.04/ROS melodic, just using command below  
        ```
        $ sudo apt-get install ros-melodic-serial 

        ```
    - Install ros message package for DVL:
    ```shell
        $ cd ~/your_path_catkin_ws/src
        $ git clone https://github.com/URIsoslab/soslab_msgs
        $ catkin_make --pkg soslab_msgs
    ```
- Install DVL driver
    ```
    $ cd ~/your_path_catkin_ws/src
    $ git clone https://github.com/URIsoslab/nortek_dvl.git
    ```

## Build
```
$ cd ~/your_path_catkin_ws/
$ catkin_make --pkg nortek_dvl
```

## Usage
- Fix USB permission: 
    ```shell
    $ ls -l /dev/ttyUSB* # check your permissions
    $ id -Gn {usrname} # replace {usrname} with your usrname, check if "dialout" is there
    $ sudo usermod -a G dialout {usrname} # add dialout in your username
    ```
- Launch
    ```shell
    $ roslaunch nortek_dvl star_dvl.launch
    ```

- Change usb port: 
    - go to /nortek_dvl/launch/star_dvl.launch
    - change "port_name" as you find the DVL

- Close the DVL:
    - Please wait for the DVL driver is fully shutdown, otherwise you will need to use another USB port, like /ttyUSB1.
    - using **ctrl+c**, not **ctrl+z**

## Current setup:

- Accoring to the setup, DVL is sending button_track msg (/nortek_dvl/dvl) and current_profile msg (/nortek_dvl/current_profile) as follow stream.

    >button_track <---1s---> button_track <---1s---> button_track <---1s---> button_track <---1s---> current_profile
- So the /nortek_dvl/dvl is 0.8 hz  /nortek_dvl/current_profile is 0.2 hz from **rostopic hz ...**
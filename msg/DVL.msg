# Button track: format as $PNORBT7
Header header

float32 dt_1 # Time from the trigger to the centre of the bottom echo [ms]
float32 dt_2 # Time from the start of the NMEA output message to the centre of the bottom echo [ms]

geometry_msgs/Vector3 speed # [m/s]

float32 figure_of_merit # [m/s] check if this precession is correct

float32[4] vertical_distance # from beam 1 ~ beam 4 [m]
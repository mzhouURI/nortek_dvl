Header header

# $PNORI1: config for current profile
int8 instrument_type
int8 beam_number
int8 cell_number
float32 blanking_distance # [m]
float32 cell_size # [m]
string coordinate_system 

# $PNORS1: status for current profile
int8 error
uint32 status
float32 battery # [v]
float32 sound_speed # [m/s]
geometry_msgs/Vector3 atitude # [deg]; x= roll; y=pitch; z = heading
geometry_msgs/Vector3 atitude_std_dev # [deg]
float32 pressure # [dBar]
float32 pressure_std_dev # [dBar]
float32 temperature # [deg C]

# $PNORC1: cell measurement
CellMeasure[] cells
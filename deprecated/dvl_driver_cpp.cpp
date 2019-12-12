/* 
* Copyright (C) 2019, Smart Ocean System Lab, University of Rhode Island.
* 
* For Nortek DVL driver
*
* Author: Lin Zhao (linzhao@uri.edu)
* 
* For NMEA message structure, look here: https://en.wikipedia.org/wiki/NMEA_0183
*/

#include <nortek_dvl/dvl_driver.h>
#include <iostream>
#include <string>
#include <nortek_dvl/tic_toc.h>
#include <unistd.h>

DVLDriver::DVLDriver(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    // signal(SIGIO, signalHandler);

    // before open it, check if it still open? 


    // check param
    private_nh.param("frame_id", config.frame_id, std::string(FRAME_ID));
    private_nh.param("port_name", config.port_name, std::string(DEFAULT_PORT));

    private_nh.param("frequency", config.frequency, DEFAULT_FREQUENCY);
    sleep_t = round(config.frequency/63.0*1000000) + 450 + 2000;
}

DVLDriver::~DVLDriver()
{
    // serial bug, if port not open, use close() will have a Segmentation fault
    if(isOpen)
        ser->close();

    delete[] ser;
}

Result DVLDriver::init()
{
    Result Err = CORRECT;
    try
    {
        ser = new serial::Serial(config.port_name,DEFAULT_BAUD,serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));

        //clear the buffer for new data
        ser->flushInput();
    }
    catch (serial::IOException& e)
    {
        Err = ERR_SERIAL;

        isOpen = false;
    }    

    return Err;
}

// void DVLDriver::signalHandler( int signum ) {
//    std::cout << "Interrupt signal (" << signum << ") received.\n";

//     // serial bug, if port not open, use close() will have a Segmentation fault
//     if(isOpen)
//         ser->close();

//     delete[] ser;

//    exit(signum);  
// }

void DVLDriver::readall()
{
    std::string data;
    std::size_t sz_t;
    // data = ser->readline(ser->available());
    // //std::cout<<"\nData:"<<data;
    // decode(data);

    
    //while(!received.check())
    int test_sz=0;
    int test_i;
    TicToc test_t;
    while(1)
    {
        data.clear();

        sz_t = ser->available();

        // the parse process take about 0.45 millis
        if(sz_t>0){
            data = ser->readline(sz_t);

            // test_t.tic();

            // test_sz = data.size();
            // std::cout<<"\nsize: "<<test_sz<<" Data:"<<data;

            // std::cout<<"data hex: ";
            // for( test_i = 0; test_i<test_sz; test_i++)
            // {
            //     std::cout<<std::hex<<int(data[test_i])<<",";
            // }
            // std::cout<<'\n';

            // std::cout <<" t: " <<test_t.toc()<<std::endl;
            // usleep(1000);
            decode(data);   

            usleep(sleep_t);

        }
        else
        {
            // ROS_WARN("bad recv");
            usleep(sleep_t);
            // 0.68s between botton track and current profile data
        }
        
    }


    // received.clean();

    //publish here
}

/*
*For test
*/
void DVLDriver::readASCII()
{
    // publish all ASCII string
}

/*
* 4 step check: 1)check size?; 2) checksum ? 3) data formate? 4) check intergatity?
*/
int DVLDriver::decode( std::string& stream )
{
    int size = stream.size();
    // std::cout<<"\nsize1: "<<stream.size()<<std::endl;

    // check integrity(first read):  $,data,00<CR><LF> 
    /* delete bad characters, like: null(ASCII:0)*/
    if(!checkIntegrity(stream)){
        
        return -1;
    }
    // std::cout<<"size2: "<<stream.size()<<std::endl;

    // check checksum
    bool checked = calcChecksum(stream);

    // check data format
    char id;
    if (checked){
        id = stream.at(5);
        // std::cout<<"id: "<<id<<std::endl;
    }
    else{
        ROS_WARN("bad checksum");
        return -1;
    }

    switch(id)
    {
        // Botton track
        case BOTTON_TRACK:
            decodeBottonTrack(stream);
            break;
        // water track
        case WATER_TRACK:
            decodeWaterTrack(stream);
            break;
        // Curret profile
        case CURRENT_PROFILE_I:
            decodeCurrentProfileI(stream);
            break;
        case CURRENT_PROFILE_S:
            decodeCurrentProfileS(stream);
            break;
        case CURRENT_PROFILE_C:
            decodeCurrentProfileC(stream);
            break;

        default:
            ROS_WARN("bad format id");
            return -1;
    }

    return 0;
}

void DVLDriver::decodeBottonTrack(const std::string& str)
{
    // long long time;
    // float time;
    char time[20];

    float dt_1;
    float dt_2;

    float vx;
    float vy;
    float vz;
    float figure_of_merit;

    float d1;
    float d2;
    float d3;
    float d4;

    const char *c = str.c_str();

    std::cout<<"\ndata: " <<str;

    if(sscanf(c, "$PNORBT7,%15s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", 
                   time, &dt_1, &dt_2, &vx, &vy, &vz, &figure_of_merit, &d1, &d2, &d3, &d4) >= 1) 
    {
        // double t = std::stof(time);
        std::cout<<" time: " << time;
        // std::cout<<" t: "<< t;
        std::cout<<" , " << dt_1 ;
        std::cout<<" , " << dt_2 ;
        std::cout<<" , " << vx ;
        std::cout<<" , " << vy;
        std::cout<<" , " << vz;
        std::cout<<" , " << figure_of_merit ;
        std::cout<<" , " << d1;
        std::cout<<" , " << d2;
        std::cout<<" , " << d3; 
        std::cout<<" , " << d4;
    }
    else
    {
        /* debug */
        std::cout<<"bt_d1: " <<-1<<std::endl;
    }
    std::cout<<"\n";

    // // If no checksum is above, then use ',' to check if any part data is missing
    // int length[DATA_NUM_BT+1];
    // length[0]=0;
    // bool isOk = getDataLocation(str,DATA_NUM_BT,length);
    // //
    // if(isOk){
    //     // std::cout<<"start"<<std::endl;

    //     time = std::stof(str.substr(length[1]+1, length[2]-length[1]-1));
    //     dt_1 = std::stof(str.substr(length[2]+1, length[3]-length[2]-1));
    //     dt_2 = std::stof(str.substr(length[3]+1, length[4]-length[3]-1));
    //     vx = std::stof(str.substr(length[4]+1, length[5]-length[4]-1));
    //     vy = std::stof(str.substr(length[5]+1, length[6]-length[5]-1));
    //     vz = std::stof(str.substr(length[6]+1, length[7]-length[6]-1));
    //     figure_of_merit = std::stof(str.substr(length[7]+1, length[8]-length[7]-1));
    //     d1 = std::stof(str.substr(length[8]+1, length[9]-length[8]-1));
    //     d2 = std::stof(str.substr(length[9]+1, length[10]-length[9]-1));
    //     d3 = std::stof(str.substr(length[10]+1, length[11]-length[10]-1));
    //     d4 = std::stof(str.substr(length[11]+1, length[12]-length[11]-1));

    //     std::cout<<"bt_d1: " <<d1<<std::endl;
    // }
    // else
    // {
    //     // debug
    //     std::cout<<"bt_d1: " <<-1<<std::endl;
    // }
    

    // received.bt_recv = true; 

}

void DVLDriver::decodeWaterTrack(const std::string& str)
{
    // If no checksum is above, then use ',' to check if any part data is missing

    // 
}

void DVLDriver::decodeCurrentProfileI(const std::string& str)
{
    int instrument_type;
    int head_id;
    int beam_number;
    int cell_number;
    float blanking_distance;
    float cell_size; 
    //std::string coordinate_system; 
    char coordinate_system[6];

    const char *c = str.c_str();

    std::cout<<"data: " <<str;

    if(sscanf(c, "$PNORI1,%d,%d,%d,%d,%f,%f,%4s", 
                   &instrument_type, &head_id, &beam_number, &cell_number, &blanking_distance, &cell_size, coordinate_system) >= 1) 
    {
        std::cout<<"cpi_type: " <<instrument_type<<std::endl;
        std::cout<<"cpi_id: " <<head_id<<std::endl;
        std::cout<<"cpi_beam: " <<beam_number<<std::endl;
        std::cout<<"cpi_cell: " <<cell_number<<std::endl;
        std::cout<<"cpi_dis: " <<blanking_distance<<std::endl;
        std::cout<<"cpi_size: " <<cell_size<<std::endl;
        std::cout<<"cpi_coord: " <<coordinate_system<<std::endl;
    }
    else
    {
        /* debug */
        std::cout<<"cpi_coord: " <<-1<<std::endl;
    }
    std::cout<<"\n";

    // // If no checksum is above, then use ',' to check if any part data is missing
    // int length[DATA_NUM_CP_I+1];
    // length[0]=0;
    // bool isOk = getDataLocation(str,DATA_NUM_CP_I,length);

    // // 
    // if(isOk){
    //     // std::cout<<"start"<<std::endl;

    //     instrument_type = std::stoi(str.substr( length[1]+1, length[2]-length[1]-1 ));
    //     head_id = std::stoi(str.substr( length[2]+1, length[3]-length[2]-1 ));
    //     beam_number = std::stoi(str.substr( length[3]+1, length[4]-length[3]-1 ));
    //     cell_number = std::stof(str.substr( length[4]+1, length[5]-length[4]-1 ));
    //     blanking_distance = std::stof(str.substr( length[5]+1, length[6]-length[5]-1 ));
    //     cell_size = std::stof(str.substr( length[6]+1, length[7]-length[6]-1 ));
    //     coordinate_system = str.substr( length[7]+1, length[8]-length[7]-1 );

    //     std::cout<<"cpi_type: " <<instrument_type<<std::endl;
    // }
    // else
    // {
    //     /* debug */
    //     std::cout<<"cpi_type: " <<-1<<std::endl;

    // }
    

    // received.cp_i_recv = true;


}

void DVLDriver::decodeCurrentProfileS(const std::string& str)
{
    TicToc t;

    int date;
    int time;
    int error;
    unsigned int status;
    float battery;
    float sound_speed;
    float heading;
    float heading_std;
    float pitch;
    float pitch_std;
    float roll;
    float roll_std;
    float pressure;
    float pressure_std_dev;
    float temperature;

    const char *c = str.c_str();

    std::cout<<"data: " <<str;

    if(sscanf(c, "$PNORS1,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", 
                   &date, &time, &error, &status, &battery, &sound_speed, &heading, &heading_std, &pitch, 
                   &pitch_std, &roll, &roll_std,&pressure, &pressure_std_dev, &temperature) >= 1) 
    {
        std::cout<<"cps_error: " <<date<<std::endl;
        std::cout<<"cps_error: " <<time<<std::endl;
        std::cout<<"cps_error: " <<error<<std::endl;
        std::cout<<"cps_error: " <<status<<std::endl;
        std::cout<<"cps_error: " <<battery<<std::endl;
        std::cout<<"cps_error: " <<sound_speed<<std::endl;
        std::cout<<"cps_error: " <<heading<<std::endl;
        std::cout<<"cps_error: " <<heading_std<<std::endl;
        std::cout<<"cps_error: " <<pitch<<std::endl;
        std::cout<<"cps_error: " <<pitch_std<<std::endl;
        std::cout<<"cps_error: " <<roll<<std::endl;
        std::cout<<"cps_error: " <<roll_std<<std::endl;
        std::cout<<"cps_error: " <<pressure<<std::endl;
        std::cout<<"cps_error: " <<pressure_std_dev<<std::endl;
        std::cout<<"cps_error: " <<temperature<<std::endl;
    }
    else
    {
        /* debug */
        std::cout<<"cps_error: " <<-1<<std::endl;
    }

    std::cout<<"\n";

    // int length[DATA_NUM_CP_S+1];
    // length[0]=0;
    // bool isOk = getDataLocation(str,DATA_NUM_CP_S,length);

    // int test_sz = str.size();
    // int test_i;

    // std::cout<<"\nsize: "<<test_sz<<" Data:"<<str;

    // std::cout<<"data hex: ";
    // for( test_i = 0; test_i<test_sz; test_i++)
    // {
    //     std::cout<<std::hex<<int(str[test_i])<<",";
    // }
    // std::cout<<'\n';

    // // 
    // if(isOk){
    //     std::cout<<"start"<<std::endl;
        
    //     std::cout<<" date: "<< str.substr( length[1]+1, length[2]-length[1]-1 );
    //     date = std::stoi(str.substr( length[1]+1, length[2]-length[1]-1 ));

    //     std::cout<<" time: "<< str.substr( length[2]+1, length[3]-length[2]-1 );
    //     time = std::stoi(str.substr( length[2]+1, length[3]-length[2]-1 ));

    //     std::cout<<" error: "<<str.substr( length[3]+1, length[4]-length[3]-1 );
    //     error = std::stoi(str.substr( length[3]+1, length[4]-length[3]-1 ));

    //     std::cout<<" status: "<<str.substr( length[4]+1, length[5]-length[4]-1 );
    //     status = std::stof(str.substr( length[4]+1, length[5]-length[4]-1 ));

    //     std::cout<<" battery: "<<str.substr( length[5]+1, length[6]-length[5]-1 );
    //     battery = std::stof(str.substr( length[5]+1, length[6]-length[5]-1 ));

    //     std::cout<<" sound: "<<str.substr( length[6]+1, length[7]-length[6]-1 );
    //     sound_speed = std::stof(str.substr( length[6]+1, length[7]-length[6]-1 ));

    //     std::cout<<" heading: "<<str.substr( length[7]+1, length[8]-length[7]-1 );
    //     heading = std::stof(str.substr( length[7]+1, length[8]-length[7]-1 ));

    //     std::cout<<" head_std: "<<str.substr( length[8]+1, length[9]-length[8]-1 );
    //     heading_std = std::stof(str.substr( length[8]+1, length[9]-length[8]-1 ));

    //     std::cout<<" pitch: "<<str.substr( length[9]+1, length[10]-length[9]-1 );
    //     pitch = std::stof(str.substr( length[9]+1, length[10]-length[9]-1 ));

    //     std::cout<<" pitch_std: "<<str.substr( length[10]+1, length[11]-length[10]-1 );
    //     pitch_std = std::stof(str.substr( length[10]+1, length[11]-length[10]-1 ));

    //     std::cout<<" roll: "<<str.substr( length[11]+1, length[12]-length[11]-1 );
    //     roll = std::stof(str.substr( length[11]+1, length[12]-length[11]-1 ));

    //     std::cout<<" roll_std: "<<str.substr( length[12]+1, length[13]-length[12]-1 );
    //     roll_std = std::stof(str.substr( length[12]+1, length[13]-length[12]-1 ));

    //     std::cout<<" press: "<<str.substr( length[13]+1, length[14]-length[13]-1 );
    //     pressure = std::stof(str.substr( length[13]+1, length[14]-length[13]-1 ));

    //     std::cout<<" press_std: "<<str.substr( length[14]+1, length[15]-length[14]-1 );
    //     pressure_std_dev = std::stof(str.substr( length[14]+1, length[15]-length[14]-1 ));

    //     std::cout<<" tmp: "<<str.substr( length[15]+1, length[16]-length[15]-1 );
    //     temperature = std::stof(str.substr( length[15]+1, length[16]-length[15]-1 ));

    //     std::cout<<"cps_error: " <<error<<std::endl;
    // }
    // else
    // {
    //     /* debug */
    //     std::cout<<"cps_error: " <<-1<<std::endl;
    // }
    

    // received.cp_s_recv = true;

    
}

void DVLDriver::decodeCurrentProfileC(const std::string& str)
{
    TicToc t;
    int date;
    int time;
    int cell_num;
    float pos; 
    float v_x; 
    float v_y;
    float v_z;
    float v_w; 
    float amp_x; 
    float amp_y;
    float amp_z;
    float amp_w;
    int cor_x; 
    int cor_y;
    int cor_z;
    int cor_w;

    const char *c = str.c_str();

    // std::cout<<"data: " <<str;

    if(sscanf(c, "$PNORC1,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d", 
                   &date, &time, &cell_num, &pos, &v_x, &v_y, &v_z, &v_w, &amp_x, 
                   &amp_y, &amp_z, &amp_w,&cor_x, &cor_y, &cor_z, &cor_w) >= 1) 
    {
        // std::cout<<"num: " <<date<<std::endl;
        // std::cout<<"num: " <<time<<std::endl;
        // std::cout<<"num: " <<cell_num<<std::endl;
        // std::cout<<"num: " <<pos<<std::endl;
        // std::cout<<"num: " <<v_x<<std::endl;
        // std::cout<<"num: " <<v_y<<std::endl;
        // std::cout<<"num: " <<v_z<<std::endl;
        // std::cout<<"num: " <<v_w<<std::endl;
        // std::cout<<"num: " <<amp_x<<std::endl;
        // std::cout<<"num: " <<amp_y<<std::endl;
        // std::cout<<"num: " <<amp_z<<std::endl;
        // std::cout<<"num: " <<amp_w<<std::endl;
        // std::cout<<"num: " <<cor_x<<std::endl;
        // std::cout<<"num: " <<cor_y<<std::endl;
        // std::cout<<"num: " <<cor_z<<std::endl;
        // std::cout<<"num: " <<cor_w<<std::endl;
    }
    else
    {
        /* debug */
        std::cout<<"num: " <<-1<<std::endl;
    }

    std::cout<<"\n";
    // // get each data length in the string 
    // int length[DATA_NUM_CP_C+1];
    // length[0]=0;
    // bool isOk = getDataLocation(str,DATA_NUM_CP_C,length);

    // if(isOk){
    //     // std::cout<<"start"<<std::endl;

    //     date = std::stof(str.substr( length[1]+1, length[2]-length[1]-1 ));
    //     time = std::stof(str.substr( length[2]+1, length[3]-length[2]-1 ));
    //     cell_num = std::stoi( str.substr( length[3]+1, length[4]-length[3]-1 ));
    //     pos = std::stof(str.substr( length[4]+1, length[5]-length[4]-1 ));
    //     //Velocity x,y,z,z2, float
    //     v_x = std::stof(str.substr( length[5]+1, length[6]-length[5]-1 ));
    //     v_y = std::stof(str.substr( length[6]+1, length[7]-length[6]-1 ));
    //     v_z = std::stof(str.substr( length[7]+1, length[8]-length[7]-1 ));
    //     v_w = std::stof(str.substr( length[8]+1, length[9]-length[8]-1 ));
    //     //Amplitude Beam 1~4, float
    //     amp_x = std::stof(str.substr( length[9]+1, length[10]-length[9]-1 ));
    //     amp_y = std::stof(str.substr( length[10]+1, length[11]-length[10]-1 ));
    //     amp_z = std::stof(str.substr( length[11]+1, length[12]-length[11]-1 ));
    //     amp_w = std::stof(str.substr( length[12]+1, length[13]-length[12]-1 ));
    //     //Correlation Beam 1~4, int 
    //     cor_x = std::stoi(str.substr( length[13]+1, length[14]-length[13]-1 ));
    //     cor_y = std::stoi(str.substr( length[14]+1, length[15]-length[14]-1 ));
    //     cor_z = std::stoi(str.substr( length[15]+1, length[16]-length[15]-1 ));
    //     cor_w = std::stoi(str.substr( length[16]+1, length[17]-length[16]-1 ));

    //     received.cp_c_recv = cell_num;

    //     std::cout<<"num: " <<cell_num<<std::endl;
    // }
    // else
    // {
    //     /* debug */
    //     std::cout<<"num: " <<-1<<std::endl;
    // }
    

    // 
}



bool DVLDriver::checkIntegrity(std::string& nmea_data)
{

    // std::cout<<"data: "<<nmea_data;
    int sz = nmea_data.size();
    int i;

    // check min size
    if(sz<=6){
        ROS_WARN("bad size");
        return false;
    }

    //$,data,...,data*00<CR><LR>
    //Check: Start delimiter, Checksum delimiter, 2 bits checksum, Carriage return, Line feed
    if(!(nmea_data.at(0)=='$' && nmea_data.at(sz - 5)=='*')){
        ROS_WARN("bad integrality");
        return false;
    }

    // find bad char
    bool isFound = false;
    std::vector<int> bad_char;
    for(i=0; i<sz-2; i++)
    {
        if( nmea_data[i] < 32 || nmea_data[i] > 126 ){//ASCII: Space
            bad_char.push_back(i);
            isFound = true;
        } 
    }
    // delete bad char
    if(isFound){
        for(i=0; i<bad_char.size(); i++)
        {
            nmea_data.erase(bad_char[i]-i,1);
        }

        ROS_WARN("bad char sz is: %ld\n", bad_char.size());
    }

    return true;
}


bool DVLDriver::calcChecksum(const std::string& nmea_data)
{
    //NMNA0183
    int sz = nmea_data.size();
    int i;
    int crc = 0;
    //$*00<LF>?
    for (i = 1; i < sz - 5; i ++) {
        crc ^= nmea_data[i];
    }
    // extract 2 bits checksum

    // std::cout<<"checksum: "<<nmea_data.substr(sz-4, 2)<<" size: "<<sz<<std::endl;

    int checksum = std::stoi(nmea_data.substr(sz-4, 2), 0, 16);

    return crc == checksum;
}

bool DVLDriver::getDataLocation(const std::string& nmea_data, int num, int len[])
{
    int i;
    // std::cout<<" data: "<<nmea_data<<std::endl;

    for( i = 0; i < num; i++ )
    {
        len[i+1] = nmea_data.find(',',len[i]+1);
            //std::cout<<" : "<< len[i+1];
        if (len[i+1] == -1) // not find ','
            return false;
    }

   // std::cout<<std::endl;
    return true;
}
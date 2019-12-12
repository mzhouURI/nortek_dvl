#include <iostream>
#include <string>
#include <vector>
#include <nortek_dvl/tic_toc.h>
#include <unistd.h>
#include <chrono>
#include <sstream>


int main()
{
    // std::string str = "$PNORC1,$111419,212748,59,29.6,-1.048,-0.035,1.267,-2.940,31.9,32.1,31.2,32.1,2,19,8,16*4C";

    // std::string strin = {0x24,0x50,0x4e,0x4f,0x52,
    //                     0x43,0x31,0x2c,0x31,0x31,
    //                     0x31,0x34,0x31,0x39,0x2c,
    //                     0x32,0x31,0x32,0x37,0x34,
    //                     0x38,0x2c,0x36,0x30,0x2c,
    //                     0x33,0x30,0x2e,0x31,0x2c,
    //                     0x2d,0x30,0x2e,0x35,0x34,
    //                     0x32,0x2c,0x0,0x30,0x2e,0x33,
    //                     0x37,0x36,0x2c,0x31,0x2e,
    //                     0x33,0x33,0x35,0x2c,0x31,
    //                     0x2e,0x32,0x34,0x30,0x2c,
    //                     0x33,0x31,0x2e,0x39,0x2c,
    //                     0x33,0x31,0x2e,0x37,0x2c,
    //                     0x33,0x31,0x2e,0x30,0x2c,
    //                     0x33,0x31,0x2e,0x30,0x2c,
    //                     0x31,0x30,0x2c,0x31,0x31,
    //                     0x2c,0x38,0x2c,0x31,0x37,
    //                     0x2a,0x37,0x32,0xd,0xa};

    // std::string str2 ={0x24,0x50,0x4e,0x4f,0x52,
    //                 0x49,0x31,0x2c,0x34,0x2c,
    //                 0x31,0x30,0x31,0x38,0x31,
    //                 0x31,0x2c,0x34,0x2c,0x36,
    //                 0x30,0x2c,0x30,0x2e,0x31,
    //                 0x30,0x2c,0x30,0x2e,0x35,
    //                 0x30,0x2c,0x58,0x59,0x5a,
    //                 0x2a,0x30,0x36,0xd,0xa};


// std::string strr;
// strr.append<int>(1,0xd);
// // str_test.append<int>(0xa);


  std::string str;
  std::string str2="Writing ";
  std::string str3="print 10 and then 5 more";

  // used in the same order as described above:
  str.append(str2);                       // "Writing "
  str.append(str3,6,3);                   // "10 "
  str.append("dots are cool",5);          // "dots "
  str.append("here: ");                   // "here: "
  str.append(10u,'.');                    // ".........."
  str.append(str3.begin()+8,str3.end());  // " and then 5 more"
  str.append(5,0x2E);                // "....."

  std::cout << str << '\n';
  return 0;

}


/************************ Test for get real time cost ***********************/
// template<typename Diff>
// void log_progress(Diff d)
// {
//     std::cout << "..("
//               << std::chrono::duration_cast<std::chrono::microseconds>(d).count()
//               << " ms).." << std::flush;
// }
// int main()
// {
    // auto t1 = std::chrono::high_resolution_clock::now();
    // usleep(1000);
    // auto t2 = std::chrono::high_resolution_clock::now();

    // log_progress(t2 - t1);
// }

/************************ Test for check time cost between iterator and for loop, in order to find specfic char location***********************/
    // int aa;
    // TicToc t;
    // int i,j;  
    // std::string::iterator  it;
    // for(i =0; i<100; i++)
    // {

    //     for( it=strin.begin(); it!=strin.end()-4; ++it)
    //     {
    //         if( *it < 32 ){//ASCII: Space
    //             // std::cout<< "i : " << it- strin.begin() +1 <<std::endl;
    //             aa = it- strin.begin() +1;
    //         } 
    //     }
    // }

    // std::cout<<"t it: " <<t.toc()<<std::endl; //0.12688

    // TicToc tt;
    
    // for( i =0; i<100; i++)
    // {

    //     for( j =0; j<strin.size();j++)
    //     {
    //         if( strin[j] < 32 ){//ASCII: Space
    //             aa = j;
    //         } 
    //     }
    // }
    // std::cout<<"tt it: " <<tt.toc()<<std::endl;//0.046279
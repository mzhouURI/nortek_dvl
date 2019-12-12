cmake_minimum_required(VERSION 2.8.3)
project(nortek_dvl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  message_generation
  geometry_msgs
)

add_message_files(
  FILES
  DVL.msg
  CurrentProfile.msg
  CellMeasure.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nortek_dvl # for new defined msg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES nortek_dvl
 CATKIN_DEPENDS roscpp rospy std_msgs serial message_runtime geometry_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# build library
add_library(dvl_driver
  src/dvl_driver.cpp
)

target_link_libraries(dvl_driver
  ${catkin_LIBRARIES}
)

# build driver
add_executable(driver_node src/driver_node.cpp)

target_link_libraries(driver_node
  dvl_driver
  ${catkin_LIBRARIES}
)

## Test code
add_executable(serial_test test/serial_test.cpp)
target_link_libraries(serial_test 
  ${catkin_LIBRARIES}
)

add_executable(code_test test/code_test.cpp)
target_link_libraries(code_test
  ${catkin_LIBRARIES}
)

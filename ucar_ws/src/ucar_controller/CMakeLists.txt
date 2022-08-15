cmake_minimum_required(VERSION 2.8.3)
project(ucar_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(catkin REQUIRED COMPONENTS
  message_generation
  genmsg
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  serial
  roslib
)

## Generate messages in the 'msg' folder
# add_message_files(FILES
#   # Battery_info.msg
#   # Health_info.msg
#   # Robot_health.msg
# )

## Generate services in the 'srv' folder
add_service_files(FILES
  GetMaxVel.srv
  SetMaxVel.srv
  GetSensorTF.srv
  SetSensorTF.srv
  GetBatteryInfo.srv
  SetLEDMode.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES ucar_controller
#  CATKIN_DEPENDS genmsg geometry_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)

include_directories(include)

add_executable(base_driver  src/base_driver.cpp)
add_dependencies(base_driver ucar_controller_generate_messages_cpp)
target_link_libraries(base_driver crc_table_fd ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## crc_table_fd
add_library(crc_table_fd src/crc_table.cpp)

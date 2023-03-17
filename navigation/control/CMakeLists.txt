cmake_minimum_required(VERSION 2.8.3)
project(controll)

set(CMAKE_BUILD_TYPE "Release")

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  sweeper_msgs
  tf2
  tf2_ros
  pcl_conversions
  pcl_ros
)

set(GLOG_LIB glog)   

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sweeper_controll 
              src/main.cpp
              src/controll.cpp 
              src/pid.cpp 
              src/ultrasonic.cpp  
              ${PROJECT_SOURCE_DIR}/../../common/watch_dog.cpp
              ${PROJECT_SOURCE_DIR}/../../common/polygon_util.cpp)

target_link_libraries(sweeper_controll ${catkin_LIBRARIES}  ${GLOG_LIB})




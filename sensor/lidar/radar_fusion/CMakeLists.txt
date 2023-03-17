cmake_minimum_required(VERSION 2.8.3)
project(radar_fusion)

set(CMAKE_BUILD_TYPE "Release")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  image_transport
  sweeper_msgs
  pcl_conversions
  pcl_ros
)

set(GLOG_LIB glog) 

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${pcl_INCLUDE_DIRS}
)

add_executable(radar_fusion src/main.cpp 
                            ${PROJECT_SOURCE_DIR}/../../../common/polygon_util.cpp
                            ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp)

target_link_libraries(radar_fusion ${catkin_LIBRARIES} ${GLOG_LIB})

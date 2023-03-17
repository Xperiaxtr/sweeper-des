cmake_minimum_required(VERSION 2.8.3)
project(app_server)

set(CMAKE_BUILD_TYPE "Release")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  sweeper_msgs
  image_transport
  cv_bridge
)

catkin_package()

set(GLOG_LIB glog) 

find_package(jsoncpp CONFIG REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)

include_directories("/usr/include/eigen3")

include_directories(
 include
 /usr/include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PROJECT_SOURCE_DIR}/)
add_executable(app_server src/main.cpp 
                          ${PROJECT_SOURCE_DIR}/../common/watch_dog.cpp
                          ${PROJECT_SOURCE_DIR}/../common/pose_util.cpp
                          ${PROJECT_SOURCE_DIR}/../common/gps_convert.cpp
                          ${PROJECT_SOURCE_DIR}/../common/frame_transform.cpp
                          ${PROJECT_SOURCE_DIR}/../common/stream/tcp_stream.cpp
                        )

target_link_libraries(app_server ${catkin_LIBRARIES}
                              ${OpenCV_LIBRARIES}
                              ${GLOG_LIB}
                              yaml-cpp
                              ${catkin_LIBRARIES} proj
                              jsoncpp_lib
                              app_server_lib)                        


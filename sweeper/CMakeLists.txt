cmake_minimum_required(VERSION 2.8.3)
project(sweeper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")


find_package(catkin REQUIRED COMPONENTS
            roscpp
            rospy
            std_msgs
            sweeper_msgs
            )
set(GLOG_LIB glog)     

FIND_PACKAGE(yaml-cpp REQUIRED)

include_directories("/usr/include/eigen3")
include_directories("/usr/include")

include_directories(
            include
            include/common
            include/protocal
            include/sweeper 
            include/state
            include/nodemanager
            ${catkin_INCLUDE_DIRS}
            )          

catkin_package(
            INCLUDE_DIRS include
            CATKIN_DEPENDS 
            roscpp 
            rospy
            std_msgs
            sweeper_msgs
            ) 

SET(SRC_LIST 
            main.cpp 
            ${PROJECT_SOURCE_DIR}/../common/pose_util.cpp
            ${PROJECT_SOURCE_DIR}/../common/watch_dog.cpp
            ${PROJECT_SOURCE_DIR}/../common/can/can_drive.cpp
            )
link_directories(${PROJECT_SOURCE_DIR}/)
add_executable(sweeper ${SRC_LIST})

target_link_libraries(sweeper 
                      ${catkin_LIBRARIES} 
                      ${GLOG_LIB}
                      yaml-cpp
                      libCanCmd.so
                      sweeper_lib
                     )




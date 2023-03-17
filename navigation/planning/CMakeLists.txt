cmake_minimum_required(VERSION 2.8.3)
project(planning)
set(CMAKE_BUILD_TYPE "Release")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sweeper_msgs
  pcl_conversions
  pcl_ros
  image_transport
  cv_bridge
) 

set(GLOG_LIB glog) 

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)

include_directories("/usr/include/eigen3")
include_directories("/usr/include")


catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(planning src/main.cpp
                        src/planning.cpp
                        src/bspline.cpp
                        src/road_edge_detect.cpp
                        src/double_circle_method.cpp
                        src/tracker/tracker.cpp
                        src/tracker/hungarian_bigragh_matcher.cpp
                        ${PROJECT_SOURCE_DIR}/../../common/watch_dog.cpp
                        ${PROJECT_SOURCE_DIR}/../../common/pose_util.cpp)
target_link_libraries(planning ${catkin_LIBRARIES}
                              ${OpenCV_LIBRARIES}
                              ${GLOG_LIB}
                              yaml-cpp)

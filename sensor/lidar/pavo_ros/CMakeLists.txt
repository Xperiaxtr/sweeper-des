cmake_minimum_required(VERSION 2.8.3)
project(pavo_ros)

set(PAVO_SDK_PATH "./sdk")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB PAVO_SDK_SRC
  "${PAVO_SDK_PATH}/src/*.cpp"
  "${PAVO_SDK_PATH}/src/impl/unix/*.cpp"
)


message("PAVO_SDK_SRC:${PAVO_SDK_SRC}")

set(ROS_BUILD_TYPE Debug)  
set(GLOG_LIB glog)   

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
  sweeper_msgs
  nav_msgs
  tf
  message_generation
  dynamic_reconfigure
  laser_geometry
)

generate_dynamic_reconfigure_options(
   cfg/pavo.cfg
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pavo_ros
#  CATKIN_DEPENDS rosconsole roscpp sensor_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${PAVO_SDK_PATH}/include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

add_executable(pavo_scan_node src/pavo_scan_node.cpp ${PAVO_SDK_SRC} ${PROJECT_SOURCE_DIR}/../../../common/pose_util.cpp
              ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp)

target_link_libraries(pavo_scan_node ${catkin_LIBRARIES} ${GLOG_LIB}  yaml-cpp)

add_executable(pavo_pcd_node src/pavo_pcd_node.cpp ${PAVO_SDK_SRC})
target_link_libraries(pavo_pcd_node ${catkin_LIBRARIES})
#############
## Install ##
#############

install(TARGETS pavo_scan_node pavo_pcd_node 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

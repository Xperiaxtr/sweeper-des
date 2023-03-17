cmake_minimum_required(VERSION 2.8.3)
project(calibration_exterior_parameter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
  std_msgs 
  cv_bridge
)

FIND_PACKAGE( OpenCV REQUIRED )
find_package(Ceres REQUIRED)
include_directories("/usr/include/eigen3")
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

generate_messages(DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
#  INCLUDE_DIRS include
#  LIBRARIES calibration_exterior_parameter
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS}
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(calib_gps_to_imu_pitch src/calib_gps_to_imu_pitch.cpp)
target_link_libraries(calib_gps_to_imu_pitch ${catkin_LIBRARIES} )

add_executable(calibration_livox_to_plane src/calibration_livox_to_plane.cpp)
target_link_libraries(calibration_livox_to_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(calib_livox_yaw src/calib_livox_yaw.cpp)
target_link_libraries(calib_livox_yaw ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})
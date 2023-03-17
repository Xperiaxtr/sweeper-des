cmake_minimum_required(VERSION 2.8.3)
project(calib_lidar_to_gps_imu)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
  std_msgs 
  PCL REQUIRED
  cv_bridge
  OpenCV REQUIRED
  sweeper_msgs
  sweeper_msgs
)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
include_directories("/usr/include/eigen3")
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)



catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS  PCL
  INCLUDE_DIRS include
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
${OpenCV_LIBRARY_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(get_feature1
  src/get_feature.cpp
)
add_dependencies(get_feature1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_feature1
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(calib_gps_odom src/calibration_odom.cpp)
add_dependencies(calib_gps_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calib_gps_odom ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(calib_lidar_to_gps_imu src/gps_correction.cpp)
target_link_libraries(calib_lidar_to_gps_imu ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(get_gps_to_lidar src/new_gps_correction.cpp)
target_link_libraries(get_gps_to_lidar ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(fusion_imu_sweeper_odom src/fusion_imu_sweeper_odom.cpp)
target_link_libraries(fusion_imu_sweeper_odom ${catkin_LIBRARIES} )

add_executable(fusion_gps_imu_odom src/fusion_gps_odom.cpp)
target_link_libraries(fusion_gps_imu_odom ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})


add_executable(calib_gps_to_livox src/calib_gps_to_livox.cpp)
target_link_libraries(calib_gps_to_livox ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(get_livox_yaw src/get_livox_yaw.cpp)
target_link_libraries(get_livox_yaw ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})


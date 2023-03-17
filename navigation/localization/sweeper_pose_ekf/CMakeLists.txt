cmake_minimum_required(VERSION 2.8.3)
project(sweeper_pose_ekf)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  sweeper_msgs
  cmake_modules
  tf
  OpenCV REQUIRED
)
set(GLOG_LIB glog)   
#find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sweeper_pose_ekf
#  CATKIN_DEPENDS geomery_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sweeper_pose_ekf
#  CATKIN_DEPENDS geomery_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib 
)

SET(EKF_LOCALIZER_LIST  src/kalman/kalman_filter.cpp
                        src/kalman/time_delay_kalman_filter.cpp
                        src/ekf_localizer.cpp
                        src/base_lidar.cpp
                        ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp
                        #${PROJECT_SOURCE_DIR}/../../mapping/src/tools/math_calculation.cpp
                        )


SET(SWEEPER_EKF_LIST    src/base_lidar.cpp
                        src/sweeper_pose_ekf_based_lidar.cpp
                        #src/fusion_localization.cpp
                        ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp
                        #${PROJECT_SOURCE_DIR}/../../mapping/src/tools/math_calculation.cpp
                        )


include_directories(include ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIRS})

#add_executable(sweeper_pose_ekf src/main.cpp src/test.cpp)
#target_link_libraries(sweeper_pose_ekf ${catkin_LIBRARIES})

#add_executable(publish_path src/publish_path.cpp)
#target_link_libraries(publish_path ${catkin_LIBRARIES}  yaml-cpp )

add_executable(sweeper_pose_ekf_based_lidar ${EKF_LOCALIZER_LIST})
target_link_libraries(sweeper_pose_ekf_based_lidar ${catkin_LIBRARIES} ${GLOG_LIB} yaml-cpp)

#add_executable(sweeper_ekf_node ${SWEEPER_EKF_LIST})
#target_link_libraries(sweeper_ekf_node ${catkin_LIBRARIES} ${GLOG_LIB} yaml-cpp)


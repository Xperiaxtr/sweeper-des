cmake_minimum_required(VERSION 2.8.3)
project(sweeper_odom)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sweeper_msgs
)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS  PCL
  INCLUDE_DIRS include
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)
set(GLOG_LIB glog)   

add_executable(fusion_imu_sweeper src/fusion_imu_sweeper_odom.cpp ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp)
target_link_libraries(fusion_imu_sweeper ${catkin_LIBRARIES} )

add_executable(test_imu_tran90 src/test_imu_tran90.cpp)
target_link_libraries(test_imu_tran90 ${catkin_LIBRARIES} )

add_executable(gps_to_map src/gps_to_map.cpp)
target_link_libraries(gps_to_map ${catkin_LIBRARIES} )


add_executable(get_lidar_odom src/get_lidar_odom.cpp src/lidar_odom.cpp ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp)
target_link_libraries(get_lidar_odom ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${GLOG_LIB})

add_executable(receive_gps_imu src/receive_gps_imu.cpp)
target_link_libraries(receive_gps_imu ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(imu_sweeper_fusion src/imu_sweeper_fusion.cpp)
target_link_libraries(imu_sweeper_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})


add_executable(fusion_gps_imu src/fusion_gps_imu.cpp ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp)
target_link_libraries(fusion_gps_imu ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

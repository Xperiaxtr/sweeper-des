cmake_minimum_required(VERSION 2.8.3)
project(radar_to_lidar)
find_package(catkin REQUIRED)
add_compile_options(-std=c++11)

catkin_package()
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        sensor_msgs
        roscpp
        pcl_conversions
        pcl_ros
        cv_bridge
        rospy
        )

include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(radar_to_lidar src/radar_to_lidar.cpp)

target_link_libraries(radar_to_lidar
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${PCL_LIBRARIES}
        )

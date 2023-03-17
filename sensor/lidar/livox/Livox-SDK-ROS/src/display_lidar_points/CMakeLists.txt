cmake_minimum_required(VERSION 2.8.3)
project(display_lidar_points)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
 set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sweeper_msgs
)


find_package(Boost REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

find_package( PCL REQUIRED )

set(GLOG_LIB glog) 

FIND_PACKAGE(yaml-cpp REQUIRED)

include_directories("/usr/include")

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

SET(SRC_DIR
    ./
)

FOREACH(dir ${SRC_DIR})
	MESSAGE(STATUS"src_dir: "${dir})
	include_directories(./include/${dir})
	include_directories(./${dir})
	AUX_SOURCE_DIRECTORY(${dir} source_list)
ENDFOREACH()

find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
	message(${APR_INCLUDE_DIRS})
    message(${APR_LIBRARIES})
endif (APR_FOUND)

include_directories(
	               ${APR_INCLUDE_DIRS}
	               )


## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_node
               ${source_list}
               ${PROJECT_SOURCE_DIR}/../../../../../../common/pose_util.cpp
               ${PROJECT_SOURCE_DIR}/../../../../../../common/watch_dog.cpp
               )

target_link_libraries(${PROJECT_NAME}_node
    livox_sdk_static.a
	${APR_LIBRARIES}
    ${PCL_LIBRARIES}
	${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GLOG_LIB}
    yaml-cpp
    -lrt
  )



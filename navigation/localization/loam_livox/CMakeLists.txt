cmake_minimum_required(VERSION 2.8.3)
project(loam_livox)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(G2O_ROOT /usr/local/include/g2o) 
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  sweeper_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
  OpenCV REQUIRED
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
#find_package(GTSAM REQUIRED QUIET)

find_package(PCL REQUIRED)
message(STATUS "***** PCL version: ${PCL_VERSION} *****")
####

find_package( G2O REQUIRED )
include_directories(${G2O_INCLUDE_DIR})

find_package( Cholmod REQUIRED )
include_directories( ${CHOLMOD_INCLUDE_DIR} )

set(GLOG_LIB glog)   

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  #${GTSAM_INCLUDE_DIR}
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	#${GTSAM_LIBRARY_DIRS}
)

#add_executable(livox_test src/test.cpp)
#target_link_libraries(livox_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#add_executable(get_feature
#  src/get_feature.cpp ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp
#)
#add_dependencies(get_feature ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(get_feature
#  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
#)


#add_executable(lidar_odom src/lidar_odom.cpp)
#target_link_libraries(lidar_odom ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#add_executable(calibration_odom src/calibration_odom.cpp)
#add_dependencies(calibration_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${G2O_LIBS})
#target_link_libraries(calibration_odom ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${CHOLMOD_LIBRARIES} ${G2O_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

#add_executable(livox_intensity_mapping src/livox_intensity_mapping.cpp ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp)
#add_dependencies(livox_intensity_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${G2O_LIBS})
#target_link_libraries(livox_intensity_mapping ${catkin_LIBRARIES} ${GLOG_LIB} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${CHOLMOD_LIBRARIES} ${G2O_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

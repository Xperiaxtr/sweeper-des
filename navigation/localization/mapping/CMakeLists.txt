cmake_minimum_required(VERSION 2.8.3)
project(mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
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
  ndt_cpu
  ndt_tku
  ndt_omp
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(PCL REQUIRED)
message(STATUS "***** PCL version: ${PCL_VERSION} *****")
####

find_package( G2O REQUIRED )
include_directories(${G2O_INCLUDE_DIR})

find_package( Cholmod REQUIRED )
include_directories( ${CHOLMOD_INCLUDE_DIR} )

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  #${GTSAM_INCLUDE_DIR}
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs ndt_cpu ndt_tku
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)


include_directories(
            include
            include/flow
            include/get_feature
            include/mapping 
            include/matcher
            include/tools
            ${catkin_INCLUDE_DIRS}
            )  

link_directories(
	include
  include/tools
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	#${GTSAM_LIBRARY_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "*_node.cpp")
#file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
file(GLOB_RECURSE FLOW_SRCS "*flow.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${FLOW_SRCS})
#list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})


SET(MATCHER_LIST  src/node/matcher_node.cpp    
                  src/flow/matcher_flow.cpp
                  src/matcher/matcher.cpp
                  src/matcher/mcl_matcher.cpp
                  src/matcher/se3_matcher.cpp
                  src/matcher/lidar_optimization.cpp
                  src/publisher/cloud_publisher.cpp
                  src/publisher/odometry_publisher.cpp
                  src/sensor_data/gnss_data.cpp
                  src/sensor_data/imu_data.cpp
                  #src/sensor_data/key_frame.cpp
                  src/subscriber/gnss_subscriber.cpp
                  src/subscriber/cloud_subscriber.cpp
                  src/subscriber/imu_subscriber.cpp
                  src/tools/math_calculation.cpp
                  src/tools/map_segment/map_segment.cpp
                  ${ALL_SRCS} ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp
)


SET(MAPPING_LIST src/node/mapping_node.cpp
                src/flow/mapping_flow.cpp
                src/mapping/back_end/back_end.cpp
                src/mapping/front_end/front_end.cpp
                src/mapping/loop_closing/loop_closing.cpp
                src/mapping/save_view_map/save_view_map.cpp
                src/matcher/matcher.cpp
                src/matcher/new_matcher.cpp
                src/matcher/se3_matcher.cpp
                src/matcher/lidar_optimization.cpp
                src/publisher/cloud_publisher.cpp
                src/publisher/odometry_publisher.cpp
                src/sensor_data/gnss_data.cpp
                src/sensor_data/imu_data.cpp
                #src/sensor_data/key_frame.cpp
                src/subscriber/cloud_subscriber.cpp
                src/subscriber/gnss_subscriber.cpp
                src/subscriber/imu_subscriber.cpp
                src/tools/math_calculation.cpp
                ${ALL_SRCS} ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp
)

SET(CALIB_LIST  
                src/calib/save_calib_data.cpp
                src/node/calib_node.cpp
                src/mapping/front_end/front_end.cpp
                src/matcher/matcher.cpp
                src/matcher/new_matcher.cpp
                src/matcher/se3_matcher.cpp
                src/matcher/lidar_optimization.cpp
                src/publisher/cloud_publisher.cpp
                src/publisher/odometry_publisher.cpp
                src/get_feature/get_feature.cpp
                #src/sensor_data/key_frame.cpp
                #src/tools/math_calculation.cpp
                #src/tools/map_segment/map_segment.cpp
                #${ALL_SRCS} ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp
)


#add_executable(test_get_feature src/node/main_node.cpp ${ALL_SRCS})
#add_executable(test_get_feature src/node/main_node.cpp src/get_feature/get_feature.cpp src/publisher/cloud_publisher.cpp src/sensor_data/feature.cpp 
#  src/subscriber/cloud_subscriber.cpp )
#target_link_libraries(test_get_feature  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})


#add_executable(front_end_node src/node/front_end_node.cpp ${ALL_SRCS})
#target_link_libraries(front_end_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(mapping_node ${MAPPING_LIST})
target_link_libraries(mapping_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${CHOLMOD_LIBRARIES} ${G2O_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY} yaml-cpp)

#add_executable(matcher_node src/node/localization_node.cpp src/flow/localization_flow.cpp ${ALL_SRCS} ${PROJECT_SOURCE_DIR}/../../../common/watch_dog.cpp)
#target_link_libraries(matcher_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${CHOLMOD_LIBRARIES} ${G2O_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

add_executable(matcher_node ${MATCHER_LIST})
target_link_libraries(matcher_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${CHOLMOD_LIBRARIES} ${G2O_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY} yaml-cpp)
add_dependencies(matcher_node ${catkin_EXPORTED_TARGETS})

add_executable(calib_node ${CALIB_LIST})
target_link_libraries(calib_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${CHOLMOD_LIBRARIES} ${G2O_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY} yaml-cpp)


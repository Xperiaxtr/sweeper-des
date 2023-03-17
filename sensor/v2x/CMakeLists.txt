cmake_minimum_required(VERSION 2.8.3)
project(v2x)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  eigen_conversions
  sweeper_msgs
)

set(GLOG_LIB glog)  

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS 
eigen_conversions)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(v2x src/main.cpp
                    src/v2x.cpp 
                    ${PROJECT_SOURCE_DIR}/../../common/frame_transform.cpp
                    ${PROJECT_SOURCE_DIR}/../../common/watch_dog.cpp)
                    
target_link_libraries(v2x
                    ${catkin_LIBRARIES} 
                    ${catkin_LIBRARIES} proj
                    ${GLOG_LIB}
                    )

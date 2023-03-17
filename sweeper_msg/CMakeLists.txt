cmake_minimum_required(VERSION 2.8.3)
project(sweeper_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  message_generation
)


 add_message_files(
   FILES
   SweeperCmd.msg
   SweeperChassisDetail.msg
   Point.msg
   Object.msg
   PerceptionObstacle.msg
   SweepMission.msg
   StateReport.msg
   SensorFaultInformation.msg
   SweeperUltraCmd.msg
   Light.msg
   Obu.msg
   RadarFormation.msg
   RadarObject.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs message_runtime geometry_msgs
  DEPENDS system_lib
)









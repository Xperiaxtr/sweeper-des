#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <gflags/gflags.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>

#include "../../../../common/log.h"
#include "../../../../common/watch_dog.h"
#include "../../../common/polygon_util.h"
#include "pid.h"
#include "sweeper_msgs/PerceptionObstacle.h"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweepMission.h"
#include "sweeper_msgs/SweeperChassisDetail.h"
#include "sweeper_msgs/SweeperCmd.h"
#include "sweeper_msgs/SweeperUltraCmd.h"
#include "ultrasonic.h"

namespace sweeper {
namespace navigation {
namespace controll {

#define PI 3.1415926
#define Todegree 180 / PI
#define RAD_TO_DEGREE (180.0 / PI)
#define DEGREE_TO_RAD (PI / 180.0)

enum RADAR_TYPE {
  FRONTRIGHTSIDE = 1,
  RIGHTFRONT,
  MIDDLEFTFRONT,
  LEFTFRONT,
  FRONTLEFTSIDE,
  MIDDLERIGHTSIDE,
  MIDDLELEFTSIDE,
  REAERRIGHTSIDE,
  REARLEFTSIDE,
};

class Controller {
 public:
  Controller(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

  ~Controller(){};

  double CalculateDistance(const double x, const double y, const double z);

 private:
  void SweepModeCallback(const sweeper_msgs::SweepMission &sweep_mission);
  void ReceivePath(nav_msgs::Path new_path);

  void SendUltraCmd(const ros::TimerEvent &e);

  void ComputeSweeperCmd(const ros::TimerEvent &e);

  void ReceiveSweeperChassis(
      const sweeper_msgs::SweeperChassisDetail &sweeprvchassis);

  double SwitchVelocityToPendal(double v_cmd, double v_current);

  double SetSteeringAngularSpeed(double steering_angle_cmd,
                                 double steering_angle_current);
  double LeadCorrectCompensation(const double kgc, const double coeffi_a,
                                 const double coeffi_b,
                                 const double value_before_correction,
                                 const double value_before_correction_last,
                                 const double value_after_correction_last);

  void ReceiveUltrasonicDatas(
      const sweeper_msgs::PerceptionObstacle &sweeper_ultrasonic);

  void ReceiveLidarDatas(const sensor_msgs::PointCloud2 &lidar_datas);

  void SelfDiagnose();

  unsigned int count_;
  int empty_path_count_, allowed_nums_empty_path_;
  int fstop_nums_, rstop_nums_, lstop_nums_, valid_stop_nums_;

  double fre_control_;
  double fre_ultra_;

  double L_, ld_, ld_2_, pos_tol_, ld_velocity_;

  double curb_v_min_lookahead_distance_, curb_v_min_lookahead_distance2_;
  double curb_v_middle_lookahead_distance_, curb_v_middle_lookahead_distance2_;
  double curb_v_max_lookahead_distance_, curb_v_max_lookahead_distance2_;

  double trace_v_min_lookahead_distance_, trace_v_min_lookahead_distance2_;
  double trace_v_middle_lookahead_distance_,
      trace_v_middle_lookahead_distance2_;
  double trace_v_max_lookahead_distance_, trace_v_max_lookahead_distance2_;

  double factor_steering_angle1_, factor_steering_angle2_;
  double proportional_gain_steering_;

  double v_, v_current_;
  double v_set_;
  double curb_v_min_, curb_v_middle_, curb_v_max_;
  double trace_v_min_, trace_v_middle_, trace_v_max_;
  double v_stop_;

  int sweeper_curb_speed_model_;

  double steering_angle_, steering_angle_cmd_last_;
  double cmd_steering_angle_last_;
  double angular_max_, angular_min_;
  double control_angular_para_;
  double pp_steering_angle_last_, lead_correction_compensation_last_;
  double diff_steering_limited_;

  double threshold_yaw_, delta_max_;
  double turn_left_threshold_;

  double ultra_forward_stopping_distance_;
  double ultra_lateral_stopping_distance_;
  double ultra_lateral_dynamic_distance_;

  double kgc_, coeffi_a_, coeffi_b_;

  double forward_x_, estop_thickness_;
  double left_width_, right_width_;
  double front_length_, back_length_, e_back_length_;

  double fstop_line_, bstop_line_;
  double lstop_line_, rstop_line_;

  int emerge_nums_;
  int forward_nums_;
  int lateral_nums_;
  int sweeper_model_;

  int path_type_;

  bool goal_reached_, flag_left_top_, flag_ultra_foward_,
      flag_dj_lidar_forward_, flag_ultra_left_, flag_ultra_right_,
      flag_lookforward_;
  bool estop_flag_, lstop_flag_, rstop_flag_, fstop_flag_;
  bool flag_is_fusion_, flag_lead_correct_compensation_, flag_smooth_steering_;
  bool flag_ultra_test_;

  std::string map_frame_id_, sweeper_frame_id_;
  std::string point_type_normal_, point_type_left_avoidance_,
      point_type_right_avoidance_;
  std::string point_type_;

  nav_msgs::Path path_;
  geometry_msgs::TransformStamped lookahead_;
  geometry_msgs::TransformStamped lookahead_2_;

  sweeper_msgs::SweeperCmd cmd_sweeper_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Ros infrastructure
  ros::Subscriber sub_path_, sub_chassis_, sub_ultrasonic_, sub_lidar_,
      sweep_mission_sub_;
  ros::Publisher pub_sweeper_cmd_;
  ros::Publisher pub_state_information_;
  ros::Publisher pub_ultra_cmd_;

  ros::Timer timer_control_;
  ros::Timer timer_ultra_;

  VelocityPID v_pid_;
  DealUltrasonic deal_ultra_;

  sweeper::common::WatchDog watch_dog_path_;
  sweeper::common::WatchDog watch_dog_fusion_lidar_;
  sweeper::common::WatchDog watch_dog_chassis_;

  sweeper::common::PolygonUtil polygon_util_;
};

}  // namespace controll
}  // namespace navigation
}  // namespace sweeper

#include "../include/controll/controll.h"

#include <thread>

#include "../include/controll/pid.h"

namespace sweeper {
namespace navigation {
namespace controll {
Controller::Controller(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : v_(0.0),
      v_stop_(0.0),
      v_current_(0.0),
      ld_(3.0),
      count_(0),
      path_type_(-1),
      empty_path_count_(0),
      fstop_nums_(0),
      rstop_nums_(0),
      lstop_nums_(0),
      sweeper_model_(0),
      goal_reached_(true),
      flag_lookforward_(false),
      tf_listener_(tf_buffer_),
      threshold_yaw_(0.2),
      flag_ultra_foward_(true),
      flag_ultra_left_(true),
      flag_ultra_right_(true),
      map_frame_id_("aft_mapped"),
      sweeper_frame_id_("camera_init"),
      point_type_("normal"),
      point_type_normal_("normal"),
      point_type_left_avoidance_("left"),
      point_type_right_avoidance_("right"),
      v_pid_(nh_private),
      estop_flag_(false),
      fstop_flag_(false),
      lstop_flag_(false),
      rstop_flag_(false),
      sweeper_curb_speed_model_(0),
      pp_steering_angle_last_(0),
      lead_correction_compensation_last_(0),
      steering_angle_cmd_last_(0),
      cmd_steering_angle_last_(0) {
  nh_private.param<double>("curb_v_min", curb_v_min_, 0.28);
  nh_private.param<double>("curb_v_middle", curb_v_middle_, 0.83);
  nh_private.param<double>("curb_v_max", curb_v_max_, 1.3);

  nh_private.param<double>("trace_v_min", trace_v_min_, 0.28);
  nh_private.param<double>("trace_v_middle", trace_v_middle_, 0.83);
  nh_private.param<double>("trace_v_max", trace_v_max_, 1.4);

  nh_private.param<double>("fre_control", fre_control_, 10.0);
  nh_private.param<double>("fre_ultra", fre_ultra_, 10.0);
  nh_private.param<double>("wheelCurvlebase", L_, 1.4);

  nh_private.param<double>("curb_v_min_lookahead_distance",
                           curb_v_min_lookahead_distance_, 3.0);
  nh_private.param<double>("curb_v_min_lookahead_distance",
                           curb_v_min_lookahead_distance2_, 3.5);
  nh_private.param<double>("curb_v_middle_lookahead_distance",
                           curb_v_middle_lookahead_distance_, 4.0);
  nh_private.param<double>("curb_v_middle_lookahead_distance2",
                           curb_v_middle_lookahead_distance2_, 4.5);
  nh_private.param<double>("curb_v_max_lookahead_distance",
                           curb_v_max_lookahead_distance_, 5.0);
  nh_private.param<double>("curb_v_max_lookahead_distance2",
                           curb_v_max_lookahead_distance2_, 5.5);

  nh_private.param<double>("trace_v_min_lookahead_distance",
                           trace_v_min_lookahead_distance_, 2.0);
  nh_private.param<double>("trace_v_min_lookahead_distance2",
                           trace_v_min_lookahead_distance2_, 2.5);
  nh_private.param<double>("trace_v_middle_lookahead_distance",
                           trace_v_middle_lookahead_distance_, 3.0);
  nh_private.param<double>("trace_v_middle_lookahead_distance2",
                           trace_v_middle_lookahead_distance2_, 3.5);
  nh_private.param<double>("trace_v_max_lookahead_distance",
                           trace_v_max_lookahead_distance_, 4.0);
  nh_private.param<double>("trace_v_max_lookahead_distance2",
                           trace_v_max_lookahead_distance2_, 4.5);

  nh_private.param<double>("position_tolerance", pos_tol_, 0.2);
  nh_private.param<double>("steering_angle_limit", delta_max_, 0.786);
  nh_private.param<double>("turn_left_threshold", turn_left_threshold_, 10.0);
  nh_private.param<double>("control_angular_para", control_angular_para_, 8.0);
  nh_private.param<double>("angular_max", angular_max_, 200.0);
  nh_private.param<double>("angular_min", angular_min_, 100.0);
  nh_private.param<double>("diff_steering_limited", diff_steering_limited_,
                           15.0);

  nh_private.param<double>("proportional_gain_steering",
                           proportional_gain_steering_, 1.35);
  nh_private.param<double>("factor_steering_angle1", factor_steering_angle1_,
                           0.9);
  nh_private.param<double>("factor_steering_angle2", factor_steering_angle2_,
                           0.1);

  nh_private.param<double>("ultra_forward_stopping_distance",
                           ultra_forward_stopping_distance_, 1.5);
  nh_private.param<double>("ultra_lateral_stopping_distance",
                           ultra_lateral_stopping_distance_, 1.6);
  nh_private.param<double>("ultra_lateral_dynamic_distance",
                           ultra_lateral_dynamic_distance_, 0.5);

  nh_private.param<double>("kgc", kgc_, 1.1);
  nh_private.param<double>("coeffi_a", coeffi_a_, 1.0);
  nh_private.param<double>("coeffi_b", coeffi_b_, 0.2);

  nh_private.param<double>("estop_thickness", estop_thickness_, 0.2);
  nh_private.param<double>("left_width", left_width_, 0.6);
  nh_private.param<double>("right_width", right_width_, 0.6);
  nh_private.param<double>("front_length", front_length_, 1.0);
  nh_private.param<double>("back_length", back_length_, 1.0);
  nh_private.param<double>("e_back_length", e_back_length_, 0.5);
  nh_private.param<double>("forward_x", forward_x_, 1.0);

  nh_private.param<double>("fstop_line", fstop_line_, 0.6);
  nh_private.param<double>("lstop_line", lstop_line_, 0.6);
  nh_private.param<double>("rstop_line", rstop_line_, 1.0);
  nh_private.param<double>("bstop_line", bstop_line_, 1.0);

  nh_private.param<int>("emerge_nums", emerge_nums_, 10);
  nh_private.param<int>("forward_nums", forward_nums_, 10);
  nh_private.param<int>("lateral_nums", lateral_nums_, 4);

  nh_private.param<int>("allowed_nums_empty_path", allowed_nums_empty_path_, 3);
  nh_private.param<int>("valid_stop_nums", valid_stop_nums_, 2);
  nh_private.param<bool>("flag_is_fusion", flag_is_fusion_, true);
  nh_private.param<bool>("flag_lead_correct_compensation",
                         flag_lead_correct_compensation_, false);
  nh_private.param<bool>("flag_smooth_steering", flag_smooth_steering_, true);
  nh_private.param<bool>("flag_ultra_test", flag_ultra_test_, false);

  cmd_sweeper_.header.frame_id = sweeper_frame_id_;
  cmd_sweeper_.steering_angle = 0.0;
  cmd_sweeper_.throttle = 0.0;
  cmd_sweeper_.brake = 0.0;

  sub_path_ = nh.subscribe("/path", 1, &Controller::ReceivePath, this);
  sub_chassis_ = nh.subscribe("/sweeper/chassis/detail", 1,
                              &Controller::ReceiveSweeperChassis, this);
  sub_ultrasonic_ = nh.subscribe("/sweeeper/sensor/ultrasonic", 1,
                                 &Controller::ReceiveUltrasonicDatas, this);
  sub_lidar_ = nh.subscribe("/sweeper/sensor/fusion/lidar_radar", 1,
                            &Controller::ReceiveLidarDatas, this);
  sweep_mission_sub_ = nh.subscribe("/sweeper/sweep_mode", 1,
                                    &Controller::SweepModeCallback, this);

  pub_sweeper_cmd_ =
      nh.advertise<sweeper_msgs::SweeperCmd>("/sweeper/control/cmd", 1);
  pub_state_information_ = nh.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/common/diagnose", 1);
  pub_ultra_cmd_ = nh.advertise<sweeper_msgs::SweeperUltraCmd>(
      "/sweeper/control/ultra_cmd", 1);

  timer_control_ = nh.createTimer(ros::Duration(1 / fre_control_),
                                  &Controller::ComputeSweeperCmd, this);
  timer_ultra_ = nh.createTimer(ros::Duration(1 / fre_ultra_),
                                &Controller::SendUltraCmd, this);
}

void Controller::SweepModeCallback(
    const sweeper_msgs::SweepMission &sweep_mission) {
  if (sweep_mission.mode == 4 || sweep_mission.mode == 5) {
    if (sweep_mission.header.frame_id == "curb_driving_state")
      sweeper_model_ = 7;
    else if (sweep_mission.header.frame_id == "manual_driving_state")
      sweeper_model_ = 0;
    else if (sweep_mission.header.frame_id == "gnss_trace_state")
      sweeper_model_ = sweep_mission.mode;
    else if (sweep_mission.header.frame_id == "lidar_trace_state")
      sweeper_model_ = sweep_mission.mode;

  } else if (sweep_mission.mode == 7)
    sweeper_model_ = sweep_mission.mode;
  ADEBUG << "sweeper_model_:" << sweeper_model_;
}

void Controller::ReceivePath(nav_msgs::Path new_path) {
  if (new_path.poses.size() > 0) {
    path_ = new_path;
    goal_reached_ = false;
    watch_dog_path_.UpdataNow();
    empty_path_count_ = 0;
  } else {
    if ((empty_path_count_ - INT16_MAX) >= 0) {
      empty_path_count_ = allowed_nums_empty_path_ + 1;
    } else {
      ++empty_path_count_;
    }

    if (empty_path_count_ > allowed_nums_empty_path_) {
      goal_reached_ = true;
      // AERROR << "Receive empty path for three consecutive times !";
      path_ = nav_msgs::Path();
    }
  }
}

void Controller::SendUltraCmd(const ros::TimerEvent &e) {
  sweeper_msgs::SweeperUltraCmd ultra_cmds;
  double ultra_lateral = 0;
  int path_points_nums = path_.poses.size();

  if (path_points_nums > 0) {
    double distance = ld_;
    for (unsigned int j = 0; j < path_points_nums; ++j) {
      double tmp_x = path_.poses[j].pose.position.x;
      double tmp_y = path_.poses[j].pose.position.y;
      double tmp_distance = CalculateDistance(tmp_x, tmp_y, 0);
      if (tmp_distance >= distance) {
        ultra_lateral = path_.poses[j].pose.position.y;
        break;
      }
    }

    if (ultra_lateral > 0.2) {
      ultra_cmds.cmd.push_back(FRONTLEFTSIDE);
      ultra_cmds.cmd.push_back(MIDDLELEFTSIDE);
      ultra_cmds.cmd.push_back(REARLEFTSIDE);
    } else if (ultra_lateral < -0.2) {
      ultra_cmds.cmd.push_back(FRONTRIGHTSIDE);
      ultra_cmds.cmd.push_back(MIDDLERIGHTSIDE);
      ultra_cmds.cmd.push_back(REAERRIGHTSIDE);
    }

    ultra_cmds.header.frame_id = "ultrasonic";
    ultra_cmds.header.stamp = ros::Time::now();
    pub_ultra_cmd_.publish(ultra_cmds);

  } else {
    ultra_cmds.header.frame_id = "ultrasonic";
    ultra_cmds.header.stamp = ros::Time::now();
    pub_ultra_cmd_.publish(ultra_cmds);
  }
}

void Controller::ComputeSweeperCmd(const ros::TimerEvent &e) {
  if (++count_ % UINT_MAX == 0) count_ = 0;
  try {
    if (!path_.poses.empty()) {
      // 获取path的属性
      std::string pathtype = path_.header.frame_id;
      path_type_ = atoi(pathtype.c_str());

      //获取路径点的属性
      point_type_ = path_.poses.front().header.frame_id;

      ADEBUG << "path_type_:" << path_type_;
      ADEBUG << "point_type_:" << point_type_;
    }

    if ((sweeper_model_ == 4 || sweeper_model_ == 5) && !path_.poses.empty()) {
      if ((point_type_ == point_type_left_avoidance_) ||
          (point_type_ == point_type_right_avoidance_)) {
        v_set_ = trace_v_middle_;
        ld_ = trace_v_middle_lookahead_distance_;
        ld_2_ = trace_v_middle_lookahead_distance2_;
      } else if (point_type_ == point_type_normal_) {
        switch (path_type_) {
          case 0:
            v_set_ = trace_v_middle_;
            ld_ = trace_v_middle_lookahead_distance_;
            ld_2_ = trace_v_middle_lookahead_distance2_;
          case 2:
            v_set_ = trace_v_middle_;
            ld_ = curb_v_middle_lookahead_distance_;
            ld_2_ = curb_v_middle_lookahead_distance2_;
          case 3:
            v_set_ = trace_v_middle_;
            ld_ = curb_v_middle_lookahead_distance_;
            ld_2_ = curb_v_middle_lookahead_distance2_;
          case 4:
            v_set_ = trace_v_middle_;
            ld_ = trace_v_min_lookahead_distance_;
            ld_2_ = trace_v_min_lookahead_distance2_;
            break;
          case 5:
            v_set_ = trace_v_min_;
            ld_ = trace_v_min_lookahead_distance_;
            ld_2_ = trace_v_min_lookahead_distance2_;
            break;
          case 6:
            v_set_ = trace_v_max_;
            ld_ = trace_v_max_lookahead_distance_;
            ld_2_ = trace_v_max_lookahead_distance2_;
            break;
          case 7:
            v_set_ = trace_v_max_;
            ld_ = trace_v_max_lookahead_distance_;
            ld_2_ = trace_v_max_lookahead_distance2_;
            break;
          case 8:
            v_set_ = trace_v_max_;
            ld_ = trace_v_max_lookahead_distance_;
            ld_2_ = trace_v_max_lookahead_distance2_;
            break;
          case 9:
            v_set_ = trace_v_middle_;
            // ld_ = trace_v_max_lookahead_distance_;
            // ld_2_ = trace_v_max_lookahead_distance2_;
            ld_ = curb_v_middle_lookahead_distance_;
            ld_2_ = curb_v_middle_lookahead_distance2_;
            break;
          case 10:
            v_set_ = trace_v_middle_;
            // ld_ = trace_v_max_lookahead_distance_;
            // ld_2_ = trace_v_max_lookahead_distance2_;
            ld_ = curb_v_middle_lookahead_distance_;
            ld_2_ = curb_v_middle_lookahead_distance2_;
            break;
        }
      }
    } else if (sweeper_model_ == 7 && !path_.poses.empty()) {
      if ((point_type_ == point_type_left_avoidance_) ||
          (point_type_ == point_type_right_avoidance_)) {
        v_set_ = curb_v_middle_;
        ld_ = curb_v_middle_lookahead_distance_;
        ld_2_ = curb_v_middle_lookahead_distance2_;
      } else if (point_type_ == point_type_normal_) {
        switch (sweeper_curb_speed_model_) {
          case 1:
            v_set_ = curb_v_min_;
            ld_ = curb_v_min_lookahead_distance_;
            ld_2_ = curb_v_min_lookahead_distance2_;
            break;
          case 0:
          case 2:
            v_set_ = curb_v_middle_;
            ld_ = curb_v_middle_lookahead_distance_;
            ld_2_ = curb_v_middle_lookahead_distance2_;
            break;
          case 3:
            v_set_ = curb_v_max_;
            ld_ = curb_v_max_lookahead_distance_;
            ld_2_ = curb_v_max_lookahead_distance2_;
            break;
        }
      }
    } else if (sweeper_model_ == 0) {
      v_set_ = v_stop_;
      ld_ = curb_v_max_lookahead_distance_;
      ld_2_ = curb_v_max_lookahead_distance2_;
    }

    ADEBUG << "ld_:" << ld_;

    unsigned int path_indx_ = 0;
    for (; path_indx_ < path_.poses.size(); ++path_indx_) {
      double tmp_x = path_.poses[path_indx_].pose.position.x;
      double tmp_y = path_.poses[path_indx_].pose.position.y;
      double tmp_distance = CalculateDistance(tmp_x, tmp_y, 0.0);
      if (tmp_distance >= ld_) {
        // Transformed lookahead to base_link frame is lateral error

        ld_ = CalculateDistance(path_.poses[path_indx_].pose.position.x,
                                path_.poses[path_indx_].pose.position.y, 0.0);
        lookahead_.transform.translation.x =
            path_.poses[path_indx_].pose.position.x;
        lookahead_.transform.translation.y =
            path_.poses[path_indx_].pose.position.y;
        lookahead_.transform.translation.z =
            path_.poses[path_indx_].pose.position.z;

        lookahead_.transform.rotation.x =
            path_.poses[path_indx_].pose.orientation.x;
        lookahead_.transform.rotation.y =
            path_.poses[path_indx_].pose.orientation.y;
        lookahead_.transform.rotation.z =
            path_.poses[path_indx_].pose.orientation.z;
        lookahead_.transform.rotation.w =
            path_.poses[path_indx_].pose.orientation.w;
        break;
      }
    }

    if (!path_.poses.empty() && path_indx_ < path_.poses.size()) {
      for (unsigned int i = 0; i < path_.poses.size(); ++i) {
        double tmp_x = path_.poses[i].pose.position.x;
        double tmp_y = path_.poses[i].pose.position.y;
        double tmp_distance = CalculateDistance(tmp_x, tmp_y, 0.0);
        if (tmp_distance >= ld_2_) {
          ld_2_ = CalculateDistance(path_.poses[i].pose.position.x,
                                    path_.poses[i].pose.position.y, 0.0);
          lookahead_2_.transform.translation.x = path_.poses[i].pose.position.x;
          lookahead_2_.transform.translation.y = path_.poses[i].pose.position.y;
          lookahead_2_.transform.translation.z = path_.poses[i].pose.position.z;

          lookahead_2_.transform.rotation.x = path_.poses[i].pose.orientation.x;
          lookahead_2_.transform.rotation.y = path_.poses[i].pose.orientation.y;
          lookahead_2_.transform.rotation.z = path_.poses[i].pose.orientation.z;
          lookahead_2_.transform.rotation.w = path_.poses[i].pose.orientation.w;

          flag_lookforward_ = true;
          break;
        }
      }
    }

    if (!path_.poses.empty() && path_indx_ >= path_.poses.size()) {
      // to judge the sweeper is reachead goal point
      if (fabs(path_.poses.back().pose.position.x) <= pos_tol_) {
        // reached the goal,Reset the path
        goal_reached_ = true;

        path_ = nav_msgs::Path();
      } else {
        double yaw;
        Eigen::Quaterniond point_quaternation(
            path_.poses.back().pose.orientation.w,
            path_.poses.back().pose.orientation.x,
            path_.poses.back().pose.orientation.y,
            path_.poses.back().pose.orientation.z);
        Eigen::Vector3d point_eulerAngle =
            point_quaternation.matrix().eulerAngles(2, 1, 0);
        yaw = point_eulerAngle(2);
        double k_end = tan(yaw);
        double l_end = path_.poses.back().pose.position.y -
                       k_end * path_.poses.back().pose.position.x;
        double a = 1 + k_end * k_end;
        double b = 2 * k_end * l_end;
        double c = l_end * l_end - ld_ * ld_;
        double D = sqrt(b * b - 4 * a * c);
        double x_ld = (-b + copysign(D, v_)) / (2 * a);
        double y_ld = k_end * x_ld + l_end;

        lookahead_.transform.translation.x = x_ld;
        lookahead_.transform.translation.y = y_ld;
        lookahead_.transform.translation.z = path_.poses.back().pose.position.z;

        lookahead_.transform.rotation.x = path_.poses.back().pose.orientation.x;
        lookahead_.transform.rotation.y = path_.poses.back().pose.orientation.y;
        lookahead_.transform.rotation.z = path_.poses.back().pose.orientation.z;
        lookahead_.transform.rotation.w = path_.poses.back().pose.orientation.w;
      }
    }

    //  find the point of veichle nearest to path and cmpute the point of
    //  preview
    unsigned idx_2 = 0;
    for (; idx_2 < path_.poses.size(); ++idx_2) {
      if (sqrt(pow(path_.poses[idx_2].pose.position.x, 2) +
               pow(path_.poses[idx_2].pose.position.y, 2)) >= ld_velocity_)
        break;
    }
    
    if (!goal_reached_) {
      // Compute steering angle and set velocity
      double ld_2 = 0.0;

      ld_2 = ld_ * ld_;

      double yt = -lookahead_.transform.translation.y;
      double steering_angle_1 =
          std::min(atan2(2 * yt * L_, ld_2), delta_max_) * Todegree;

      double ld2_2 = 0.0;
      double yt_2 = 0.0;
      double steering_angle_2 = 0.0;

      ADEBUG << "lateroal error: " << yt;
      double pp_steering_angle = 0;

      if (flag_lookforward_) {
        ld2_2 = pow(ld_2_, 2);
        yt_2 = -lookahead_2_.transform.translation.y;
        steering_angle_2 =
            std::min(atan2(2 * yt_2 * L_, ld2_2), delta_max_) * Todegree;

        pp_steering_angle = proportional_gain_steering_ *
                            (factor_steering_angle1_ * steering_angle_1 +
                             factor_steering_angle2_ * steering_angle_2);
        flag_lookforward_ = false;
      } else {
        pp_steering_angle = proportional_gain_steering_ * steering_angle_1;
      }

      double lead_correct_compensation = LeadCorrectCompensation(
          kgc_, coeffi_a_, coeffi_b_, pp_steering_angle,
          pp_steering_angle_last_, lead_correction_compensation_last_);

      pp_steering_angle_last_ = pp_steering_angle;
      lead_correction_compensation_last_ = lead_correct_compensation;

      if (flag_lead_correct_compensation_ &&
          (sweeper_model_ == 4 || sweeper_model_ == 5) && path_type_ != 4 &&
          v_set_ >= trace_v_max_) {
        if (v_current_ > 1.0) {
          double diff_steering =
              fabs(steering_angle_cmd_last_ - lead_correct_compensation);
          if (diff_steering > diff_steering_limited_ && flag_smooth_steering_) {
            cmd_sweeper_.steering_angle = 0.5 * lead_correct_compensation +
                                          0.5 * steering_angle_cmd_last_;
          } else {
            cmd_sweeper_.steering_angle = lead_correct_compensation;
          }
        }
        steering_angle_cmd_last_ = cmd_sweeper_.steering_angle;
      } else {
        cmd_sweeper_.steering_angle = pp_steering_angle;
        pp_steering_angle_last_ = 0;
        lead_correction_compensation_last_ = 0;
      }

      cmd_steering_angle_last_ = cmd_sweeper_.steering_angle;

      v_ = v_set_;

      cmd_sweeper_.steering_angular_speed =
          SetSteeringAngularSpeed(cmd_sweeper_.steering_angle, steering_angle_);

      if ((!flag_ultra_left_ || !flag_ultra_right_) && flag_is_fusion_) {
        v_ = v_stop_;
      }

      if ((estop_flag_ || fstop_flag_ || rstop_flag_ || lstop_flag_) &&
          flag_is_fusion_) {
        v_ = v_stop_;
      }

      if (v_ < 0.0001 && v_ > -0.001)
        cmd_sweeper_.steering_angle = cmd_steering_angle_last_;
      else
        cmd_steering_angle_last_ = cmd_sweeper_.steering_angle;

      if (cmd_sweeper_.steering_angle > -0.00001 &&
          cmd_sweeper_.steering_angle < 0.00001)
        cmd_sweeper_.steering_angle = 0.0;

      double pendal = SwitchVelocityToPendal(v_, v_current_);
      if (pendal > 0.0000001) {
        cmd_sweeper_.throttle = pendal;
        cmd_sweeper_.brake = 0.0;
      } else if (pendal < -0.0000001) {
        cmd_sweeper_.brake = -pendal;
        cmd_sweeper_.throttle = 0.0;
      }
      cmd_sweeper_.header.stamp = ros::Time::now();
      cmd_sweeper_.speed = v_;
    } else {
      // At the goal! Stop the vehicle
      cmd_sweeper_.header.stamp = ros::Time::now();
      cmd_sweeper_.speed = 0.0;
      cmd_sweeper_.steering_angular_speed = 150.0;
      cmd_sweeper_.steering_angle = cmd_steering_angle_last_;
      cmd_sweeper_.throttle = 0.0;
      cmd_sweeper_.brake = 0.0;
    }

    ADEBUG << "steering_angle:" << cmd_sweeper_.steering_angle;
    ADEBUG << "v_:" << v_;

    pub_sweeper_cmd_.publish(cmd_sweeper_);
    if (count_ % 4 == 0) SelfDiagnose();
  } catch (tf2::TransformException &ex) {
    AERROR << ex.what();
  }
}

void Controller::ReceiveSweeperChassis(
    const sweeper_msgs::SweeperChassisDetail &sweeprvchassis) {
  watch_dog_chassis_.UpdataNow();
  v_current_ = sweeprvchassis.vehicle_speed_output;
  steering_angle_ = sweeprvchassis.steering_angle_output;
  sweeper_curb_speed_model_ = (int)sweeprvchassis.chassis_speed_mode;
}

double Controller::SwitchVelocityToPendal(double v_cmd, double v_current) {
  double output = v_pid_.VelocityControl(v_cmd, v_current);
  return output;
}

double Controller::SetSteeringAngularSpeed(double steering_angle_cmd,
                                           double steering_angle_current) {
  double output = 0.0;
  double error = fabs(steering_angle_cmd - steering_angle_current);

  if (error > 3.0) {
    output = control_angular_para_ * error;
  } else {
    output = angular_min_;
  }

  if (output > angular_max_) {
    output = angular_max_;
  } else if (output < angular_min_) {
    output = angular_min_;
  }
  return output;
}

double Controller::LeadCorrectCompensation(
    const double kgc, const double coeffi_a, const double coeffi_b,
    const double value_before_correction,
    const double value_before_correction_last,
    const double value_after_correction_last) {
  const double correct_first_item = kgc * (1 / fre_control_ + 2 * coeffi_a) /
                                    (1 / fre_control_ + 2 * coeffi_b);
  const double correct_second_item = kgc * (1 / fre_control_ - 2 * coeffi_a) /
                                     (1 / fre_control_ + 2 * coeffi_b);
  const double correct_third_item =
      -(1 / fre_control_ - 2 * coeffi_b) / (1 / fre_control_ + 2 * coeffi_b);
  const double lead_correct_compensation =
      correct_first_item * value_before_correction +
      correct_second_item * value_before_correction_last +
      correct_third_item * value_after_correction_last;
  return lead_correct_compensation;
}

void Controller::ReceiveUltrasonicDatas(
    const sweeper_msgs::PerceptionObstacle &sweeper_ultrasonic) {
  int num = sweeper_ultrasonic.object.size();
  for (unsigned int m = 0; m < num; ++m) {
    deal_ultra_.InterpretUltrasonicData(sweeper_ultrasonic, m);
  }

  double ultra_detecing_width_left = ultra_lateral_stopping_distance_;
  double ultra_detecing_width_right = ultra_lateral_stopping_distance_;
  if (point_type_ == point_type_left_avoidance_) {
    ultra_detecing_width_left =
        ultra_lateral_stopping_distance_ + ultra_lateral_dynamic_distance_;
    ultra_detecing_width_right = ultra_lateral_stopping_distance_;
  } else if (point_type_ == point_type_right_avoidance_) {
    ultra_detecing_width_left = ultra_lateral_stopping_distance_;
    ultra_detecing_width_right =
        ultra_lateral_stopping_distance_ + ultra_lateral_dynamic_distance_;
  } else if (point_type_ == point_type_normal_) {
    ultra_detecing_width_left = ultra_lateral_stopping_distance_;
    ultra_detecing_width_right = ultra_lateral_stopping_distance_;
  }

  if (!path_.poses.empty()) {
    // if ((deal_ultra_.ultra_2_filter_data_ > 0.8 &&
    //      deal_ultra_.ultra_2_filter_data_ <=
    //          ultra_forward_stopping_distance_) ||
    //     (deal_ultra_.ultra_3_filter_data_ > 0.00000001 &&
    //      deal_ultra_.ultra_4_filter_data_ <=
    //          ultra_forward_stopping_distance_) ||
    //     (deal_ultra_.ultra_4_filter_data_>0.8 &&
    //     deal_ultra_.ultra_4_filter_data_<=
    //     ultra_forward_stopping_distance_)) {
    //   flag_ultra_foward_ = false;
    // } else {
    //   flag_ultra_foward_ = true;
    // }

    if (((deal_ultra_.ultra_1_filter_data_ > 0.00000001 &&
          deal_ultra_.ultra_1_filter_data_ <= ultra_detecing_width_left) ||
         (deal_ultra_.ultra_6_filter_data_ > 0.00000001 &&
          deal_ultra_.ultra_6_filter_data_ <= ultra_detecing_width_left) ||
         (deal_ultra_.ultra_8_filter_data_ > 0.0000001 &&
          deal_ultra_.ultra_8_filter_data_ <= ultra_detecing_width_left))) {
      flag_ultra_left_ = false;
    } else {
      flag_ultra_left_ = true;
    }

    if (((deal_ultra_.ultra_5_filter_data_ > 0.00001 &&
          deal_ultra_.ultra_5_filter_data_ <= ultra_detecing_width_right) ||
         (deal_ultra_.ultra_7_filter_data_ > 0.00001 &&
          deal_ultra_.ultra_7_filter_data_ <= ultra_detecing_width_right) ||
         (deal_ultra_.ultra_9_filter_data_ > 0.00001 &&
          deal_ultra_.ultra_9_filter_data_ <= ultra_detecing_width_right))) {
      flag_ultra_right_ = false;
    } else {
      flag_ultra_right_ = true;
    }
  }
}

void Controller::ReceiveLidarDatas(
    const sensor_msgs::PointCloud2 &lidar_datas) {
  watch_dog_fusion_lidar_.UpdataNow();
  if (path_.poses.empty()) return;
  sensor_msgs::PointCloud lidar_cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(lidar_datas, lidar_cloud);

  int estop_count = 0, fstop_count = 0;
  int lstop_count = 0, rstop_count = 0;

  double lstop_width = left_width_ + estop_thickness_;
  double rstop_width = right_width_ + estop_thickness_;
  double theta = -steering_angle_ * DEGREE_TO_RAD;
  sweeper::common::PolygonDType emergecy_area, f_area, l_area, r_area;
  emergecy_area.resize(6);
  emergecy_area[0].x = -0.45;
  emergecy_area[0].y = lstop_width;
  emergecy_area[1].x = -front_length_;
  emergecy_area[1].y = lstop_width;
  emergecy_area[2].x = -front_length_ - e_back_length_ * cos(theta);
  emergecy_area[2].y = lstop_width + e_back_length_ * sin(theta);
  emergecy_area[3].x = -front_length_ - e_back_length_ * cos(theta);
  emergecy_area[3].y = -rstop_width + e_back_length_ * sin(theta);
  emergecy_area[4].x = -front_length_;
  emergecy_area[4].y = -rstop_width;
  emergecy_area[5].x = -0.45;
  emergecy_area[5].y = -rstop_width;

  f_area.resize(4);
  f_area[0].x = fstop_line_;
  f_area[0].y = left_width_ + 0.15;
  f_area[1].x = 0.01;
  f_area[1].y = left_width_ + 0.15;
  f_area[2].x = 0.01;
  f_area[2].y = -right_width_ - 0.15;
  f_area[3].x = fstop_line_;
  f_area[3].y = -right_width_ - 0.15;

  l_area.resize(4);
  l_area[0].x = forward_x_;
  l_area[0].y = lstop_line_;
  l_area[1].x = -bstop_line_;
  l_area[1].y = lstop_line_;
  l_area[2].x = -bstop_line_;
  l_area[2].y = 0.01;
  l_area[3].x = forward_x_;
  l_area[3].y = 0.01;

  r_area.resize(4);
  r_area[0].x = forward_x_;
  r_area[0].y = -0.01;
  r_area[1].x = -bstop_line_;
  r_area[1].y = -0.01;
  r_area[2].x = -bstop_line_;
  r_area[2].y = -rstop_line_;
  r_area[3].x = forward_x_;
  r_area[3].y = -rstop_line_;

  bool obstacle_flag = false;
  bool left_curb_flag = false, right_curb_flag = false;
  if (path_type_ == 2 || path_type_ == 10)
    right_curb_flag = true;
  else if (path_type_ == 2 || path_type_ == 9) {
    left_curb_flag = true;
  }

  if (point_type_ == "left" || point_type_ == "right") obstacle_flag = true;

  for (size_t i = 0; i < lidar_cloud.points.size(); ++i) {
    pcl::PointXYZI point;
    point.x = lidar_cloud.points[i].x;
    point.y = lidar_cloud.points[i].y;
    point.z = lidar_cloud.points[i].z;

    if (polygon_util_.IsXyPointIn2dXyPolygon(point, emergecy_area)) {
      estop_count++;
    }
    if (estop_count > emerge_nums_) {
      AWARN << "Obstacle in emerge area.";
      estop_flag_ = true;
      break;
    } else
      estop_flag_ = false;
    if (polygon_util_.IsXyPointIn2dXyPolygon(point, f_area)) {
      fstop_count++;
    }
    if (fstop_count > forward_nums_) {
      AWARN << "Obstacle in front area.";
      fstop_flag_ = true;
      break;
    } else
      fstop_flag_ = false;
    if (right_curb_flag &&
        ((obstacle_flag && steering_angle_ < -1.0) || steering_angle_ < -3.0) &&
        polygon_util_.IsXyPointIn2dXyPolygon(point, l_area)) {
      lstop_count++;
    }
    if (lstop_count > lateral_nums_) {
      AWARN << "Obstacle in left lateral area.";
      lstop_flag_ = true;
      break;
    } else
      lstop_flag_ = false;
    if (left_curb_flag &&
        (((obstacle_flag && steering_angle_ > 1.0) || steering_angle_ > 3.0)) &&
        polygon_util_.IsXyPointIn2dXyPolygon(point, r_area)) {
      rstop_count++;
    }
    if (rstop_count > lateral_nums_) {
      AWARN << "Obstacle in right lateral area.";
      rstop_flag_ = true;
      break;
    } else
      rstop_flag_ = false;
  }
}

double Controller::CalculateDistance(const double x, const double y,
                                     const double z) {
  double output = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  return output;
}

void Controller::SelfDiagnose() {
  sweeper_msgs::SensorFaultInformation state_control;
  if (!watch_dog_path_.DogIsOk(3)) {
    state_control.state_code.push_back(3301);
    path_ = nav_msgs::Path();
    goal_reached_ = true;
  }

  if (!watch_dog_chassis_.DogIsOk(5)) {
    state_control.state_code.push_back(3302);
  }

  if (!watch_dog_fusion_lidar_.DogIsOk(5)) {
    state_control.state_code.push_back(3303);
  }

  if (fstop_flag_) {
    state_control.state_code.push_back(3207);
  }

  if (estop_flag_) {
    state_control.state_code.push_back(3305);
  }

  if (lstop_flag_) {
    state_control.state_code.push_back(3307);
  }

  if (rstop_flag_) {
    state_control.state_code.push_back(3308);
  }

  if (!flag_ultra_foward_ || !flag_ultra_left_ || !flag_ultra_right_) {
    state_control.state_code.push_back(3306);
  }

  if (state_control.state_code.empty()) {
    state_control.state_code.push_back(3300);
    state_control.header.frame_id = "control";
    state_control.header.stamp = ros::Time::now();
  } else {
    state_control.header.frame_id = "control";
    state_control.header.stamp = ros::Time::now();
  }
  pub_state_information_.publish(state_control);
}

}  // namespace controll
}  // namespace navigation
}  // namespace sweeper

#include "../include/sweeper_pose_ekf/sweeper_pose_ekf_based_lidar.h"

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>

namespace sweeper {
namespace navigation {
namespace localization {
namespace fusion_location {
SweeperLocalization::SweeperLocalization(ros::NodeHandle nh,
                                         ros::NodeHandle nh_private)
    : flag_filter_init_(false),
      open_fusion_localization_(false),
      untrusted_number_(0),
      receive_new_gnss_data_(false),
      receive_new_lidar_data_(false),
      receive_new_imu_data_(false),
      now_lidar_time_(0.0),
      now_gnss_time_(0.0),
      use_lidar_data_(true),
      use_gnss_data_(true),
      get_gps_to_map_(false),
      get_map_to_image_(false),
      flag_receive_imu_(false),
      angular_velocity_yaw_(0.0),
      angular_acc_(0.2) {
  nh_private.param<bool>("open_fusion_localization", open_fusion_localization_,
                         false);
  nh_private.param<bool>("use_lidar_data", use_lidar_data_, true);
  nh_private.param<bool>("use_gnss_data", use_gnss_data_, true);
  nh_private.param<double>("omga_variance", omga_variance_, 15.0);
  nh_private.param<double>("speed_acc", acc_, 1);
  nh_private.param<double>("slam_cov_mag", slam_cov_mag_, 0.03);

  sub_gps_ = nh.subscribe("/sweeper/localization/gnss", 1,
                          &SweeperLocalization::ReceiveGpsDatas, this);
  sub_slam_ = nh.subscribe("/sweeper/matcher/lidar_odom", 1,
                           &SweeperLocalization::ReceiveSlamDatas, this);
  sub_imu_ = nh.subscribe("/sweeper/sensor/imu", 1,
                          &SweeperLocalization::ReceiveImuDatas, this);
  sub_speed_ = nh.subscribe("/sweeper/main_control/chassis_detail", 1,
                            &SweeperLocalization::ReceiveSpeedDatas, this);

  sub_state_localization_information_ =
      nh.subscribe<sweeper_msgs::SensorFaultInformation>(
          "/sweeper/matcher_state", 1,
          &SweeperLocalization::ReceiveLocalizationDiagnose, this);
  sub_sweeper_mode_ = nh.subscribe<sweeper_msgs::SweepMission>(
      "/sweeper/main_control/sweep_mode", 1,
      &SweeperLocalization::ReceiveSweeperMode, this);

  pub_fusion_pose_ =
      nh.advertise<nav_msgs::Odometry>("/sweeper/localization/fusion_pose", 1);
  pub_map_pose_ =
      nh.advertise<nav_msgs::Odometry>("/sweeper/localization/map_pose", 1);
  // pub_speed_pose_ =
  //     nh.advertise<nav_msgs::Odometry>("/sweeper/localization/sweeper_pose",
  //     1);
  pub_gnss_pose_ =
      nh.advertise<nav_msgs::Odometry>("/sweeper/localization/gnss_pose", 1);
  pub_image_pose_ =
      nh.advertise<nav_msgs::Odometry>("/sweeper/localization/image_pose", 1);

  pub_fusion_state_information_ =
      nh.advertise<sweeper_msgs::SensorFaultInformation>("/sweeper/state_code",
                                                         1);
  timer_ekf_navigation_ = nh.createTimer(
      ros::Duration(0.2), &SweeperLocalization::GetSweeperInformation, this);

  R_slam_.setZero();
  R_gps_.setZero();
  last_pcd_name_ = "unname";
  std::cout << "param : " << omga_variance_ << " " << acc_ << " "
            << slam_cov_mag_ << std::endl;
  
  xyz_acc_ = Eigen::Vector3d(0.2, 0.2, 0.2);
  // std::string gnss_config_path =
  //     "../sweeper_ws/src/sweeper_haide/calibration/data/"
  //     "livox_to_gps.yaml";
  // GetCalibYamlParam(gnss_config_path, gnss_to_lidar_quater_,
  //                   gnss_to_lidar_pos_);

  // std::string mapping_config_path =
  //     "../sweeper_ws/src/sweeper_haide/modules/calibration/data/"
  //     "mapping_localization.yaml";
  // GetCalibGnssPoseYamlParam(mapping_config_path, gnss_origin_pos_);
}

void SweeperLocalization::ReceiveSpeedDatas(
    const sweeper_msgs::SweeperChassisDetail sweeper_chassis_detail) {
  watch_dog_vehical_.UpdataNow();
  if (!open_fusion_localization_) return;
  AINFO << "receive speed data";
  double now_speed_time = sweeper_chassis_detail.header.stamp.toSec();
  double kmph_to_mps = 1000.0 / 3600.0;
  double sweep_vehicle =
      sweeper_chassis_detail.vehicle_speed_output * kmph_to_mps * 0.93;

  double sweeper_vehicle_plane = sweep_vehicle;// * cos(imu_pitch_);
  double delta_t = now_speed_time - last_speed_time_;
  if (delta_t > 1.0) {
    last_speed_time_ = now_speed_time;
    return;
  }

  AWARN << "data : "
        << "sweep_vehicle : " << sweep_vehicle
        << " sweeper_vehicle_plane : " << sweeper_vehicle_plane
        << " delta_t: " << delta_t << " flag_filter_init_ "
        << flag_filter_init_;

  if (flag_filter_init_) {
    // //发布里程计,判断车速
    // static double odom_x = gps_observation_(0), odom_y = gps_observation_(1),
    //               odom_yaw = gps_observation_(2);
    // odom_x += sweeper_vehicle_plane * delta_t * cos(gps_observation_(2));
    // odom_y += sweeper_vehicle_plane * delta_t * sin(gps_observation_(2));
    // odom_yaw += angular_velocity_yaw_ * delta_t;

    // nav_msgs::Odometry sweeper_position;
    // sweeper_position.header.frame_id = "camera_init";
    // sweeper_position.header.stamp = ros::Time::now();
    // sweeper_position.pose.pose.position.x = odom_x;
    // sweeper_position.pose.pose.position.y = odom_y;
    // geometry_msgs::Quaternion observation_pose_quat =
    //     tf::createQuaternionMsgFromYaw(gps_observation_(2));
    // sweeper_position.pose.pose.orientation = observation_pose_quat;
    // pub_speed_pose_.publish(sweeper_position);
    /*
      预测模型
        x = v/w * sin(w * delta_t + yaw) - v/w * sin(yaw) +x0;
        y = -v/w * cos(w * delta_t + yaw) + v/w* cos(yaw) +y0;
        v=v;
        yaw = yaw0 + w*delta_t
        w =w;
        速度取平均速度，
     */
    //积分得到imu预测的位置
    
    double predict_angle_velocity, predict_x, predict_y;
    if (flag_receive_imu_) {
      flag_receive_imu_ = false;
      predict_angle_velocity = (angular_velocity_yaw_ + x_last_(4)) / 2.0;
    } else {
      predict_angle_velocity = x_last_(4);
    }
    sweeper_vehicle_plane = x_last_(2);
    // x y v, yaw ,w
    double predict_v = sweeper_vehicle_plane;
    double predict_yaw = predict_angle_velocity * delta_t + x_last_(3);

    if (fabs(predict_angle_velocity) > 0.00001) {
      AWARN << "cin";
      predict_x = x_last_(0) +
                  ((sweeper_vehicle_plane + x_last_(2)) /
                   (2.0 * predict_angle_velocity)) *
                      sin(predict_angle_velocity * delta_t + x_last_(3)) -
                  ((sweeper_vehicle_plane + x_last_(2)) /
                   (2.0 * predict_angle_velocity)) *
                      sin(x_last_(3));

      predict_y = x_last_(1) -
                  ((sweeper_vehicle_plane + x_last_(2)) /
                   (2.0 * predict_angle_velocity)) *
                      cos(predict_angle_velocity * delta_t + x_last_(3)) +
                  ((sweeper_vehicle_plane + x_last_(2)) /
                   (2.0 * predict_angle_velocity)) *
                      cos(x_last_(3));
    } else {
      predict_y = x_last_(1) + predict_v * sin(x_last_(3)) * delta_t;
      predict_x = x_last_(0) + predict_v * cos(x_last_(3)) * delta_t;
    }
    NormalizeYaw(predict_yaw);
    AWARN << "predict data : " << predict_x << " " << predict_y << " "
          << predict_v << " " << predict_yaw << " " << predict_angle_velocity;
    Eigen::Matrix<double, 5, 1> x_predict;
    x_predict << predict_x, predict_y, predict_v, predict_yaw,
        predict_angle_velocity;
    x_last_ = x_predict;
    AWARN << "before predict: " << x_last_(0) << " " << x_last_(1) << " "
          << x_last_(2) << " " << x_last_(3) << " " << x_last_(4);
    //计算雅可比矩阵
    UpdateJacobianMatrix(x_last_, delta_t);
    ekf_filter_.SetTransitionMatrix(jacobian_);
    Eigen::Matrix<double, 5, 5> Q =
        SetProcessNoise(delta_t);
    ekf_filter_.SetTransitionNoise(Q);
    EkfLastParam ekf_last_param;
    //更新状态协方差矩阵p
    Eigen::Matrix<double, 1, 1> input;
    ekf_filter_.GetP();
    ekf_filter_.x_ = x_predict;
    ekf_last_param.last_x = x_predict;
    ekf_last_param.last_p = ekf_filter_.GetStateCovariance();
    ekf_last_params_.push_front(ekf_last_param);

    if (receive_new_gnss_data_) {
      receive_new_gnss_data_ = false;
      double ekf_yaw = x_last_(3);
      double yaw_error = NormalizeYaw(gps_observation_(2) - ekf_yaw);
      gps_observation_(2) = ekf_yaw + yaw_error;
      //判断数据是否符合
      // if (fabs(x_predict(0) - gps_observation_(0)) < 0.3 &&
      //         fabs(x_predict(1) - gps_observation_(1)) < 0.3 &&
      //         fabs(yaw_error) < 0.45 ||
      //     untrusted_number_ > 30)  // 25
      if(1)
      {
        AWARN << "gnss data is right !";
        AWARN << "gnss data: " << gps_observation_;
        ekf_filter_.SetObservationNoise(R_gps_);
        ekf_filter_.Correct(gps_observation_);
        x_last_ = ekf_filter_.x_;
        untrusted_number_ = 0;
        AWARN << "gnss : "
              << " x_last_ x: " << x_last_(0) << " y: " << x_last_(1)
              << " v: " << x_last_(2) << " yaw: " << x_last_(3)
              << " w: " << x_last_(4);
      } else {
        AWARN << "gnss data is error !"
              << "\n"
              << "predict: "
              << " x: " << x_predict(0) << " y:" << x_predict(1)
              << " yaw : " << x_predict(3) * 57.3 << "\n"
              << "gnss data: "
              << " x : " << gps_observation_(0) << " y: " << gps_observation_(1)
              << " yaw " << gps_observation_(2) * 57.3;
      }
    }

    if (receive_new_lidar_data_) {
      AWARN << "Receive new lidar data";
      receive_new_lidar_data_ = false;
      //找到临近的预测位置
      predict_delta_time_ = now_speed_time - now_lidar_time_;
      int delay_times = round(predict_delta_time_ / 0.02);
      AWARN << "lidar delta time " << predict_delta_time_
            << " times: " << delay_times;
      if (delay_times < 20 && delay_times < ekf_last_params_.size()) {
        //将过去预测的x和状态协方差p导入
        EkfLastParam lidar_ekf_param = ekf_last_params_[delay_times];
        //检查是否在区间内，并判断数据是否符合
        double ekf_yaw = lidar_ekf_param.last_x(3);
        double yaw_error = NormalizeYaw(lidar_observation_(2) - ekf_yaw);
        lidar_observation_(2) = ekf_yaw + yaw_error;
        if (/*fabs(lidar_ekf_param.last_x(0) - lidar_observation_(0)) < 0.3 &&
                fabs(lidar_ekf_param.last_x(1) - lidar_observation_(1)) < 0.3 &&
                fabs(yaw_error) < 0.45 ||
            untrusted_number_ > 30*/1) {
          ekf_filter_.SetStateCovariance(lidar_ekf_param.last_p);
          ekf_filter_.x_ = lidar_ekf_param.last_x;
          AWARN << "lidar data is right !";

          ekf_filter_.SetObservationNoise(R_slam_);
          ekf_filter_.Correct(lidar_observation_);

          //预测到当前时刻
          UpdateJacobianMatrix(ekf_filter_.x_, predict_delta_time_);
          ekf_filter_.SetTransitionMatrix(jacobian_);
          ekf_filter_.Predict(input);
          x_last_ = ekf_filter_.x_;
          untrusted_number_ = 0;
          AWARN << "lidar data: " << lidar_observation_;
          AWARN << "delay_times : " << delay_times
                << " x_last_ x: " << x_last_(0) << " y: " << x_last_(1)
                << " v: " << x_last_(2) << " yaw: " << x_last_(3)
                << " w: " << x_last_(4);
        } else {
          AWARN << "lidar data is error !"
                << "\n"
                << "predict: "
                << " x: " << lidar_ekf_param.last_x(0)
                << " y:" << lidar_ekf_param.last_x(1)
                << " yaw : " << lidar_ekf_param.last_x(3) * 57.3 << "\n"
                << "gnss data: "
                << " x : " << lidar_observation_(0)
                << " y: " << lidar_observation_(1) << " yaw "
                << lidar_observation_(2) * 57.3;
        }
      } else {
        AWARN << "lidar delta time " << predict_delta_time_
              << " times: " << delay_times;
      }
    }
    x_last_ = ekf_filter_.x_;
    AWARN << "correct : " << x_last_(0) << " " << x_last_(1) << " "
          << x_last_(2) << " " << x_last_(3) << " " << x_last_(4) << " "
          << x_last_(5) << " " << x_last_(6);
    if (ekf_last_params_.size() > 20) {
      ekf_last_params_.pop_back();
    }
    untrusted_number_++;
    fusion_yaw_last_ = x_last_(3);
    tf::Vector3 fusion_pose_position(x_last_(0), x_last_(1), 0.0);
    tf::Quaternion fusion_pose_quat =
        tf::createQuaternionFromYaw(fusion_yaw_last_);

    //发布位置
    std::string localization_frame_id;
    if (untrusted_number_ < 10)
      localization_frame_id = "best";
    else if (untrusted_number_ < 40)
      localization_frame_id = "good";
    else
      localization_frame_id = "bad";
    nav_msgs::Odometry fusion_position;
    fusion_position.header.frame_id = "lidar_map";
    fusion_position.child_frame_id = localization_frame_id;
    fusion_position.header.stamp = sweeper_chassis_detail.header.stamp;

    // AWARN << "fusion pos : " << fusion_position.pose.pose.position.x
    //       << " y: " << fusion_position.pose.pose.position.y
    //       << " yaw: " << fusion_yaw_last_ * 57.3;
    fusion_position.pose.pose.position.x = fusion_pose_position.x();
    fusion_position.pose.pose.position.y = fusion_pose_position.y();
    fusion_position.pose.pose.position.z = fusion_pose_position.z();

    fusion_position.pose.pose.orientation.x = fusion_pose_quat.x();
    fusion_position.pose.pose.orientation.y = fusion_pose_quat.y();
    fusion_position.pose.pose.orientation.z = fusion_pose_quat.z();
    fusion_position.pose.pose.orientation.w = fusion_pose_quat.w();
    pub_fusion_pose_.publish(fusion_position);

    //发布tf
    static tf::TransformBroadcaster br;
    tf::Transform transform1;
    transform1.setOrigin(tf::Vector3(x_last_(0), x_last_(1), 0.0));
    transform1.setRotation(fusion_pose_quat);
    br.sendTransform(tf::StampedTransform(transform1,
                                          sweeper_chassis_detail.header.stamp,
                                          "/base_link", "/lidar_map"));
    static tf::TransformBroadcaster br_cost;
    tf::Transform transform_90 =
        tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, 3.1415 / 2.0),
                      tf::Vector3(0.0, 0.0, 0.0));
    br_cost.sendTransform(
        tf::StampedTransform(transform_90, sweeper_chassis_detail.header.stamp,
                             "/lidar_map", "/map"));

    if (get_gps_to_map_) {
      static tf::TransformBroadcaster br_cost;
      tf::Transform gnss_to_map = tf::Transform(
          tf::Quaternion(gnss_to_map_quater_.x(), gnss_to_map_quater_.y(),
                         gnss_to_map_quater_.z(), gnss_to_map_quater_.w()),
          tf::Vector3(gnss_to_map_pos_.x(), gnss_to_map_pos_.y(),
                      gnss_to_map_pos_.z()));
      br_cost.sendTransform(
          tf::StampedTransform(gnss_to_map, sweeper_chassis_detail.header.stamp,
                               "/gnss", "/lidar_map"));
    }

    if (get_map_to_image_) {
      //发布图像位置
      nav_msgs::Odometry image_pose;
      image_pose.header.frame_id = "image";
      image_pose.child_frame_id = fusion_position.child_frame_id;
      image_pose.header.stamp = sweeper_chassis_detail.header.stamp;
      image_pose.pose.pose.position.y =
          (-fusion_pose_position.x() + origin_x_) / resolution_;
      image_pose.pose.pose.position.x =
          (-fusion_pose_position.y() + origin_y_) / resolution_;
      fusion_position.pose.pose.orientation.x = fusion_pose_quat.x();
      fusion_position.pose.pose.orientation.y = fusion_pose_quat.y();
      fusion_position.pose.pose.orientation.z = fusion_pose_quat.z();
      fusion_position.pose.pose.orientation.w = fusion_pose_quat.w();
      // image_pose.pose.pose.orientation =
      //     tf::createQuaternionMsgFromYaw(fusion_yaw_last_);
      pub_image_pose_.publish(image_pose);
    }

    //发布map位置
    nav_msgs::Odometry map_pose;
    map_pose.header.frame_id = "map";
    map_pose.child_frame_id = fusion_position.child_frame_id;
    map_pose.header.stamp = fusion_position.header.stamp;
    tf::Vector3 map_pose_position = transform_90 * fusion_pose_position;
    tf::Quaternion map_pose_quater = transform_90 * fusion_pose_quat;
    map_pose.pose.pose.position.x = map_pose_position.x();
    map_pose.pose.pose.position.y = map_pose_position.y();
    map_pose.pose.pose.position.z = map_pose_position.z();

    map_pose.pose.pose.orientation.x = map_pose_quater.x();
    map_pose.pose.pose.orientation.y = map_pose_quater.y();
    map_pose.pose.pose.orientation.z = map_pose_quater.z();
    map_pose.pose.pose.orientation.w = map_pose_quater.w();
    pub_map_pose_.publish(map_pose);

  } else if (receive_new_gnss_data_) {
    AWARN << "start init gnss";
    receive_new_gnss_data_ = false;
    x_last_(0) = gps_observation_(0);
    x_last_(1) = gps_observation_(1);
    x_last_(2) = sweeper_vehicle_plane;
    x_last_(3) = gps_observation_(2);
    x_last_(4) = angular_velocity_yaw_;

    AWARN << "receive gnss to init";
    EkfInit();
  } else if (receive_new_lidar_data_ && R_slam_(0, 0) < 0.5) {
    AWARN << "start init gnss";
    receive_new_lidar_data_ = false;
    x_last_(0) = lidar_observation_(0);
    x_last_(1) = lidar_observation_(1);
    x_last_(2) = sweeper_vehicle_plane;
    x_last_(3) = lidar_observation_(2);
    x_last_(4) = angular_velocity_yaw_;
    AWARN << "receive lidar to init";
    EkfInit();
    use_gnss_data_ = false;
  }

  last_speed_time_ = now_speed_time;
}

Eigen::Matrix<double, 5, 5> SweeperLocalization::SetProcessNoise(double dt) {
  Eigen::Matrix<double, 5, 5> Q;
  double linear_acc =
      sqrt(xyz_acc_.x() * xyz_acc_.x() + xyz_acc_.y() * xyz_acc_.y());
  double angular_acc = angular_acc_;

  Q << pow(0.5 * dt * dt * linear_acc * cos(x_last_(3)), 2),
      0.25 * pow(dt * dt * linear_acc, 2) * sin(x_last_(3)) * cos(x_last_(3)),
      0.5 * dt * dt * dt * linear_acc * linear_acc * cos(x_last_(3)), 0.0, 0.0,
      pow(0.5 * dt * dt * linear_acc, 2) * sin(x_last_(3)) * cos(x_last_(3)),
      pow(0.5 * dt * dt * linear_acc * sin(x_last_(3)), 2),
      0.5 * dt * dt * dt * linear_acc * linear_acc * sin(x_last_(3)), 0.0, 0.0,
      0.5 * dt * dt * dt * linear_acc * linear_acc * cos(x_last_(3)),
      0.5 * dt * dt * dt * linear_acc * linear_acc * sin(x_last_(3)),
      dt * dt * linear_acc * linear_acc, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.25 * pow(dt, 4) * pow(angular_acc, 2),
      0.5 * pow(dt, 3) * pow(angular_acc, 2), 0.0, 0.0, 0.0,
      0.5 * pow(dt, 3) * pow(angular_acc, 2), pow(dt, 2) * pow(angular_acc, 2);
  return Q;
}

void SweeperLocalization::ReceiveImuDatas(
    const sensor_msgs::Imu::ConstPtr &input) {
  watch_dog_imu_.UpdataNow();
  AINFO << "receive imu data";
  const ros::Time current_time = input->header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  double imu_yaw, imu_roll, imu_pitch;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  imu_pitch_ = RpyToSignRpy(imu_pitch);
  imu_yaw = RpyToSignRpy(imu_yaw);
  static double previous_imu_yaw = imu_yaw;
  // const double diff_imu_yaw = CalcDiffForRadian(imu_yaw, previous_imu_yaw);
  const double diff_imu_yaw = NormalizeYaw(imu_yaw - previous_imu_yaw);
  // angular_velocity_yaw_ = diff_imu_yaw / diff_time;
  angular_velocity_yaw_ = input->angular_velocity.z;

  //去除重力加速度
  double acc_x = input->linear_acceleration.x + sin(imu_pitch_) * 9.81;
  double acc_y =
      input->linear_acceleration.y - sin(imu_roll) * cos(imu_pitch_) * 9.81;

  double acc_z =
      input->linear_acceleration.z - cos(imu_roll) * cos(imu_pitch_) * 9.81;
  xyz_acc_ = Eigen::Vector3d(acc_x, acc_y, acc_z);

  angular_acc_ = input->angular_velocity.z / diff_time;

  previous_time = current_time;
  // previous_imu_yaw = imu_yaw;
  receive_new_imu_data_ = true;
  AINFO << "receive imu data ok!";
}

void SweeperLocalization::ReceiveSweeperMode(
    sweeper_msgs::SweepMission sweeper_mode_command) {
  std::string frame_id = sweeper_mode_command.header.frame_id;
  if (sweeper_mode_command.mode != 2 && frame_id != "mapping") {
    switch (sweeper_mode_command.cmd)
    case 1:  //开始
    {
      if (sweeper_mode_command.map.size() <= 0 ||
          !sweeper_mode_command.map.compare(last_pcd_name_))
        break;
      position_processing_ = true;
      // get_gps_to_map_yaml
      std::string gnss_config_path = "../sweeper_ws/src/sweeper_haide/data/map/" +
                                     sweeper_mode_command.map +
                                     "/gps_to_map.yaml";
      get_gps_to_map_ = GetCalibYamlParam(gnss_config_path, gnss_to_map_quater_,
                                          gnss_to_map_pos_);
      // image_yaml
      std::string image_config_path =
          "../sweeper_ws/src/sweeper_haide/data/map/" + sweeper_mode_command.map +
          "/map_to_image.yaml";
      get_map_to_image_ = GetImageYamlParam(image_config_path, origin_x_,
                                            origin_y_, resolution_);

      ekf_filter_.ExtendedKalmanFilterClear();
      use_lidar_data_ = true;
      last_pcd_name_ = sweeper_mode_command.map;
      open_fusion_localization_ = true;
      AWARN << "start fusion localization!";
      break;
    }

  } else {
    use_lidar_data_ = false;
    last_pcd_name_ = "unname";
    open_fusion_localization_ = false;
    flag_filter_init_ = false;
    ekf_filter_.ExtendedKalmanFilterClear();
    AWARN << "not open fusion localization!";
    get_map_to_image_ = false;
    get_gps_to_map_ = false;
    position_processing_ = false;
  }
}

void SweeperLocalization::ReceiveLocalizationDiagnose(
    const sweeper_msgs::SensorFaultInformation localization_state) {
  for (size_t i = 0; i < localization_state.state_code.size(); i++) {
    receive_localization_state_code_.push_back(
        localization_state.state_code[i]);
  }
}

void SweeperLocalization::GetSweeperInformation(const ros::TimerEvent &e) {
  sweeper_msgs::SensorFaultInformation fusion_navigation_code;
  fusion_navigation_code.header.frame_id = "localization";
  fusion_navigation_code.header.stamp = ros::Time::now();

  if (position_processing_) {
    for (int i = 0; i < receive_localization_state_code_.size(); i++) {
      fusion_navigation_code.state_code.push_back(
          receive_localization_state_code_[i]);
    }
  }
  receive_localization_state_code_.clear();

  if (!watch_dog_imu_.DogIsOk(3)) {  //没有接收imu
    fusion_navigation_code.state_code.push_back(UN_RECEIVED_IMU_DATA_);
  }
  if (!watch_dog_vehical_.DogIsOk(3)) {  //没有接收车速信息
    fusion_navigation_code.state_code.push_back(UN_RECEIVED_VEHICAL_DATA_);
  }
  if (!watch_dog_gnss_.DogIsOk(3)) {  //没有接收gps融合数据
    fusion_navigation_code.state_code.push_back(UN_RECEIVED_GNSS_DATA_);
  }
  if (!watch_dog_lidar_odom_.DogIsOk(3) &&
      open_fusion_localization_) {  //没有接收到lidarodom数据
    fusion_navigation_code.state_code.push_back(
        UN_RECEIVED_LIDAR_MATCHER_DATA_);
  }

  if (fusion_navigation_code.state_code.empty())
    fusion_navigation_code.state_code.push_back(LOCALIZATION_MODE_NORMAL_);
  pub_fusion_state_information_.publish(fusion_navigation_code);
}

Eigen::Matrix<double, 5, 1> SweeperLocalization::StateTransition(
    const Eigen::Matrix<double, 5, 1> &state,
    const Eigen::Matrix<double, 1, 1> &input) {
  double dt = predict_delta_time_;
  Eigen::Matrix<double, 5, 1> x_next;
  if (fabs(state(4)) < 0.01) {
    x_next(0) = state(0) + state(2) * cos(state(3)) * dt;
    x_next(1) = state(1) + state(2) * sin(state(3)) * dt;
    x_next(2) = state(2);
    x_next(3) = state(4) * dt + state(3);
    x_next(4) = state(4);
  } else {
    x_next(0) = state(0) + (state(2) / state(4)) *
                               (sin(state(4) * dt + state(3)) - sin(state(3)));
    x_next(1) = state(1) + (state(2) / state(4)) *
                               (cos(state(3)) - cos(state(4) * dt + state(3)));
    x_next(2) = state(2);
    x_next(3) = state(4) * dt + state(3);
    x_next(4) = state(4);
  }

  return x_next;
}

Eigen::Matrix<double, 3, 1> SweeperLocalization::StateObservation(
    const Eigen::Matrix<double, 5, 1> &state,
    const Eigen::Matrix<double, 3, 5> &observe_matrix) {
  Eigen::Matrix<double, 3, 1> state_observation;
  state_observation = observe_matrix * state;
  return state_observation;
}

void SweeperLocalization::UpdateJacobianMatrix(
    const Eigen::Matrix<double, 5, 1> &state_last, const double dt) {
  jacobian_.setZero();
  if (fabs(state_last(4)) < 0.01) {
    jacobian_(0, 0) = 1.0;
    jacobian_(0, 2) = dt * cos(state_last(3));
    jacobian_(0, 3) = -dt * state_last(2) * sin(state_last(3));
    jacobian_(0, 4) = 0;

    jacobian_(1, 1) = 1.0;
    jacobian_(1, 2) = dt * sin(state_last(3));
    jacobian_(1, 3) = dt * state_last(2) * cos(state_last(3));
    jacobian_(1, 4) = 0;

    jacobian_(2, 2) = 1.0;
    jacobian_(3, 3) = 1.0;
    jacobian_(3, 4) = dt;
    jacobian_(4, 4) = 1.0;
  } else {
    jacobian_(0, 0) = 1.0;
    jacobian_(0, 2) =
        (1 / state_last(4)) *
        (sin(state_last(4) * dt + state_last(3)) - sin(state_last(3)));
    jacobian_(0, 3) =
        (state_last(2) / state_last(4)) *
        (cos(state_last(4) * dt + state_last(3)) - cos(state_last(3)));
    jacobian_(0, 4) =
        (state_last(2) / pow(state_last(4), 2)) *
            (sin(state_last(3)) - sin(state_last(4) * dt + state_last(3))) +
        (state_last(2) * dt / state_last(4)) *
            cos(state_last(4) * dt + state_last(3));

    jacobian_(1, 1) = 1.0;
    jacobian_(1, 2) =
        1 / state_last(4) *
        (cos(state_last(3)) - cos(state_last(4) * dt + state_last(3)));
    jacobian_(1, 3) =
        (state_last(2) / state_last(4)) *
        (sin(state_last(4) * dt + state_last(3)) - sin(state_last(3)));
    jacobian_(1, 4) =
        (state_last(2) / pow(state_last(4), 2)) *
            (cos(state_last(4) * dt + state_last(3)) - cos(state_last(3))) +
        (state_last(2) * dt / state_last(4)) *
            sin(state_last(4) * dt + state_last(3));

    jacobian_(2, 2) = 1.0;
    jacobian_(3, 3) = 1.0;
    jacobian_(3, 4) = dt;
    jacobian_(4, 4) = 1.0;
  }
}

void SweeperLocalization::ReceiveGpsDatas(const nav_msgs::Odometry &gps_datas) {
  AINFO << "reacive gps data";
  watch_dog_gnss_.UpdataNow();
  if (!get_gps_to_map_) return;
  double gps_time_now = gps_datas.header.stamp.toSec();

  Eigen::Quaterniond gps_quater_now(
      gps_datas.pose.pose.orientation.w, gps_datas.pose.pose.orientation.x,
      gps_datas.pose.pose.orientation.y, gps_datas.pose.pose.orientation.z);

  Eigen::Vector3d gps_pos_now(gps_datas.pose.pose.position.x,
                              gps_datas.pose.pose.position.y,
                              gps_datas.pose.pose.position.z);
  SensorToCenter(gnss_to_map_quater_, gnss_to_map_pos_, gps_quater_now,
                 gps_pos_now);

  tf::Matrix3x3 gps_mat(tf::Quaternion(gps_quater_now.x(), gps_quater_now.y(),
                                       gps_quater_now.z(), gps_quater_now.w()));
  double gps_roll, gps_pitch, gps_yaw;
  gps_mat.getRPY(gps_roll, gps_pitch, gps_yaw);

  R_gps_.setZero();
  R_gps_(0, 0) = gps_datas.pose.covariance[0];
  R_gps_(1, 1) = gps_datas.pose.covariance[7];
  R_gps_(2, 2) = gps_datas.pose.covariance[35];
  gps_observation_ << gps_pos_now.x(), gps_pos_now.y(), gps_yaw;
  // receive_new_gnss_data_ = true;
  AWARN << "reacive gps data ok!";

  nav_msgs::Odometry gnss_position;
  gnss_position.header.frame_id = "lidar_map";
  gnss_position.header.stamp = ros::Time::now();
  gnss_position.pose.pose.position.x = gps_pos_now.x();
  gnss_position.pose.pose.position.y = gps_pos_now.y();
  gnss_position.pose.pose.orientation.x = gps_quater_now.x();
  gnss_position.pose.pose.orientation.y = gps_quater_now.y();
  gnss_position.pose.pose.orientation.z = gps_quater_now.z();
  gnss_position.pose.pose.orientation.w = gps_quater_now.w();
  pub_gnss_pose_.publish(gnss_position);

  if (gps_datas.pose.covariance[0] > 0.2 ||
      gps_datas.pose.covariance[7] > 0.2 || gps_datas.pose.covariance[35] > 0.2)
    return;
  double gps_roll, gps_pitch, gps_yaw;
  gps_mat.getRPY(gps_roll, gps_pitch, gps_yaw);

  R_gps_.setZero();
  R_gps_(0, 0) = gps_datas.pose.covariance[0];
  R_gps_(1, 1) = gps_datas.pose.covariance[7];
  R_gps_(2, 2) = gps_datas.pose.covariance[35];
  gps_observation_ << gps_pos_now.x(), gps_pos_now.y(), gps_yaw;
  receive_new_gnss_data_ = true;
  AINFO << "reacive gps data ok!";
}

void SweeperLocalization::ReceiveSlamDatas(
    const nav_msgs::Odometry &slam_datas) {
  AINFO << "reacive lidar data";
  watch_dog_lidar_odom_.UpdataNow();
  if (!use_lidar_data_) return;
  double lidar_odom_cov = slam_datas.pose.covariance[0];
  now_lidar_time_ = slam_datas.header.stamp.toSec();
  if (lidar_odom_cov > 1.5) {
    return;
  }
  tf::Matrix3x3 slam_mat(tf::Quaternion(
      slam_datas.pose.pose.orientation.x, slam_datas.pose.pose.orientation.y,
      slam_datas.pose.pose.orientation.z, slam_datas.pose.pose.orientation.w));
  double slam_roll, slam_pitch, slam_yaw;
  slam_mat.getRPY(slam_roll, slam_pitch, slam_yaw);

  R_slam_.setZero();
  R_slam_(0, 0) = lidar_odom_cov;
  R_slam_(1, 1) = lidar_odom_cov;
  R_slam_(2, 2) = lidar_odom_cov;

  lidar_observation_ << slam_datas.pose.pose.position.x,
      slam_datas.pose.pose.position.y, slam_yaw;
  receive_new_lidar_data_ = true;
  AINFO << "reacive lidar data ok!";
}

void SweeperLocalization::EkfInit() {
  Eigen::Matrix<double, 5, 5> P0;
  P0.setZero();
  P0(0, 0) = 10.0;
  P0(1, 1) = 10.0;
  P0(2, 2) = 10.0;
  P0(3, 3) = 10.0;
  P0(4, 4) = 10.0;

  auto h = std::bind(&SweeperLocalization::StateObservation, this,
                     std::placeholders::_1, std::placeholders::_2);
  auto f = std::bind(&SweeperLocalization::StateTransition, this,
                     std::placeholders::_1, std::placeholders::_2);

  Eigen::Matrix<double, 3, 5> H;
  H.setZero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 3) = 1.0;

  ekf_filter_.SetObservationModel(h, H);
  ekf_filter_.SetTransitionModel(f);
  ekf_filter_.SetStateEstimate(x_last_, P0);
  flag_filter_init_ = true;
}

void SweeperLocalization::EkfPrediction(double delta_t) {
  UpdateJacobianMatrix(x_last_, delta_t);
  ekf_filter_.SetTransitionMatrix(jacobian_);

  double linear_acc =
      sqrt(xyz_acc_.x() * xyz_acc_.x() + xyz_acc_.y() * xyz_acc_.y());
  double angular_acc = angular_acc_;

  Eigen::Matrix<double, 5, 5> Q;
  Q << pow(0.5 * delta_t_ * delta_t_ * linear_acc * cos(x_last_(3)), 2),
      0.25 * pow(delta_t_ * delta_t_ * linear_acc, 2) * sin(x_last_(3)) *
          cos(x_last_(3)),
      0.5 * delta_t_ * delta_t_ * delta_t_ * linear_acc * linear_acc *
          cos(x_last_(3)),
      0.0, 0.0,
      pow(0.5 * delta_t_ * delta_t_ * linear_acc, 2) * sin(x_last_(3)) *
          cos(x_last_(3)),
      pow(0.5 * delta_t_ * delta_t_ * linear_acc * sin(x_last_(3)), 2),
      0.5 * delta_t_ * delta_t_ * delta_t_ * linear_acc * linear_acc *
          sin(x_last_(3)),
      0.0, 0.0,
      0.5 * delta_t_ * delta_t_ * delta_t_ * linear_acc * linear_acc *
          cos(x_last_(3)),
      0.5 * delta_t_ * delta_t_ * delta_t_ * linear_acc * linear_acc *
          sin(x_last_(3)),
      delta_t_ * delta_t_ * linear_acc * linear_acc, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.25 * pow(delta_t_, 4) * pow(angular_acc, 2),
      0.5 * pow(delta_t_, 3) * pow(angular_acc, 2), 0.0, 0.0, 0.0,
      0.5 * pow(delta_t_, 3) * pow(angular_acc, 2),
      pow(delta_t_, 2) * pow(angular_acc, 2);

  ekf_filter_.SetTransitionNoise(Q);
  Eigen::Matrix<double, 1, 1> input;
  input.setZero();
  ekf_filter_.Predict(input);
  x_last_ = ekf_filter_.x_;
}

void SweeperLocalization::FixYawAdaptPrediction(double &yaw) {
  if (yaw > PI) {
    yaw = yaw - 2.0 * PI;
  } else if (yaw < -PI) {
    yaw = yaw + 2.0 * PI;
  }
}

double SweeperLocalization::NormalizeYaw(const double &yaw) {
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

//得到标定参数
bool SweeperLocalization::GetCalibYamlParam(const std::string file_path,
                                            Eigen::Quaterniond &rotation,
                                            Eigen::Vector3d &translation) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["transform"]) {
      return false;
    }
    if (config["transform"]["translation"] && config["transform"]["rotation"]) {
      double tx = config["transform"]["translation"]["x"].as<double>();
      double ty = config["transform"]["translation"]["y"].as<double>();
      double tz = config["transform"]["translation"]["z"].as<double>();

      double qx = config["transform"]["rotation"]["x"].as<double>();
      double qy = config["transform"]["rotation"]["y"].as<double>();
      double qz = config["transform"]["rotation"]["z"].as<double>();
      double qw = config["transform"]["rotation"]["w"].as<double>();

      rotation = Eigen::Quaterniond(qw, qx, qy, qz);
      translation = Eigen::Vector3d(tx, ty, tz);
      AWARN << "qx, qy, qz, qw : " << qx << " " << qy << " " << qz << " " << qw;
      AWARN << "tx, ty, tz : " << tx << " " << ty << " " << tz;
    }
  } catch (const YAML::Exception &e) {
    AERROR << "load extrinsics: " << file_path
           << " failed! error: " << e.what();
    return false;
  }
  return true;
}

bool SweeperLocalization::GetCalibGnssPoseYamlParam(
    const std::string file_path, Eigen::Vector3d &translation) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["gnss"]) {
      return false;
    }
    double tx = config["gnss"]["gnss_origin_x"].as<double>();
    double ty = config["gnss"]["gnss_origin_y"].as<double>();
    double tz = config["gnss"]["gnss_origin_z"].as<double>();
    translation = Eigen::Vector3d(tx, ty, tz);
    AWARN << "tx, ty, tz : " << tx << " " << ty << " " << tz;

  } catch (const YAML::Exception &e) {
    AERROR << "load extrinsics: " << file_path
           << " failed! error: " << e.what();
    return false;
  }
  return true;
}

bool SweeperLocalization::GetImageYamlParam(const std::string file_path,
                                            double &origin_x, double &origin_y,
                                            double &resolution) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    // if (!config["gnss"]) {
    //   return false;
    // }
    origin_x = config["origin"][0].as<double>();
    origin_y = config["origin"][1].as<double>();
    resolution = config["resolution"].as<double>();
    // translation = Eigen::Vector3d(tx, ty, tz);
    // AWARN << "tx, ty, tz : " << tx << " " << ty << " " << tz;

  } catch (const YAML::Exception &e) {
    AERROR << "load extrinsics: " << file_path
           << " failed! error: " << e.what();
    return false;
  }
  return true;
}

//传感器矫正
void SweeperLocalization::SensorToCenter(const Eigen::Quaterniond &tran_quater,
                                         const Eigen::Vector3d &tran_pos,
                                         Eigen::Quaterniond &sensor_quater,
                                         Eigen::Vector3d &sensor_pos) {
  Eigen::Quaterniond sensor_quat = sensor_quater;
  Eigen::Vector3d sensor_position = sensor_pos;
  sensor_quater = tran_quater * sensor_quat;
  sensor_pos = tran_quater * sensor_position + tran_pos;
}

double SweeperLocalization::RpyToSignRpy(double rpy) {
  if (rpy >= M_PI) {
    rpy -= 2.0 * M_PI;
  }
  return rpy;
}
double SweeperLocalization::CalcDiffForRadian(const double now_rad,
                                              const double before_rad) {
  double diff_rad = now_rad - before_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI) {
    diff_rad = diff_rad + 2 * M_PI;
  }
  return diff_rad;
}

SweeperLocalization::~SweeperLocalization() {}

}  // namespace fusion_location
}  // namespace localization
}  // namespace navigation
}  // namespace sweeper

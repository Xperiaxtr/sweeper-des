#include "ekf_localizer.hpp"

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>

namespace sweeper {
namespace navigation {
namespace localization {
namespace fusion_location {
EKFLocalizer::EKFLocalizer(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : flag_filter_init_(false),
      open_fusion_localization_(false),
      untrusted_number_(0),
      flag_receive_new_gnss_(false),
      flag_receive_new_lidar_data_(false),
      flag_receive_new_imu_data_(false),
      get_gps_to_map_(false),
      get_gnss_fixed_point_(false),
      // get_map_to_image_(false),
      angular_velocity_yaw_(0.0),
      pitch_car_(0.0),
      pitch_car_last_(0.0),
      sweeper_vehicle_plane_(0.0),
      dim_x_(7),
      flag_livox_to_gps_(false) {
  nh_private.param<bool>("open_fusion_localization", open_fusion_localization_,
                         false);
  nh_private.param<bool>("use_lidar_data", use_lidar_data_, true);
  nh_private.param<bool>("use_gnss_data", flag_use_gps_data_, true);
  nh_private.param<double>("predict_frequency", ekf_rate_, 50.0);
  ekf_dt_ = 1.0 / ekf_rate_;
  //   nh_private.param("enable_yaw_bias_estimation",
  //   enable_yaw_bias_estimation_,
  //              bool(true));
  nh_private.param("extend_state_step", extend_state_step_, int(50));

  /* pose measurement */
  nh_private.param("pose_additional_delay", pose_additional_delay_,
                   double(0.0));
  nh_private.param("pose_measure_uncertainty_time",
                   pose_measure_uncertainty_time_, double(0.01));
  nh_private.param("pose_rate", pose_rate_,
                   double(10.0));  // used for covariance calculation
  nh_private.param("pose_stddev_x", pose_stddev_x_, double(0.05));
  nh_private.param("pose_stddev_y", pose_stddev_y_, double(0.05));
  nh_private.param("pose_stddev_yaw", pose_stddev_yaw_, double(0.035));
  nh_private.param("use_pose_with_covariance", use_pose_with_covariance_,
                   bool(true));

  //   /* twist measurement */
  //   nh_private.param("twist_additional_delay", twist_additional_delay_,
  //   double(0.0)); nh_private.param("twist_rate", twist_rate_, double(10.0));
  //   // used for covariance calculation nh_private.param("twist_gate_dist",
  //   twist_gate_dist_, double(10000.0)); // Mahalanobis limit
  //   nh_private.param("twist_stddev_vx", twist_stddev_vx_, double(0.2));
  //   nh_private.param("twist_stddev_wz", twist_stddev_wz_, double(0.03));

  /* process noise */
  double proc_stddev_yaw_c, proc_stddev_v_c, proc_stddev_w_c, proc_stddev_wb_c,
      proc_stddev_vs_c;
  nh_private.param("proc_stddev_yaw_c", proc_stddev_yaw_c, double(0.05));
  //   nh_private.param("proc_stddev_yaw_bias_c", proc_stddev_yaw_bias_c,
  //   double(0.001));
  nh_private.param("proc_stddev_v_c", proc_stddev_v_c, double(2.0));
  nh_private.param("proc_stddev_w_c", proc_stddev_w_c, double(0.2));
  nh_private.param("proc_stddev_wb_c", proc_stddev_wb_c, double(2.0));
  nh_private.param("proc_stddev_vs_c", proc_stddev_vs_c, double(0.2));

  //   if (!enable_yaw_bias_estimation_) {
  //     proc_stddev_yaw_bias_c = 0.0;
  //   }

  /* convert to continuous to discrete */
  cov_v_d_ = std::pow(proc_stddev_v_c, 2.0) * ekf_dt_;
  cov_w_d_ = std::pow(proc_stddev_w_c, 2.0) * ekf_dt_;
  cov_yaw_d_ = std::pow(proc_stddev_yaw_c, 2.0) * ekf_dt_;
  cov_vs_d_ = std::pow(proc_stddev_vs_c, 2.0) * ekf_dt_;
  cov_wb_d_ = std::pow(proc_stddev_wb_c, 2.0) * ekf_dt_;
  //   cov_yaw_bias_d_ = std::pow(proc_stddev_yaw_bias_c, 2.0) * ekf_dt_;

  sub_gps_ = nh.subscribe("/sweeper/localization/gnss", 1,
                          &EKFLocalizer::ReceiveGpsDatas, this);
  sub_slam_ = nh.subscribe("/sweeper/navigation/lidar_odom", 1,
                           &EKFLocalizer::ReceiveSlamDatas, this);
  sub_imu_ = nh.subscribe("/sweeper/sensor/imu", 1,
                          &EKFLocalizer::ReceiveImuDatas, this);
  sub_speed_ = nh.subscribe("/sweeper/chassis/detail", 1,
                            &EKFLocalizer::ReceiveSpeedDatas, this);

  sub_state_localization_information_ =
      nh.subscribe<sweeper_msgs::SensorFaultInformation>(
          "/sweeper/localization/diagnose", 1,
          &EKFLocalizer::ReceiveLocalizationDiagnose, this);
  sub_sweeper_mode_ = nh.subscribe<sweeper_msgs::SweepMission>(
      "/sweeper/sweep_mode", 1, &EKFLocalizer::ReceiveSweeperMode, this);

  pub_fusion_pose_ = nh.advertise<nav_msgs::Odometry>(
      "/sweeper/localization/fusion_position", 1);
  pub_image_pose_ =
      nh.advertise<nav_msgs::Odometry>("/sweeper/localization/image_pose", 1);
  pub_gnss_pose_ =
      nh.advertise<nav_msgs::Odometry>("/sweeper/localization/gnss_pose", 1);

  pub_fusion_state_information_ =
      nh.advertise<sweeper_msgs::SensorFaultInformation>(
          "/sweeper/common/diagnose", 1);
  timer_ekf_navigation_ = nh.createTimer(
      ros::Duration(0.1), &EKFLocalizer::GetSweeperInformation, this);

  timer_control_ = nh.createTimer(ros::Duration(ekf_dt_),
                                  &EKFLocalizer::TimerCallback, this);

  dim_x_ex_ = dim_x_ * extend_state_step_;

  std::string path_livox_to_gps =
      "../sweeper_ws/src/sweeper_haide/calibration/data/livox_to_gps.yaml";
  flag_livox_to_gps_ = GetCalibYamlParam(
      path_livox_to_gps, quater_livox_to_gps_, pos_livox_to_gps_);
}

void EKFLocalizer::TimerCallback(const ros::TimerEvent &e) {
  if (!open_fusion_localization_) return;
  AINFO << "---------------------timer called--------------------------";
  if (flag_filter_init_) {
    // now_time_ = ros::Time::now();
    // predit
    double time_start = std::clock();
    PredictEKFModel();
    double time_predict = clock();
    // AINFO << "predict diff time: " << time_predict - time_start;

    /*gnss measurement update */
    if (flag_receive_new_gnss_) {
      flag_receive_new_gnss_ = false;
      double time_gnss_start = clock();
      GnssMeasurementUpdatePose();
      // AINFO << "gnss updata time: " << clock() - time_gnss_start;
      Eigen::MatrixXd X_curr(dim_x_, 1);  // curent state
      ekf_filter_.getLatestX(X_curr);
      AINFO << "gnss result: " << X_curr(0) << " " << X_curr(1) << " "
            << X_curr(2) << " " << X_curr(3) << " " << X_curr(4) << " "
            << X_curr(5) << " " << X_curr(6);
    }

    /* lidar measurement update*/
    if (flag_receive_new_lidar_data_) {
      flag_receive_new_lidar_data_ = false;
      double time_lidar_start = clock();
      SlamMeasurementUpdatePose();
      // AINFO << "lidar updata time: " << clock() - time_lidar_start;
      Eigen::MatrixXd X_curr(dim_x_, 1);  // curent state
      ekf_filter_.getLatestX(X_curr);
      AINFO << "lidar result: " << X_curr(0) << " " << X_curr(1) << " "
            << X_curr(2) << " " << X_curr(3) << " " << X_curr(4) << " "
            << X_curr(5) << " " << X_curr(6);
    }

    // SetCurrentResult();
    PublishFusionResult();
  } else if (flag_receive_new_gnss_) {
    Eigen::Matrix<double, 7, 1> x_last_;
    flag_receive_new_gnss_ = false;
    x_last_(0) = gps_observation_(0);
    x_last_(1) = gps_observation_(1);
    x_last_(2) = gps_observation_(2);
    x_last_(3) = sweeper_vehicle_plane_;
    x_last_(4) = angular_velocity_yaw_;
    x_last_(5) = 1.0;
    x_last_(6) = 0.0;
    EkfInit(x_last_);
  } else if (flag_receive_new_lidar_data_) {
    Eigen::Matrix<double, 7, 1> x_last_;
    x_last_(0) = lidar_observation_(0);
    x_last_(1) = lidar_observation_(1);
    x_last_(2) = lidar_observation_(2);
    x_last_(3) = sweeper_vehicle_plane_;
    x_last_(4) = angular_velocity_yaw_;
    x_last_(5) = 1.0;
    x_last_(6) = 0.0;
    flag_receive_new_lidar_data_ = false;
    EkfInit(x_last_);
  }
}

void EKFLocalizer::PredictEKFModel() {
  Eigen::MatrixXd X_curr(dim_x_, 1);  // curent state
  Eigen::MatrixXd X_next(dim_x_, 1);  // predicted state
  ekf_filter_.getLatestX(X_curr);

  AINFO << "before: " << X_curr(0) << " " << X_curr(1) << " " << X_curr(2)
        << " " << X_curr(3) << " " << X_curr(4) << " " << X_curr(5) << " "
        << X_curr(6);
  // DEBUG_PRINT_MAT(X_curr.transpose());

  Eigen::MatrixXd P_curr;
  ekf_filter_.getLatestP(P_curr);

  int d_dim_x = dim_x_ex_ - dim_x_;
  double w_bias = X_curr(IDX::WB);
  double v_scale = X_curr(IDX::VS);

  if (v_scale > 1.1)
    v_scale = 1.1;
  else if (v_scale < 0.9)
    v_scale = 0.9;
  if (fabs(w_bias) > 0.01) w_bias = 0.0;
  double w, v, yaw, y, x, center_w, center_v;
  if (flag_receive_new_vehical_data_) {
    flag_receive_new_vehical_data_ = false;
    v = (sweeper_vehicle_plane_ + X_curr(IDX::V)) / 2.0;
  } else {
    v = X_curr(IDX::V);
  }
  if (flag_receive_new_imu_data_) {
    flag_receive_new_imu_data_ = false;
    w = (angular_velocity_yaw_ + X_curr(IDX::W)) / 2.0;
  } else {
    w = X_curr(IDX::W);
  }
  AINFO << "ekf_dt_ : " << ekf_dt_;
  center_w = w + w_bias;
  center_v = v * v_scale;
  yaw = X_curr(IDX::YAW) + center_w * ekf_dt_;
  yaw = std::atan2(std::sin(yaw), std::cos(yaw));

  AINFO << "before X_next: " << x << " " << y << " " << yaw << " " << center_v
        << " " << center_w << " " << v_scale << " " << w_bias;

  // if (fabs(center_w) > 0.001) {
  //   y = X_curr(IDX::Y) -
  //       center_v / center_w * cos(center_w * ekf_dt_ + X_curr(IDX::YAW)) +
  //       center_v / center_w * cos(X_curr(IDX::YAW));
  //   AINFO << " " << center_v / center_w << " "
  //         << cos(center_w * ekf_dt_ + X_curr(IDX::YAW)) << " "
  //         << cos(X_curr(IDX::YAW));
  //   x = X_curr(IDX::X) +
  //       center_v / center_w * sin(center_w * ekf_dt_ + X_curr(IDX::YAW)) -
  //       center_v / center_w * sin(X_curr(IDX::YAW));

  //   AINFO << " " << center_v / center_w << " "
  //         << sin(center_w * ekf_dt_ + X_curr(IDX::YAW)) << " "
  //         << cos(X_curr(IDX::YAW))<<" "<<ekf_dt_;
  // } else {
  y = X_curr(IDX::Y) + center_v * sin(X_curr(IDX::YAW)) * ekf_dt_;
  x = X_curr(IDX::X) + center_v * cos(X_curr(IDX::YAW)) * ekf_dt_;
  // }

  /* Update for latest state */
  X_next(IDX::X) = x;
  X_next(IDX::Y) = y;
  X_next(IDX::YAW) = yaw;
  X_next(IDX::V) = v;
  X_next(IDX::W) = w;
  X_next(IDX::VS) = v_scale;
  X_next(IDX::WB) = w_bias;
  X_next(IDX::YAW) =
      std::atan2(std::sin(X_next(IDX::YAW)), std::cos(X_next(IDX::YAW)));

  AINFO << "X_next: " << X_next(0) << " " << X_next(1) << " " << X_next(2)
        << " " << X_next(3) << " " << X_next(4) << " " << X_next(5) << " "
        << X_next(6);
  Eigen::MatrixXd A = UpdateJacobianMatrix(X_next, ekf_dt_);

  const double dvx = std::sqrt(P_curr(IDX::VS, IDX::VS));
  const double dyaw = std::sqrt(P_curr(IDX::YAW, IDX::YAW));

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  if (dvx < 5.0 && dyaw < 3.0) {
    // auto covariance calculate for x, y assuming vx & yaw estimation
    // covariance is small

    /* Set covariance matrix Q for process noise. Calc Q by velocity and yaw
     angle covariance : dx = Ax + Jp*w -> Q = Jp*w_cov*Jp'          */
    Eigen::MatrixXd Jp =
        Eigen::MatrixXd::Zero(2, 2);  // coeff of deviation of vx & yaw
    Jp << cos(yaw), -v * sin(yaw), sin(yaw), v * cos(yaw);
    Eigen::MatrixXd Q_vx_yaw =
        Eigen::MatrixXd::Zero(2, 2);  // cov of vx and yaw

    Q_vx_yaw(0, 0) =
        P_curr(IDX::VS, IDX::VS) * ekf_dt_;  // covariance of vx - vx
    Q_vx_yaw(1, 1) =
        P_curr(IDX::YAW, IDX::YAW) * ekf_dt_;  // covariance of yaw - yaw
    Q_vx_yaw(0, 1) =
        P_curr(IDX::VS, IDX::YAW) * ekf_dt_;  // covariance of vx - yaw
    Q_vx_yaw(1, 0) =
        P_curr(IDX::YAW, IDX::VS) * ekf_dt_;  // covariance of yaw - vx
    Q.block(0, 0, 2, 2) = Jp * Q_vx_yaw * Jp.transpose();  // for pos_x & pos_y
  } else {
    // vx & vy is not converged yet, set constant value.
    Q(IDX::X, IDX::X) = 0.1;
    Q(IDX::Y, IDX::Y) = 0.1;
  }

  Q(IDX::YAW, IDX::YAW) = cov_yaw_d_;  // for yaw
  //   Q(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;  // for yaw bias
  Q(IDX::V, IDX::V) = cov_v_d_;     // for vx
  Q(IDX::W, IDX::W) = cov_w_d_;     // for wz
  Q(IDX::VS, IDX::VS) = cov_vs_d_;  // for vx
  Q(IDX::WB, IDX::WB) = cov_wb_d_;  // for wz

  ekf_filter_.predictWithDelay(X_next, A, Q);
}

/*
 * measurementUpdatePose
 */
void EKFLocalizer::GnssMeasurementUpdatePose() {
  Eigen::MatrixXd X_curr(dim_x_, 1);  // curent state
  ekf_filter_.getLatestX(X_curr);
  //   DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 3;  // pos_x, pos_y, yaw, depending on Pose output
                            //   const ros::Time t_curr_ = ros::Time::now();
  /* Calculate delay step */
  double delay_time = (now_time_ - gnss_time_).toSec() + pose_additional_delay_;
  if (delay_time < 0.0) {
    delay_time = 0.0;
    // ROS_WARN_DELAYED_THROTTLE(
    //     1.0, "Pose time stamp is inappropriate, set delay to 0[s]. delay =
    //     %f", delay_time);
  }
  AWARN << "diff time: " << delay_time;
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1)  // 1s
  {
    // ROS_WARN_DELAYED_THROTTLE(
    //     1.0,
    //     "Pose delay exceeds the compensation limit, ignored. delay: %f[s], "
    //     "limit = extend_state_step * ekf_dt : %f [s]",
    //     delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  //   DEBUG_INFO("delay_time: %f [s]", delay_time);
  /* Set yaw */
  // const double yaw_curr =
  //     ekf_filter_.getXelement((unsigned int)(delay_step * dim_x_ +
  //     IDX::YAW));
  double yaw = gps_observation_(2);
  const double ekf_yaw =
      ekf_filter_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error =
      NormalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi
  yaw = yaw_error + ekf_yaw;
  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << gps_observation_(0), gps_observation_(1), yaw;

  AINFO << "observation gnss data: " << y;
  if (isnan(y.array()).any() || isinf(y.array()).any()) {
    // ROS_WARN(
    //     "[EKF] pose measurement matrix includes NaN of Inf. ignore update. "
    //     "check pose message.");
    return;
  }
  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_filter_.getXelement(delay_step * dim_x_ + IDX::X),
      ekf_filter_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw;
  Eigen::MatrixXd P_curr, P_y;
  ekf_filter_.getLatestP(P_curr);
  P_y = P_curr.block(0, 0, dim_y, dim_y);
  //   if (!mahalanobisGate(pose_gate_dist_, y_ekf, y, P_y)) {
  //     ROS_WARN_DELAYED_THROTTLE(
  //         2.0,
  //         "[EKF] Pose measurement update, mahalanobis distance is over limit.
  //         " "ignore measurement data.");
  //     return;
  //   }

  //   DEBUG_PRINT_MAT(y.transpose());
  //   DEBUG_PRINT_MAT(y_ekf.transpose());
  //   DEBUG_PRINT_MAT((y - y_ekf).transpose());
  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::X) = 1.0;    // for pos x
  C(1, IDX::Y) = 1.0;    // for pos y
  C(2, IDX::YAW) = 1.0;  // for yaw

  /* Set measurement noise covariancs */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (use_pose_with_covariance_) {
    R(0, 0) = R_gps_(0, 0);
    R(1, 1) = R_gps_(1, 1);
    R(2, 2) = R_gps_(2, 2);
  } else {
    const double ekf_yaw = ekf_filter_.getXelement(IDX::YAW);
    const double v = ekf_filter_.getXelement(IDX::V);
    const double w = ekf_filter_.getXelement(IDX::W);
    const double cov_pos_x =
        std::pow(pose_measure_uncertainty_time_ * v * cos(ekf_yaw), 2.0);
    const double cov_pos_y =
        std::pow(pose_measure_uncertainty_time_ * v * sin(ekf_yaw), 2.0);
    const double cov_yaw = std::pow(pose_measure_uncertainty_time_ * w, 2.0);
    R(0, 0) = std::pow(pose_stddev_x_, 2) + cov_pos_x;  // pos_x
    R(1, 1) = std::pow(pose_stddev_y_, 2) + cov_pos_y;  // pos_y
    R(2, 2) = std::pow(pose_stddev_yaw_, 2) + cov_yaw;  // yaw
  }
  /* In order to avoid a large change at the time of updating, measuremeent
   * update is performed by dividing at every step. */
  R *= (ekf_rate_ / pose_rate_);

  ekf_filter_.updateWithDelay(y, C, R, delay_step);
  //   // debug
  //   Eigen::MatrixXd X_result(dim_x_, 1);
  //   ekf_filter_.getLatestX(X_result);
  //   DEBUG_PRINT_MAT(X_result.transpose());
  //   DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdatePose
 */
void EKFLocalizer::SlamMeasurementUpdatePose() {
  Eigen::MatrixXd X_curr(dim_x_, 1);  // curent state
  ekf_filter_.getLatestX(X_curr);
  //   DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 3;  // pos_x, pos_y, yaw, depending on Pose output
                            //   const ros::Time t_curr_ = ros::Time::now();

  /* Calculate delay step */
  double delay_time = (now_time_ - slam_time_).toSec() + pose_additional_delay_;
  if (delay_time < 0.0) {
    delay_time = 0.0;
    // ROS_WARN_DELAYED_THROTTLE(
    //     1.0, "Pose time stamp is inappropriate, set delay to 0[s]. delay =
    //     %f", delay_time);
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1)  // 1s
  {
    // ROS_WARN_DELAYED_THROTTLE(
    //     1.0,
    //     "Pose delay exceeds the compensation limit, ignored. delay: %f[s], "
    //     "limit = extend_state_step * ekf_dt : %f [s]",
    //     delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  //   DEBUG_INFO("delay_time: %f [s]", delay_time);

  /* Set yaw */
  // const double yaw_curr =
  //     ekf_filter_.getXelement((unsigned int)(delay_step * dim_x_ +
  //     IDX::YAW));
  double yaw = lidar_observation_(2);
  const double ekf_yaw =
      ekf_filter_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error =
      NormalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi
  yaw = yaw_error + ekf_yaw;

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << lidar_observation_(0), lidar_observation_(1), yaw;

  AINFO << "lidar observation: " << y;

  if (isnan(y.array()).any() || isinf(y.array()).any()) {
    // ROS_WARN(
    //     "[EKF] pose measurement matrix includes NaN of Inf. ignore update. "
    //     "check pose message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_filter_.getXelement(delay_step * dim_x_ + IDX::X),
      ekf_filter_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw;
  Eigen::MatrixXd P_curr, P_y;
  ekf_filter_.getLatestP(P_curr);
  P_y = P_curr.block(0, 0, dim_y, dim_y);
  //   if (!mahalanobisGate(pose_gate_dist_, y_ekf, y, P_y)) {
  //     ROS_WARN_DELAYED_THROTTLE(
  //         2.0,
  //         "[EKF] Pose measurement update, mahalanobis distance is over limit.
  //         " "ignore measurement data.");
  //     return;
  //   }

  //   DEBUG_PRINT_MAT(y.transpose());
  //   DEBUG_PRINT_MAT(y_ekf.transpose());
  //   DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::X) = 1.0;    // for pos x
  C(1, IDX::Y) = 1.0;    // for pos y
  C(2, IDX::YAW) = 1.0;  // for yaw

  /* Set measurement noise covariancs */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (use_pose_with_covariance_) {
    R(0, 0) = R_slam_(0, 0);
    R(1, 1) = R_slam_(1, 1);
    R(2, 2) = R_slam_(2, 2);
  } else {
    const double ekf_yaw = ekf_filter_.getXelement(IDX::YAW);
    const double vx = ekf_filter_.getXelement(IDX::VS);
    const double wz = ekf_filter_.getXelement(IDX::WB);
    const double cov_pos_x =
        std::pow(pose_measure_uncertainty_time_ * vx * cos(ekf_yaw), 2.0);
    const double cov_pos_y =
        std::pow(pose_measure_uncertainty_time_ * vx * sin(ekf_yaw), 2.0);
    const double cov_yaw = std::pow(pose_measure_uncertainty_time_ * wz, 2.0);
    R(0, 0) = std::pow(pose_stddev_x_, 2) + cov_pos_x;  // pos_x
    R(1, 1) = std::pow(pose_stddev_y_, 2) + cov_pos_y;  // pos_y
    R(2, 2) = std::pow(pose_stddev_yaw_, 2) + cov_yaw;  // yaw
  }

  /* In order to avoid a large change at the time of updating, measuremeent
   * update is performed by dividing at every step. */
  R *= (ekf_rate_ / pose_rate_);

  ekf_filter_.updateWithDelay(y, C, R, delay_step);

  //   // debug
  //   Eigen::MatrixXd X_result(dim_x_, 1);
  //   ekf_filter_.getLatestX(X_result);
  //   DEBUG_PRINT_MAT(X_result.transpose());
  //   DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

void EKFLocalizer::PublishFusionResult() {
  Eigen::MatrixXd now_x(dim_x_, 1);
  ekf_filter_.getLatestX(now_x);
  tf::Vector3 fusion_pose_position(now_x(0), now_x(1), 0.0);
  tf::Quaternion fusion_pose_quat = tf::createQuaternionFromYaw(now_x(2));

  std::string localization_frame_id;
  if (untrusted_number_ < 10)
    localization_frame_id = "best";
  else if (untrusted_number_ < 40)
    localization_frame_id = "good";
  else
    localization_frame_id = "bad";
  nav_msgs::Odometry fusion_position;
  fusion_position.header.frame_id = "camera_init";
  fusion_position.child_frame_id = localization_frame_id;
  fusion_position.header.stamp = ros::Time::now();
  fusion_position.pose.pose.position.x = fusion_pose_position.x();
  fusion_position.pose.pose.position.y = fusion_pose_position.y();
  fusion_position.pose.pose.position.z = fusion_pose_position.z();

  fusion_position.pose.pose.orientation.x = fusion_pose_quat.x();
  fusion_position.pose.pose.orientation.y = fusion_pose_quat.y();
  fusion_position.pose.pose.orientation.z = fusion_pose_quat.z();
  fusion_position.pose.pose.orientation.w = fusion_pose_quat.w();
  // fusion_position.twist.twist.linear.x = predict_v;
  pub_fusion_pose_.publish(fusion_position);

  Eigen::Vector3d map_pose(now_x(0), now_x(1), 1.0);
  Eigen::Matrix<double, 2, 1> image_pose = map_to_image_ * map_pose;

  double yaw = -now_x(2) - M_PI_2;
  tf::Quaternion image_pose_quat = tf::createQuaternionFromYaw(yaw);
  nav_msgs::Odometry image_position;
  image_position.header.frame_id = "camera_init";
  image_position.child_frame_id = localization_frame_id;
  image_position.header.stamp = ros::Time::now();
  image_position.pose.pose.position.x = image_pose.x();
  image_position.pose.pose.position.y = image_pose.y();
  // image_position.pose.pose.position.z = fusion_pose_position.z();
  image_position.pose.pose.orientation.x = image_pose_quat.x();
  image_position.pose.pose.orientation.y = image_pose_quat.y();
  image_position.pose.pose.orientation.z = image_pose_quat.z();
  image_position.pose.pose.orientation.w = image_pose_quat.w();
  // fusion_position.twist.twist.linear.x = predict_v;
  pub_image_pose_.publish(image_position);
}

/*
    w_bias = last_bias;
    v_scale = last_sacle;
    w = new_w;
    v = new_v;

        center_w = (w + last_w)/2.0 + w_bias;
        center_v = (v + last_v)/2.0 * v_scale;

    yaw = last_yaw + center_w * dt;
    y = last_y - center_v/center_w *cos(center_w *dt + last_yaw) +
   center_v/center_w*cos(last_yaw); x = last_x + center_v/center_w
   *sin(center_w *dt + last_yaw) - center_v/center_w*sin(last_yaw);


    当w = 0时，为直线运动
    w_bias = last_bias;
    v_scale = last_sacle;
    w = new_w;
    v = new_v;

        center_w = (w + last_w)/2.0 + w_bias;
        center_v = (v + last_v)/2.0 * v_scale;

    yaw = last_yaw + center_w * dt;
    y = last_y - center_v *sin(yaw);
    x = last_x + center_v * cos(yaw);
*/

void EKFLocalizer::ReceiveSpeedDatas(
    const sweeper_msgs::SweeperChassisDetail sweeper_chassis_detail) {
  watch_dog_vehical_.UpdataNow();
  if (!open_fusion_localization_) return;
  AINFO << "receive speed data";
  double now_speed_time = sweeper_chassis_detail.header.stamp.toSec();
  now_time_ = sweeper_chassis_detail.header.stamp;
  double kmph_to_mps = 1000.0 / 3600.0;
  double sweep_vehicle =
      sweeper_chassis_detail.vehicle_speed_output * kmph_to_mps;

  sweeper_vehicle_plane_ = sweep_vehicle * cos(pitch_car_);
  // double delta_t = now_speed_time - last_speed_time_;
  flag_receive_new_vehical_data_ = true;

  AWARN << "data : "
        << "sweep_vehicle : " << sweep_vehicle
        << " sweeper_vehicle_plane : " << sweeper_vehicle_plane_
        << " flag_filter_init_ " << flag_filter_init_;
}

Eigen::Matrix<double, 7, 7> EKFLocalizer::SetProcessNoise(
    double dt, Eigen::Matrix<double, 7, 1> x_last) {
  Eigen::Matrix<double, 7, 7> Q;
  Eigen::Matrix<double, 7, 3> Q_left;
  Eigen::Matrix<double, 3, 3> Q_center;
  //角度变换
  double pitch2 = pitch_car_, pitch1 = pitch_car_last_, scale_noise_ = 0.1,
         bias_noise_ = 0.1, v = x_last(3), w = x_last(4), yaw = x_last(2);
  pitch_car_last_ = pitch_car_;
  double delta_cos_pitch = cos(pitch2) - cos(pitch1);
  double scale_noise = scale_noise_;
  double bias_noise = bias_noise_;
  Q_center << delta_cos_pitch * delta_cos_pitch, 0.0, 0.0, 0.0,
      scale_noise * scale_noise, 0.0, 0.0, 0.0, bias_noise * bias_noise;

  Q_left << v * cos(yaw) * dt, 0.0, 0.0, v * sin(yaw) * dt, 0.0, 0.0, w * dt,
      0.0, 0.0, v, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;

  Q = Q_left * Q_center * Q_left.transpose();

  return Q;
}

void EKFLocalizer::ReceiveImuDatas(const sensor_msgs::Imu::ConstPtr &input) {
  watch_dog_imu_.UpdataNow();
  AINFO << "receive imu data   " << input->angular_velocity.z;
  const ros::Time current_time = input->header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();
  if (diff_time > 0 && diff_time < 0.03)
    angular_acc_ =
        (-input->angular_velocity.z - angular_velocity_yaw_) / diff_time;
  angular_velocity_yaw_ = input->angular_velocity.z * cos(pitch_car_);

  previous_time = current_time;
  flag_receive_new_imu_data_ = true;
  AINFO << "receive imu data ok! "
        << "   w:   " << angular_velocity_yaw_;
}

void EKFLocalizer::ReceiveSweeperMode(
    sweeper_msgs::SweepMission sweeper_mode_command) {
  std::string frame_id = sweeper_mode_command.header.frame_id;
  if (sweeper_mode_command.mode == 5 /* && frame_id != "mapping"*/) {
    switch (sweeper_mode_command.start_cmd)
    case 1:  //开始
    {
      if (sweeper_mode_command.map.size() <= 0 ||
          !sweeper_mode_command.map.compare(last_pcd_name_))
        break;
      position_processing_ = true;
      // get_gps_to_map_yaml

      std::string gnss_to_map_path =
          "../sweeper_ws/src/sweeper_haide/data/path/" +
          sweeper_mode_command.map;
      std::cout << "yaml : " << gnss_to_map_path << std::endl;
      gnss_to_map_path.erase(gnss_to_map_path.end() - 5,
                             gnss_to_map_path.end());

      std::cout << "yaml : " << gnss_to_map_path << std::endl;
      map_to_image_ = GetMapToImageParam(gnss_to_map_path);

      AWARN << "yaml : " << gnss_to_map_path + "gps_to_map.yaml";
      get_gps_to_map_ =
          GetCalibYamlParam(gnss_to_map_path + "gps_to_map.yaml",
                            gnss_to_map_quater_, gnss_to_map_pos_);
      get_gnss_fixed_point_ = GetFixedGpsPoint(
          gnss_to_map_path + "gps_to_map.yaml", gnss_fixed_pos_);
      last_pcd_name_ = sweeper_mode_command.map;
      open_fusion_localization_ = true;
      AWARN << "start fusion localization!";
      break;
    }

  } else {
    last_pcd_name_ = "unname";
    open_fusion_localization_ = false;
    flag_filter_init_ = false;
    AWARN << "not open fusion localization!";
    // get_map_to_image_ = false;
    get_gnss_fixed_point_ = get_gps_to_map_ = false;
    position_processing_ = false;
  }
}

void EKFLocalizer::ReceiveLocalizationDiagnose(
    const sweeper_msgs::SensorFaultInformation localization_state) {
  for (size_t i = 0; i < localization_state.state_code.size(); i++) {
    receive_localization_state_code_.push_back(
        localization_state.state_code[i]);
  }
}

void EKFLocalizer::GetSweeperInformation(const ros::TimerEvent &e) {
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
    fusion_navigation_code.state_code.push_back(3105);
  }
  if (!watch_dog_vehical_.DogIsOk(3)) {  //没有接收车速信息
    fusion_navigation_code.state_code.push_back(3106);
  }
  if (!watch_dog_gnss_.DogIsOk(3)) {  //没有接收gps数据
    fusion_navigation_code.state_code.push_back(3107);
  }
  if (!watch_dog_lidar_odom_.DogIsOk(3) &&
      open_fusion_localization_) {  //没有接收到lidarodom数据
    fusion_navigation_code.state_code.push_back(3108);
  }
  //没有读取gps变换矩阵
  if (open_fusion_localization_ &&
      !(get_gps_to_map_ && flag_livox_to_gps_ && get_gnss_fixed_point_)) {
    fusion_navigation_code.state_code.push_back(3119);
  }

  if (open_fusion_localization_)
    fusion_navigation_code.state_code.push_back(3102);
  //进入寻迹模式
  if (fusion_navigation_code.state_code.empty())
    fusion_navigation_code.state_code.push_back(3100);
  pub_fusion_state_information_.publish(fusion_navigation_code);
}

// Eigen::Matrix<double, 7, 1> EKFLocalizer::StateTransition(
//     const Eigen::Matrix<double, 7, 1> &state,
//     const Eigen::Matrix<double, 1, 1> &input) {
//   double dt = predict_delta_time_;
//   Eigen::Matrix<double, 7, 1> x_next;
//   if (fabs(state(4)) < 0.01) {
//     x_next(0) = state(0) + (state(3) * state(5)) * cos(state(2)) * dt;
//     x_next(1) = state(1) + (state(3) * state(5)) * sin(state(2)) * dt;
//     x_next(2) = (state(4) + state(6)) * dt + state(2);
//     x_next(3) = state(3);
//     x_next(4) = state(4);
//     x_next(5) = state(5);
//     x_next(6) = state(6);
//   } else {
//     x_next(0) = state(0) + ((state(3) * state(5)) / (state(4) + state(6))) *
//                                (sin((state(4) + state(6)) * dt + state(2)) -
//                                 sin(state(2)));
//     x_next(1) = state(1) + ((state(3) * state(5)) / (state(4) + state(6))) *
//                                (cos(state(2)) -
//                                 cos((state(4) + state(6)) * dt + state(2)));
//     x_next(2) = (state(4) + state(6)) * dt + state(2);
//     x_next(3) = state(3);
//     x_next(4) = state(4);
//     x_next(5) = state(5);
//     x_next(6) = state(6);
//   }

//   return x_next;
// }

// Eigen::Matrix<double, 3, 1> EKFLocalizer::StateObservation(
//     const Eigen::Matrix<double, 7, 1> &state,
//     const Eigen::Matrix<double, 3, 7> &observe_matrix) {
//   Eigen::Matrix<double, 3, 1> state_observation;
//   state_observation = observe_matrix * state;
//   return state_observation;
// }

Eigen::Matrix<double, 7, 7> EKFLocalizer::UpdateJacobianMatrix(
    const Eigen::Matrix<double, 7, 1> &state_last, const double dt) {
  Eigen::Matrix<double, 7, 7> jacobian;
  jacobian.setZero();

  double yaw = state_last(2);
  double v = state_last(3);
  double w = state_last(4);
  double scale = state_last(5);
  double bias = state_last(6);
  // if (fabs(state_last(4)) < 0.001) {
  jacobian(0, 0) = 1.0;
  jacobian(0, 2) = -dt * v * scale * sin(yaw);
  jacobian(0, 3) = dt * scale * cos(yaw);
  jacobian(0, 5) = dt * v * cos(yaw);

  jacobian(1, 1) = 1.0;
  jacobian(1, 2) = dt * v * scale * cos(yaw);
  jacobian(1, 3) = dt * scale * sin(yaw);
  jacobian(1, 5) = dt * v * sin(yaw);

  jacobian(2, 2) = 1.0;
  jacobian(2, 4) = dt;
  jacobian(2, 6) = dt;

  jacobian(3, 3) = 1.0;
  jacobian(4, 4) = 1.0;
  jacobian(5, 5) = 1.0;
  jacobian(6, 6) = 1.0;

  // } else {
  //   jacobian(0, 0) = 1.0;
  //   // v*scale/(w+bias)*(cos((w+bias)*dt +yaw) - cos(yaw))
  //   jacobian(0, 2) =
  //       v * scale / (w + bias) * (cos((w + bias) * dt + yaw) - cos(yaw));
  //   // v_sacle/(w+bias) * (sin((w+bias)*dt + yaw) - sin(yaw))
  //   jacobian(0, 3) =
  //       state_last(5) / (state_last(4) + state_last(6)) *
  //       (sin((state_last(4) + state_last(6)) * dt + state_last(2)) -
  //        sin(state_last(2)));
  //   jacobian(0, 3) =
  //       scale / (w + bias) * (sin((w + bias) * dt + yaw) - sin(yaw));
  //   // v*sacle *dt/(w+bias) * cos((w+bias)dt +yaw) -
  //   // v*sacle/((w+bias)*(w+bias))* (sin((w+bias)*dt +yaw) - sin(yaw))
  //   jacobian(0, 4) = v * scale * dt / (w + bias) * cos((w + bias) * dt + yaw)
  //   -
  //                    v * scale / ((w + bias) * (w + bias)) *
  //                        (sin((w + bias) * dt + yaw) - sin(yaw));
  //   jacobian(0, 5) = v / (w + bias) * (sin((w + bias) * dt + yaw) -
  //   sin(yaw)); jacobian(0, 6) = v * scale * dt / (w + bias) * cos((w + bias)
  //   * dt + yaw) -
  //                    v * scale / ((w + bias) * (w + bias)) *
  //                        (sin((w + bias) * dt + yaw) - sin(yaw));

  //   jacobian(1, 1) = 1.0;
  //   jacobian(1, 2) =
  //       v * scale / (w + bias) * (sin((w + bias) * dt + yaw) - sin(yaw));
  //   jacobian(1, 3) =
  //       scale / (w + bias) * (cos(yaw) - cos((w + bias) * dt + yaw));
  //   jacobian(1, 4) =
  //       v * scale * dt / (w + bias) * (sin((w + bias) * dt + yaw)) -
  //       v * scale / ((w + bias) * (w + bias)) *
  //           (-cos((w + bias) * dt + yaw) + cos(yaw));
  //   jacobian(1, 5) = v / (w + bias) * (-cos((w + bias) * dt + yaw) +
  //   cos(yaw));

  //   jacobian(1, 6) =
  //       v * scale * dt / (w + bias) * (sin((w + bias) * dt + yaw)) -
  //       v * scale / ((w + bias) * (w + bias)) *
  //           (-cos((w + bias) * dt + yaw) + cos(yaw));

  //   jacobian(2, 2) = 1.0;
  //   jacobian(2, 4) = dt;
  //   jacobian(2, 6) = dt;

  //   jacobian(3, 3) = 1.0;
  //   jacobian(4, 4) = 1.0;

  //   jacobian(5, 5) = 1.0;
  //   jacobian(6, 6) = 1.0;
  // }
  // for (size_t i = 0; i < 7; i++) {
  //   for (size_t j = 0; j < 7; j++) {
  //     std::cout << " " << jacobian(i, j);
  //   }
  //   AINFO << " ";
  // }

  return jacobian;
}

void EKFLocalizer::ReceiveGpsDatas(const nav_msgs::Odometry &gps_datas) {
  AINFO << "reacive gps data";
  watch_dog_gnss_.UpdataNow();
  if (!flag_use_gps_data_ || !get_gps_to_map_ || !flag_livox_to_gps_ ||
      !get_gnss_fixed_point_)
    return;
  //   double gps_time_now = gps_datas.header.stamp.toSec();
  gnss_time_ = gps_datas.header.stamp;

  Eigen::Quaterniond gps_quater_now(
      gps_datas.pose.pose.orientation.w, gps_datas.pose.pose.orientation.x,
      gps_datas.pose.pose.orientation.y, gps_datas.pose.pose.orientation.z);

  Eigen::Vector3d gps_pos_now(
      gps_datas.pose.pose.position.x - gnss_fixed_pos_.x(),
      gps_datas.pose.pose.position.y - gnss_fixed_pos_.y(),
      gps_datas.pose.pose.position.z - gnss_fixed_pos_.z());
  // SensorToCenter(gnss_to_map_quater_, gnss_to_map_pos_, gps_quater_now,
  //                gps_pos_now);
  // gps->lidar, ->to map
  Eigen::Quaterniond quater_lidar_gps = gps_quater_now * quater_livox_to_gps_;
  Eigen::Vector3d pos_lidar_gps =
      gps_quater_now * pos_livox_to_gps_ + gps_pos_now;

  // to map
  Eigen::Quaterniond quater_map_gps = gnss_to_map_quater_ * quater_lidar_gps;
  Eigen::Vector3d pos_map_gps =
      gnss_to_map_quater_ * pos_lidar_gps + gnss_to_map_pos_;

  tf::Matrix3x3 gps_mat(tf::Quaternion(quater_map_gps.x(), quater_map_gps.y(),
                                       quater_map_gps.z(), quater_map_gps.w()));

  if (gps_datas.pose.covariance[0] > 0.15 ||
      gps_datas.pose.covariance[7] > 0.15)
    return;

  nav_msgs::Odometry gnss_position;
  gnss_position.header.frame_id = "camera_init";
  gnss_position.header.stamp = ros::Time::now();
  gnss_position.pose.pose.position.x = pos_map_gps.x();
  gnss_position.pose.pose.position.y = pos_map_gps.y();
  gnss_position.pose.pose.orientation.x = quater_map_gps.x();
  gnss_position.pose.pose.orientation.y = quater_map_gps.y();
  gnss_position.pose.pose.orientation.z = quater_map_gps.z();
  gnss_position.pose.pose.orientation.w = quater_map_gps.w();
  pub_gnss_pose_.publish(gnss_position);

  double gps_roll, gps_pitch, gps_yaw;
  gps_mat.getRPY(gps_roll, gps_pitch, gps_yaw);

  pitch_car_ = gps_pitch;
  R_gps_.setZero();
  R_gps_(0, 0) = gps_datas.pose.covariance[0];
  R_gps_(1, 1) = gps_datas.pose.covariance[7];
  R_gps_(2, 2) = gps_datas.pose.covariance[35];
  gps_observation_ << pos_map_gps.x(), pos_map_gps.y(), gps_yaw;
  flag_receive_new_gnss_ = true;
  AINFO << "reacive gps data ok! : " << gps_observation_(0) << " "
        << gps_observation_(1) << " " << gps_observation_(2) << " "
        << R_gps_(0, 0) << " " << R_gps_(1, 1) << " " << R_gps_(2, 2);
}

void EKFLocalizer::ReceiveSlamDatas(const nav_msgs::Odometry &slam_datas) {
  AINFO << "reacive lidar data";
  watch_dog_lidar_odom_.UpdataNow();
  if (!use_lidar_data_) return;
  double lidar_odom_cov = slam_datas.pose.covariance[0];
  //   now_lidar_time_ = slam_datas.header.stamp.toSec();
  slam_time_ = slam_datas.header.stamp;
  if (lidar_odom_cov > 2.0) {
    return;
  }
  tf::Matrix3x3 slam_mat(tf::Quaternion(
      slam_datas.pose.pose.orientation.x, slam_datas.pose.pose.orientation.y,
      slam_datas.pose.pose.orientation.z, slam_datas.pose.pose.orientation.w));
  double slam_roll, slam_pitch, slam_yaw;
  slam_mat.getRPY(slam_roll, slam_pitch, slam_yaw);

  pitch_car_ = slam_pitch;
  R_slam_.setZero();
  R_slam_(0, 0) = lidar_odom_cov;
  R_slam_(1, 1) = lidar_odom_cov;
  R_slam_(2, 2) = lidar_odom_cov;

  lidar_observation_ << slam_datas.pose.pose.position.x,
      slam_datas.pose.pose.position.y, slam_yaw;
  flag_receive_new_lidar_data_ = true;
  AINFO << "reaceive lidar data ok! "
        << " " << lidar_observation_(0) << " " << lidar_observation_(1) << " "
        << lidar_observation_(2) << " " << R_slam_(0, 0) << " " << R_slam_(1, 1)
        << " " << R_slam_(2, 2);
}

void EKFLocalizer::EkfInit(Eigen::MatrixXd X) {
  // Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd P =
      Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 100.00;  // for x & y
  P(IDX::YAW, IDX::YAW) = 50.0;                            // for yaw
  //   P(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;          // for yaw bias
  P(IDX::V, IDX::V) = 100.0;  // for vx
  P(IDX::W, IDX::W) = 50.0;   // for wz
  P(IDX::VS, IDX::VS) = cov_vs_d_;
  P(IDX::WB, IDX::WB) = cov_wb_d_;

  AWARN << "X: " << X(0) << " " << X(1) << " " << X(2) << " " << X(3) << " "
        << X(4) << " " << X(5) << " " << X(6);
  ekf_filter_.init(X, P, extend_state_step_);
  flag_filter_init_ = true;
}

void EKFLocalizer::FixYawAdaptPrediction(double &yaw) {
  if (yaw > PI) {
    yaw = yaw - 2.0 * PI;
  } else if (yaw < -PI) {
    yaw = yaw + 2.0 * PI;
  }
}

double EKFLocalizer::NormalizeYaw(const double &yaw) {
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

//得到标定参数
bool EKFLocalizer::GetCalibYamlParam(const std::string file_path,
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

bool EKFLocalizer::GetFixedGpsPoint(const std::string file_path,
                                    Eigen::Vector3d &translation) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["fixed"]) {
      AERROR << "load extrinsics: " << file_path << " not fixed";
      translation = Eigen::Vector3d(0.0, 0.0, 0.0);
      return false;
    }
    double tx = config["fixed"]["x"].as<double>();
    double ty = config["fixed"]["y"].as<double>();
    double tz = config["fixed"]["z"].as<double>();

    translation = Eigen::Vector3d(tx, ty, tz);
    AWARN << "tx, ty, tz : " << tx << " " << ty << " " << tz;
  } catch (const YAML::Exception &e) {
    AERROR << "load extrinsics: " << file_path
           << " failed! error: " << e.what();
    translation = Eigen::Vector3d(0.0, 0.0, 0.0);
    return false;
  }
  return true;
}

Eigen::Matrix<double, 2, 3> EKFLocalizer::GetMapToImageParam(
    const std::string file_path) {
  std::string yaml_path = file_path + "map_to_image.yaml";
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  cv::Mat map_to_image;
  fs["map_to_image"] >> map_to_image;
  Eigen::Matrix<double, 2, 3> eigen_map_to_image;
  eigen_map_to_image << map_to_image.at<double>(0, 0),
      map_to_image.at<double>(0, 1), map_to_image.at<double>(0, 2),
      map_to_image.at<double>(1, 0), map_to_image.at<double>(1, 1),
      map_to_image.at<double>(1, 2);
  std::cout << "map_to_image: " << eigen_map_to_image << std::endl;
  return eigen_map_to_image;
}

// bool EKFLocalizer::GetCalibGnssPoseYamlParam(const std::string file_path,
//                                              Eigen::Vector3d &translation) {
//   try {
//     YAML::Node config = YAML::LoadFile(file_path);
//     if (!config["gnss"]) {
//       return false;
//     }
//     double tx = config["gnss"]["gnss_origin_x"].as<double>();
//     double ty = config["gnss"]["gnss_origin_y"].as<double>();
//     double tz = config["gnss"]["gnss_origin_z"].as<double>();
//     translation = Eigen::Vector3d(tx, ty, tz);
//     AWARN << "tx, ty, tz : " << tx << " " << ty << " " << tz;

//   } catch (const YAML::Exception &e) {
//     AERROR << "load extrinsics: " << file_path
//            << " failed! error: " << e.what();
//     return false;
//   }
//   return true;
// }

//传感器矫正
void EKFLocalizer::SensorToCenter(const Eigen::Quaterniond &tran_quater,
                                  const Eigen::Vector3d &tran_pos,
                                  Eigen::Quaterniond &sensor_quater,
                                  Eigen::Vector3d &sensor_pos) {
  Eigen::Quaterniond sensor_quat = sensor_quater;
  Eigen::Vector3d sensor_position = sensor_pos;
  sensor_quater = tran_quater * sensor_quat;
  sensor_pos = tran_quater * sensor_position + tran_pos;
}

double EKFLocalizer::RpyToSignRpy(double rpy) {
  if (rpy >= M_PI) {
    rpy -= 2.0 * M_PI;
  }
  return rpy;
}
double EKFLocalizer::CalcDiffForRadian(const double now_rad,
                                       const double before_rad) {
  double diff_rad = now_rad - before_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI) {
    diff_rad = diff_rad + 2 * M_PI;
  }
  return diff_rad;
}

EKFLocalizer::~EKFLocalizer() {}

}  // namespace fusion_location
}  // namespace localization
}  // namespace navigation
}  // namespace sweeper

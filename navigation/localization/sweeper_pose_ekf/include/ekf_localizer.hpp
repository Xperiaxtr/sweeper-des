#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <deque>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "../../../../common/log.h"
#include "../../../../common/watch_dog.h"
#include "kalman/kalman_filter.hpp"
#include "kalman/time_delay_kalman_filter.hpp"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweepMission.h"
#include "sweeper_msgs/SweeperChassisDetail.h"

// #define UN_RECEIVED_LIDAR_ODOM 3117
// #define UN_RECEIVED_SWEEPER_ODOM 3118
// #define UN_RECEIVED_GNSS_DATA 3110
// #define CIN_CRUISE_TRACE_MODE 3104
// #define CRUISE_TRACE_MODE_SUCESS 3102
// #define UN_RECEIVED_SPEED_DATA 3106

#define LOCALIZATION_MODE_NORMAL_ 3300
#define LOCALIZATION_PROCESSING_ 3301
#define UN_RECEIVED_GNSS_DATA_ 3304
#define UN_RECEIVED_VEHICAL_DATA_ 3306
#define UN_RECEIVED_LIDAR_MATCHER_DATA_ 3308
#define UN_RECEIVED_IMU_DATA_ 3305

#define PI 3.1415926
const int imu_length_ = 200;
const int gps_length_ = 3;
namespace sweeper {
namespace navigation {
namespace localization {
namespace fusion_location {
class EKFLocalizer {
 public:
  EKFLocalizer(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~EKFLocalizer();

 private:
  //   Eigen::Matrix<double, 7, 1> StateTransition(
  //       const Eigen::Matrix<double, 7, 1> &state,
  //       const Eigen::Matrix<double, 1, 1> &input);
  //   Eigen::Matrix<double, 3, 1> StateObservation(
  //       const Eigen::Matrix<double, 7, 1> &state,
  //       const Eigen::Matrix<double, 3, 7> &observe_matrix);
  void ReceiveSweeperMode(sweeper_msgs::SweepMission sweeper_mode_command);
  void GetSweeperInformation(const ros::TimerEvent &e);
  void ReceiveLocalizationDiagnose(
      const sweeper_msgs::SensorFaultInformation mapping_state);
  void ReceiveSlamDatas(const nav_msgs::Odometry &slam_datas);
  void ReceiveGpsDatas(const nav_msgs::Odometry &gps_datas);
  void ReceiveImuDatas(const sensor_msgs::Imu::ConstPtr &input);
  void ReceiveSpeedDatas(
      const sweeper_msgs::SweeperChassisDetail sweeper_chassis_detail);
  Eigen::Matrix<double, 7, 7> UpdateJacobianMatrix(
      const Eigen::Matrix<double, 7, 1> &state_last, const double dt);
  double NormalizeYaw(const double &yaw);
  void FixYawAdaptPrediction(double &yaw);
  //   void FusionProcess(const ros::Time sensor_time);
  void EkfPrediction(double delta_t);
  void EkfInit(Eigen::MatrixXd X);
  bool GetCalibYamlParam(const std::string file_path,
                         Eigen::Quaterniond &rotation,
                         Eigen::Vector3d &translation);
  void SensorToCenter(const Eigen::Quaterniond &tran_quater,
                      const Eigen::Vector3d &tran_pos,
                      Eigen::Quaterniond &sensor_quater,
                      Eigen::Vector3d &sensor_pos);
  double RpyToSignRpy(double rpy);
  Eigen::Matrix<double, 7, 7> SetProcessNoise(
      double dt, Eigen::Matrix<double, 7, 1> x_last);
  double CalcDiffForRadian(const double now_rad, const double before_rad);
  bool GetCalibGnssPoseYamlParam(const std::string file_path,
                                 Eigen::Vector3d &translation);
  bool GetFixedGpsPoint(const std::string file_path,
                                      Eigen::Vector3d &translation);

  void GnssMeasurementUpdatePose();
  void PredictEKFModel();
  void PublishFusionResult();
  void SlamMeasurementUpdatePose();
  void TimerCallback(const ros::TimerEvent &e);
  Eigen::Matrix<double, 2, 3> GetMapToImageParam(const std::string file_path);

 private:
  enum IDX { X = 0, Y = 1, YAW = 2, V = 3, W = 4, VS = 5, WB = 6 };

  bool flag_filter_init_;

  //   Eigen::Matrix<double, 7, 7> jacobian_;
  Eigen::Matrix<double, 3, 1> gps_observation_;
  Eigen::Matrix<double, 3, 1> lidar_observation_;
  //   Eigen::Matrix<double, 7, 1> x_last_;
  Eigen::Matrix<double, 3, 3> R_gps_;
  Eigen::Matrix<double, 3, 3> R_slam_;

  //   Eigen::Quaterniond gnss_to_lidar_quater_;
  //   Eigen::Vector3d gnss_to_lidar_pos_;
  //   Eigen::Vector3d gnss_origin_pos_;
  //   Eigen::Vector3d xyz_acc_;
  double angular_acc_;

  sweeper::common::WatchDog watch_dog_imu_;
  sweeper::common::WatchDog watch_dog_gnss_;
  sweeper::common::WatchDog watch_dog_lidar_odom_;
  sweeper::common::WatchDog watch_dog_vehical_;
  sweeper_msgs::SensorFaultInformation::_state_code_type
      receive_localization_state_code_;

  bool open_fusion_localization_;
  bool flag_receive_new_gnss_;
  bool flag_receive_new_lidar_date_;
  //   bool flag_receive_new_imu_data_;
  bool use_small_map_mode_;
  double now_lidar_time_;
  double now_gnss_time_;
  double last_speed_time_;
  double angular_velocity_yaw_;
  double imu_pitch_;
  double predict_delta_time_;

  bool use_lidar_data_;
  bool flag_use_gps_data_;
  bool position_processing_;

  //   std::deque<EkfLastParam> ekf_last_params_;

  int untrusted_number_;

  ros::Subscriber sub_gps_, sub_slam_, sub_odom_, sub_sweeper_mode_, sub_imu_,
      sub_speed_;
  ros::Subscriber sub_state_localization_information_;
  ros::Publisher pub_fusion_pose_, pub_speed_pose_, pub_gnss_pose_,
      pub_image_pose_;
  ros::Publisher pub_fusion_state_information_;
  ros::Timer timer_ekf_navigation_;
  ros::Timer timer_control_;

  std::string last_pcd_name_;
  Eigen::Vector3d gnss_to_map_pos_, pos_livox_to_gps_, gnss_fixed_pos_;
  Eigen::Quaterniond gnss_to_map_quater_, quater_livox_to_gps_;
  bool get_gps_to_map_, flag_livox_to_gps_, get_gnss_fixed_point_;
  double origin_x_, origin_y_, resolution_;
  double pitch_car_, pitch_car_last_;
  double ekf_rate_, ekf_dt_;
  double sweeper_vehicle_plane_;
  bool flag_receive_new_vehical_data_, flag_receive_new_imu_data_,
      flag_receive_new_lidar_data_;
  int dim_x_, dim_x_ex_;
  double cov_yaw_d_, cov_v_d_, cov_w_d_, cov_wb_d_, cov_vs_d_;
  ros::Time gnss_time_, slam_time_, now_time_;
  bool use_pose_with_covariance_;
  double pose_measure_uncertainty_time_;
  int extend_state_step_;
  double pose_additional_delay_, pose_rate_;
  double pose_stddev_x_, pose_stddev_y_, pose_stddev_yaw_;
  Eigen::Matrix<double, 2, 3> map_to_image_;

  TimeDelayKalmanFilter ekf_filter_;
};

}  // namespace fusion_location
}  // namespace localization
}  // namespace navigation
}  // namespace sweeper

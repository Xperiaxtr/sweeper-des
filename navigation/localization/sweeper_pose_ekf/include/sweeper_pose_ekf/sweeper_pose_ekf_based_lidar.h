#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <deque>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include "../../../../common/log.h"
#include "../../../../common/math/extended_kalman_filter.h"
#include "../../../../common/watch_dog.h"
// #include "../../mapping/include/tools/math_calculation.hpp"
// #include
// "../../../../../navigation/mapping/include/tools/math_calculation.hpp"
#include <tf/transform_broadcaster.h>

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
class SweeperLocalization {
 public:
  struct EkfLastParam {
    Eigen::Matrix<double, 5, 1> last_x;
    Eigen::Matrix<double, 5, 5> last_p;
  };

 public:
  SweeperLocalization(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~SweeperLocalization();

 private:
  Eigen::Matrix<double, 5, 1> StateTransition(
      const Eigen::Matrix<double, 5, 1> &state,
      const Eigen::Matrix<double, 1, 1> &input);
  Eigen::Matrix<double, 3, 1> StateObservation(
      const Eigen::Matrix<double, 5, 1> &state,
      const Eigen::Matrix<double, 3, 5> &observe_matrix);
  void ReceiveSweeperMode(sweeper_msgs::SweepMission sweeper_mode_command);
  void GetSweeperInformation(const ros::TimerEvent &e);
  void ReceiveLocalizationDiagnose(
      const sweeper_msgs::SensorFaultInformation mapping_state);
  void ReceiveSlamDatas(const nav_msgs::Odometry &slam_datas);
  void ReceiveGpsDatas(const nav_msgs::Odometry &gps_datas);
  void ReceiveImuDatas(const sensor_msgs::Imu::ConstPtr &input);
  void ReceiveSpeedDatas(
      const sweeper_msgs::SweeperChassisDetail sweeper_chassis_detail);
  void UpdateJacobianMatrix(const Eigen::Matrix<double, 5, 1> &state_last,
                            const double dt);
  double NormalizeYaw(const double &yaw);
  void FixYawAdaptPrediction(double &yaw);
  //   void FusionProcess(const ros::Time sensor_time);
  void EkfPrediction(double delta_t);
  void EkfInit();
  bool GetCalibYamlParam(const std::string file_path,
                         Eigen::Quaterniond &rotation,
                         Eigen::Vector3d &translation);
  void SensorToCenter(const Eigen::Quaterniond &tran_quater,
                      const Eigen::Vector3d &tran_pos,
                      Eigen::Quaterniond &sensor_quater,
                      Eigen::Vector3d &sensor_pos);
  double RpyToSignRpy(double rpy);
  Eigen::Matrix<double, 5, 5> SetProcessNoise(double dt);
  double CalcDiffForRadian(const double now_rad, const double before_rad);
  bool GetCalibGnssPoseYamlParam(const std::string file_path,
                                 Eigen::Vector3d &translation);
  bool GetImageYamlParam(const std::string file_path, double &origin_x,
                         double &origin_y, double &resolution);

  double omga_variance_;
  double acc_;
  double slam_cov_mag_;
  double delta_t_;
  double fusion_yaw_last_;
  bool flag_filter_init_;

  Eigen::Matrix<double, 5, 5> jacobian_;
  Eigen::Matrix<double, 3, 1> gps_observation_;
  Eigen::Matrix<double, 3, 1> lidar_observation_;
  Eigen::Matrix<double, 5, 1> x_last_;
  Eigen::Matrix<double, 3, 3> R_gps_;
  Eigen::Matrix<double, 3, 3> R_slam_;

  //   Eigen::Quaterniond gnss_to_lidar_quater_;
  //   Eigen::Vector3d gnss_to_lidar_pos_;
  Eigen::Vector3d gnss_origin_pos_;
  Eigen::Vector3d xyz_acc_;
  double angular_acc_;

  sweeper::common::WatchDog watch_dog_imu_;
  sweeper::common::WatchDog watch_dog_gnss_;
  sweeper::common::WatchDog watch_dog_lidar_odom_;
  sweeper::common::WatchDog watch_dog_vehical_;
  sweeper_msgs::SensorFaultInformation::_state_code_type
      receive_localization_state_code_;

  bool open_fusion_localization_;
  bool receive_new_gnss_data_;
  bool receive_new_lidar_data_;
  bool receive_new_imu_data_;
  bool use_small_map_mode_;
  double now_lidar_time_;
  double now_gnss_time_;
  double last_speed_time_;
  double angular_velocity_yaw_;
  double imu_pitch_;
  double predict_delta_time_;

  bool use_lidar_data_;
  bool use_gnss_data_;
  bool position_processing_;

  std::deque<EkfLastParam> ekf_last_params_;

  int untrusted_number_;

  ros::Subscriber sub_gps_, sub_slam_, sub_odom_, sub_sweeper_mode_, sub_imu_,
      sub_speed_;
  ros::Subscriber sub_state_localization_information_;
  ros::Publisher pub_fusion_pose_, pub_speed_pose_, pub_gnss_pose_,
      pub_image_pose_, pub_map_pose_;
  ros::Publisher pub_fusion_state_information_;
  ros::Timer timer_ekf_navigation_;

  std::string last_pcd_name_;
  Eigen::Vector3d gnss_to_map_pos_;
  Eigen::Quaterniond gnss_to_map_quater_;
  bool get_gps_to_map_, get_map_to_image_, flag_receive_imu_;
  double origin_x_, origin_y_, resolution_;


  sweeper::common::math::ExtendedKalmanFilter<double, 5, 3, 1> ekf_filter_;
};

}  // namespace fusion_location
}  // namespace localization
}  // namespace navigation
}  // namespace sweeper

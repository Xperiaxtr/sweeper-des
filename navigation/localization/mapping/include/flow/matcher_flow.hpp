/*
融合定位模块，以激光定位为主。
gps数据提供初始位置，imu与底盘速度提供预测模块，
lidar定位对imu与底盘速度模块进行更正，同时imu与底盘模块对lidar进行修正。
当lidar误差很大时，gps再次初始化。
*/

#ifndef MAPPING_FLOW_MATCHER_FLOW_HPP_
#define MAPPING_FLOW_MATCHER_FLOW_HPP_

#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/registration/ndt.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
#include <mutex>
#include <random>

#include "../../../../common/log.h"
#include "../../../../common/watch_dog.h"
#include "get_feature/get_feature.hpp"
#include "matcher/matcher.hpp"
#include "matcher/new_matcher.hpp"
#include "matcher/se3_matcher.hpp"
#include "mcl_matcher.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "sensor_data/cloud_data.hpp"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweepMission.h"
#include "sweeper_msgs/SweeperChassisDetail.h"
#include "tools/map_segment/map_segment.hpp"
// #include "tools/math_calculation.hpp"
#include <ndt_cpu/NormalDistributionsTransform.h>
#include <omp.h>
#include <pclomp/ndt_omp.h>

#define LOCALIZATION_FUCTION_NORMAL_ 3102
#define ENTER_LOCALIZATION_FUCTION_ 3104
#define UN_RECEIVED_GNSS_DATA_ 3107
#define UN_RECEIVED_LIVOX_DATA_ 3108
#define UN_RECEIVE_INIT_POSE_ 3113
#define INIT_POSE_FAIL_ 3114
#define LOAD_MAP_FAIL_ 3116
#define CAR_FRONT_DIS_ 0.60
#define CAR_BACK_DIS_ 0.85

namespace mapping {
class MatcherFlow {
 public:
  struct Pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
  };
  struct Point2 {
    double score;
    int ind;
  };

  struct SweeperInformation {
    double time;
    double distance;
    Eigen::Quaterniond diff_quater;
  };
  struct ImuInformation {
    double time;
    // Eigen::Quaterniond diff_quater;
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
  };

  MatcherFlow(ros::NodeHandle nh, ros::NodeHandle nh_private);
  void InitMatcherParam();
  void CurbAndLoadPcd(std::string load_pcd_name);

  double RpyToSignRpy(double rpy);
  double CalcDiffForRadian(const double now_rad, const double before_rad);

  Pose ConvertPoseIntoRelativeCoordinate(const Pose target_pose,
                                         const Pose reference_pose);

  void ReceiveImuData(const sensor_msgs::Imu::ConstPtr& input);
  void ReceiveLivoxData(const sensor_msgs::PointCloud2::ConstPtr& input);
  void ReceiveVehicleData(
      const sweeper_msgs::SweeperChassisDetail sweeper_chassis_detail);
  void ReceiveGnssData(const nav_msgs::Odometry::ConstPtr& input);

  bool UpdateImuData(ros::Time current_time);
  bool UpdateVehicleData(ros::Time current_time);
  bool UpdataImuVehicleData(ros::Time current_time);

  void PubLocalizationInformation(const ros::TimerEvent& e);
  void ReceiveSweeperMode(sweeper_msgs::SweepMission sweeper_mode_command);
  void EnterFilterParam(double corner_filter_size, double surf_filter_size,
                        double intensity_filter_size,
                        double corner_map_filter_size,
                        double surf_map_filter_size,
                        double intensity_map_filter_size);

  bool SegmentMap(Eigen::Vector3d pose);
  static bool GetMaxValue(Point2 i, Point2 j);
  bool MCLInit(Eigen::Matrix4d init_pose, int particle_num,
               Eigen::Matrix<double, 1, 4> pose_cov);
  double GetMCLScore(Pose& pose, Feature& now_feature);
  void MatcherNode();

  Eigen::Matrix4d PredictLidarPose(double lidar_time);

  void CloudAdjustMotion(const Eigen::Matrix4d& tran_pose,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud_ptr);
  double GetMatcherScore(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& score_cloud,
      const Eigen::Matrix4f& tran_pose, const double& range,
      const double& height);

 private:
  cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> anh_ndt_;
  pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr
      ndt_ptr_;
  pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr
      register_;  // register object
  pcl::VoxelGrid<CloudData::POINT> cloud_filter_;
  pcl::VoxelGrid<CloudData::POINT> local_map_filter_;
  pcl::VoxelGrid<CloudData::POINT> display_filter_;

  pcl::VoxelGrid<CloudData::POINT> corner_filter_;
  pcl::VoxelGrid<CloudData::POINT> surf_filter_;
  pcl::VoxelGrid<CloudData::POINT> intensity_filter_;
  pcl::VoxelGrid<CloudData::POINT> map_corner_filter_;
  pcl::VoxelGrid<CloudData::POINT> map_surf_filter_;
  pcl::VoxelGrid<CloudData::POINT> map_intensity_filter_;

  CloudData::CLOUD_PTR local_map_ptr_;
  SE3Matcher* matcher_;
  //   NewMatcher *new_matcher_;
  MCLMatcher mcl_matcher_;
  MapSegment map_segment_;
  sensor_msgs::Imu imu_;

  ros::Subscriber sub_gnss_data_, sub_livox_data_, sub_vehicle_data_,
      sub_imu_data, sub_sweeper_mode_;
  ros::Publisher pub_localizer_odom_, pub_livox_odom_, pub_map_, pub_now_cloud_,
      pub_cov_odom_, pub_state_information_;
  ros::Timer timer_matcher_state_;
  std::shared_ptr<CloudPublisher> local_map_pub_ptr;
  std::shared_ptr<CloudPublisher> cloud_pub_ptr;
  std::shared_ptr<CloudPublisher> segment_map_pub_ptr_;
  std::shared_ptr<OdometryPublisher> odom_pub_ptr_;
  std::shared_ptr<OdometryPublisher> local_odom_pub_ptr_;
  std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
  std::shared_ptr<OdometryPublisher> init_pose_pub_ptr_, pred_pub_ptr_;

  pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_from_seg_map_;

 private:
  Eigen::Quaterniond livox_to_gnss_quater_;
  Eigen::Vector3d livox_to_gnss_pos_;
  Eigen::Quaterniond gnss_to_map_quater_;
  Eigen::Vector3d gnss_to_map_pos_;
  bool flag_gnss_to_map_;
  bool flag_livox_to_gnss_;
  bool flag_start_lidar_matcher_;
  bool flag_get_fixed_point_;

 private:
  std::deque<SweeperInformation> sweeper_information_deque_;
  std::deque<ImuInformation> imu_information_deque_;
  GetFeature* get_feature_;
  Feature now_feature_;
  bool get_new_feature_;

  CloudData::CLOUD_PTR global_map_corner_ptr_;
  CloudData::CLOUD_PTR global_map_surf_ptr_;
  CloudData::CLOUD_PTR global_map_intensity_ptr_;

  double sweep_vehicle_;
  bool map_loaded_, init_pose_set_;
  bool use_imu_data_, use_odom_data_, use_gnss_data_;
  bool use_ndt_mode_;  //龙马采用ndt模式
  std::string read_pcd_name_;
  std::string last_pcd_name_;
  sweeper::common::WatchDog watch_dog_livox_, watch_dog_imu_,
      watch_dog_vehical_, watch_dog_gnss_;
  Eigen::Vector3d gnss_fixed_pos_;

  double fitness_score_;
  bool get_new_lidar_data_;
  Eigen::Matrix4d now_pose_, predict_pose_, init_pose_;
  bool flag_receive_init_pose_;
  std::mutex predict_mutex_;
};
}  // namespace mapping

#endif

//地图定位的流程
/* 1.载入地图
2.得到初始位置
3.计算得到匹配位姿
*/
#ifndef MAPPING_FLOW_LOCALOZATION_FLOW_HPP_
#define MAPPING_FLOW_LOCALOZATION_FLOW_HPP_

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>

#include "../../../../common/log.h"
#include "../../../../common/watch_dog.h"
#include "get_feature/get_feature.hpp"
#include "matcher/matcher.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweepMission.h"
#include "tools/math_calculation.hpp"

#define LOCALIZATION_FUCTION_NORMAL_ 3102
#define ENTER_LOCALIZATION_FUCTION_ 3104
#define UN_RECEIVED_GNSS_DATA_ 3107
#define UN_RECEIVED_LIVOX_DATA_ 3108
#define UN_RECEIVE_INIT_POSE_ 3113
#define INIT_POSE_FAIL_ 3114
#define LOAD_MAP_FAIL_ 3116

namespace mapping {
class LocalizationFlow {
 public:
 public:
  LocalizationFlow(ros::NodeHandle nh, ros::NodeHandle nh_private);
  bool GetInitPose(Eigen::Quaterniond init_quater,
                   Eigen::Vector3d init_position);
  bool ReadAndLoadMap(std::string map_name);
  void Process();
  void ResetLidarOdomParam();
  void PubLocalizationInformation();
  void PubInformationThread();
  void ReceiveSweeperMode(sweeper_msgs::SweepMission sweeper_mode_command);
  void EnterFilterParam(double corner_filter_size, double surf_filter_size,
                        double intensity_filter_size,
                        double corner_map_filter_size,
                        double surf_map_filter_size,
                        double intensity_map_filter_size);

 private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
  std::shared_ptr<CloudPublisher> corner_bag_pub_ptr_;
  std::shared_ptr<CloudPublisher> corner_feature_pub_ptr_;
  std::shared_ptr<CloudPublisher> surf_feature_pub_ptr_;
  std::shared_ptr<CloudPublisher> intensity_feature_pub_ptr_;
  ros::Subscriber sub_sweeper_mode_;

  std::shared_ptr<CloudPublisher> corner_pub_ptr_;
  std::shared_ptr<CloudPublisher> map_pub_ptr_;
  std::shared_ptr<OdometryPublisher> odom_pub_ptr_;
  ros::Publisher pub_state_information_;
  // ros::Timer timer_lidar_odom_;

  pcl::VoxelGrid<CloudData::POINT> corner_filter_;
  pcl::VoxelGrid<CloudData::POINT> surf_filter_;
  pcl::VoxelGrid<CloudData::POINT> intensity_filter_;
  pcl::VoxelGrid<CloudData::POINT> map_corner_filter_;
  pcl::VoxelGrid<CloudData::POINT> map_surf_filter_;
  pcl::VoxelGrid<CloudData::POINT> map_intensity_filter_;

  sweeper::common::WatchDog watch_dog_livox_;
  // sweeper::common::WatchDog watch_dog_gnss_;
  GetFeature get_feature_;
  double lidar_match_cov_;
  Eigen::Vector3d now_pos_;
  Eigen::Quaterniond now_quater_;
  std::string pcd_name_;
  std::string last_pcd_name_;
  bool load_map_success_;
  bool init_pose_sucess_;
  bool allow_localization_mode_;
  Eigen::Matrix4d init_pose_;
  std::deque<GNSSData> unsynced_gnss_;
  Matcher matcher_;
  int icp_times_;
};
}  // namespace mapping

#endif
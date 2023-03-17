#ifndef MAPPING_MAPPING_FRONT_END_FRONT_END_HPP_
#define MAPPING_MAPPING_FRONT_END_FRONT_END_HPP_

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>

#include "../../../../common/log.h"
#include "mapping/back_end/back_end.hpp"
#include "matcher/matcher.hpp"
#include "matcher/new_matcher.hpp"
#include "matcher/se3_matcher.hpp"
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/feature.hpp"
#include "sensor_data/imu_data.hpp"
#include "sensor_data/key_frame.hpp"
#include "sensor_data/odom_data.hpp"
#include "sensor_data/vehical_speed.hpp"

namespace mapping {
class FrontEnd {
 public:
  // std::deque<KeyFrame> key_frames_;
  // std::deque<Feature> key_frames_cloud_;
  // std::vector<KeyFrame> frist_10_frames_;
  // std::vector<Feature> frist_10_frames_cloud_;
  Eigen::Quaterniond now_quater_;
  Eigen::Vector3d now_pos_;
  double lidar_match_cov_;
  // Feature receive_feature_;
  CloudData::CLOUD_PTR corner_map_bag_;
  CloudData::CLOUD_PTR surf_map_bag_;
  CloudData::CLOUD_PTR intensity_map_bag_;

 public:
  FrontEnd(double corner_filter_size, double surf_filter_size,
           double intensity_filter_size, int icp_times, int bag_frames_num);
  void Process(const pcl::PointCloud<pcl::PointXYZI>::Ptr &corner_cloud,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_cloud,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &intensity_cloud);
  void GetKeyFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr &corner_cloud,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_cloud,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &intensity_cloud);
  void GetMapBag();
  bool OutputNewKeyFrame(Eigen::Matrix4d &odom_pose);
  void PredictLidarPose(Eigen::Quaterniond predict_quater,
                        Eigen::Vector3d predict_pos);
  void ResetFrontEndParam();

 private:
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> corner_key_frames_;
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> surf_key_frames_;
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> intensity_key_frames_;
  pcl::VoxelGrid<CloudData::POINT> map_corner_filter_;
  pcl::VoxelGrid<CloudData::POINT> map_surf_filter_;
  pcl::VoxelGrid<CloudData::POINT> map_intensity_filter_;
  Eigen::Quaterniond last_quater_;
  Eigen::Vector3d last_position_, last_key_pos_;
  int frame_counts_;
  SE3Matcher *matcher;
  bool get_new_key_frame_;
  int bag_frames_num_;
};
}  // namespace mapping

#endif
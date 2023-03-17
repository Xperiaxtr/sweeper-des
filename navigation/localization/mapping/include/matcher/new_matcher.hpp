#pragma once

#include <math.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Dense>
// #include "ceres_matcher.hpp"

class NewMatcher {
 public:
  int icp_max_iterations_;
  float transform_sum[6] = {0};
  float transform_incre[6] = {0};
  float transformTobeMapped[6] = {0};
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surf_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_intensity_map_;

 public:
  NewMatcher(int icp_max_iterations);
  ~NewMatcher();
  //   void PointAssociateToMap(pcl::PointXYZI pi, pcl::PointXYZI &po);

  void CopyMapToMacther(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_intensity_map);
  double CloudToMapMacth(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_intensity);
  void PointAssociateToMap(pcl::PointXYZI const *const pi,
                           pcl::PointXYZI *const po);
  double GetCloudToMapMacth(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_intensity,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner_map,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf_map,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_intensity_map);
  double rad2deg(double radians);
  void PredictMatcherPose(const Eigen::Quaterniond &predict_quater,
                          const Eigen::Vector3d &predict_position);

 public:
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_corner_from_map_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surf_from_map_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_intensity_from_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri;
  pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel;
  Eigen::Quaterniond m_q_w_curr_;
  Eigen::Vector3d m_t_w_curr_;
};

// NewMatcher::NewMatcher(/* args */) {
//   laserCloudOri.reset(new pcl::PointCloud<pcl::PointXYZI>());
//   coeffSel.reset(new pcl::PointCloud<pcl::PointXYZI>());

//   kdtree_corner_from_map_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
//       new pcl::KdTreeFLANN<pcl::PointXYZI>());
//   kdtree_surf_from_map_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
//       new pcl::KdTreeFLANN<pcl::PointXYZI>());
//   kdtree_intensity_from_map_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
//       new pcl::KdTreeFLANN<pcl::PointXYZI>());
// }

// NewMatcher::~NewMatcher() {}

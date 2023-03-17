#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

#include "lidar_optimization.hpp"

class SE3Matcher {
 public:
  int icp_max_iterations_;
  float transform_sum[6] = {0};
  float transform_incre[6] = {0};
  float transformTobeMapped[6] = {0};

 public:
  SE3Matcher(int icp_max_iterations);
  ~SE3Matcher();
  //   void PointAssociateToMap(pcl::PointXYZI pi, pcl::PointXYZI &po);
  //   void PointAssociateToMap(pcl::PointXYZI const *const pi,
  //                            pcl::PointXYZI *const po);
  double GetCloudToMapMacth(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &intensity_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_edge_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_surf_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_intensity_in);
  double OutputMatcherScore(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &intensity_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_edge_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_surf_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_intensity_in);
  void PredictMatcherPose(const Eigen::Quaterniond &predict_quater,
                          const Eigen::Vector3d &predict_position);
  double CloudToMapMacth(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &intensity_in);
  void CopyMapToMacther(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner_map,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf_map,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_intensity_map);

 public:
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_corner_from_map_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surf_from_map_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_intensity_from_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_edge_in_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_surf_in_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_intensity_in_;
  //   pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri;
  //   pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel;
  //   Eigen::Quaterniond m_q_w_curr_;
  //   Eigen::Vector3d m_t_w_curr_;
  double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> m_q_w_curr_ =
      Eigen::Map<Eigen::Quaterniond>(parameters);
  Eigen::Map<Eigen::Vector3d> m_t_w_curr_ =
      Eigen::Map<Eigen::Vector3d>(parameters + 4);

 private:
  void AddEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in,
                         ceres::Problem &problem,
                         ceres::LossFunction *loss_function);
  void AddIntensityCostFactor(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in,
      ceres::Problem &problem, ceres::LossFunction *loss_function);
  void AddSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in,
                         ceres::Problem &problem,
                         ceres::LossFunction *loss_function);
  void PointAssociateToMap(pcl::PointXYZI const *const pi,
                           pcl::PointXYZI *const po);
};
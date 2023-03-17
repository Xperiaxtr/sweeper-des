#ifndef MAPPING_MATCHER_MCL_MATCHER_HPP_
#define MAPPING_MATCHER_MCL_MATCHER_HPP_

#include <pcl/kdtree/kdtree_flann.h>

#include <eigen3/Eigen/Dense>

#include "../../../../common/log.h"
#include "matcher/ceres_matcher.hpp"
#include "sensor_data/cloud_data.hpp"
#include "tools/math_calculation.hpp"

namespace mapping {
class MCLMatcher {
 public:
  int icp_max_iterations_;
  double lidar_match_score_;
  double matcher_cost_;
  int avail_num_;
  bool mapping;
  CloudData::CLOUD_PTR laser_cloud_corner_map_;
  CloudData::CLOUD_PTR laser_cloud_surf_map_;
  CloudData::CLOUD_PTR laser_cloud_intensity_map_;

  double parameters_[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  double last_parameters_[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  double incre_parameters_[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  Eigen::Map<Eigen::Quaterniond> m_q_w_curr_ =
      Eigen::Map<Eigen::Quaterniond>(parameters_);
  Eigen::Map<Eigen::Vector3d> m_t_w_curr_ =
      Eigen::Map<Eigen::Vector3d>(parameters_ + 4);
  Eigen::Map<Eigen::Quaterniond> m_q_w_last_ =
      Eigen::Map<Eigen::Quaterniond>(last_parameters_);
  Eigen::Map<Eigen::Vector3d> m_t_w_last_ =
      Eigen::Map<Eigen::Vector3d>(last_parameters_ + 4);
  Eigen::Map<Eigen::Quaterniond> m_q_w_incre_ =
      Eigen::Map<Eigen::Quaterniond>(incre_parameters_);
  Eigen::Map<Eigen::Vector3d> m_t_w_incre_ =
      Eigen::Map<Eigen::Vector3d>(incre_parameters_ + 4);

  pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_corner_from_map_;
  pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_surf_from_map_;
  pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_intensity_from_map_;

 public:
  MCLMatcher();
  void ResetIncremtalParam();
  Eigen::Matrix<double, 3, 1> PclPointToEigen(CloudData::POINT &pt);
  void PointAssociateToMap(CloudData::POINT pi, CloudData::POINT &po);
  double GetCloudToMapMacth(const CloudData::CLOUD_PTR &laser_cloud_corner,
                            const CloudData::CLOUD_PTR &laser_cloud_surf,
                            const CloudData::CLOUD_PTR &laser_cloud_intensity);

  void InitMatcherPose(Eigen::Quaterniond init_quater,
                       Eigen::Vector3d init_position, int icp_times);
  void PredictMatcherPose(Eigen::Quaterniond predict_quater,
                          Eigen::Vector3d predict_position);
  void GetMatcherMap(const CloudData::CLOUD_PTR &laser_cloud_corner_map,
                     const CloudData::CLOUD_PTR &laser_cloud_surf_map,
                     const CloudData::CLOUD_PTR &laser_cloud_intensity_map);
  double GetMatcherScore(double max_range,
                         CloudData::CLOUD_PTR &laser_cloud_surf);
  void set_ceres_solver_bound(ceres::Problem &problem);
  void GetKdtree();
  //   Eigen::Matrix<double, 3, 1> PclPointToEigen(CloudData::POINT& pt);

 public:
};
}  // namespace mapping

#endif
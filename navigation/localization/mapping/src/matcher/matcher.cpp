#include "matcher/matcher.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
namespace mapping {

Matcher::Matcher(int icp_max_iterations)
    : laser_cloud_corner_map_(new CloudData::CLOUD()),
      laser_cloud_surf_map_(new CloudData::CLOUD()),
      laser_cloud_intensity_map_(new CloudData::CLOUD()) {
  kdtree_corner_from_map_ = pcl::KdTreeFLANN<CloudData::POINT>::Ptr(
      new pcl::KdTreeFLANN<CloudData::POINT>());
  kdtree_surf_from_map_ = pcl::KdTreeFLANN<CloudData::POINT>::Ptr(
      new pcl::KdTreeFLANN<CloudData::POINT>());
  kdtree_intensity_from_map_ = pcl::KdTreeFLANN<CloudData::POINT>::Ptr(
      new pcl::KdTreeFLANN<CloudData::POINT>());
  m_t_w_incre_ = m_t_w_last_ = m_t_w_curr_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  m_q_w_curr_ = m_q_w_last_ = m_q_w_incre_ =
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  icp_max_iterations_ = icp_max_iterations;
  mapping = true;
}

void Matcher::set_ceres_solver_bound(ceres::Problem &problem) {
  for (unsigned int i = 0; i < 3; i++) {
    double max_speed = 3.0;
    problem.SetParameterLowerBound(incre_parameters_ + 4, i, -max_speed);
    problem.SetParameterUpperBound(incre_parameters_ + 4, i, +max_speed);
  }
}

void Matcher::InitMatcherPose(Eigen::Quaterniond init_quater,
                              Eigen::Vector3d init_position, int icp_times) {
  m_t_w_curr_ = m_t_w_last_ = init_position;
  m_q_w_curr_ = m_q_w_last_ = init_quater;
  // icp_max_iterations_ = icp_times;
}

void Matcher::PredictMatcherPose(Eigen::Quaterniond predict_quater,
                                 Eigen::Vector3d predict_position) {
  m_t_w_curr_ = m_t_w_last_ = predict_position;
  m_q_w_curr_ = m_q_w_last_ = predict_quater;
}

void Matcher::ResetIncremtalParam() {
  // if (mapping) {
  //   m_t_w_curr_ = m_q_w_last_ * m_t_w_incre_ + m_t_w_last_;
  //   m_q_w_curr_ = m_q_w_last_ * m_q_w_incre_;
  // }
  for (size_t i = 0; i < 7; i++) {
    incre_parameters_[i] = 0;
  }
  incre_parameters_[3] = 1.0;
  m_t_w_incre_ = m_t_w_incre_ * 0;
  m_q_w_incre_ = Eigen::Map<Eigen::Quaterniond>(incre_parameters_);
}

Eigen::Matrix<double, 3, 1> Matcher::PclPointToEigen(CloudData::POINT &pt) {
  return Eigen::Matrix<double, 3, 1>(pt.x, pt.y, pt.z);
}

void Matcher::PointAssociateToMap(CloudData::POINT const *const pi, CloudData::POINT *const po) {
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = m_q_w_curr_ * point_curr + m_t_w_curr_;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
  po->intensity = pi->intensity;
}

void Matcher::CopyMapToMacther(
    const CloudData::CLOUD_PTR &laser_cloud_corner_map,
    const CloudData::CLOUD_PTR &laser_cloud_surf_map,
    const CloudData::CLOUD_PTR &laser_cloud_intensity_map) {
  *laser_cloud_corner_map_ = *laser_cloud_corner_map;
  *laser_cloud_surf_map_ = *laser_cloud_surf_map;
  *laser_cloud_intensity_map_ = *laser_cloud_intensity_map;
  kdtree_corner_from_map_->setInputCloud(laser_cloud_corner_map_);
  kdtree_surf_from_map_->setInputCloud(laser_cloud_surf_map_);
  kdtree_intensity_from_map_->setInputCloud(laser_cloud_intensity_map_);
}

// void Matcher::GetKdtree() {
//   kdtree_corner_from_map_->setInputCloud(laser_cloud_corner_map_);
//   kdtree_surf_from_map_->setInputCloud(laser_cloud_surf_map_);
//   kdtree_intensity_from_map_->setInputCloud(laser_cloud_intensity_map_);
// }
double Matcher::CloudToMapMacth(    
  const CloudData::CLOUD_PTR &laser_cloud_corner,
    const CloudData::CLOUD_PTR &laser_cloud_surf,
    const CloudData::CLOUD_PTR &laser_cloud_intensity){
      ResetIncremtalParam();
  int laser_corner_point_num = laser_cloud_corner->points.size();
  int laser_surf_point_num = laser_cloud_surf->points.size();
  int laser_intensity_point_num = laser_cloud_intensity->points.size();

  AWARN << "size corner, surf, intensity : " << laser_corner_point_num << " "
        << laser_surf_point_num << " " << laser_intensity_point_num;

  AWARN << "map size corner, surf, intensity : "
        << laser_cloud_corner_map_->points.size() << " "
        << laser_cloud_surf_map_->points.size() << " "
        << laser_cloud_intensity_map_->points.size();
  ceres::Solver::Summary summary;
  matcher_cost_ = summary.final_cost;
  float angular_diff = 0;
  int surf_num = 0;
  int corner_num = 0;
  int intensity_num = 0;
  CloudData::POINT point_origin;
  CloudData::POINT point_select;
  std::vector<int> point_search_idx;
  std::vector<float> point_search_dis;

  // kdtree_corner_from_map_->setInputCloud(laser_cloud_corner_map_);
  // kdtree_surf_from_map_->setInputCloud(laser_cloud_surf_map_);
  // kdtree_intensity_from_map_->setInputCloud(laser_cloud_intensity_map_);
  AWARN<<"times: "<<icp_max_iterations_;
  for (int iter_count = 0; iter_count < icp_max_iterations_; iter_count++) {
    surf_num = 0;
    corner_num = 0;
    intensity_num = 0;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization =
        new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;
    ceres::ResidualBlockId block_id;
    ceres::Problem problem(problem_options);
    std::vector<ceres::ResidualBlockId> residual_block_ids;

    problem.AddParameterBlock(incre_parameters_, 4, q_parameterization);
    problem.AddParameterBlock(incre_parameters_ + 4, 3);

    for (int i = 0; i < laser_corner_point_num; i++) {
      point_origin = laser_cloud_corner->points[i];
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_corner_from_map_->nearestKSearch(point_select, 5, point_search_idx,
                                              point_search_dis);
      if (point_search_dis[4] < 2.0) {
        bool line_is_avail = true;
        std::vector<Eigen::Vector3d> near_centers;
        Eigen::Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; j++) {
          Eigen::Vector3d tmp(
              laser_cloud_corner_map_->points[point_search_idx[j]].x,
              laser_cloud_corner_map_->points[point_search_idx[j]].y,
              laser_cloud_corner_map_->points[point_search_idx[j]].z);
          center += tmp;
          near_centers.push_back(tmp);
        }
        center = center / 5.0;  //中点的xyz值
        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
        for (int j = 0; j < 5; j++) {
          Eigen::Matrix<double, 3, 1> tmpZeroMean = near_centers[j] - center;
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
          line_is_avail = true;
        } else {
          line_is_avail = false;
        }
        Eigen::Vector3d curr_point(point_origin.x, point_origin.y,
                                   point_origin.z);
        if (line_is_avail) {
          ceres::CostFunction *cost_function;
          cost_function = ceres_icp_point2line<double>::Create(
              curr_point,
              PclPointToEigen(
                  laser_cloud_corner_map_->points[point_search_idx[0]]),
              PclPointToEigen(
                  laser_cloud_corner_map_->points[point_search_idx[1]]),
              1.0,
              Eigen::Matrix<double, 4, 1>(m_q_w_last_.w(), m_q_w_last_.x(),
                                          m_q_w_last_.y(), m_q_w_last_.z()),
              m_t_w_last_);
          block_id = problem.AddResidualBlock(cost_function, loss_function,
                                              incre_parameters_,
                                              incre_parameters_ + 4);
          residual_block_ids.push_back(block_id);
          corner_num++;
        }
      }
    }

    for (int i = 0; i < laser_intensity_point_num; i++) {
      point_origin = laser_cloud_intensity->points[i];
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_intensity_from_map_->nearestKSearch(
          point_select, 5, point_search_idx, point_search_dis);
      if (point_search_dis[4] < 2.0) {
        bool line_is_avail = true;
        std::vector<Eigen::Vector3d> near_intensitys;
        Eigen::Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; j++) {
          Eigen::Vector3d tmp(
              laser_cloud_intensity_map_->points[point_search_idx[j]].x,
              laser_cloud_intensity_map_->points[point_search_idx[j]].y,
              laser_cloud_intensity_map_->points[point_search_idx[j]].z);
          center = center + tmp;
          near_intensitys.push_back(tmp);
        }
        center = center / 5.0;
        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
        for (int j = 0; j < 5; j++) {
          Eigen::Matrix<double, 3, 1> tmpZeroMean = near_intensitys[j] - center;
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
          line_is_avail = true;
        } else {
          line_is_avail = false;
        }
        Eigen::Vector3d curr_point(point_origin.x, point_origin.y,
                                   point_origin.z);

        if (line_is_avail) {
          ceres::CostFunction *cost_function;
          cost_function = ceres_icp_point2line<double>::Create(
              curr_point,
              PclPointToEigen(
                  laser_cloud_intensity_map_->points[point_search_idx[0]]),
              PclPointToEigen(
                  laser_cloud_intensity_map_->points[point_search_idx[1]]),
              1.0,
              Eigen::Matrix<double, 4, 1>(m_q_w_last_.w(), m_q_w_last_.x(),
                                          m_q_w_last_.y(), m_q_w_last_.z()),
              m_t_w_last_);

          block_id = problem.AddResidualBlock(cost_function, loss_function,
                                              incre_parameters_,
                                              incre_parameters_ + 4);
          residual_block_ids.push_back(block_id);
          intensity_num++;
        }
      }
    }

    for (int i = 0; i < laser_surf_point_num; i++) {
      point_origin = laser_cloud_surf->points[i];
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_surf_from_map_->nearestKSearch(point_select, 5, point_search_idx,
                                            point_search_dis);

      if (point_search_dis[4] < 10.0) {
        bool plane_is_avail = true;
        std::vector<Eigen::Vector3d> near_corners;
        Eigen::Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; j++) {
          Eigen::Vector3d tmp(
              laser_cloud_surf_map_->points[point_search_idx[j]].x,
              laser_cloud_surf_map_->points[point_search_idx[j]].y,
              laser_cloud_surf_map_->points[point_search_idx[j]].z);
          center = center + tmp;
          near_corners.push_back(tmp);
        }

        center = center / (float)(5);
        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();

        for (int j = 0; j < 5; j++) {
          Eigen::Matrix<double, 3, 1> tmpZeroMean = near_corners[j] - center;
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

        if ((saes.eigenvalues()[2] > 3 * saes.eigenvalues()[0]) &&
            (saes.eigenvalues()[2] < 10 * saes.eigenvalues()[1])) {
          plane_is_avail = true;
        } else {
          plane_is_avail = false;
        }

        Eigen::Vector3d curr_point(point_origin.x, point_origin.y,
                                   point_origin.z);
        if (plane_is_avail) {
          ceres::CostFunction *cost_function;
          cost_function = ceres_icp_point2plane<double>::Create(
              curr_point,
              PclPointToEigen(
                  laser_cloud_surf_map_->points[point_search_idx[0]]),
              PclPointToEigen(
                  laser_cloud_surf_map_->points[point_search_idx[5 / 2]]),
              PclPointToEigen(
                  laser_cloud_surf_map_->points[point_search_idx[5 - 1]]),
              1.0,
              Eigen::Matrix<double, 4, 1>(m_q_w_last_.w(), m_q_w_last_.x(),
                                          m_q_w_last_.y(), m_q_w_last_.z()),
              m_t_w_last_);

          block_id = problem.AddResidualBlock(cost_function, loss_function,
                                              incre_parameters_,
                                              incre_parameters_ + 4);
          residual_block_ids.push_back(block_id);

          surf_num++;
        }
      }
    }

    ceres::Solver::Options options;
    std::vector<ceres::ResidualBlockId> residual_block_ids_bak;
    residual_block_ids_bak = residual_block_ids;

      options.linear_solver_type = ceres::DENSE_QR;
      options.logging_type = ceres::SILENT;
      options.minimizer_progress_to_stdout = false;
      options.check_gradients = false;
    options.max_num_iterations = 10;
    set_ceres_solver_bound(problem);
    ceres::Solve(options, &problem, &summary);

    m_t_w_curr_ = m_q_w_last_ * m_t_w_incre_ + m_t_w_last_;
    m_q_w_curr_ = m_q_w_last_ * m_q_w_incre_;

    angular_diff = (float)m_q_w_curr_.angularDistance(m_q_w_last_) * 57.3;
    matcher_cost_ = summary.final_cost;
    avail_num_ = corner_num + intensity_num + 0.7 * surf_num;
  }

  // if(m_t_w_incre_.x() > 0.5 || m_t_w_incre_.y()> 0.5)return 100.0;
  // AWARN << "lidar match socre : " << lidar_match_score_;
  // AWARN << "cost " << matcher_cost_;
  // if (angular_diff > 20.0 || matcher_cost_ > 1.0 || lidar_match_score_ > 0.02) {
  //   for (int i = 0; i < 7; i++) {
  //     parameters_[i] = last_parameters_[i];
  //   }
  //   m_t_w_incre_ = m_t_w_incre_ * 0;
  //   m_q_w_incre_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  //   m_q_w_curr_ = m_q_w_last_;
  //   m_t_w_curr_ = m_t_w_last_;

  //   // std::cout << "error : the distance is far" << std::endl;
  // }

  m_q_w_last_ = m_q_w_curr_;
  m_t_w_last_ = m_t_w_curr_;


    double fitness_score = 0.0;
    std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);
  int nr = 0;
  for (int i = 0; i < laser_corner_point_num; i++) {
    point_origin = laser_cloud_corner->points[i];
    PointAssociateToMap(&point_origin, &point_select);
    kdtree_corner_from_map_->nearestKSearch(point_select, 1, nn_indices,
                                            nn_dists);
        // if (nn_dists[0] <= 20) {
      fitness_score += nn_dists[0];
      nr++;
    // }
  }
    for (int i = 0; i < laser_intensity_point_num; i++) {
    point_origin = laser_cloud_intensity->points[i];
    PointAssociateToMap(&point_origin, &point_select);
    kdtree_intensity_from_map_->nearestKSearch(point_select, 1, nn_indices,
                                            nn_dists);
        if (nn_dists[0] <= 30) {
      fitness_score += nn_dists[0];
      nr++;
    }
  }
    for (int i = 0; i < laser_surf_point_num; i++) {
    point_origin = laser_cloud_surf->points[i];
    PointAssociateToMap(&point_origin, &point_select);
    kdtree_surf_from_map_->nearestKSearch(point_select, 1, nn_indices,
                                            nn_dists);
        if (nn_dists[0] <= 30) {
      fitness_score += nn_dists[0]*0.7;
      nr++;
    }
  }
  // AWARN<<"fitness score : "<<fitness_score;
  // return score;
    if (nr > 100)
    return (fitness_score / nr);
  else
    return 1000.0;


}


double Matcher::GetCloudToMapMacth(
    const CloudData::CLOUD_PTR &laser_cloud_corner,
    const CloudData::CLOUD_PTR &laser_cloud_surf,
    const CloudData::CLOUD_PTR &laser_cloud_intensity,
    const CloudData::CLOUD_PTR &laser_cloud_corner_map,
    const CloudData::CLOUD_PTR &laser_cloud_surf_map,
    const CloudData::CLOUD_PTR &laser_cloud_intensity_map) {
  ResetIncremtalParam();
  int laser_corner_point_num = laser_cloud_corner->points.size();
  int laser_surf_point_num = laser_cloud_surf->points.size();
  int laser_intensity_point_num = laser_cloud_intensity->points.size();

  AWARN << "size corner, surf, intensity : " << laser_corner_point_num << " "
        << laser_surf_point_num << " " << laser_intensity_point_num;

  AWARN << "map size corner, surf, intensity : "
        << laser_cloud_corner_map->points.size() << " "
        << laser_cloud_surf_map->points.size() << " "
        << laser_cloud_intensity_map->points.size();
  ceres::Solver::Summary summary;
  matcher_cost_ = summary.final_cost;
  float angular_diff = 0;
  int surf_num = 0;
  int corner_num = 0;
  int intensity_num = 0;
  CloudData::POINT point_origin;
  CloudData::POINT point_select;
  std::vector<int> point_search_idx;
  std::vector<float> point_search_dis;

  kdtree_corner_from_map_->setInputCloud(laser_cloud_corner_map);
  kdtree_surf_from_map_->setInputCloud(laser_cloud_surf_map);
  kdtree_intensity_from_map_->setInputCloud(laser_cloud_intensity_map);
  AWARN<<"times: "<<icp_max_iterations_;
  for (int iter_count = 0; iter_count < icp_max_iterations_; iter_count++) {
    surf_num = 0;
    corner_num = 0;
    intensity_num = 0;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization =
        new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;
    ceres::ResidualBlockId block_id;
    ceres::Problem problem(problem_options);
    std::vector<ceres::ResidualBlockId> residual_block_ids;

    problem.AddParameterBlock(incre_parameters_, 4, q_parameterization);
    problem.AddParameterBlock(incre_parameters_ + 4, 3);

    for (int i = 0; i < laser_corner_point_num; i++) {
      point_origin = laser_cloud_corner->points[i];
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_corner_from_map_->nearestKSearch(point_select, 5, point_search_idx,
                                              point_search_dis);
      if (point_search_dis[4] < 2.0) {
        bool line_is_avail = true;
        std::vector<Eigen::Vector3d> near_centers;
        Eigen::Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; j++) {
          Eigen::Vector3d tmp(
              laser_cloud_corner_map->points[point_search_idx[j]].x,
              laser_cloud_corner_map->points[point_search_idx[j]].y,
              laser_cloud_corner_map->points[point_search_idx[j]].z);
          center += tmp;
          near_centers.push_back(tmp);
        }
        center = center / 5.0;  //中点的xyz值
        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
        for (int j = 0; j < 5; j++) {
          Eigen::Matrix<double, 3, 1> tmpZeroMean = near_centers[j] - center;
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
          line_is_avail = true;
        } else {
          line_is_avail = false;
        }
        Eigen::Vector3d curr_point(point_origin.x, point_origin.y,
                                   point_origin.z);
        if (line_is_avail) {
          ceres::CostFunction *cost_function;
          cost_function = ceres_icp_point2line<double>::Create(
              curr_point,
              PclPointToEigen(
                  laser_cloud_corner_map->points[point_search_idx[0]]),
              PclPointToEigen(
                  laser_cloud_corner_map->points[point_search_idx[1]]),
              1.0,
              Eigen::Matrix<double, 4, 1>(m_q_w_last_.w(), m_q_w_last_.x(),
                                          m_q_w_last_.y(), m_q_w_last_.z()),
              m_t_w_last_);
          block_id = problem.AddResidualBlock(cost_function, loss_function,
                                              incre_parameters_,
                                              incre_parameters_ + 4);
          residual_block_ids.push_back(block_id);
          corner_num++;
        }
      }
    }

    for (int i = 0; i < laser_intensity_point_num; i++) {
      point_origin = laser_cloud_intensity->points[i];
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_intensity_from_map_->nearestKSearch(
          point_select, 5, point_search_idx, point_search_dis);
      if (point_search_dis[4] < 2.0) {
        bool line_is_avail = true;
        std::vector<Eigen::Vector3d> near_intensitys;
        Eigen::Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; j++) {
          Eigen::Vector3d tmp(
              laser_cloud_intensity_map->points[point_search_idx[j]].x,
              laser_cloud_intensity_map->points[point_search_idx[j]].y,
              laser_cloud_intensity_map->points[point_search_idx[j]].z);
          center = center + tmp;
          near_intensitys.push_back(tmp);
        }
        center = center / 5.0;
        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
        for (int j = 0; j < 5; j++) {
          Eigen::Matrix<double, 3, 1> tmpZeroMean = near_intensitys[j] - center;
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
          line_is_avail = true;
        } else {
          line_is_avail = false;
        }
        Eigen::Vector3d curr_point(point_origin.x, point_origin.y,
                                   point_origin.z);

        if (line_is_avail) {
          ceres::CostFunction *cost_function;
          cost_function = ceres_icp_point2line<double>::Create(
              curr_point,
              PclPointToEigen(
                  laser_cloud_intensity_map->points[point_search_idx[0]]),
              PclPointToEigen(
                  laser_cloud_intensity_map->points[point_search_idx[1]]),
              1.0,
              Eigen::Matrix<double, 4, 1>(m_q_w_last_.w(), m_q_w_last_.x(),
                                          m_q_w_last_.y(), m_q_w_last_.z()),
              m_t_w_last_);

          block_id = problem.AddResidualBlock(cost_function, loss_function,
                                              incre_parameters_,
                                              incre_parameters_ + 4);
          residual_block_ids.push_back(block_id);
          intensity_num++;
        }
      }
    }

    for (int i = 0; i < laser_surf_point_num; i++) {
      point_origin = laser_cloud_surf->points[i];
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_surf_from_map_->nearestKSearch(point_select, 5, point_search_idx,
                                            point_search_dis);

      if (point_search_dis[4] < 10.0) {
        bool plane_is_avail = true;
        std::vector<Eigen::Vector3d> near_corners;
        Eigen::Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; j++) {
          Eigen::Vector3d tmp(
              laser_cloud_surf_map->points[point_search_idx[j]].x,
              laser_cloud_surf_map->points[point_search_idx[j]].y,
              laser_cloud_surf_map->points[point_search_idx[j]].z);
          center = center + tmp;
          near_corners.push_back(tmp);
        }

        center = center / (float)(5);
        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();

        for (int j = 0; j < 5; j++) {
          Eigen::Matrix<double, 3, 1> tmpZeroMean = near_corners[j] - center;
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

        if ((saes.eigenvalues()[2] > 3 * saes.eigenvalues()[0]) &&
            (saes.eigenvalues()[2] < 10 * saes.eigenvalues()[1])) {
          plane_is_avail = true;
        } else {
          plane_is_avail = false;
        }

        Eigen::Vector3d curr_point(point_origin.x, point_origin.y,
                                   point_origin.z);
        if (plane_is_avail) {
          ceres::CostFunction *cost_function;
          cost_function = ceres_icp_point2plane<double>::Create(
              curr_point,
              PclPointToEigen(
                  laser_cloud_surf_map->points[point_search_idx[0]]),
              PclPointToEigen(
                  laser_cloud_surf_map->points[point_search_idx[5 / 2]]),
              PclPointToEigen(
                  laser_cloud_surf_map->points[point_search_idx[5 - 1]]),
              1.0,
              Eigen::Matrix<double, 4, 1>(m_q_w_last_.w(), m_q_w_last_.x(),
                                          m_q_w_last_.y(), m_q_w_last_.z()),
              m_t_w_last_);

          block_id = problem.AddResidualBlock(cost_function, loss_function,
                                              incre_parameters_,
                                              incre_parameters_ + 4);
          residual_block_ids.push_back(block_id);

          surf_num++;
        }
      }
    }

    // AWARN << "corner surf intensity num: " << corner_num << " " << surf_num
    //       << " " << intensity_num;
    // AWARN << "block size: " << residual_block_ids.size();

    ceres::Solver::Options options;
    std::vector<ceres::ResidualBlockId> residual_block_ids_bak;
    residual_block_ids_bak = residual_block_ids;

    for (size_t ii = 0; ii < 1; ii++) {
      options.linear_solver_type = ceres::DENSE_QR;
      // options.max_num_iterations = m_para_cere_max_iterations;
      options.max_num_iterations = 5;
      options.logging_type = ceres::SILENT;
      options.minimizer_progress_to_stdout = false;
      options.check_gradients = false;
      // options.gradient_check_relative_precision = 1e-10;
      // options.function_tolerance = 1e-100; // default 1e-6

      //   if (0) {
      //     // NOTE Optimize T first and than R
      //     if (iterCount < (m_para_icp_max_iterations - 2) / 2)
      //       problem.SetParameterBlockConstant(incre_parameters_ + 4);
      //     else if (iterCount < m_para_icp_max_iterations - 2)
      //       problem.SetParameterBlockConstant(incre_parameters_);
      //   }

      // set_ceres_solver_bound(problem);
      // ceres::Solve(options, &problem, &summary);
      // // Remove outliers
      // residual_block_ids_bak.clear();
      // if (1) {
      //   ceres::Problem::EvaluateOptions eval_options;
      //   eval_options.residual_blocks = residual_block_ids;
      //   double total_cost = 0.0;
      //   double avr_cost;
      //   std::vector<double> residuals;
      //   problem.Evaluate(eval_options, &total_cost, &residuals, nullptr,
      //                    nullptr);
      //   avr_cost = total_cost / residual_block_ids.size();
      //   lidar_match_score_ = avr_cost;
      //   for (unsigned int i = 0; i < residual_block_ids.size(); i++) {
      //     if ((fabs(residuals[3 * i + 0]) + fabs(residuals[3 * i + 1]) +
      //          fabs(residuals[3 * i + 2])) >
      //         std::min(0.1, 10 * avr_cost))  // std::min( 1.0, 10 * avr_cost )
      //     {
      //       problem.RemoveResidualBlock(residual_block_ids[i]);
      //     } else {
      //       residual_block_ids_bak.push_back(residual_block_ids[i]);
      //     }
      //   }
      // }

      // residual_block_ids = residual_block_ids_bak;
    }

    // AWARN << "block after size: " << residual_block_ids.size();
    // options.logging_type = ceres::SILENT;
    // options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 10;
    set_ceres_solver_bound(problem);
    ceres::Solve(options, &problem, &summary);

    // AWARN << "matcher ok";
    m_t_w_curr_ = m_q_w_last_ * m_t_w_incre_ + m_t_w_last_;
    m_q_w_curr_ = m_q_w_last_ * m_q_w_incre_;

    angular_diff = (float)m_q_w_curr_.angularDistance(m_q_w_last_) * 57.3;
    matcher_cost_ = summary.final_cost;
    avail_num_ = corner_num + intensity_num + 0.7 * surf_num;
  }

  // if(m_t_w_incre_.x() > 0.5 || m_t_w_incre_.y()> 0.5)return 100.0;
  // AWARN << "lidar match socre : " << lidar_match_score_;
  // AWARN << "cost " << matcher_cost_;
  // if (angular_diff > 20.0 || matcher_cost_ > 1.0 || lidar_match_score_ > 0.02) {
  //   for (int i = 0; i < 7; i++) {
  //     parameters_[i] = last_parameters_[i];
  //   }
  //   m_t_w_incre_ = m_t_w_incre_ * 0;
  //   m_q_w_incre_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  //   m_q_w_curr_ = m_q_w_last_;
  //   m_t_w_curr_ = m_t_w_last_;

  //   // std::cout << "error : the distance is far" << std::endl;
  // }

  m_q_w_last_ = m_q_w_curr_;
  m_t_w_last_ = m_t_w_curr_;

  // return 0.1;

    double fitness_score = 0.0;
    std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);
  int nr = 0;
  for (int i = 0; i < laser_corner_point_num; i++) {
    point_origin = laser_cloud_corner->points[i];
    PointAssociateToMap(&point_origin, &point_select);
    kdtree_corner_from_map_->nearestKSearch(point_select, 1, nn_indices,
                                            nn_dists);
        // if (nn_dists[0] <= 20) {
      fitness_score += nn_dists[0];
      nr++;
    // }
  }
    for (int i = 0; i < laser_intensity_point_num; i++) {
    point_origin = laser_cloud_intensity->points[i];
    PointAssociateToMap(&point_origin, &point_select);
    kdtree_intensity_from_map_->nearestKSearch(point_select, 1, nn_indices,
                                            nn_dists);
        if (nn_dists[0] <= 30) {
      fitness_score += nn_dists[0];
      nr++;
    }
  }
    for (int i = 0; i < laser_surf_point_num; i++) {
    point_origin = laser_cloud_surf->points[i];
    PointAssociateToMap(&point_origin, &point_select);
    kdtree_surf_from_map_->nearestKSearch(point_select, 1, nn_indices,
                                            nn_dists);
        if (nn_dists[0] <= 30) {
      fitness_score += nn_dists[0]*0.7;
      nr++;
    }
  }
  // AWARN<<"fitness score : "<<fitness_score;
  // return score;
    if (nr > 100)
    return (fitness_score / nr);
  else
    return 1000.0;
}

double Matcher::GetMatcherScore(double max_range,
                                CloudData::CLOUD_PTR &laser_cloud_surf) {
  double fitness_score = 0.0;
  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);
  int nr = 0;
  CloudData::POINT point_select;
  for (size_t i = 0; i < laser_cloud_surf->points.size(); i++) {
    CloudData::POINT point_origin = laser_cloud_surf->points[i];
    PointAssociateToMap(&point_origin, &point_select);
    kdtree_surf_from_map_->nearestKSearch(point_select, 1, nn_indices,
                                          nn_dists);
    if (nn_dists[0] <= max_range) {
      fitness_score += nn_dists[0];
      nr++;
    }
  }
  if (nr > 0)
    return (fitness_score / nr);
  else
    return 1000.0;
}

}  // namespace mapping
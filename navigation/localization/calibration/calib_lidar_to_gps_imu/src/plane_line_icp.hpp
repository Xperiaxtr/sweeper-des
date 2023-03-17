#ifndef PLANE_LINE_ICP_HPP
#define PLANE_LINE_ICP_HPP

#include <ceres/ceres.h>
#include <math.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>

#include "../include/tools/common.h"
#include "../include/tools/logger.hpp"
#include "ceres_icp.hpp"
#include "ros/ros.h"

#define ICP_PLANE 1
#define ICP_LINE 1

#define BLUR_SCALE 1.0

using namespace std;

class Plane_Line_ICP {
 public:
  ros::Publisher o_pub_all_cloud;
  int MOTION_DEBLUR = 0;
  int line_search_num = 5;
  int IF_LINE_FEATURE_CHECK = 1;
  int plane_search_num = 5;
  int IF_PLANE_FEATURE_CHECK = 1;

  int m_max_buffer_size = 50000000;
  int m_para_icp_max_iterations = 15;
  int m_para_cere_max_iterations = 100;
  float m_para_max_angular_rate = 20.0;  // max angular rate = 90.0 /50.0 deg/s
  float m_para_max_speed = 0.50;         // max speed = 10 m/s
  float m_max_final_cost = 1.0;
  // int   m_kmean_filter_count = 3;
  // int   m_kmean_filter_threshold = 2.0;

  float m_minimum_pt_time_stamp = 0;
  float m_maximum_pt_time_stamp = 1.0;

  double m_interpolatation_theta;
  Eigen::Matrix<double, 3, 1> m_interpolatation_omega;
  Eigen::Matrix<double, 3, 3> m_interpolatation_omega_hat;
  Eigen::Matrix<double, 3, 3> m_interpolatation_omega_hat_sq2;

  // map中提取的匹配使用的边沿点
  // surround points in map to build tree
  pcl::PointCloud<PointType>::Ptr m_laser_cloud_corner_from_map;
  pcl::PointCloud<PointType>::Ptr m_laser_cloud_surf_from_map;

  // input & output: points in one frame. local --> global
  pcl::PointCloud<PointType>::Ptr m_laser_cloud_full_res;

  // kd-tree
  pcl::KdTreeFLANN<PointType>::Ptr m_kdtree_corner_from_map;
  pcl::KdTreeFLANN<PointType>::Ptr m_kdtree_surf_from_map;

  double m_para_buffer_RT[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  double m_para_buffer_RT_last[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  double m_para_buffer_incremental[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  const Eigen::Quaterniond m_q_I = Eigen::Quaterniond(1, 0, 0, 0);

  Eigen::Map<Eigen::Quaterniond> m_q_w_curr =
      Eigen::Map<Eigen::Quaterniond>(m_para_buffer_RT);
  Eigen::Map<Eigen::Vector3d> m_t_w_curr =
      Eigen::Map<Eigen::Vector3d>(m_para_buffer_RT + 4);

  Eigen::Map<Eigen::Quaterniond> m_q_w_last =
      Eigen::Map<Eigen::Quaterniond>(m_para_buffer_RT_last);
  Eigen::Map<Eigen::Vector3d> m_t_w_last =
      Eigen::Map<Eigen::Vector3d>(m_para_buffer_RT_last + 4);

  Eigen::Map<Eigen::Quaterniond> m_q_w_incre =
      Eigen::Map<Eigen::Quaterniond>(m_para_buffer_incremental);
  Eigen::Map<Eigen::Vector3d> m_t_w_incre =
      Eigen::Map<Eigen::Vector3d>(m_para_buffer_incremental + 4);

  pcl::VoxelGrid<PointType> m_down_sample_filter_corner;
  pcl::VoxelGrid<PointType> m_down_sample_filter_surface;
  // pcl::VoxelGrid<PointType>                 m_down_sample_filter_surface_map;
  pcl::StatisticalOutlierRemoval<PointType> m_filter_k_means;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;

  std::vector<int> m_point_search_Idx;
  std::vector<float> m_point_search_sq_dis;

  // Eigen::Quaterniond covariance_q = Eigen::Quaterniond( 1, 0, 0, 0 );
  // Eigen::Vector3d covariance_t = Eigen::Vector3d(0.0, 0.0, 0.0);
  double odom_covariance[7] = {0};
  double odom_cov_;

  Plane_Line_ICP() {
    // ros::NodeHandle nh;
    m_down_sample_filter_corner.setLeafSize(0.2, 0.2, 0.2);
    m_down_sample_filter_surface.setLeafSize(1.0, 1.0, 1.0);
    // m_down_sample_filter_surface_map.setLeafSize(0.2,0.2,0.2);
    // o_pub_all_cloud = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    m_laser_cloud_corner_from_map =
        pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    m_laser_cloud_surf_from_map =
        pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    m_laser_cloud_full_res =
        pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

    m_kdtree_corner_from_map =
        pcl::KdTreeFLANN<PointType>::Ptr(new pcl::KdTreeFLANN<PointType>());
    m_kdtree_surf_from_map =
        pcl::KdTreeFLANN<PointType>::Ptr(new pcl::KdTreeFLANN<PointType>());
    laserCloudCornerStack =
        pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    laserCloudSurfStack =
        pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

    cout << "Laser_mapping init OK" << endl;
  };

  ~Plane_Line_ICP(){};

  //对点云中的点进行修正
  void pointAssociateToMap(PointType const *const pi, PointType *const po,
                           double interpolate_s = 1.0, int if_undistore = 0) {
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w;

    if (MOTION_DEBLUR == 0 || if_undistore == 0 || interpolate_s == 1.0) {
      point_w = m_q_w_curr * point_curr + m_t_w_curr;
    } else {
      if (interpolate_s > 1.0 || interpolate_s < 0.0) {
        // printf( "Input interpolate_s = %.5f\r\n", interpolate_s );
        // assert( interpolate_s <= 1.0 && interpolate_s >= 0.0 );
      }

      if (1)  // Using rodrigues for fast compute.
      {
        // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
        Eigen::Vector3d interpolate_T =
            m_t_w_incre * (interpolate_s * BLUR_SCALE);
        double interpolate_R_theta = m_interpolatation_theta * interpolate_s;
        Eigen::Matrix<double, 3, 3> interpolate_R_mat;

        interpolate_R_mat =
            Eigen::Matrix3d::Identity() +
            sin(interpolate_R_theta) * m_interpolatation_omega_hat +
            (1 - cos(interpolate_R_theta)) * m_interpolatation_omega_hat_sq2;
        point_w =
            m_q_w_last * (interpolate_R_mat * point_curr + interpolate_T) +
            m_t_w_last;
      } else {
        Eigen::Quaterniond interpolate_q =
            m_q_I.slerp(interpolate_s * BLUR_SCALE, m_q_w_incre);
        Eigen::Vector3d interpolate_T =
            m_t_w_incre * (interpolate_s * BLUR_SCALE);
        point_w = m_q_w_last * (interpolate_q * point_curr + interpolate_T) +
                  m_t_w_last;
      }
    }

    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    // po->intensity = 1.0;
  }

  //将点云修正转换到地图中
  unsigned int pointcloudAssociateToMap(pcl::PointCloud<PointType> const &pc_in,
                                        pcl::PointCloud<PointType> &pt_out,
                                        int if_undistore = 0) {
    unsigned int points_size = pc_in.points.size();
    pt_out.points.resize(points_size);

    for (unsigned int i = 0; i < points_size; i++) {
      pointAssociateToMap(
          &pc_in.points[i], &pt_out.points[i],
          refine_blur(pc_in.points[i].intensity, m_minimum_pt_time_stamp,
                      m_maximum_pt_time_stamp),
          if_undistore);
    }

    return points_size;
  }

  //将某点转换到地图坐标系中
  void pointAssociateTobeMapped(PointType const *const pi,
                                PointType *const po) {
    Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_curr = m_q_w_curr.inverse() * (point_w - m_t_w_curr);
    po->x = point_curr.x();
    po->y = point_curr.y();
    po->z = point_curr.z();
    po->intensity = pi->intensity;
  }

  //将点云转换到坐标系中
  unsigned int pointcloudAssociateTbeoMapped(
      pcl::PointCloud<PointType> const &pc_in,
      pcl::PointCloud<PointType> &pt_out) {
    unsigned int points_size = pc_in.points.size();
    pt_out.points.resize(points_size);

    for (unsigned int i = 0; i < points_size; i++) {
      pointAssociateTobeMapped(&pc_in.points[i], &pt_out.points[i]);
    }

    return points_size;
  }

  void set_ceres_solver_bound(ceres::Problem &problem) {
    for (unsigned int i = 0; i < 3; i++) {
      problem.SetParameterLowerBound(m_para_buffer_incremental + 4, i,
                                     -m_para_max_speed);
      problem.SetParameterUpperBound(m_para_buffer_incremental + 4, i,
                                     +m_para_max_speed);
    }
  }

  Eigen::Matrix<double, 3, 1> pcl_pt_to_eigend(PointType &pt) {
    return Eigen::Matrix<double, 3, 1>(pt.x, pt.y, pt.z);
  }

  float refine_blur(float in_blur, const float &min_blur,
                    const float &max_blur) {
    return (in_blur - min_blur) / (max_blur - min_blur);
  }

  void reset_incremtal_parameter() {
    m_t_w_curr = m_q_w_last * m_t_w_incre + m_t_w_last;
    m_q_w_curr = m_q_w_last * m_q_w_incre;
    for (size_t i = 0; i < 7; i++) {
      m_para_buffer_incremental[i] = 0;
    }
    m_para_buffer_incremental[3] = 1.0;
    m_t_w_incre = m_t_w_incre * 0;
    m_q_w_incre = Eigen::Map<Eigen::Quaterniond>(m_para_buffer_incremental);

    m_interpolatation_theta = 0;
    m_interpolatation_omega_hat.setZero();
    m_interpolatation_omega_hat_sq2.setZero();
  }

  float compute_fov_angle(const PointType &pt) {
    float sq_xy = sqrt(std::pow(pt.y / pt.x, 2) + std::pow(pt.z / pt.x, 2));
    return atan(sq_xy) * 57.3;
  }

  void GetCornerSurfMap(pcl::PointCloud<PointType> laserCloudMap) {
    for (size_t i = 0; i < laserCloudMap.points.size(); i++) {
      PointType point_now = laserCloudMap.points[i];
      if (point_now.intensity > 0.6)
        m_laser_cloud_corner_from_map->points.push_back(point_now);
      else
        m_laser_cloud_surf_from_map->points.push_back(point_now);
    }
  }
  void ReceivePosition(Eigen::Quaterniond cin_gps_quaterniond,
                       Eigen::Vector3d cin_gps_position) {
    m_t_w_incre = cin_gps_position;
    m_q_w_incre = cin_gps_quaterniond;
    m_q_w_last = m_q_w_incre;
    m_t_w_last = m_t_w_incre;
  }

  void GetPL_ICP(pcl::PointCloud<PointType>::Ptr m_laser_cloud_corner_last,
                 pcl::PointCloud<PointType>::Ptr m_laser_cloud_surf_last) {
    // m_q_w_last = m_q_w_curr;
    // m_t_w_last = m_t_w_curr;
    reset_incremtal_parameter();
    // pcl::PointCloud<PointType>::Ptr laserCloudCornerStack( new
    // pcl::PointCloud<PointType>() );
    m_down_sample_filter_corner.setInputCloud(m_laser_cloud_corner_last);
    m_down_sample_filter_corner.filter(*laserCloudCornerStack);
    int laser_corner_pt_num = laserCloudCornerStack->points.size();

    // pcl::PointCloud<PointType>::Ptr laserCloudSurfStack( new
    // pcl::PointCloud<PointType>() );
    m_down_sample_filter_surface.setInputCloud(m_laser_cloud_surf_last);
    m_down_sample_filter_surface.filter(*laserCloudSurfStack);
    int laser_surface_pt_num = laserCloudSurfStack->points.size();

    printf("map corner num %d  surf num %d \n", laser_corner_pt_num,
           laser_surface_pt_num);

    int surf_avail_num = 0;
    int corner_avail_num = 0;
    ceres::Solver::Summary summary;
    float angular_diff = 0;
    // float                  t_diff = 0;
    float minimize_cost = summary.final_cost;
    PointType pointOri, pointSel;
    int corner_rejection_num = 0;
    int surface_rejecetion_num = 0;
    int if_undistore_in_matching = 0;
    m_kdtree_corner_from_map->setInputCloud(m_laser_cloud_corner_from_map);
    m_kdtree_surf_from_map->setInputCloud(m_laser_cloud_surf_from_map);

    double covariance_xx[4 * 4];
    double covariance_yy[3 * 3];  //协方差矩阵

    for (int iterCount = 0; iterCount < m_para_icp_max_iterations;
         iterCount++) {
      corner_avail_num = 0;  //特征明显的点
      surf_avail_num = 0;
      corner_rejection_num = 0;
      surface_rejecetion_num = 0;

      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *q_parameterization =
          new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options problem_options;
      ceres::ResidualBlockId block_id;
      ceres::Problem problem(problem_options);
      std::vector<ceres::ResidualBlockId> residual_block_ids;

      problem.AddParameterBlock(m_para_buffer_incremental, 4,
                                q_parameterization);
      problem.AddParameterBlock(m_para_buffer_incremental + 4, 3);

      for (int i = 0; i < laser_corner_pt_num; i++) {
        pointOri = laserCloudCornerStack->points[i];
        pointAssociateToMap(
            &pointOri, &pointSel,
            refine_blur(pointOri.intensity, m_minimum_pt_time_stamp,
                        m_maximum_pt_time_stamp),
            if_undistore_in_matching);
        m_kdtree_corner_from_map->nearestKSearch(pointSel, line_search_num,
                                                 m_point_search_Idx,
                                                 m_point_search_sq_dis);
        if (m_point_search_sq_dis[line_search_num - 1] < 2.0) {
          bool line_is_avail = true;
          std::vector<Eigen::Vector3d> nearCorners;
          Eigen::Vector3d center(0, 0, 0);
          if (IF_LINE_FEATURE_CHECK) {
            for (int j = 0; j < line_search_num; j++) {
              Eigen::Vector3d tmp(
                  m_laser_cloud_corner_from_map->points[m_point_search_Idx[j]]
                      .x,
                  m_laser_cloud_corner_from_map->points[m_point_search_Idx[j]]
                      .y,
                  m_laser_cloud_corner_from_map->points[m_point_search_Idx[j]]
                      .z);
              center = center + tmp;
              nearCorners.push_back(tmp);
            }

            center = center / ((float)line_search_num);  //中点的xyz值

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();

            for (int j = 0; j < line_search_num; j++) {
              Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
              covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            // if is indeed line feature
            // note Eigen library sort eigenvalues in increasing order
            //如果确实是线要素
            //注意特征库按递增顺序排序特征值
            //如果最大的特征值大于第二大的特征值三倍以上,就是特征明显的话
            // std::cout<<"corner  saes"<<saes.eigenvalues()[ 0 ]<<"
            // "<<saes.eigenvalues()[ 1 ]<<"  "<<saes.eigenvalues()[ 2
            // ]<<std::endl;
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
              line_is_avail = true;
            } else {
              line_is_avail = false;
            }
          }

          Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);

          if (line_is_avail) {
            if (ICP_LINE) {
              ceres::CostFunction *cost_function;
              if (MOTION_DEBLUR) {
                cost_function = ceres_icp_point2line<double>::Create(
                    curr_point,
                    pcl_pt_to_eigend(m_laser_cloud_corner_from_map
                                         ->points[m_point_search_Idx[0]]),
                    pcl_pt_to_eigend(m_laser_cloud_corner_from_map
                                         ->points[m_point_search_Idx[1]]),
                    refine_blur(pointOri.intensity, m_minimum_pt_time_stamp,
                                m_maximum_pt_time_stamp) *
                        1.0,
                    Eigen::Matrix<double, 4, 1>(m_q_w_last.w(), m_q_w_last.x(),
                                                m_q_w_last.y(), m_q_w_last.z()),
                    m_t_w_last);  // pointOri.intensity );
              } else {
                cost_function = ceres_icp_point2line<double>::Create(
                    curr_point,
                    pcl_pt_to_eigend(m_laser_cloud_corner_from_map
                                         ->points[m_point_search_Idx[0]]),
                    pcl_pt_to_eigend(m_laser_cloud_corner_from_map
                                         ->points[m_point_search_Idx[1]]),
                    1.0,
                    Eigen::Matrix<double, 4, 1>(m_q_w_last.w(), m_q_w_last.x(),
                                                m_q_w_last.y(), m_q_w_last.z()),
                    m_t_w_last);
              }
              block_id = problem.AddResidualBlock(
                  cost_function, loss_function, m_para_buffer_incremental,
                  m_para_buffer_incremental + 4);
              residual_block_ids.push_back(block_id);
            }
            corner_avail_num++;
          } else {
            corner_rejection_num++;
          }
        }
      }
      for (int i = 0; i < laser_surface_pt_num; i++) {
        pointOri = laserCloudSurfStack->points[i];
        int planeValid = true;
        pointAssociateToMap(
            &pointOri, &pointSel,
            refine_blur(pointOri.intensity, m_minimum_pt_time_stamp,
                        m_maximum_pt_time_stamp),
            if_undistore_in_matching);
        m_kdtree_surf_from_map->nearestKSearch(pointSel, plane_search_num,
                                               m_point_search_Idx,
                                               m_point_search_sq_dis);

        if (m_point_search_sq_dis[plane_search_num - 1] < 10.0) {
          std::vector<Eigen::Vector3d> nearCorners;
          Eigen::Vector3d center(0, 0, 0);
          if (IF_PLANE_FEATURE_CHECK) {
            for (int j = 0; j < plane_search_num; j++) {
              Eigen::Vector3d tmp(
                  m_laser_cloud_surf_from_map->points[m_point_search_Idx[j]].x,
                  m_laser_cloud_surf_from_map->points[m_point_search_Idx[j]].y,
                  m_laser_cloud_surf_from_map->points[m_point_search_Idx[j]].z);
              center = center + tmp;
              nearCorners.push_back(tmp);
            }

            center = center / (float)(plane_search_num);

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();

            for (int j = 0; j < plane_search_num; j++) {
              Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
              covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
            // std::cout<<"surf  saes"<<saes.eigenvalues()[ 0 ]<<"
            // "<<saes.eigenvalues()[ 1 ]<<"  "<<saes.eigenvalues()[ 2
            // ]<<std::endl;
            if ((saes.eigenvalues()[2] > 3 * saes.eigenvalues()[0]) &&
                (saes.eigenvalues()[2] < 10 * saes.eigenvalues()[1])) {
              planeValid = true;
            } else {
              planeValid = false;
            }
          }

          Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);

          if (planeValid) {
            if (ICP_PLANE) {
              ceres::CostFunction *cost_function;

              if (MOTION_DEBLUR)  //对点云进行运动补偿
              {
                cost_function = ceres_icp_point2plane<double>::Create(
                    curr_point,
                    pcl_pt_to_eigend(m_laser_cloud_surf_from_map
                                         ->points[m_point_search_Idx[0]]),
                    pcl_pt_to_eigend(
                        m_laser_cloud_surf_from_map
                            ->points[m_point_search_Idx[plane_search_num / 2]]),
                    pcl_pt_to_eigend(
                        m_laser_cloud_surf_from_map
                            ->points[m_point_search_Idx[plane_search_num - 1]]),
                    refine_blur(pointOri.intensity, m_minimum_pt_time_stamp,
                                m_maximum_pt_time_stamp) *
                        BLUR_SCALE,
                    Eigen::Matrix<double, 4, 1>(m_q_w_last.w(), m_q_w_last.x(),
                                                m_q_w_last.y(), m_q_w_last.z()),
                    m_t_w_last);  // pointOri.intensity );
              } else {
                cost_function = ceres_icp_point2plane<double>::Create(
                    curr_point,
                    pcl_pt_to_eigend(m_laser_cloud_surf_from_map
                                         ->points[m_point_search_Idx[0]]),
                    pcl_pt_to_eigend(
                        m_laser_cloud_surf_from_map
                            ->points[m_point_search_Idx[plane_search_num / 2]]),
                    pcl_pt_to_eigend(
                        m_laser_cloud_surf_from_map
                            ->points[m_point_search_Idx[plane_search_num - 1]]),
                    1.0,
                    Eigen::Matrix<double, 4, 1>(m_q_w_last.w(), m_q_w_last.x(),
                                                m_q_w_last.y(), m_q_w_last.z()),
                    m_t_w_last);
              }
              block_id = problem.AddResidualBlock(
                  cost_function, loss_function, m_para_buffer_incremental,
                  m_para_buffer_incremental + 4);
              residual_block_ids.push_back(block_id);
            }
            surf_avail_num++;
          } else {
            surface_rejecetion_num++;
          }
        }
      }

      ceres::Solver::Options options;

      std::vector<ceres::ResidualBlockId> residual_block_ids_bak;
      residual_block_ids_bak = residual_block_ids;
      for (size_t ii = 0; ii < 1; ii++) {
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = m_para_cere_max_iterations;
        options.max_num_iterations = 5;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        // options.gradient_check_relative_precision = 1e-10;
        // options.function_tolerance = 1e-100; // default 1e-6

        if (0) {
          // NOTE Optimize T first and than R
          if (iterCount < (m_para_icp_max_iterations - 2) / 2)
            problem.SetParameterBlockConstant(m_para_buffer_incremental + 4);
          else if (iterCount < m_para_icp_max_iterations - 2)
            problem.SetParameterBlockConstant(m_para_buffer_incremental);
        }

        set_ceres_solver_bound(problem);
        ceres::Solve(options, &problem, &summary);

        // Remove outliers
        residual_block_ids_bak.clear();

        // if ( summary.final_cost > m_max_final_cost * 0.001 )
        if (1) {
          ceres::Problem::EvaluateOptions eval_options;
          eval_options.residual_blocks = residual_block_ids;
          double total_cost = 0.0;
          double avr_cost;
          vector<double> residuals;
          problem.Evaluate(eval_options, &total_cost, &residuals, nullptr,
                           nullptr);
          avr_cost = total_cost / residual_block_ids.size();
          odom_cov_ = avr_cost;

          for (unsigned int i = 0; i < residual_block_ids.size(); i++) {
            if ((fabs(residuals[3 * i + 0]) + fabs(residuals[3 * i + 1]) +
                 fabs(residuals[3 * i + 2])) >
                std::min(0.1, 5 * avr_cost))  // std::min( 1.0, 10 * avr_cost )
            {
              problem.RemoveResidualBlock(residual_block_ids[i]);
            } else {
              residual_block_ids_bak.push_back(residual_block_ids[i]);
            }
          }
        }

        residual_block_ids = residual_block_ids_bak;
      }
      options.max_num_iterations = m_para_cere_max_iterations;
      set_ceres_solver_bound(problem);
      ceres::Solve(options, &problem, &summary);

      ceres::Covariance::Options cov_options;
      ceres::Covariance covariance(cov_options);
      vector<pair<const double *, const double *> > covariance_blocks;
      covariance_blocks.push_back(
          make_pair(m_para_buffer_incremental, m_para_buffer_incremental));
      covariance_blocks.push_back(make_pair(m_para_buffer_incremental + 4,
                                            m_para_buffer_incremental + 4));
      // covariance_blocks.push_back(make_pair(m_para_buffer_incremental,
      // m_para_buffer_incremental +4));

      CHECK(covariance.Compute(covariance_blocks, &problem));

      // double covariance_xx[4 * 4];
      // double covariance_yy[3 * 3];
      // double covariance_xy[4 * 3];
      covariance.GetCovarianceBlock(m_para_buffer_incremental,
                                    m_para_buffer_incremental, covariance_yy);
      covariance.GetCovarianceBlock(m_para_buffer_incremental + 4,
                                    m_para_buffer_incremental + 4,
                                    covariance_yy);
      // for(size_t i = 0; i < 16; i++)
      // {
      //     if(i%4==0)std::cout<<std::endl;
      //     std::cout<<" "<<covariance_xx[i];
      // }

      // for(size_t i = 0; i < 9; i++)
      // {
      //     if(i%3==0)std::cout<<std::endl;
      //     std::cout<<" "<<covariance_yy[i];
      // }

      if (MOTION_DEBLUR) {
        // compute_interpolatation_rodrigue( m_q_w_incre,
        // m_interpolatation_omega, m_interpolatation_theta,
        // m_interpolatation_omega_hat ); m_interpolatation_omega_hat_sq2 =
        // m_interpolatation_omega_hat * m_interpolatation_omega_hat;
      }
      m_t_w_curr = m_q_w_last * m_t_w_incre + m_t_w_last;
      m_q_w_curr = m_q_w_last * m_q_w_incre;
      angular_diff = (float)m_q_w_curr.angularDistance(m_q_w_last) * 57.3;
      // t_diff = ( m_t_w_curr - m_t_w_last ).norm();
      minimize_cost = summary.final_cost;

      // Eigen::Vector3d eulerAngle = m_q_w_incre.matrix().eulerAngles(2,1,0);

      // double delta_t = sqrt(m_t_w_incre.x() * m_t_w_incre.x() +
      // m_t_w_incre.y() * m_t_w_incre.y() + m_t_w_incre.z() * m_t_w_incre.z());
      // double delta_r = sqrt(eulerAngle(0) * eulerAngle(0) + eulerAngle(1) *
      // eulerAngle(1) + eulerAngle(2) * eulerAngle(2)); std::cout<<"delta T
      // "<<delta_t<<" delta R "<<delta_r<<std::endl; std::cout<<"icp num
      // "<<iterCount<<std::endl; if( delta_t < 0.005 && delta_r < 0.0001)
      // break; std::cout<<"icp num "<<m_para_icp_max_iterations<<std::endl;
      std::cout << "avail : " << corner_avail_num << " " << surf_avail_num
                << " " << surface_rejecetion_num << std::endl;
      ROS_INFO("coner_ration , surf_avail_ration, %d, %d, %d", corner_avail_num,
               surf_avail_num, surface_rejecetion_num);
      if (odom_cov_ < 0.0008) break;
    }
    // std::cout<<"t_now  "<<m_t_w_curr<<std::endl;
    // std::cout<<"q_now  "<<m_t_w_curr<<std::endl;
    // std::cout<<"corner_avail_num
    // "<<corner_avail_num/corner_rejection_num<<"surf_avail_num
    // "<<surf_avail_num/surf_rejecetion_num<<std::endl; ROS_INFO("coner_ration
    // , surf_avail_ration, %f, %f",corner_avail_num/(corner_rejection_num
    // +corner_avail_num),
    // surf_avail_num/(surface_rejecetion_num+surf_avail_num));

    if (angular_diff > m_para_max_angular_rate ||
        minimize_cost > m_max_final_cost) {
      for (int i = 0; i < 7; i++) {
        m_para_buffer_RT[i] = m_para_buffer_RT_last[i];
      }

      m_q_w_curr = m_q_w_last;
      m_t_w_curr = m_t_w_last;

      std::cout << "error : the distance is far" << std::endl;
    }
    m_q_w_last = m_q_w_curr;
    m_t_w_last = m_t_w_curr;
    // covariance_q = Eigen::Quaterniond(covariance_xx[0], covariance_xx[5],
    // covariance_xx[10], covariance_xx[15]); covariance_t =
    // Eigen::Vector3d(covariance_yy[0], covariance_yy[4], covariance_yy[8]);
    odom_covariance[0] = covariance_yy[0];
    odom_covariance[1] = covariance_yy[4];
    odom_covariance[2] = covariance_yy[8];
    odom_covariance[3] = covariance_xx[0];
    odom_covariance[4] = covariance_xx[5];
    odom_covariance[5] = covariance_xx[10];
    odom_covariance[6] = covariance_xx[15];
    // for(int i = 0; i < 7; i++)
    // {
    //     std::cout<<"cov num "<<odom_covariance[i]<<std::endl;
    // }
  }
};

#endif  // LASER_MAPPING_HPP

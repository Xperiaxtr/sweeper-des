#include "se3_matcher.hpp"

SE3Matcher::SE3Matcher(int icp_max_iterations)
    : map_edge_in_(new pcl::PointCloud<pcl::PointXYZI>()),
      map_surf_in_(new pcl::PointCloud<pcl::PointXYZI>()),
      map_intensity_in_(new pcl::PointCloud<pcl::PointXYZI>()) {
  kdtree_corner_from_map_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_surf_from_map_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_intensity_from_map_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());
  //   m_t_w_incre_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  //   m_q_w_curr_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  icp_max_iterations_ = icp_max_iterations;
}

double SE3Matcher::GetCloudToMapMacth(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &intensity_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_edge_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_surf_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_intensity_in) {
  if (map_edge_in->points.size() > 10 && map_surf_in->points.size() > 50) {
    kdtree_corner_from_map_->setInputCloud(map_edge_in);
    kdtree_surf_from_map_->setInputCloud(map_surf_in);
    kdtree_intensity_from_map_->setInputCloud(map_intensity_in);

    // ceres优化，附近的多帧点云构建优化问题
    for (int iterCount = 0; iterCount < icp_max_iterations_; iterCount++) {
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);  // 鲁棒核
      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options);  // 优化问题

      // 残差块
      problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());

      // 对两种特征点云分别构建点到平面、点到边缘的残差项
      AddEdgeCostFactor(edge_in, map_edge_in, problem, loss_function);
      AddIntensityCostFactor(intensity_in, map_intensity_in, problem,
                             loss_function);
      AddSurfCostFactor(surf_in, map_surf_in, problem, loss_function);

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 4;
      options.minimizer_progress_to_stdout = false;
      options.check_gradients = false;
      options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;

      // 求解问题, 优化完成后的结果更新在 q_w_curr、t_w_curr
      ceres::Solve(options, &problem, &summary);
    }
  } else {
    printf("not enough points in map to associate, map error");
  }
  return OutputMatcherScore(edge_in, surf_in, intensity_in, map_edge_in,
                            map_surf_in, map_intensity_in);
}

double SE3Matcher::CloudToMapMacth(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &intensity_in) {
  // ceres优化，附近的多帧点云构建优化问题
  for (int iterCount = 0; iterCount < icp_max_iterations_; iterCount++) {
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);  // 鲁棒核
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);  // 优化问题

    // 残差块
    problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());

    // 对两种特征点云分别构建点到平面、点到边缘的残差项
    AddEdgeCostFactor(edge_in, map_edge_in_, problem, loss_function);
    AddIntensityCostFactor(intensity_in, map_intensity_in_, problem,
                           loss_function);
    AddSurfCostFactor(surf_in, map_surf_in_, problem, loss_function);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 4;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    ceres::Solver::Summary summary;

    // 求解问题, 优化完成后的结果更新在 q_w_curr、t_w_curr
    ceres::Solve(options, &problem, &summary);
  }
  return OutputMatcherScore(edge_in, surf_in, intensity_in, map_edge_in_,
                            map_surf_in_, map_intensity_in_);
}

void SE3Matcher::CopyMapToMacther(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_intensity_map) {
  *map_edge_in_ = *laser_cloud_corner_map;
  *map_surf_in_ = *laser_cloud_surf_map;
  *map_intensity_in_ = *laser_cloud_intensity_map;
  kdtree_corner_from_map_->setInputCloud(map_edge_in_);
  kdtree_surf_from_map_->setInputCloud(map_surf_in_);
  kdtree_intensity_from_map_->setInputCloud(map_intensity_in_);
}

double SE3Matcher::OutputMatcherScore(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &intensity_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_edge_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_surf_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_intensity_in) {
  double fitness_score = 0.0;
  int nr = 0;
  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);
  pcl::PointXYZI point_origin, point_select;
  for (size_t i = 0; i < edge_in->points.size(); i++) {
    point_origin = edge_in->points[i];
    PointAssociateToMap(&point_origin, &point_select);
    kdtree_corner_from_map_->nearestKSearch(point_select, 1, nn_indices,
                                            nn_dists);
    if (nn_dists[0] <= 30.0) {
      fitness_score += nn_dists[0];
      nr++;
    }
  }
  for (size_t i = 0; i < intensity_in->points.size(); i++) {
    point_origin = intensity_in->points[i];
    PointAssociateToMap(&point_origin, &point_select);
    kdtree_intensity_from_map_->nearestKSearch(point_select, 1, nn_indices,
                                               nn_dists);
    if (nn_dists[0] <= 30.0) {
      fitness_score += nn_dists[0];
      nr++;
    }
  }
  for (size_t i = 0; i < surf_in->points.size(); i++) {
    point_origin = surf_in->points[i];
    PointAssociateToMap(&point_origin, &point_select);
    kdtree_surf_from_map_->nearestKSearch(point_select, 1, nn_indices,
                                          nn_dists);
    if (nn_dists[0] <= 30.0) {
      fitness_score += nn_dists[0];
      nr++;
    }
  }

  if (nr > 100)
    return (fitness_score / (double)nr);
  else
    return 1000.0;
}

void SE3Matcher::PointAssociateToMap(pcl::PointXYZI const *const pi,
                                     pcl::PointXYZI *const po) {
  // 点转换到地图坐标系
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = m_q_w_curr_ * point_curr + m_t_w_curr_;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
  po->intensity = pi->intensity;
}

void SE3Matcher::AddEdgeCostFactor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in, ceres::Problem &problem,
    ceres::LossFunction *loss_function) {
  int corner_num = 0;
  for (int i = 0; i < (int)pc_in->points.size(); i++) {
    pcl::PointXYZI point_temp;
    PointAssociateToMap(&(pc_in->points[i]),
                        &point_temp);  // 将当前点转换到地图坐标系

    // 寻找最近邻的5个点
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtree_corner_from_map_->nearestKSearch(point_temp, 5, pointSearchInd,
                                            pointSearchSqDis);
    if (pointSearchSqDis[4] < 1.0) {
      std::vector<Eigen::Vector3d> nearCorners;
      // 计算均值
      Eigen::Vector3d center(0, 0, 0);
      for (int j = 0; j < 5; j++) {
        Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                            map_in->points[pointSearchInd[j]].y,
                            map_in->points[pointSearchInd[j]].z);
        center = center + tmp;
        nearCorners.push_back(tmp);
      }
      center = center / 5.0;
      // 计算方差
      Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
      for (int j = 0; j < 5; j++) {
        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
      }

      //  主成分分析，获取直线的参数
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
      Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y,
                                 pc_in->points[i].z);
      if (saes.eigenvalues()[2] >
          3 * saes.eigenvalues()
                  [1]) {  // 如果最大的特征向量，明显比第二大的大，则认为是直线
        Eigen::Vector3d point_on_line = center;
        Eigen::Vector3d point_a, point_b;
        // 取直线的两点，中点+-0.1×(直线方向单位向量)
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;

        // 用点O，A，B构造点到线的距离的残差项，注意这三个点都是在上一帧的Lidar坐标系下，即，残差
        // = 点O到直线AB的距离
        ceres::CostFunction *cost_function =
            new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
        // 添加边缘点关联构建的残差项
        problem.AddResidualBlock(cost_function, loss_function, parameters);
        corner_num++;
      }
    }
  }
  if (corner_num < 20) {
    printf("not enough correct points");
  }
}

void SE3Matcher::AddIntensityCostFactor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in, ceres::Problem &problem,
    ceres::LossFunction *loss_function) {
  int corner_num = 0;
  for (int i = 0; i < (int)pc_in->points.size(); i++) {
    pcl::PointXYZI point_temp;
    PointAssociateToMap(&(pc_in->points[i]),
                        &point_temp);  // 将当前点转换到地图坐标系

    // 寻找最近邻的5个点
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtree_intensity_from_map_->nearestKSearch(point_temp, 5, pointSearchInd,
                                               pointSearchSqDis);
    if (pointSearchSqDis[4] < 1.0) {
      std::vector<Eigen::Vector3d> nearCorners;
      // 计算均值
      Eigen::Vector3d center(0, 0, 0);
      for (int j = 0; j < 5; j++) {
        Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                            map_in->points[pointSearchInd[j]].y,
                            map_in->points[pointSearchInd[j]].z);
        center = center + tmp;
        nearCorners.push_back(tmp);
      }
      center = center / 5.0;
      // 计算方差
      Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
      for (int j = 0; j < 5; j++) {
        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
      }

      //  主成分分析，获取直线的参数
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
      Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y,
                                 pc_in->points[i].z);
      if (saes.eigenvalues()[2] >
          3 * saes.eigenvalues()
                  [1]) {  // 如果最大的特征向量，明显比第二大的大，则认为是直线
        Eigen::Vector3d point_on_line = center;
        Eigen::Vector3d point_a, point_b;
        // 取直线的两点，中点+-0.1×(直线方向单位向量)
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;

        // 用点O，A，B构造点到线的距离的残差项，注意这三个点都是在上一帧的Lidar坐标系下，即，残差
        // = 点O到直线AB的距离
        ceres::CostFunction *cost_function =
            new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
        // 添加边缘点关联构建的残差项
        problem.AddResidualBlock(cost_function, loss_function, parameters);
        corner_num++;
      }
    }
  }
  if (corner_num < 20) {
    printf("not enough correct points");
  }
}

void SE3Matcher::AddSurfCostFactor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in, ceres::Problem &problem,
    ceres::LossFunction *loss_function) {
  int surf_num = 0;
  for (int i = 0; i < (int)pc_in->points.size(); i++) {
    pcl::PointXYZI point_temp;
    PointAssociateToMap(&(pc_in->points[i]), &point_temp);
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtree_surf_from_map_->nearestKSearch(point_temp, 5, pointSearchInd,
                                          pointSearchSqDis);

    // 与上面的建立corner特征点之间的关联类似，寻找平面特征点O的最近邻点ABC，
    // 即基于最近邻原理建立surf特征点之间的关联，find correspondence for plane
    // features 寻找五个紧邻点
    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 =
        -1 * Eigen::Matrix<double, 5, 1>::Ones();
    if (pointSearchSqDis[4] < 1.0) {
      // 5*3的矩阵
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
        matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
        matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
      }
      // 计算平面的法向量
      // 平面方程Ax+By+Cz+D = 0 =>  (x,y,z)(A/D,B/D,C/D)^T = -1  => 求解  Ax = b
      // ,x即为法向量
      Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
      double negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();

      // 判断平面是否有效
      // Here n(pa, pb, pc) is unit norm of plane   X^T*n = -1 =>  X^T*n/|n| +
      // 1/|n| = 0  如果结果>0.2则认为有误
      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        // if OX * n > 0.2, then plane is not fit well
        if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                 norm(1) * map_in->points[pointSearchInd[j]].y +
                 norm(2) * map_in->points[pointSearchInd[j]].z +
                 negative_OA_dot_norm) > 0.2) {
          planeValid = false;
          break;
        }
      }
      Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y,
                                 pc_in->points[i].z);
      if (planeValid) {
        // 有效的话t添加点到平面的残差项
        // 用点O，A，B，C构造点到面的距离的残差项，注意这三个点都是在上一帧的Lidar坐标系下，即，残差
        // = 点O到平面ABC的距离
        ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(
            curr_point, norm, negative_OA_dot_norm);
        problem.AddResidualBlock(cost_function, loss_function, parameters);

        surf_num++;
      }
    }
  }
  if (surf_num < 20) {
    printf("not enough correct points");
  }
}

void SE3Matcher::PredictMatcherPose(const Eigen::Quaterniond &predict_quater,
                                    const Eigen::Vector3d &predict_position) {
  m_t_w_curr_ = predict_position;
  m_q_w_curr_ = predict_quater;
}

SE3Matcher::~SE3Matcher() {}
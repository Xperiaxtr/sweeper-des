//后端优化
/*
//流程：
1.输入位姿与gps等数据
2.添加到g2o中，符合要求就进行优化
*/
#ifndef MAPPING_MAPPING_BACK_END_BACK_END_HPP_
#define MAPPING_MAPPING_BACK_END_BACK_END_HPP_
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <mutex>

#include "sensor_data/cloud_data.hpp"
#include "sensor_data/feature.hpp"
#include "sensor_data/key_frame.hpp"
#include "sensor_data/odom_data.hpp"
#include "tools/g2o/edge_se3_priorxyz.hpp"
#include "tools/math_calculation.hpp"

namespace mapping {
class BackEnd {
 public:
  g2o::SparseOptimizer optimizer;
  BackEnd();
  void InitOptimizeParam();
  void AddVertexToG2O(const Eigen::Matrix4d &lidar_odom);
  void AddLidarOdomToG2O(Eigen::Matrix4d lidar_odom_last,
                         Eigen::Matrix4d lidar_odom_now, double matcher_score);
  void AddGNSSToG2O(const Eigen::Vector3d &gps_pose, const int &index,
                    const Eigen::Vector3d &gnss_cov);
  void AddImuToG2O(Eigen::Matrix4d imu_odom_last, Eigen::Matrix4d imu_odom_now,
                   int index_last, int index_now);
  void AddLoopToG2O(const Eigen::Isometry3d &loop_odom, const int &index_last,
                    const int &index_now, const double &matcher_score);
  void AddDRToG2O(const OdomData &last_odom_data, const OdomData &now_odom_data,
                  const int &last_index, const int &now_index);
  bool BackEndOptimize(bool start_optimize);
  Eigen::Matrix4d OutputOptimizedPose(std::vector<KeyFrame> &key_frames);
  // void SetLidarOdomInformation(double lidar_odom_cov);
  bool SetGNSSInformation(Eigen::Matrix<double, 6, 6> pos_cov);
  void ResetBackEndParam();

 private:
  int optimize_num_;
  int optimize_step_gps_;
  int optimize_step_imu_;
  int optimize_step_loop_;
  int optimize_step_odom_;
  // Eigen::Matrix<double, 6, 6> lidar_odom_information_;
  // Eigen::Matrix<double, 3, 3> gnss_information_;
  std::mutex back_mutex_;
  bool flag_frist_gnss_;
  // Eigen::Vector3d last_gps_pos_;
};
}  // namespace mapping

#endif
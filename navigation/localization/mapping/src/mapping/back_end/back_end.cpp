#include "mapping/back_end/back_end.hpp"

namespace mapping {

BackEnd::BackEnd() {
  InitOptimizeParam();
  optimize_num_ = 0;
  optimize_step_odom_ = 0;
  optimize_step_gps_ = 0;
  optimize_step_imu_ = 0;
  optimize_step_loop_ = 0;
  // last_gps_pos_ = Eigen::Vector3d(0, 0, 0);
  optimizer.clear();
  flag_frist_gnss_ = true;
}

void BackEnd::InitOptimizeParam() {
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
  std::unique_ptr<Block::LinearSolverType> linearSolver(
      new g2o::LinearSolverCholmod<Block::PoseMatrixType>());
  std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);
}

void BackEnd::AddLidarOdomToG2O(Eigen::Matrix4d lidar_odom_last,
                                Eigen::Matrix4d lidar_odom_now,
                                double matcher_score) {
  Eigen::Matrix4d last_to_now_tran = lidar_odom_last.inverse() * lidar_odom_now;

  AWARN << "now_in_last: " << last_to_now_tran;

  int index = optimizer.vertices().size();
  AWARN << "index: " << index;
  if (index <= 1) return;
  g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
  edge->vertices()[0] = optimizer.vertex(index - 2);
  edge->vertices()[1] = optimizer.vertex(index - 1);

  Eigen::Matrix<double, 6, 6> information =
      Eigen::Matrix<double, 6, 6>::Identity();
  information = MathCalculation::CalInformationMatrix(matcher_score);
  // information(0, 0) = information(1, 1) = information(2, 2) =
  //     1.0 / matcher_score;
  // information(3, 3) = information(4, 4) = information(5, 5) =
  //     1.0 / matcher_score;

  AWARN << "matcher: " << information(0, 0);
  edge->setInformation(information);
  Eigen::Isometry3d last_to_now =
      MathCalculation::Matrix4dToIsometry3d(last_to_now_tran);
  edge->setMeasurement(last_to_now);
  edge->setRobustKernel(new g2o::RobustKernelHuber());
  optimizer.addEdge(edge);
  optimize_step_odom_++;
  // AWARN << "odom edge information : " << information(0, 0);
}

void BackEnd::AddVertexToG2O(const Eigen::Matrix4d &lidar_odom) {
  g2o::VertexSE3 *v = new g2o::VertexSE3();
  Eigen::Isometry3d odom = MathCalculation::Matrix4dToIsometry3d(lidar_odom);
  int index = optimizer.vertices().size();
  AWARN << "frist size: " << index;
  v->setId(index);
  v->setEstimate(odom);
  if (index < 1) v->setFixed(true);
  optimizer.addVertex(v);
}

void BackEnd::AddDRToG2O(const OdomData &last_odom_data,
                         const OdomData &now_odom_data, const int &last_index,
                         const int &now_index) {
  g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
  edge->vertices()[0] = optimizer.vertex(last_index);
  edge->vertices()[1] = optimizer.vertex(now_index);

  //计算变换矩阵
  Eigen::Isometry3d odom_last = Eigen::Isometry3d::Identity();
  odom_last.rotate(last_odom_data.quater.matrix());
  odom_last.pretranslate(last_odom_data.pos);
  Eigen::Isometry3d odom_now = Eigen::Isometry3d::Identity();
  odom_last.rotate(now_odom_data.quater.matrix());
  odom_last.pretranslate(now_odom_data.pos);

  Eigen::Matrix<double, 6, 6> information =
      Eigen::Matrix<double, 6, 6>::Identity();
  information(0, 0) = information(1, 1) = information(2, 2) = 100000;
  information(3, 3) = information(4, 4) = information(5, 5) = 100000;
  edge->setInformation(information);
  Eigen::Isometry3d last_to_now = odom_last.inverse() * odom_now;
  edge->setMeasurement(last_to_now);
  edge->setRobustKernel(new g2o::RobustKernelHuber());
  optimizer.addEdge(edge);
}

void BackEnd::AddGNSSToG2O(const Eigen::Vector3d &gps_pose, const int &index,
                           const Eigen::Vector3d &gnss_cov) {
  Eigen::Matrix3d gnss_information = Eigen::Matrix3d::Identity();
  gnss_information(0, 0) = 1.0 / gnss_cov.x();  // * gnss_cov.x());
  gnss_information(1, 1) = 1.0 / gnss_cov.y();  // * gnss_cov.y());
  gnss_information(2, 2) = 1.0 / gnss_cov.z();  // * gnss_cov.z());
  g2o::EdgeSE3PriorXYZ *edge(new g2o::EdgeSE3PriorXYZ());
  edge->vertices()[0] = optimizer.vertex(index);
  edge->setInformation(gnss_information);
  edge->setMeasurement(gps_pose);

  AWARN << " gnss information: " << gnss_information;
  AWARN << "index : " << index << "   vertex: " << optimizer.vertices().size();

  //锁住第一个点，放弃锁住第一个点
  if (flag_frist_gnss_) {
    optimizer.vertex(0)->setFixed(false);
    optimizer.vertex(index)->setFixed(true);
    flag_frist_gnss_ = false;
    // edge->setRobustKernel(new g2o::RobustKernelHuber());
  } else
    edge->setRobustKernel(new g2o::RobustKernelHuber());
  AWARN << "before gps edge size : " << optimizer.edges().size();
  optimizer.addEdge(edge);
  AWARN << "after gps edge size : " << optimizer.edges().size();
  optimize_step_gps_++;
  AWARN << " idx is : " << index << " add gnss edge : " << gps_pose;
}

// void BackEnd::AddImuToG2O(Eigen::Matrix4d imu_odom, int index_0, int index_1)
// {}
void BackEnd::AddLoopToG2O(const Eigen::Isometry3d &loop_odom,
                           const int &index_last, const int &index_now,
                           const double &matcher_score) {
  AWARN << " index_last: " << index_last << " index_now: " << index_now
        << " size: " << optimizer.vertices().size();
  g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
  edge->vertices()[0] = optimizer.vertex(index_last);
  edge->vertices()[1] = optimizer.vertex(index_now);
  Eigen::Matrix<double, 6, 6> information =
      Eigen::Matrix<double, 6, 6>::Identity();
  // information(0, 0) = information(1, 1) = information(2, 2) =
  //     1.0 / matcher_score;
  // information(3, 3) = information(4, 4) = information(5, 5) =
  //     1.0 / matcher_score;
  information = MathCalculation::CalInformationMatrix(matcher_score);
  edge->setInformation(information);
  edge->setMeasurement(loop_odom);
  edge->setRobustKernel(new g2o::RobustKernelHuber());
  AWARN << "loop edge information : " << information(0, 0);
  optimizer.addEdge(edge);
  optimize_step_loop_++;
}

bool BackEnd::BackEndOptimize(bool start_optimize) {
  // if (/* optimize_step_odom_ > 20 || */ optimize_step_gps_ >= 1 ||
  //     optimize_step_loop_ >= 1 || optimize_step_imu_ > 10 || start_optimize)
  //     {
  back_mutex_.lock();

  std::cout << std::endl;
  std::cout << "--- pose graph optimization ---" << std::endl;
  std::cout << "nodes: " << optimizer.vertices().size()
            << "   edges: " << optimizer.edges().size() << std::endl;
  std::cout << "optimizing... " << std::flush;

  // optimizer
  //     .ComputeE

  optimizer.initializeOptimization();
  optimizer.setVerbose(false);

  std::cout << "chi2" << std::endl;
  double chi2 = optimizer.chi2();

  auto t1 = ros::WallTime::now();
  int optimizer_time = optimizer.optimize(30);  //可以指定优化步数
  // optimizer.save("../result_after.g2o");
  auto t2 = ros::WallTime::now();
  optimize_num_ = optimizer.vertices().size();
  back_mutex_.unlock();

  optimize_step_odom_ = 0;
  optimize_step_gps_ = 0;
  optimize_step_imu_ = 0;
  optimize_step_loop_ = 0;

  std::cout << "done" << std::endl;
  std::cout << "iterations: " << optimizer_time << " / " << 30 << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << optimizer.chi2()
            << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]"
            << std::endl;

  AWARN << "Optimize sucess! save g2o!";
  return true;
  // } else {
  //   return false;
  //   AWARN << "Optimize faile! g2o edge is not enough!";
  // }
}

void BackEnd::ResetBackEndParam() {
  optimizer.clear();
  flag_frist_gnss_ = true;
}

// void BackEnd::SetLidarOdomInformation(double lidar_odom_cov) {
//   lidar_odom_information_ = Eigen::Matrix<double, 6, 6>::Identity();
//   // double lidar_odom_information = CovToInformation(lidar_odom_cov);
//   double lidar_odom_information = lidar_odom_cov;
//   lidar_odom_information_(0, 0) =
//       1.0 / lidar_odom_information * lidar_odom_information;
//   lidar_odom_information_(1, 1) =
//       1.0 / lidar_odom_information * lidar_odom_information;
//   lidar_odom_information_(2, 2) =
//       1.0 / lidar_odom_information * lidar_odom_information;
//   lidar_odom_information_(3, 3) =
//       1.0 / lidar_odom_information * lidar_odom_information;
//   lidar_odom_information_(4, 4) =
//       1.0 / lidar_odom_information * lidar_odom_information;
//   lidar_odom_information_(5, 5) =
//       1.0 / lidar_odom_information * lidar_odom_information;
// }

// bool BackEnd::SetGNSSInformation(Eigen::Matrix<double, 6, 6> pos_cov) {
//   if (pos_cov(0, 0) > 0.05 || pos_cov(5, 5) > 0.5) return false;
//   gnss_information_ = Eigen::Matrix<double, 3, 3>::Identity();
//   pos_cov(0, 0) = pos_cov(1, 1) = pos_cov(2, 2) = 0.2;
//   gnss_information_(0, 0) = 5.0 / (pos_cov(0, 0));
//   gnss_information_(1, 1) = 5.0 / (pos_cov(1, 1));
//   gnss_information_(2, 2) = 2.0 / (pos_cov(2, 2));
//   AWARN << "gnss edge information : " << gnss_information_(0, 0) << " "
//         << gnss_information_(1, 1) << " " << gnss_information_(2, 2);
//   return true;
// }

Eigen::Matrix4d BackEnd::OutputOptimizedPose(
    std::vector<KeyFrame> &key_frames) {
  // if (optimize_num_ < 20) return;
  AWARN << "optimize_num_ : " << optimize_num_;
  if (optimize_num_ == 0) return Eigen::Matrix4d::Identity();
  back_mutex_.lock();
  for (int i = 0; i < optimize_num_; i++) {
    g2o::VertexSE3 *vertex =
        dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
    Eigen::Isometry3d pose = vertex->estimate();  //该帧优化后的位姿
    // for (size_t j = 0; j < 4; j++) {
    //   for (size_t k = 0; k < 4; k++) {
    //     // std::cout << pose(j, k) << " ";
    //   }
    //   std::cout << std::endl;
    // }
    Eigen::Matrix4d matrix_pose = MathCalculation::Isometry3dToMatrix4d(pose);
    // key_frames[i].AddPoseToKeyFrame(matrix_pose);
    key_frames[i].pose = matrix_pose.cast<float>();
    key_frames[i].optimized = true;
  }
  back_mutex_.unlock();
  std::cout << "out optimize end" << std::endl;

  AWARN << "key_frame size : " << key_frames.size()
        << "   optimize_num_: " << optimize_num_;
  return key_frames[optimize_num_ - 1].pose.cast<double>();
}

}  // namespace mapping
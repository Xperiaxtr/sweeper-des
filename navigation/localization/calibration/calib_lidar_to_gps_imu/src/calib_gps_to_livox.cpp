
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <utility>

#include "tf/transform_datatypes.h"

using namespace std;

struct CalibData {
  Eigen::Matrix4d lidar_pose;
  Eigen::Matrix4d gnss_pose;
};

struct GNSSData {
  Eigen::Matrix4d gnss_pose;
  double gnss_time;
};

void ReceiveGNSSData(const nav_msgs::OdometryConstPtr &receive_data);
void ReceiveLidarData(const nav_msgs::OdometryConstPtr &receive_data);
void Calib();
Eigen::Quaterniond SolveRotation(
    const std::vector<std::pair<Eigen::Quaterniond, Eigen::Quaterniond>>
        &corres);
Eigen::Vector3d SloveTransform(
    const std::vector<std::pair<Eigen::Matrix4d, Eigen::Matrix4d>> &corres,
    const Eigen::Quaterniond &q_l_g);
Eigen::MatrixXd EigenSVDInverse(Eigen::MatrixXd &origin, float er);

std::deque<GNSSData> gnss_deque_data_;
std::vector<CalibData> calib_vector_data_;
ros::Publisher pub_calib_param_;

void ReceiveGNSSData(const nav_msgs::OdometryConstPtr &receive_data) {
  double time = receive_data->header.stamp.toSec();
  Eigen::Quaterniond gnss_quater(receive_data->pose.pose.orientation.w,
                                 receive_data->pose.pose.orientation.x,
                                 receive_data->pose.pose.orientation.y,
                                 receive_data->pose.pose.orientation.z);
  Eigen::Vector3d gnss_pos(receive_data->pose.pose.position.x,
                           receive_data->pose.pose.position.y,
                           receive_data->pose.pose.position.z);
  if (receive_data->pose.covariance[0] > 0.2 ||
      receive_data->pose.covariance[7] > 0.2 ||
      receive_data->pose.covariance[14] > 0.2 ||
      receive_data->pose.covariance[21] > 0.2 ||
      receive_data->pose.covariance[28] > 0.2 ||
      receive_data->pose.covariance[35] > 0.2)
    return;
  Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();
  gnss_pose.block(0, 0, 3, 3) = gnss_quater.matrix();
  gnss_pose.block(0, 3, 3, 1) = gnss_pos;

  GNSSData now_gnss_data;
  now_gnss_data.gnss_pose = gnss_pose;
  now_gnss_data.gnss_time = time;

  gnss_deque_data_.push_back(now_gnss_data);
}

void ReceiveLidarData(const nav_msgs::OdometryConstPtr &receive_data) {
  double time = receive_data->header.stamp.toSec();
  Eigen::Quaterniond lidar_quater(receive_data->pose.pose.orientation.w,
                                  receive_data->pose.pose.orientation.x,
                                  receive_data->pose.pose.orientation.y,
                                  receive_data->pose.pose.orientation.z);
  Eigen::Vector3d lidar_pos(receive_data->pose.pose.position.x,
                            receive_data->pose.pose.position.y,
                            receive_data->pose.pose.position.z);
  Eigen::Matrix4d lidar_pose = Eigen::Matrix4d::Identity();
  lidar_pose.block(0, 0, 3, 3) = lidar_quater.matrix();
  lidar_pose.block(0, 3, 3, 1) = lidar_pos;

  Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();
  double min_deta_time = 0.1;
  int min_deta_time_index = 0;
  for (size_t i = 0; i < gnss_deque_data_.size(); i++) {
    if (fabs(gnss_deque_data_[i].gnss_time - time) < min_deta_time) {
      min_deta_time = fabs(gnss_deque_data_[i].gnss_time - time);
      gnss_pose = gnss_deque_data_[i].gnss_pose;
      min_deta_time_index = i;
    }
  }
  gnss_deque_data_.erase(gnss_deque_data_.begin(),
                         gnss_deque_data_.begin() + min_deta_time_index);
  if (min_deta_time > 0.08) return;

  CalibData calib_data;
  calib_data.lidar_pose = lidar_pose;
  calib_data.gnss_pose = gnss_pose;

  calib_vector_data_.push_back(calib_data);

  std::cout << " " << calib_vector_data_.size() << std::endl;
  Calib();
}

void Calib() {
  if (calib_vector_data_.size() < 10) return;

  std::vector<std::pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres(
      0);  // lidar, gnss
  std::vector<std::pair<Eigen::Matrix4d, Eigen::Matrix4d>> matrix_corres(0);
  //计算相对匹配变化参数

  Eigen::Matrix4d frist_lidar_pose = calib_vector_data_[0].lidar_pose;
  Eigen::Matrix4d frist_gnss_pose = calib_vector_data_[0].gnss_pose;
  for (int i = 10; i < calib_vector_data_.size(); i++) {
    Eigen::Matrix4d lidar_pose = calib_vector_data_[i].lidar_pose;
    Eigen::Matrix4d gnss_pose = calib_vector_data_[i].gnss_pose;

    Eigen::Matrix4d tran_lidar_pose = frist_lidar_pose.inverse() * lidar_pose;
    Eigen::Matrix4d tran_gnss_pose = frist_gnss_pose.inverse() * gnss_pose;

    Eigen::Matrix3d rotation_lidar_tran = tran_lidar_pose.block(0, 0, 3, 3);
    Eigen::Matrix3d rotation_gnss_tran = tran_gnss_pose.block(0, 0, 3, 3);
    Eigen::Quaterniond lidar_tran_quater(rotation_lidar_tran),
        gnss_tran_quater(rotation_gnss_tran);
    // lidar_tran_quater = tran_lidar_pose.block(0, 0, 3, 3);
    // gnss_tran_quater = tran_gnss_pose.block(0, 0, 3, 3);

    corres.push_back(move(std::pair<Eigen::Quaterniond, Eigen::Quaterniond>(
        lidar_tran_quater, gnss_tran_quater)));
    matrix_corres.push_back(move(std::pair<Eigen::Matrix4d, Eigen::Matrix4d>(
        tran_lidar_pose, tran_gnss_pose)));

    frist_lidar_pose = calib_vector_data_[i - 10].lidar_pose;
    frist_gnss_pose = calib_vector_data_[i - 10].gnss_pose;
  }

  Eigen::Quaterniond quater_gnss_to_lidar = SolveRotation(corres);
  Eigen::Vector3d vector_gnss_to_lidar =
      SloveTransform(matrix_corres, quater_gnss_to_lidar);

  double roll, pitch, yaw;
  tf::Quaternion quat(quater_gnss_to_lidar.x(), quater_gnss_to_lidar.y(),
                      quater_gnss_to_lidar.z(), quater_gnss_to_lidar.w());
  tf::Matrix3x3(quat).getRPY(yaw, roll, pitch);
  std::cout << "roll, pitch, yaw: " << roll * 57.3 << " " << pitch * 57.3 << " "
            << yaw * 57.3 << std::endl;
  std::cout << "x, y, z: " << vector_gnss_to_lidar.x() << " "
            << vector_gnss_to_lidar.y() << " " << vector_gnss_to_lidar.z()
            << std::endl;

  geometry_msgs::PoseStamped msg;

  // assign value to poseStamped

  // First assign value to "header".
  ros::Time currentTime = ros::Time::now();
  msg.header.stamp = currentTime;

  // Then assign value to "pose", which has member position and orientation
  msg.pose.position.x = vector_gnss_to_lidar.x();
  msg.pose.position.y = vector_gnss_to_lidar.y();
  msg.pose.position.z = vector_gnss_to_lidar.z();

  msg.pose.orientation.x = quater_gnss_to_lidar.x();
  msg.pose.orientation.y = quater_gnss_to_lidar.y();
  msg.pose.orientation.z = quater_gnss_to_lidar.z();
  msg.pose.orientation.w = quater_gnss_to_lidar.w();
  pub_calib_param_.publish(msg);
}

Eigen::Quaterniond SolveRotation(
    const std::vector<std::pair<Eigen::Quaterniond, Eigen::Quaterniond>>
        &corres) {
  if (corres.size() == 0) {
    std::cout << "no found tran quater!!!" << std::endl;
    return move(Eigen::Quaterniond().Identity());
  }

  // for(size_t i = 0; i < corres.size(); i++)

  //四元数转换为反对称矩阵
  // transform quaternion to skew symmetric matrix
  auto toSkewSymmetric = [](const Eigen::Vector3d &q) -> Eigen::Matrix3d {
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    mat(0, 1) = -q.z();
    mat(0, 2) = q.y();
    mat(1, 0) = q.z();
    mat(1, 2) = -q.x();
    mat(2, 0) = -q.y();
    mat(2, 1) = q.x();

    return move(mat);
  };

  //构建齐次线性方程组
  // create homogeneous linear equations
  Eigen::MatrixXd A(corres.size() * 4, 4);
  for (int i = 0; i < corres.size(); i++) {
    // get relative transform
    const auto &q_l2_l1 = corres[i].first;
    const auto &q_b2_b1 = corres[i].second;

    //左积矩阵
    // get left product matrix
    Eigen::Vector3d q_b2_b1_vec = q_b2_b1.vec();
    Eigen::Matrix4d left_Q_b2_b1 = Eigen::Matrix4d::Zero();
    left_Q_b2_b1.block<1, 3>(0, 1) = -q_b2_b1_vec.transpose();
    left_Q_b2_b1.block<3, 1>(1, 0) = q_b2_b1_vec;
    left_Q_b2_b1.block<3, 3>(1, 1) = toSkewSymmetric(q_b2_b1_vec);
    left_Q_b2_b1 += q_b2_b1.w() * Eigen::Matrix4d::Identity();

    //右积矩阵
    // get right product matrix
    Eigen::Vector3d q_l2_l1_vec = q_l2_l1.vec();
    Eigen::Matrix4d right_Q_l2_l1 = Eigen::Matrix4d::Zero();
    right_Q_l2_l1.block<1, 3>(0, 1) = -q_l2_l1_vec.transpose();
    right_Q_l2_l1.block<3, 1>(1, 0) = q_l2_l1_vec;
    right_Q_l2_l1.block<3, 3>(1, 1) = -toSkewSymmetric(q_l2_l1_vec);
    right_Q_l2_l1 += q_l2_l1.w() * Eigen::Matrix4d::Identity();

    // add loss function
    double angle_distance = 180.0 / M_PI * q_b2_b1.angularDistance(q_b2_b1);
    double huber = angle_distance > 5.0 ? 5.0 / angle_distance : 1.0;

    A.block<4, 4>(i * 4, 0) = huber * (left_Q_b2_b1 - right_Q_l2_l1);
  }

  // solve homogeneous linear equations by svd method
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
  Eigen::Quaterniond q_l_b(x(0), x(1), x(2), x(3));
  q_l_b.normalize();

  std::cout << "quater: " << q_l_b.x() << " " << q_l_b.y() << " " << q_l_b.z()
            << " " << q_l_b.w() << std::endl;
  return move(q_l_b);
}

Eigen::Vector3d SloveTransform(
    const std::vector<std::pair<Eigen::Matrix4d, Eigen::Matrix4d>> &corres,
    const Eigen::Quaterniond &q_l_g) {
  if (corres.size() == 0) {
    std::cout << "no found tran quater!!!" << std::endl;
    return move(Eigen::Vector3d().Zero());
  }
  Eigen::Matrix3d r_q_l_b = q_l_g.matrix();

  Eigen::MatrixXd A(corres.size() * 3, 3);
  Eigen::MatrixXd B(corres.size() * 3, 1);
  for (int i = 0; i < corres.size(); i++) {
    // get relative transform
    const auto &t_l2_l1 = corres[i].first;
    const auto &t_g2_g1 = corres[i].second;

    //转换为旋转矩阵
    Eigen::Matrix3d r_l2_l1 = t_l2_l1.block(0, 0, 3, 3);
    Eigen::Matrix3d r_g2_g1 = t_g2_g1.block(0, 0, 3, 3);

    Eigen::Vector3d v_l2_l1 = t_l2_l1.block(0, 3, 3, 1);
    Eigen::Vector3d v_g2_g1 = t_g2_g1.block(0, 3, 3, 1);

    Eigen::Matrix3d left_rotation = r_g2_g1 - Eigen::Matrix3d::Identity();
    Eigen::Vector3d right_vector = r_q_l_b * v_l2_l1 - v_g2_g1;

    A.block<3, 3>(i * 3, 0) = left_rotation;
    B.block<3, 1>(i * 3, 0) = right_vector;
  }
  // slove
  // Ax = B

  Eigen::MatrixXd A_inverse = EigenSVDInverse(A, 0);
  Eigen::Vector3d x = A_inverse * B;

  std::cout << "vector: " << x << std::endl;
  return x;
}

// 利用Eigen库，采用SVD分解的方法求解矩阵伪逆，默认误差er为0
Eigen::MatrixXd EigenSVDInverse(Eigen::MatrixXd &origin, float er) {
  // 进行svd分解
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(
      origin, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // 构建SVD分解结果
  Eigen::MatrixXd U = svd_holder.matrixU();
  Eigen::MatrixXd V = svd_holder.matrixV();
  Eigen::MatrixXd D = svd_holder.singularValues();

  // 构建S矩阵
  Eigen::MatrixXd S(V.cols(), U.cols());
  S.setZero();

  for (unsigned int i = 0; i < D.size(); ++i) {
    if (D(i, 0) > er) {
      S(i, i) = 1 / D(i, 0);
    } else {
      S(i, i) = 0;
    }
  }

  // pinv_matrix = V * S * U^T
  return V * S * U.transpose();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "get_feature");
  ros::NodeHandle nh;
  ros::Subscriber sub_lidar_pose, sub_gps_pose;
  sub_lidar_pose =
      nh.subscribe<nav_msgs::Odometry>("lidar_odom", 1, ReceiveLidarData);
  sub_gps_pose = nh.subscribe<nav_msgs::Odometry>("/sweeper/localization/gnss",
                                                  1, ReceiveGNSSData);
  pub_calib_param_ =
      nh.advertise<geometry_msgs::PoseStamped>("/sweeper/calib_param", 1);
  ros::spin();
}
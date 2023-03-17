#include <ceres/ceres.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <stdio.h>

#include <Eigen/Eigen>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <opencv2/core/core.hpp>
#include <vector>

#include "tf/transform_datatypes.h"

using namespace std;
using namespace cv;
std::string path_name_;

struct ceres_get_relative_position {
  ceres_get_relative_position(Eigen::Vector3d _odom_t,
                              Eigen::Quaterniond _odom_q,
                              Eigen::Vector3d _gps_t, Eigen::Quaterniond _gps_q,
                              Eigen::Vector3d _frist_gps_t,
                              Eigen::Quaterniond _frist_gps_q)
      : odom_t(_odom_t),
        odom_q(_odom_q),
        gps_t(_gps_t),
        gps_q(_gps_q),
        frist_gps_t(_frist_gps_t),
        frist_gps_q(_frist_gps_q) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    // gps点
    Eigen::Quaternion<T> q_gps((T)gps_q.w(), (T)gps_q.x(), (T)gps_q.y(),
                               (T)gps_q.z());
    Eigen::Matrix<T, 3, 1> t_gps = gps_t.template cast<T>();
    //第一个gps点
    Eigen::Quaternion<T> frist_q_gps((T)frist_gps_q.w(), (T)frist_gps_q.x(),
                                     (T)frist_gps_q.y(), (T)frist_gps_q.z());
    Eigen::Matrix<T, 3, 1> frist_t_gps = frist_gps_t.template cast<T>();

    //激光odom的变换
    Eigen::Quaternion<T> q_odom((T)odom_q.w(), (T)odom_q.x(), (T)odom_q.y(),
                                (T)odom_q.z());
    Eigen::Matrix<T, 3, 1> t_odom = odom_t.template cast<T>();

    Eigen::Quaternion<T> q_incre{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_incre{t[0], t[1], t[2]};

    Eigen::Matrix<T, 4, 4> lidar_gps_tran;

    lidar_gps_tran.block(0, 0, 3, 3) = q_incre.matrix();
    lidar_gps_tran.block(0, 3, 3, 1) = t_incre;
    lidar_gps_tran(3, 3) = (T)1;
    lidar_gps_tran(3, 0) = (T)0;
    lidar_gps_tran(3, 1) = (T)0;
    lidar_gps_tran(3, 2) = (T)0;

    Eigen::Matrix<T, 4, 4> gps_world_tran;
    gps_world_tran.block(0, 0, 3, 3) = q_gps.matrix();
    gps_world_tran.block(0, 3, 3, 1) = t_gps;
    gps_world_tran(3, 3) = (T)1;
    gps_world_tran(3, 0) = (T)0;
    gps_world_tran(3, 1) = (T)0;
    gps_world_tran(3, 2) = (T)0;

    Eigen::Matrix<T, 4, 4> frist_gps_world_tran;
    frist_gps_world_tran.block(0, 0, 3, 3) = frist_q_gps.matrix();
    frist_gps_world_tran.block(0, 3, 3, 1) = frist_t_gps;
    frist_gps_world_tran(3, 3) = (T)1;
    frist_gps_world_tran(3, 0) = (T)0;
    frist_gps_world_tran(3, 1) = (T)0;
    frist_gps_world_tran(3, 2) = (T)0;

    Eigen::Matrix<T, 4, 4> lidar_world_tran = lidar_gps_tran.inverse() *
                                              frist_gps_world_tran.inverse() *
                                              gps_world_tran * lidar_gps_tran;

    residual[0] = lidar_world_tran(0, 3) - t_odom[0];
    residual[1] = lidar_world_tran(1, 3) - t_odom[1];
    residual[2] = lidar_world_tran(2, 3) - t_odom[2];

    return true;
  }
  static ceres::CostFunction *Create(const Eigen::Vector3d _odom_t,
                                     const Eigen::Quaterniond _odom_q,
                                     const Eigen::Vector3d _gps_t,
                                     const Eigen::Quaterniond _gps_q,
                                     const Eigen::Vector3d _frist_gps_t,
                                     const Eigen::Quaterniond _frist_gps_q) {
    // TODO: can be vector or distance
    return (
        new ceres::AutoDiffCostFunction<ceres_get_relative_position, 3, 4, 3>(
            new ceres_get_relative_position(_odom_t, _odom_q, _gps_t, _gps_q,
                                            _frist_gps_t, _frist_gps_q)));
  }
  Eigen::Vector3d odom_t, gps_t, frist_gps_t;
  Eigen::Quaterniond odom_q, gps_q, frist_gps_q;
};

void ComputedMeanOfAngle(std::vector<double> angles, double &angle) {
  if (angles.size() > 0 && angles.size() < 2) {
    angle = angles[0];
    return;
  }
  double angle_all;
  bool angle_in_pai = false;
  bool angle_less_zero = false;
  bool angle_greater_zero = false;
  for (size_t i = 0; i < angles.size(); i++) {
    if (angles[i] < 0) angle_less_zero = true;
    if (angles[i] >= 0) angle_greater_zero = true;
    if (angles[i] > 3.0 * 3.1415926 / 4.0) angle_in_pai = true;
  }
  //如果在pai附近
  if (angle_less_zero && angle_greater_zero && angle_in_pai) {
    for (size_t i = 0; i < angles.size(); i++) {
      if (angles[i] < 0)
        angle += angles[i] + 2 * M_PI;
      else {
        angle += angles[i];
      }
    }
    // std::cout << "angle " << angle << " size : " << angles.size() <<
    // std::endl;
    angle = angle / angles.size();
    if (angle > M_PI) angle -= 2 * M_PI;
    // if(angle < -M_PI)
    //     angle = angle + 2 * M_PI;
    return;
  }
  //如果不在pai附近
  else {
    for (int i = 0; i < angles.size(); i++) {
      angle += angles[i];
      // std::cout<<"172 "<<angles[i]<<std::endl;
    }
    // std::cout<<"angle "<<angle<<" "<<angles[0]<<" "<<angles[1]<<std::endl;
    // std::cout << "angle " << angle << " size : " << angles.size() <<
    // std::endl;
    angle = angle / angles.size();
    // std::cout<<"angle "<<angle<<" "<<angles.size()<<std::endl;
    return;
  }
}

double m_para_buffer_incremental[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

Eigen::Map<Eigen::Quaterniond> m_q_w_incre =
    Eigen::Map<Eigen::Quaterniond>(m_para_buffer_incremental);
Eigen::Map<Eigen::Vector3d> m_t_w_incre =
    Eigen::Map<Eigen::Vector3d>(m_para_buffer_incremental + 4);

std::vector<Eigen::Quaterniond> gps_quters;
std::vector<Eigen::Vector3d> gps_positions;

std::vector<Eigen::Quaterniond> odom_quters;
// std::vector<Eigen::Vector3d> odom_angle;
std::vector<Eigen::Vector3d> odom_positions;

void set_ceres_solver_bound(ceres::Problem &problem) {
  for (unsigned int i = 0; i < 3; i++) {
    problem.SetParameterLowerBound(m_para_buffer_incremental + 4, i, -5.0);
    problem.SetParameterUpperBound(m_para_buffer_incremental + 4, i, +5.0);
  }
}

std::vector<std::string> StringSplit(const std::string &str,
                                     const std::string &delim) {
  std::vector<std::string> res;
  if ("" == str) return res;
  char *strs = new char[str.length() + 1];
  strcpy(strs, str.c_str());

  char *d = new char[delim.length() + 1];
  strcpy(d, delim.c_str());

  char *p = strtok(strs, d);
  while (p) {
    std::string s = p;
    res.push_back(s);
    p = strtok(NULL, d);
  }

  return res;
}

static void toEulerAngle(const Eigen::Quaterniond &q, double &yaw,
                         double &pitch, double &roll) {
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_to_imu_gps");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.param<string>("path_name", path_name_, "no/name");
  std::cout << "path_name : " << path_name_ << std::endl;

  string pkg_name = ros::package::getPath("calib_lidar_to_gps_imu");
  path_name_ = pkg_name + path_name_;
  std::cout << path_name_ << std::endl;

  ros::Publisher o_pub_odom = nh.advertise<nav_msgs::Odometry>(
      "/sweeper/navigation/localization/gps", 1);
  ros::Publisher o_pub_odom_odom = nh.advertise<nav_msgs::Odometry>(
      "/sweeper/navigation/localization/odom", 1);
  ros::Publisher o_pub_gps_before = nh.advertise<nav_msgs::Odometry>(
      "/sweeper/navigation/localization/gps_before", 1);

  std::vector<Eigen::Quaterniond> gps_quaterniond;
  std::vector<Eigen::Quaterniond> odom_quaterniond;

  std::vector<Eigen::Vector3d> gps_transformation;
  std::vector<Eigen::Vector3d> odom_transformation;

  std::string line, str;
  ifstream inf;
  inf.open(path_name_, ios::in);
  int i = 0;
  vector<std::string> two_string;
  // std::cout << "105" << std::endl;
  while (getline(inf, line)) {
    istringstream stream(line);
    if (i % 2 == 1) {
      two_string.clear();
      while (stream >> str) {
        two_string.push_back(str);
      }
      Eigen::Quaterniond gps_qj;
      Eigen::Vector3d gps_tj;
      gps_tj = Eigen::Vector3d(atof(two_string[0].c_str()) * 0.29,
                               atof(two_string[1].c_str()) * 0.29,
                               atof(two_string[2].c_str()) * 0.29);
      gps_qj.x() = atof(two_string[3].c_str());
      gps_qj.y() = atof(two_string[4].c_str());
      gps_qj.z() = atof(two_string[5].c_str());
      gps_qj.w() = atof(two_string[6].c_str());

      gps_quaterniond.push_back(gps_qj);
      gps_transformation.push_back(gps_tj);
    } else {
      two_string.clear();
      while (stream >> str) {
        two_string.push_back(str);
      }
      Eigen::Quaterniond odom_qj;
      Eigen::Vector3d odom_tj;
      odom_tj = Eigen::Vector3d(atof(two_string[0].c_str()),
                                atof(two_string[1].c_str()),
                                atof(two_string[2].c_str()));

      odom_qj.x() = atof(two_string[3].c_str());
      odom_qj.y() = atof(two_string[4].c_str());
      odom_qj.z() = atof(two_string[5].c_str());
      odom_qj.w() = atof(two_string[6].c_str());
      odom_quaterniond.push_back(odom_qj);
      odom_transformation.push_back(odom_tj);
    }
    i++;
  }
  std::cout << "size : " << odom_transformation.size() << std::endl;
  ceres::Solver::Summary summary;
  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.0001);
  ceres::LocalParameterization *q_parameterization =
      new ceres::EigenQuaternionParameterization();
  ceres::Problem::Options problem_options;
  ceres::ResidualBlockId block_id;
  ceres::Problem problem(problem_options);
  std::vector<ceres::ResidualBlockId> residual_block_ids;

  problem.AddParameterBlock(m_para_buffer_incremental, 4, q_parameterization);
  problem.AddParameterBlock(m_para_buffer_incremental + 4, 3);

  for (size_t i = 0; i < odom_transformation.size(); i++) {
    ceres::CostFunction *cost_function;
    cost_function = ceres_get_relative_position::Create(
        odom_transformation[i], odom_quaterniond[i], gps_transformation[i],
        gps_quaterniond[i], gps_transformation[0], gps_quaterniond[0]);

    block_id = problem.AddResidualBlock(cost_function, loss_function,
                                        m_para_buffer_incremental,
                                        m_para_buffer_incremental + 4);
    residual_block_ids.push_back(block_id);
  }

  ceres::Solver::Options options;
  std::vector<ceres::ResidualBlockId> residual_block_ids_bak;
  residual_block_ids_bak = residual_block_ids;

  options.linear_solver_type = ceres::DENSE_QR;

  options.minimizer_progress_to_stdout = false;
  options.check_gradients = false;

  if (0) {
    options.max_num_iterations = 20;
    set_ceres_solver_bound(problem);
    ceres::Solve(options, &problem, &summary);

    // Remove outliers
    residual_block_ids_bak.clear();

    ceres::Problem::EvaluateOptions eval_options;
    eval_options.residual_blocks = residual_block_ids;
    double total_cost = 0.0;
    double avr_cost;
    vector<double> residuals;
    problem.Evaluate(eval_options, &total_cost, &residuals, nullptr, nullptr);
    avr_cost = total_cost / residual_block_ids.size();

    for (unsigned int i = 0; i < residual_block_ids.size(); i++) {
      if ((fabs(residuals[3 * i + 0]) + fabs(residuals[3 * i + 1]) +
           fabs(residuals[3 * i + 2])) >
          7 * avr_cost)  // std::min( 1.0, 10 * avr_cost )
      {
        problem.RemoveResidualBlock(residual_block_ids[i]);
      } else {
        residual_block_ids_bak.push_back(residual_block_ids[i]);
      }
    }
    residual_block_ids = residual_block_ids_bak;
  }

  if (0) {
    options.max_num_iterations = 20;
    set_ceres_solver_bound(problem);
    ceres::Solve(options, &problem, &summary);

    // Remove outliers
    residual_block_ids_bak.clear();

    ceres::Problem::EvaluateOptions eval_options;
    eval_options.residual_blocks = residual_block_ids;
    double total_cost = 0.0;
    double avr_cost;
    vector<double> residuals;
    problem.Evaluate(eval_options, &total_cost, &residuals, nullptr, nullptr);
    avr_cost = total_cost / residual_block_ids.size();

    for (unsigned int i = 0; i < residual_block_ids.size(); i++) {
      if ((fabs(residuals[3 * i + 0]) + fabs(residuals[3 * i + 1]) +
           fabs(residuals[3 * i + 2])) >
          7 * avr_cost)  // std::min( 1.0, 10 * avr_cost )
      {
        problem.RemoveResidualBlock(residual_block_ids[i]);
      } else {
        residual_block_ids_bak.push_back(residual_block_ids[i]);
      }
    }
    residual_block_ids = residual_block_ids_bak;
  }

  std::cout << " is iteration" << std::endl;
  options.max_num_iterations = 50;
  set_ceres_solver_bound(problem);
  ceres::Solve(options, &problem, &summary);

  std::cout << std::setprecision(15) << "incre t  : " << m_t_w_incre(0) << " "
            << m_t_w_incre(1) << " " << m_t_w_incre(2) << " "
            << "\n"
            << m_q_w_incre.w() << " " << m_q_w_incre.x() << " "
            << m_q_w_incre.y() << " " << m_q_w_incre.z() << std::endl;

  std::cout << "angle  : " << m_q_w_incre.matrix().eulerAngles(2, 1, 0)[0]
            << " " << m_q_w_incre.matrix().eulerAngles(2, 1, 0)[1] << " "
            << m_q_w_incre.matrix().eulerAngles(2, 1, 0)[2] << std::endl;

  Eigen::Isometry3d lidar_to_gps = Eigen::Isometry3d::Identity();
  // lidar_to_gps.
  lidar_to_gps.rotate(m_q_w_incre.matrix());
  lidar_to_gps.pretranslate(m_t_w_incre);

  Eigen::Isometry3d gps_to_lidar = lidar_to_gps.inverse();
  Eigen::Matrix3d gps_to_lidar_matrix;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      gps_to_lidar_matrix(i, j) = gps_to_lidar(i, j);
    }
  }
  Eigen::Quaterniond gps_to_lidar_quater;
  gps_to_lidar_quater = gps_to_lidar_matrix;
  std::cout << "gps_to_lidar_ea  : "
            << gps_to_lidar_matrix.eulerAngles(2, 1, 0)[0] * 57.3 << " "
            << gps_to_lidar_matrix.eulerAngles(2, 1, 0)[1] * 57.3 << " "
            << gps_to_lidar_matrix.eulerAngles(2, 1, 0)[2] * 57.3 << std::endl;
  std::cout << "gps_to_lidar : " << gps_to_lidar(0, 3) << " "
            << gps_to_lidar(1, 3) << " " << gps_to_lidar(2, 3) << " "
            << std::endl;
  std::cout << "gps_to_lidar quater: " << gps_to_lidar_quater.x() << " "
            << gps_to_lidar_quater.y() << " " << gps_to_lidar_quater.z() << " "
            << gps_to_lidar_quater.w() << std::endl;

  std::cout << "vector  : "
            << m_q_w_incre.matrix().eulerAngles(2, 1, 0)[0] * 57.3 << " "
            << m_q_w_incre.matrix().eulerAngles(2, 1, 0)[1] * 57.3 << " "
            << m_q_w_incre.matrix().eulerAngles(2, 1, 0)[2] * 57.3 << std::endl;

  //算出实际的delta_gps值
  // Eigen::Quaterniond delta_gps
  // m_t_w_incre = Eigen::Vector3d(0.540927338082619,
  // 0.517892452768321, 2.10387247263517); m_q_w_incre =
  // Eigen::Quaterniond(0.998331907815185, 0.0163162424449328,
  // -0.00721288088429475, 0.0549104399902179);
  m_q_w_incre = m_q_w_incre.normalized();
  while (ros::ok()) {
    for (size_t i = 0; i < odom_transformation.size(); i++) {
      Eigen::Matrix<double, 4, 4> lidar_to_gps;
      lidar_to_gps.block(0, 0, 3, 3) = m_q_w_incre.matrix();
      lidar_to_gps.block(0, 3, 3, 1) = m_t_w_incre;

      lidar_to_gps(3, 3) = 1;
      lidar_to_gps(3, 0) = 0;
      lidar_to_gps(3, 1) = 0;
      lidar_to_gps(3, 2) = 0;

      Eigen::Matrix<double, 4, 4> gps_to_frist;
      gps_to_frist.block(0, 0, 3, 3) = gps_quaterniond[i].matrix();
      gps_to_frist.block(0, 3, 3, 1) = gps_transformation[i];

      gps_to_frist(3, 3) = 1;
      gps_to_frist(3, 0) = 0;
      gps_to_frist(3, 1) = 0;
      gps_to_frist(3, 2) = 0;

      Eigen::Matrix<double, 4, 4> frist_gps_to_frist;
      frist_gps_to_frist.block(0, 0, 3, 3) = gps_quaterniond[0].matrix();
      frist_gps_to_frist.block(0, 3, 3, 1) = gps_transformation[0];

      frist_gps_to_frist(3, 3) = 1;
      frist_gps_to_frist(3, 0) = 0;
      frist_gps_to_frist(3, 1) = 0;
      frist_gps_to_frist(3, 2) = 0;

      Eigen::Matrix<double, 4, 4> lidar_in_map;
      lidar_in_map = lidar_to_gps.inverse() * frist_gps_to_frist.inverse() *
                     gps_to_frist * lidar_to_gps;
      Eigen::Matrix<double, 3, 3> lidar_to_map_matrix;
      lidar_to_map_matrix = lidar_in_map.block(0, 0, 3, 3);
      Eigen::Quaterniond lidar_to_map_quater(lidar_to_map_matrix);
      nav_msgs::Odometry odomAftMapped;
      odomAftMapped.header.frame_id = "/camera_init";
      odomAftMapped.header.stamp = ros::Time::now();
      odomAftMapped.pose.pose.orientation.x = lidar_to_map_quater.x();
      odomAftMapped.pose.pose.orientation.y = lidar_to_map_quater.y();
      odomAftMapped.pose.pose.orientation.z = lidar_to_map_quater.z();
      odomAftMapped.pose.pose.orientation.w = lidar_to_map_quater.w();
      odomAftMapped.pose.pose.position.x = lidar_in_map(0, 3);
      odomAftMapped.pose.pose.position.y = lidar_in_map(1, 3);
      odomAftMapped.pose.pose.position.z = lidar_in_map(2, 3);
      o_pub_odom.publish(odomAftMapped);

      nav_msgs::Odometry odomAftMapped1;
      odomAftMapped1.header.frame_id = "/camera_init";
      odomAftMapped1.header.stamp = ros::Time::now();
      odomAftMapped1.pose.pose.orientation.x = odom_quaterniond[i].x();
      odomAftMapped1.pose.pose.orientation.y = odom_quaterniond[i].y();
      odomAftMapped1.pose.pose.orientation.z = odom_quaterniond[i].z();
      odomAftMapped1.pose.pose.orientation.w = odom_quaterniond[i].w();

      odomAftMapped1.pose.pose.position.x = odom_transformation[i].x();
      odomAftMapped1.pose.pose.position.y = odom_transformation[i].y();
      odomAftMapped1.pose.pose.position.z = odom_transformation[i].z();
      o_pub_odom_odom.publish(odomAftMapped1);

      nav_msgs::Odometry odomAftMapped2;
      odomAftMapped2.header.frame_id = "/camera_init";
      odomAftMapped2.header.stamp = ros::Time::now();
      odomAftMapped2.pose.pose.orientation.x = gps_quaterniond[i].x();
      odomAftMapped2.pose.pose.orientation.y = gps_quaterniond[i].y();
      odomAftMapped2.pose.pose.orientation.z = gps_quaterniond[i].z();
      odomAftMapped2.pose.pose.orientation.w = gps_quaterniond[i].w();

      odomAftMapped2.pose.pose.position.x =
          gps_transformation[i].x() - gps_transformation[0].x();
      odomAftMapped2.pose.pose.position.y =
          gps_transformation[i].y() - gps_transformation[0].y();
      odomAftMapped2.pose.pose.position.z =
          gps_transformation[i].z() - gps_transformation[0].z();
      o_pub_gps_before.publish(odomAftMapped2);
    }
  }
}

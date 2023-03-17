#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "sweeper_msgs/SweeperChassisDetail.h"

int main() {
  Eigen::Vector3d raw_angle(0.0, 1.0, -1.0);
  Eigen::Vector3d tran_90_yaw(-3.1415926 / 2.0, 0.0, 0.0);
  Eigen::Quaterniond tran_90_yaw_quater =
      Eigen::AngleAxisd(tran_90_yaw[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(tran_90_yaw[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(tran_90_yaw[2], Eigen::Vector3d::UnitX());
  Eigen::Quaterniond raw_angle_quater =
      Eigen::AngleAxisd(raw_angle[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(raw_angle[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(raw_angle[2], Eigen::Vector3d::UnitX());
  Eigen::Quaterniond calib_angle = tran_90_yaw_quater * raw_angle_quater;

  Eigen::Matrix<double, 3, 3> tran_90 = tran_90_yaw_quater.matrix();
  Eigen::Vector3d tran_imu(1.0, 2.0, 3.0);
  Eigen::Vector3d tran_imu_90 = tran_90 * tran_imu;
  std::cout << "tran 90 : " << tran_imu_90 << std::endl;

  // std::cout<<"quater : "<<tran_90_yaw_quater[0]<<"
  // "<<tran_90_yaw_quater[1]<<" "<<tran_90_yaw_quater[2]<<"
  // "<<tran_90_yaw_quater[3]<<std::endl; tf::Quaternion quat(calib_angle.x(),
  // calib_angle.y(), calib_angle.z(), calib_angle.w()); double roll, pitch,
  // yaw; tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", raw_angle[2],
           raw_angle[1], raw_angle[0]);
  // Eigen::Vector3d eulerAngle4 = calib_angle.matrix().eulerAngles(2,1,0);
  // std::cout << "yaw(z) pitch(y) roll(x) = " << eulerAngle4.transpose() <<
  // std::endl; ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", roll,
  // pitch, yaw);
  Eigen::Vector3d tran(2, -1, 3);

  Eigen::Quaterniond tran_angular(tran_imu_90 * tran_imu.transpose());

  // tf::Quaternion quat(tran_angular.x(), tran_angular.y(), tran_angular.z(),
  // tran_angular.w());
  tf::Quaternion quat(0.00391485, 0.169227, -0.0250145, 0.985252);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  yaw = yaw*0.3; //   ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", roll*57.3
  //   pitch57.3, yaw*57.3);
  std::cout << "rpy : " << roll * 57.3 << " " << pitch * 57.3 << " "
            << yaw * 57.3 << std::endl;

  yaw = 1.17960606709062 / 57.3 * 1.5;
  geometry_msgs::Quaternion fusion_quater =
      tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  std::cout << "quater : " << fusion_quater.x << " " << fusion_quater.y << " "
            << fusion_quater.z << " " << fusion_quater.w << std::endl;

  //     Eigen::Vector3d euler_angles = tran_angular.eulerAngles(2, 1, 0);
  // std::cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() <<
  // std::endl;
}
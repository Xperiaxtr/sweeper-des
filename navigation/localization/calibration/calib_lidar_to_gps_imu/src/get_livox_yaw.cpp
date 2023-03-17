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

int main(int argc, char **argv) {
      ros::init(argc, argv, "loam_new");
  // Eigen::Quaterniond livox_plane(0.0, 0.0, 0.0, 1.0);

  double livox_gnss_yaw = 0.0;
  tf::Quaternion livox_quater(0.0, 0.0, 0.0, 1.0);
  double roll, pitch, yaw;
  tf::Matrix3x3(livox_quater).getRPY(roll, pitch, yaw);
  std::cout << "livox_plane angle: " << roll * 57.3 << " " << pitch * 57.3
            << " " << yaw * 57.3 << std::endl;

  Eigen::Quaterniond cal_livox_to_car =
      Eigen::AngleAxisd(livox_gnss_yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  std::cout << "livox_to_car : " << cal_livox_to_car.x() << " "
            << cal_livox_to_car.y() << " " << cal_livox_to_car.z() << " "
            << cal_livox_to_car.w() << std::endl;
  return 0;
}
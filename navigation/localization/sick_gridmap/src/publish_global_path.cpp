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

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_to_imu_gps");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::Publisher o_pub_odom = nh.advertise<nav_msgs::Odometry>(
      "/sweeper/navigation/localization/path", 1);
  std::vector<Eigen::Quaterniond> odom_quaterniond;
  std::vector<Eigen::Vector3d> odom_transformation;

  std::string path_name_ =
      "../sweeper_ws/src/sweeper_haide/data/path/L_1711.txt";
  //   std::string line, str;
  //   ifstream inf;
  //   inf.open(path_name_, ios::in);
  //   int i = 0;
  //   vector<std::string> two_string;
  //   while (getline(inf, line)) {
  //     istringstream stream(line);
  //     two_string.clear();
  //     while (stream >> str) {
  //       two_string.push_back(str);
  //     }
  //     Eigen::Quaterniond odom_qj;
  //     Eigen::Vector3d odom_tj;
  //     odom_tj = Eigen::Vector3d(atof(two_string[3].c_str()),
  //                               atof(two_string[4].c_str()),
  //                               atof(two_string[5].c_str()));

  //     odom_qj.x() = atof(two_string[6].c_str());
  //     odom_qj.y() = atof(two_string[7].c_str());
  //     odom_qj.z() = atof(two_string[8].c_str());
  //     odom_qj.w() = atof(two_string[9].c_str());
  //     odom_quaterniond.push_back(odom_qj);
  //     odom_transformation.push_back(odom_tj);
  //   }
  std::fstream fs(path_name_, std::ios_base::in);
  std::string line_str;
  while (getline(fs, line_str, '\n')) {
    std::vector<std::string> path_str = StringSplit(line_str, "|");
    // rode_num = atoi(path_str[0].c_str());
    Eigen::Quaterniond odom_qj;
    Eigen::Vector3d odom_tj;
    odom_tj = Eigen::Vector3d(atof(path_str[3].c_str()),
                              atof(path_str[4].c_str()),
                              atof(path_str[5].c_str()));

    odom_qj.x() = atof(path_str[6].c_str());
    odom_qj.y() = atof(path_str[7].c_str());
    odom_qj.z() = atof(path_str[8].c_str());
    odom_qj.w() = atof(path_str[9].c_str());
    odom_quaterniond.push_back(odom_qj);
    odom_transformation.push_back(odom_tj);
  }
  while (ros::ok()) {
    for (size_t i = 0; i < odom_transformation.size(); i++) {
      nav_msgs::Odometry odomAftMapped;
      odomAftMapped.header.frame_id = "/camera_init";
      odomAftMapped.header.stamp = ros::Time::now();
      odomAftMapped.pose.pose.orientation.x = odom_quaterniond[i].x();
      odomAftMapped.pose.pose.orientation.y = odom_quaterniond[i].y();
      odomAftMapped.pose.pose.orientation.z = odom_quaterniond[i].z();
      odomAftMapped.pose.pose.orientation.w = odom_quaterniond[i].w();
      odomAftMapped.pose.pose.position.x = odom_transformation[i].x();
      odomAftMapped.pose.pose.position.y = odom_transformation[i].y();
      odomAftMapped.pose.pose.position.z = odom_transformation[i].z();
      o_pub_odom.publish(odomAftMapped);
    }
  }
}
#pragma once

#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <opencv2/opencv.hpp>
#include <stack>
#include <string>
#include <vector>

#include "object.h"
#include "ros/package.h"

namespace sweeper {
namespace navigation {

struct tracker {
  tracker(std::shared_ptr<Object> obj_ptr) : object_ptr(obj_ptr) {}
  std::shared_ptr<Object> object_ptr;
  int num_points;  // the numbers of object points

  /**< 8 corners of boundingbox */
  cv::Point3f corners[8];

  /**< x:length, y:width, z:height, notice that length >= width*/
  cv::Point3f size;

  /**<box geometry center*/
  Eigen::Vector3f center;
  Eigen::Vector3d local_center;
  int track_id = 0;
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  int class_type = 0;

  // age of the tracked object
  int untracked_time = 0;
  double latest_tracked_time = 0.0;
  float max_height = 0.;
  // kalman filter
  Eigen::Matrix2f Af;
  Eigen::Matrix2f R, Q, k_filter;
  Eigen::Matrix2f pre_true_covariance, estimate_true_covariance;
  Eigen::Vector2f predict_vel, measure_vel, estimate_vel;
  float last_pose_x = 0, last_pose_y = 0;
  float all_vel_x = 0, all_vel_y = 0;
  int tracked_time = 0;
  void kalman_init() {
    Af << 1, 0, 0, 1;
    R << 5, 0, 0, 5;
    Q << 1, 0, 0, 1;
    k_filter << 1, 0, 0, 1;
    estimate_true_covariance << 1, 0, 0, 1;
    pre_true_covariance << 1, 0, 0, 1;
    predict_vel << 0, 0;
    measure_vel << 0, 0;
    estimate_vel << 0, 0;
  }

  void vel_kalman(float pose_x, float pose_y, double frame_time) {
    double diff_time = frame_time - latest_tracked_time;
    Q << 1, 0, 0, 1;
    R << 100 * diff_time, 0, 0, 100 * diff_time;
    float vel_x = (pose_x - last_pose_x) / diff_time;
    float vel_y = (pose_y - last_pose_y) / diff_time;
    // std::cout<<"the diff time is "<<diff_time<<std::endl;
    // std::cout<<"the pose is "<<(pose_y - last_pose_y) <<","<<(pose_x -
    // last_pose_x)<<std::endl;
    if (fabs(vel_x) < 100 && fabs(vel_y) < 100) {
      measure_vel << vel_x, vel_y;
    } else {
      measure_vel << 0, 0;
    }

    pre_true_covariance = Af * estimate_true_covariance * Af.transpose() + Q;
    k_filter = pre_true_covariance * (pre_true_covariance + R).inverse();
    predict_vel = Af * estimate_vel;
    estimate_vel = predict_vel + k_filter * (measure_vel - predict_vel);
    estimate_true_covariance =
        (Eigen::Matrix2f::Identity() - k_filter) * pre_true_covariance;
    last_pose_x = pose_x;
    last_pose_y = pose_y;
    // if(fabs(estimate_vel[0])>10.)
    // {
    //     estimate_vel[0]=0;
    //     estimate_vel[1]=0;
    // }
    velocity << estimate_vel[0], estimate_vel[1], 0;
  }
};

}  // namespace navigation
}  // namespace sweeper

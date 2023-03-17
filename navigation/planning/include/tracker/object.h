#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "Eigen/Core"

namespace sweeper {
namespace navigation {

struct alignas(16) Object {
  // Object();
  // object id per frame
  int id = 0;

  bool inside_road_edge_flag;
  // point cloud of the object
  pcl::PointCloud<pcl::PointXYZI> cloud;

  float height_max = -FLT_MAX;
  float height_min = FLT_MAX;

  // oriented boundingbox information
  // main direction
  Eigen::Vector3d direction = Eigen::Vector3d(1, 0, 0);
  // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
  double theta = 0.0;
  // ground center of the object (cx, cy, z_min)
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  // size of the oriented bbox, length is the size in the main direction
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;

  cv::Point dis_x_y;
  cv::Point bottom_left_point;
  cv::Point bottom_right_point;

  int obstacle_expand_width;
  // contours
  std::vector<cv::Point> contour;

  // RotatedRect box
  cv::RotatedRect box;

  // contours area
  int area;

  // origin rect points
  cv::Point2f rect_points[4];

  // expand rect points
  cv::Point2f expand_rect_points[4];

  // shape feature used for tracking
  std::vector<float> shape_features;

  // foreground score/probability
  float score = 0.0;

  float distance = 0.0f;

  // Probability of each type, used for track type.
  std::vector<float> type_probs;

  // fg/bg flag
  bool is_background = false;

  // tracking information
  int track_id = 0;
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  // age of the tracked object
  double tracking_time = 0.0;
  double latest_tracked_time = 0.0;
  double timestamp = 0.0;

  // stable anchor_point during time, e.g., barycenter
  Eigen::Vector3d anchor_point;

  // noise covariance matrix for uncertainty of position and velocity
  Eigen::Matrix3d position_uncertainty;
  Eigen::Matrix3d velocity_uncertainty;

  // modeling uncertainty from sensor level tracker
  Eigen::Matrix4d state_uncertainty = Eigen::Matrix4d::Identity();
  // Tailgating (trajectory of objects)
  std::vector<Eigen::Vector3d> drops;

  // local lidar track id
  int local_lidar_track_id = -1;
  // local radar track id
  int local_radar_track_id = -1;
  // local camera track id
  int local_camera_track_id = -1;

  // local lidar track ts
  double local_lidar_track_ts = -1;
  // local radar track ts
  double local_radar_track_ts = -1;
  // local camera track ts
  double local_camera_track_ts = -1;
};
}  // namespace navigation
}  // namespace sweeper

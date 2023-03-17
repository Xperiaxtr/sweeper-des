#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../../../common/log.h"
#include "../../../common/pose_util.h"
#include "../../../common/watch_dog.h"
#include "../common/data_center.h"
#include "sweeper_msgs/SweeperChassisDetail.h"

#define PI 3.1415926
#define RAD_TO_DEGREE (180.0 / PI)

using sweeper::common::WatchDog;

namespace sweeper {
typedef struct PointType {
  int point_type;
  int crossing_name;
  Eigen::Vector3d point;
  Eigen::Vector3d gps_point;
  Eigen::Quaterniond quat;
  Eigen::Vector3d euler_angle;
} PointType;
typedef std::vector<PointType> LineType;
class FindGlobalPath {
 public:
  FindGlobalPath(SweepDataCenter *sweep_data_center);
  ~FindGlobalPath();

 public:

  void AddStateCode(const int code);
  bool FindStateCode(const int code);

  void GnssCallback(const nav_msgs::Odometry &gnss_location);
  void FusionCallback(const nav_msgs::Odometry &fusion_location);

  void WorldToBynavAffine(const PointType &point, Eigen::Affine3d *affine,
                          Eigen::Vector3d *rpy_vec);

  bool ReadLidarEntirePath(const std::string &file_name);
  bool ReadGnssEntirePath(const std::string &file_name);

  void GnssReset();
  void LidarReset();

  void GnssReferenceLine();
  void GnssInitialIndex();
  void GnssCurrentIndex();
  void GnssCurrentScene();
  void GnssConvertToCar();
  void GnssCurrentLine();

  void LidarReferenceLine();
  void LidarInitialIndex();
  void LidarCurrentIndex();
  void LidarCurrentScene();
  void LidarConvertToCar();
  void LidarCurrentLine();

  double LidarInitialPose(geometry_msgs::Pose *init_pose);
  void ConvertLidarNavMsg(nav_msgs::Path *reference_path,
                     nav_msgs::Path *boudary_path);
  void ConvertGnssNavMsg(nav_msgs::Path *reference_path,
                     nav_msgs::Path *boudary_path);

  std::vector<std::string> StringSplit(const std::string &str,
                                       const std::string &delim);

  bool gnss_fix_flag_;
  bool fusion_fix_flag_;

  double left_offset_ = 0.7;
  double right_offset_ = 0.7;
  double antenna_dis_ = 0.0;

  int gnss_path_index_;
  int gnss_last_index_;
  int gnss_current_index_;
  int gnss_last_scene_type_;
  int gnss_current_scene_type_;

  int lidar_path_index_;
  int lidar_last_index_;
  int lidar_current_index_;
  int lidar_last_scene_type_;
  int lidar_current_scene_type_;

  std::string file_path_;
  Eigen::Affine3d bynav_livox_extrinsic_;
  Eigen::Affine3d world_to_bynav_affine_;
  Eigen::Affine3d world_to_car_affine_;

  PointType current_location_;

  LineType entire_lidar_line_;
  LineType entire_gnss_line_;
  
  LineType gnss_reference_line_, gnss_left_boundary_line_, gnss_right_boundary_line_; 
  LineType lidar_reference_line_, lidar_left_boundary_line_, lidar_right_boundary_line_;
  LineType global_left_boundary_line_, global_right_boundary_line_, global_center_reference_line_;

  SweepDataCenter *sweep_data_center_;

  nav_msgs::Odometry gnss_location_;
  nav_msgs::Odometry fusion_location_;

  ros::Subscriber gnss_sub_;
  ros::Subscriber fusion_sub_;
  ros::Subscriber chassis_sub_;

  WatchDog gps_watch_dog_;
  WatchDog fusion_watch_dog_;
};

}  // namespace sweeper
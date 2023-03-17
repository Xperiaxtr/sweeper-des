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
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <memory>
#include <opencv2/opencv.hpp>
#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "base_tracker.h"
#include "tracker_object.h"
#include "object.h"
#include "ros/package.h"

namespace sweeper {
namespace navigation {
class ObjectTracker {
 public:
  ObjectTracker(){};
  ~ObjectTracker() {}
  // @brief initialize tracker's configs
  // @return true if initialize successfully, otherwise return false
  bool Init();

  // @brief track detected objects over consecutive frames
  // @params[IN] objects: recently detected objects
  // @params[IN] timestamp: timestamp of recently detected objects
  // @params[IN] options: tracker options with necessary information
  // @params[OUT] tracked_objects: tracked objects with tracking information
  // @return true if track successfully, otherwise return false
  bool Track(const std::vector<std::shared_ptr<Object>> objects,
             double timestamp,
             std::vector<std::shared_ptr<Object>> *tracked_objects);

  std::string name() const { return "ObjectTracker"; }

  void setNewObjects(std::vector<tracker> trackers, double frame_time);
  void getObjects(std::vector<tracker> &trackers);

 private:
  void predictOldTrackers(double time);
  void updateTrackers(std::vector<tracker> trackers,
                      std::vector<std::pair<int, int>> assignments,
                      std::vector<int> unassigned_tracks,
                      std::vector<int> unassigned_objects, double frame_time);
  void createNewTrackers(std::vector<tracker> trackers);
  void computeAssociateMatrix(
      const std::vector<tracker> &tracks,
      const std::vector<tracker> &new_objects,
      std::vector<std::vector<double>> &association_mat);
  void AssignObjectsToTracks(std::vector<int> tracks_idx,
                             std::vector<int> objects_idx,
                             std::vector<std::vector<double>> cost,
                             std::vector<std::pair<int, int>> *assignments,
                             std::vector<int> *unassigned_tracks,
                             std::vector<int> *unassigned_objects);

  std::vector<tracker> Tracked_object_;          //跟踪物体
  std::vector<tracker> Current_Tracked_object_;  //与当前帧有关的跟踪物体
  double last_frame_time_;
  double odo_time_;
  ros::Publisher marker_pub;
  Eigen::Matrix3f car_pose_;
  double first_gps_x;
  double first_gps_y;
  double first_gps_z;
  bool first_gps;
  int id_;
};
}  // namespace navigation
}  // namespace sweeper

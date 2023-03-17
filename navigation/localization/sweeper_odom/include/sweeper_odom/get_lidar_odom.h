#include <pcl/point_cloud.h>
#include <ros/ros.h>
// msgs type and conversion
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
// pcd io
#include <pcl/io/pcd_io.h>
// point types
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>
// #include "../../loam_livox/src/plane_line_icp.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <time.h>

#include <fstream>
#include <iostream>
#include <string>

#include "../../../../common/log.h"
#include "../../../../common/watch_dog.h"
#include "../../loam_livox/src/point_cloud_matcher.hpp"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweepMission.h"
#include "tf/transform_datatypes.h"

#define un_received_livox_data_ 3108
#define un_received_frist_pose_ 3113
#define un_load_map_ 3116
#define long_time_matcher_error_ 3115
#define frist_matcher_error_ 3114
#define cin_cruise_trace_mode_ 3104
#define cruise_trace_mode_sucess_ 3102
struct Frist_Odom_Pose {
  bool received_odom_pose = false;
  bool receive_odom_pose = false;
  Eigen::Quaterniond frist_pose_quater;
  Eigen::Vector3d frist_pose_position;
  void GetFristPose(
      const geometry_msgs::PoseWithCovarianceStamped receivepose) {
    Eigen::Vector3d angle_rotation(0.0, 0.0, 0.0);  //欧拉角度
    tf::Quaternion quat(receivepose.pose.pose.orientation.x,
                        receivepose.pose.pose.orientation.y,
                        receivepose.pose.pose.orientation.z,
                        receivepose.pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(angle_rotation[2], angle_rotation[1],
                               angle_rotation[0]);
    frist_pose_quater =
        Eigen::AngleAxisd(angle_rotation[0] - M_PI_2,
                          Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd((double)0.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd((double)0.0, Eigen::Vector3d::UnitX());
    // frist_pose_quater =
    // Eigen::Quaterniond(receivepose.pose.pose.orientation.w,
    // receivepose.pose.pose.orientation.x, receivepose.pose.pose.orientation.y,
    // receivepose.pose.pose.orientation.z);
    frist_pose_position = Eigen::Vector3d(receivepose.pose.pose.position.y,
                                          -receivepose.pose.pose.position.x,
                                          receivepose.pose.pose.position.z);
    receive_odom_pose = true;
  }
};

struct LidarOdomInformation {
  std::string pcd_name;
  int swpper_mode;
};
class GetLidarOdom {
 public:
  void ParamInit();
  void ReadPcdName();
  void ResetOdomParam();
  void ReceiveFusionOdom(const nav_msgs::Odometry input_odom);
  void ReceiveSweeperMode(sweeper_msgs::SweepMission sweeper_mode_command);
  void ReceiveGpsPosition(const nav_msgs::Odometry input_odom);
  void FristOdomHander(
      const geometry_msgs::PoseWithCovarianceStamped receivepose);
  void MatcherCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
  void GetSweeperInformation(const ros::TimerEvent &e);
  bool InitializeLidarPosition();
  void LoadMapInformation();
  void TransformationCoordinate(pcl::PointCloud<pcl::PointXYZI> &raw_cloud);
  void ResetLidarOdomParam();
  void InitLidarOdom();

  GetLidarOdom(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~GetLidarOdom();

 private:
  int lidar_odom_error_times_;
  Eigen::Quaterniond fusion_quater_;
  Eigen::Vector3d fusion_odom_;
  bool receive_new_fusion_odom_;
  ros::Time fusion_time_;
  bool showed_map_;
  ros::Publisher lidar_odom_pub_, cloud_map_pub_, pub_frist_odom_;
  ros::Subscriber sub_livox_lidar_, sub_frist_odom_, sub_fusion_odom_;
  ros::Subscriber sub_sweeper_mode_, sub_gps_position_;
  ros::Timer timer_lidar_odom_;
  ros::Publisher pub_state_information_;
  PointCloudMatcher plicp_;
  Frist_Odom_Pose frist_odom_pose_;
  LidarOdomInformation sweeper_information_;
  std::string pcd_name_;
  bool allow_open_lidar_odom_;
  bool init_lidar_odom_;
  bool load_map_success_;
  bool receive_frist_pose_;
  bool cruise_trace_mode_;
  bool receive_pcd_name_sucessed_;
  bool frist_matcher_;
  int frist_matcher_score_;
  std::string get_pcd_name_;
  std::string last_pcd_name_;
  int receive_sweeper_mode_;
  int icp_max_times_;
  bool reset_param_;

  pcl::VoxelGrid<PointType> down_size_filter_corner_;
  pcl::VoxelGrid<PointType> down_size_filter_surf_;
  pcl::VoxelGrid<PointType> down_size_filter_intensity_;

  Eigen::Quaterniond init_quater_;
  Eigen::Vector3d init_pose_;
  sweeper::common::WatchDog watch_dog_livox_;
  // sweeper::common::WatchDog watch_dog_gps_;

  std::vector<std::string> maps_name_;
  std::vector<double> gps_points_x_;
  std::vector<double> gps_points_y_;
};
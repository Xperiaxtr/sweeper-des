#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <fstream>
#include <iostream>
#include <thread>
#include <utility>

#include "get_feature/get_feature.hpp"
#include "mapping/front_end/front_end.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/feature.hpp"
#include "sensor_data/key_frame.hpp"
#include "sweeper_msgs/SweeperChassisDetail.h"
#include "tf/transform_datatypes.h"

using namespace mapping;
using namespace std;

#define CAR_FRONT_DIS_ 0.60
#define CAR_BACK_DIS_ 0.85

class SaveCalibData {
 public:
  struct SweeperInformation {
    double time;
    double distance;
    Eigen::Quaterniond diff_quater;
  };
  struct ImuInformation {
    double time;
    // Eigen::Quaterniond diff_quater;
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
  };
  struct GNSSData {
    // Eigen::Matrix4d gnss_pose;
    Eigen::Quaterniond gnss_quater;
    Eigen::Vector3d gnss_vector;
    double gnss_time;
  };

 private:
  GetFeature *get_feature_;
  FrontEnd *front_end_;
  std::deque<GNSSData> gnss_deque_data_;
  std::deque<SweeperInformation> sweeper_information_deque_;
  std::deque<ImuInformation> imu_information_deque_;
  std::vector<Eigen::Matrix4f> lidar_pose_deque_;
  Eigen::Quaterniond predict_quater_, now_quater_;
  Eigen::Vector3d predict_pos_, now_pos_;
  double lidar_time_;
  std::string key_frames_path_;
  int save_index_;
  bool flag_frist_gnss_;
  Eigen::Vector3d fixed_gnss_point_;

  pcl::VoxelGrid<CloudData::POINT> map_filter_;

  std::mutex predict_mutex_, save_mutex_,gnss_pose_mutex_, sweeper_mutex_, imu_mutex_;

  ros::Subscriber sub_gnss_data_, sub_livox_data_, sub_sweeper_data_,
      sub_imu_data_;

  std::shared_ptr<CloudPublisher> corner_feature_pub_ptr_;
  std::shared_ptr<CloudPublisher> surf_feature_pub_ptr_;
  std::shared_ptr<CloudPublisher> intensity_feature_pub_ptr_;
  std::shared_ptr<CloudPublisher> corner_bag_pub_ptr_;
  std::shared_ptr<CloudPublisher> map_pub_ptr_;
  std::shared_ptr<OdometryPublisher> odom_pub_ptr_;
  std::shared_ptr<OdometryPublisher> vehical_pub_ptr_;

 private:
  void ReceiveImuData(const sensor_msgs::Imu::ConstPtr &imu_msg_ptr);
  void ReceiveSweeperInformation(
      const sweeper_msgs::SweeperChassisDetail &sweeper_chassis_detail);
  void ReceiveGnssData(const nav_msgs::OdometryConstPtr &receive_data);
  void ReceiveLidarData(const sensor_msgs::PointCloud2::ConstPtr &input);
  void CloudAdjustMotion(const Eigen::Matrix4d &tran_pose,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud_ptr);
  void PredictLidarPose();
  void AddCloudToFile(const CloudData::CLOUD_PTR &cloud_ptr,
                      const int &key_frame_index);

  void SaveCalibPoseToFile(const Eigen::Quaterniond &lidar_quater,
                           const Eigen::Quaterniond &gnss_quater,
                           const Eigen::Vector3d &lidar_pos,
                           const Eigen::Vector3d &gnss_pos);
  void SaveMapCloud();

  bool ResetParam();
  int RemoveDir(const char *dir);

 public:
  void MapStructNode();
  void MatcherNode();

 public:
  SaveCalibData(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~SaveCalibData();
};

#ifndef MAPPING_FLOW_MAPPING_FLOW_HPP_
#define MAPPING_FLOW_MAPPING_FLOW_HPP_

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <fstream>
#include <string>

#include "../../../../common/log.h"
#include "../../../../common/watch_dog.h"
#include "get_feature/get_feature.hpp"
#include "mapping/back_end/back_end.hpp"
#include "mapping/front_end/front_end.hpp"
#include "mapping/loop_closing/loop_closing.hpp"
#include "mapping/save_view_map/save_view_map.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "sensor_data/back_data.hpp"
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/feature.hpp"
#include "sensor_data/gnss_data.hpp"
#include "sensor_data/key_frame.hpp"
#include "sensor_data/odom_data.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/vehical_subscriber.hpp"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweepMission.h"
#include "tools/file_manager/file_manager.hpp"
#include "tools/math_calculation.hpp"

#define MAPPING_FUCTION_NORMAL_ 3101
#define ENTER_MAPPING_FUCTION_ 3103
#define UN_RECEIVE_GNSS_DATA_ 3107
#define UN_RECEIVE_LIVOX_DATA_ 3108
#define MAKE_FILE_FAIL 3111
#define SAVE_DATA_FAIL_ 3112
#define CAR_FRONT_DIS_ 0.60
#define CAR_BACK_DIS_ 0.85

namespace mapping {
class MappingFlow /*  : public LoopClosing */ {
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

 public:
  MappingFlow(ros::NodeHandle nh, ros::NodeHandle nh_private);
  void Run();
  void ViewGlobalMap();
  void OptimizeKeyFrame();
  void ResetMappingParam();
  void ReceiveMappingInformation(
      sweeper_msgs::SweepMission sweeper_mode_command);
  void ReceiveImuData(const sensor_msgs::Imu::ConstPtr &imu_msg_ptr);
  void ReceiveSweeperInformation(
      const sweeper_msgs::SweeperChassisDetail &sweeper_chassis_detail);
  void ReceiveVehicalData(
      const sweeper_msgs::SweeperChassisDetail sweeper_chassis_detail);
  void GetMappingStateCode();
  void GetLoopClosure();
  void AddCloudToFile(const Feature &now_feature, const int &key_frame_index);
  bool InitMapppingParam();
  void PublishMappingState();
  void FrontNode();
  void PredictLidarPose();
  void BackEndFlow();
  void PublishBackPath();
  double CalDistance(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2);
  void VisualizeLoopClosure();
  void CloudAdjustMotion(const Eigen::Matrix4d &tran_pose,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud_ptr);

 public:
  BackEnd back_end_;
  // BackEnd *back_end_;
  FrontEnd *front_end_;
  KeyFrame last_key_frame_;
  SaveAndViewMap *save_and_view_map_;
  LoopClosing *loop_closing_;
  // SaveAndViewMap save_and_view_map_;
  std::vector<KeyFrame> key_frames_;
  // KeyFrame last_key_frame_;
  std::deque<GNSSData> unsynced_gnss_;
  std::deque<IMUData> unsynced_imu_;
  std::string key_frames_path_;
  // std::deque<IMUData> unsynced_imu_;
  // std::deque<VelocityData> unsynced_velocity_;

 private:
  std::deque<SweeperInformation> sweeper_information_deque_;
  std::deque<ImuInformation> imu_information_deque_;
  std::vector<BackData> back_datas_;

  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
  std::shared_ptr<VehicalSubscriber> vehical_sub_ptr_;
  // std::shared_ptr<SweepModeSubscriber> sweep_mode_sub_ptr_;
  ros::Subscriber sub_mapping_information_, sub_vehical_information_,
      sub_imu_information_;

  std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> corner_bag_pub_ptr_;
  std::shared_ptr<CloudPublisher> corner_feature_pub_ptr_;
  std::shared_ptr<CloudPublisher> surf_feature_pub_ptr_;
  std::shared_ptr<CloudPublisher> intensity_feature_pub_ptr_;
  std::shared_ptr<OdometryPublisher> odom_pub_ptr_;
  std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
  std::shared_ptr<OdometryPublisher> vehical_odom_pub_ptr_;
  std::shared_ptr<OdometryPublisher> map_odom_pub_ptr_;
  ros::Publisher pub_state_information_, pub_optimize_path_,
      pub_loop_constraints_;
  image_transport::Publisher pub_map_image_;
  GetFeature *get_feature_;

  // SweepModeSubscriber::SweepMode last_sweep_mode_;

  // odom ,车速, imu相关参数
  std::deque<OdomData> odom_datas_;
  double vehical_last_time_;
  double vehical_speed_last_;
  Eigen::Vector3d last_odom_pos_;
  OdomData last_odom_data_;
  Eigen::Matrix4d frist_odom_data_;
  bool frist_odom_ = true;

  sweeper::common::WatchDog watch_dog_lidar_;
  sweeper::common::WatchDog watch_dog_gnss_;
  std::string save_pcd_name_;
  std::string save_line_name_;
  std::string save_dir_name_;

  Eigen::Vector3d gnss_origin_pos_;
  Eigen::Vector3d gnss_to_lidar_pos_;
  Eigen::Quaterniond gnss_to_lidar_quater_;

  Eigen::Vector3d lidar_to_vehical_pos_;
  Eigen::Quaterniond lidar_to_vehical_quater_;
  // std::string key_frames_path_;
  int save_now_pcd_num_;
  int last_mapping_state_;

  bool need_init_mapping_;
  bool init_mapping_;
  bool allow_mapping_mode_;
  bool make_file_sucess_;
  bool save_data_sucess_;
  bool enter_mapping_mode_;
  bool using_loop_;
  bool using_gps_data_;
  bool using_imu_data_;
  bool pub_global_map_;
  bool use_loop_closure_;
  int icp_times_;
  bool use_small_map_mode_;
  int point_attribute_;
  double lidar_time_;
  Eigen::Quaterniond now_quater_, predict_quater_;
  Eigen::Vector3d now_pos_, predict_pos_, last_gps_pos_;
  std::mutex predict_mutex_, back_mutex_, view_mutex_;
  bool flag_gps_to_map_;
  Eigen::Matrix4d gps_to_map_, fixed_pose_;
  Eigen::Matrix4d lidar_to_gps_;
  int key_frame_index_;
  Eigen::Matrix4d odom_to_map_, prev_lidar_odom_;
  std::vector<int> loop_points_;
  std::vector<int> gps_load_points_;
  std::vector<Eigen::Matrix4d> loop_trans_;
  cv::Mat jpg_map_pub_;
};
}  // namespace mapping

#endif
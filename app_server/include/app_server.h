#pragma once

#include <arpa/inet.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <jsoncpp/json/json.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <tf/transform_broadcaster.h>
#include <time.h>

#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "../../common/frame_transform.h"
#include "../../common/gps_convert.h"
#include "../../common/log.h"
#include "../../common/stream/tcp_stream.h"
#include "../../common/watch_dog.h"
#include "sweeper_msgs/Point.h"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/StateReport.h"
#include "sweeper_msgs/SweepMission.h"
#include "yaml-cpp/yaml.h"

#define PI 3.1415926
#define RAD_TO_DEGREE (180.0 / PI)
#define DEGREE_TO_RAD (PI / 180.0)

using namespace std;

namespace sweeper {
namespace app_server {
typedef std::vector<std::vector<cv::Point>> ContoursVector;

class AppServer {
 public:
  AppServer(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  ~AppServer();

  bool Init();
  void Stop();
  void Reset();

 private:
  bool ConnectServer();
  bool IsJsonData(std::string strData);
  bool SendData(const std::string &str, bool gps_flag = false);
  void TcpConnectThread();
  void GpsFixCallback(const sensor_msgs::NavSatFix &msg);
  void ImagePoseCallback(const nav_msgs::Odometry &msg);
  void StateReportCallback(const sweeper_msgs::StateReport &msg);
  void PlanningImgCallback(const sensor_msgs::ImagePtr &msg);
  void MappingImgCallback(const sensor_msgs::ImagePtr &msg);
  void CameraFrontCallback(const sensor_msgs::ImagePtr &msg);
  void SelfDiagnose(const ros::TimerEvent &event);
  void ParseCmdData(const std::string &data);
  void GetFiles(std::string path, std::vector<std::string> &files);
  int RemoveDir(const char *dir);
  std::vector<std::string> SplitString(const std::string &str,
                                       const std::string &delim);

  bool reset_flag_;

  int socket_fd_;
  int port_;
  int gps_count_;
  int side_;
  int state_;
  int heart_count_;
  int show_num_;
  int image_pose_count_;

  float resize_rate_;

  struct sockaddr_in addr_;

  std::string car_name_;
  std::string line_path_;
  std::string map_name_;
  std::string ip_;
  std::string show_img_;
  std::string txt_name_;
  sweeper::WGS84Corr gps_fix_;

  ros::Subscriber gps_fix_sub_;
  ros::Subscriber state_report_sub_;
  ros::Subscriber planning_img_sub_;
  ros::Subscriber mapping_img_sub_;
  ros::Subscriber image_pose_sub_;
  ros::Subscriber camera_front_sub_;

  ros::Publisher sweep_mission_pub_;
  ros::Publisher state_code_pub_;

  ros::Timer state_code_timer_;

  GpsConvert gps_convert_;
  sweeper::FrameTransform gps_utm_trans_;
  sweeper::common::TcpStream *tcp_stream_;
};
}  // namespace app_server
}  // namespace sweeper

#pragma once

#include <arpa/inet.h>
#include <nav_msgs/Odometry.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <serial/serial.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <tf/transform_broadcaster.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "../../../common/frame_transform.h"
#include "../../../common/log.h"
#include "../../../common/watch_dog.h"
#include "sweeper_msgs/SensorFaultInformation.h"

#define PI 3.1415926
#define TORAD PI / 180.0
#define ANGLE_SCALE_FACTOR (3.0517578125*1e-5L)
#define SPEED_SCALE_FACTOR (3.74094009399414*1e-6L)

namespace sweeper {
namespace gnss {
using namespace std;
using namespace ros;
class Gnss {
 public:
  Gnss(ros::NodeHandle &node, ros::NodeHandle &private_nh);
  ~Gnss();

  bool Init();

 private:
  int TcpClient(const std::string &addr, const int port,
                struct sockaddr_in *servaddr);
  void GpsToOdom(const sensor_msgs::NavSatFix &navSat,
                 nav_msgs::Odometry &odom);
  void ParseNtripData(std::string info);
  int SendBaseToBynav();
  int RecieveGnssData();
  void DiffDataForward();
  void ParseGpgga(std::string &data, std::vector<std::string> &list);
  void ParseInspvaxa(std::string &data);
  void ParseRawimusa(std::string &data);
  void JudgeAccuracyLevel();
  void PublishOdom();
  void SelfDiagnose();
  std::string encode_base64(const std::string &in);
  std::vector<std::string> SplitNtripMsg(const std::string &str,
                                         const std::string &delim);

  int base_station_port_, bynav_send_port_, bynav_receive_port_;
  double install_angle_;
  double error_position_;
  int flag_position_accuracy_;
  bool flag_diff_write_, flag_cus_write_, flag_cus_read_;
  bool flag_longtitude_error_;
  bool recieve_base_data_flag_;
  bool connect_base_flag_, connect_bynav_receive_flag_,
      connect_bynav_send_flag_;
  bool initialized_;
  bool recieve_gpgga_flag_;
  bool recieve_inspvaxa_flag_;

  double yaw_, roll_, pitch_;
  double yaw_dev_, roll_dev_, pitch_dev_;

  int pos_satellite_num_, pos_state_;
  int sockfd_bynav_receive_, sockfd_bynav_send_, socketfd_base_;
  float pos_diff_edge_;

  std::string base_station_ip_;
  std::string bynav_ip_;
  std::string user_;
  std::string password_;
  std::string login_data_;
  std::string gnss_gpgga_;
  std::string gnss_mode_;
  std::string protocol_;
  std::string solve_state_;
  sensor_msgs::NavSatFix nav_sat_fix_;
  nav_msgs::Odometry odom_gps_, localization_gps_;
  sweeper::FrameTransform gps_transform_;
  sweeper::common::WatchDog watch_dog_reci_ctrl_, watch_dog_reci_all_, watch_dog_reci_imu_, watch_dog_reci_diff_, watch_dog_write_diff_;
  ros::Publisher gps_fix_pub_, gps_odom_pub_, gnss_diagnose_pub_, imu_raw_pub_, gps_localization_pub_;
  geometry_msgs::Quaternion localization_gps_quaternion_;
  int counts_;
  double pub_gpgga_rate_;
};

}  // namespace gnss
}  // namespace sweeper

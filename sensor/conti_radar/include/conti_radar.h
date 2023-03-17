#pragma once

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "pose_util.h"
#include "../common/can_common/socket_can_client.h"
#include "../common/can_common/type.h"
#include "../common/log.h"
#include "../common/watch_dog.h"
#include "sweeper_msgs/RadarFormation.h"
#include "sweeper_msgs/RadarObject.h"
#include "sweeper_msgs/SensorFaultInformation.h"

#define PI 3.1415926
#define TORAD PI / 180.0

namespace sweeper {
namespace sensor {
struct SetRadarParam {
  bool max_distance_valid;
  bool sensor_id_valid;
  bool radar_power_valid;
  bool output_type_valid;
  bool send_quality_valid;
  bool send_ext_info_valid;
  bool sort_index_valid;
  bool store_in_nvm_valid;
  bool ctrl_relay_valid;
  bool rcs_threshold_valid;

  double max_distance;
  int sensor_id;
  int output_type;
  int radar_power;
  int ctrl_relay;
  int send_ext_info;
  int send_quality;
  int sort_index;
  int store_in_nvm;
  int rcs_threshold;
};

struct RadarConfig {
  double max_distance;
  int sensor_id;
  int output_type;
  int radar_power;
  int send_ext_info;
  int send_quality;
  int sort_index;
  int rcs_threshold;
};

class ContiRadar {
 public:
  ContiRadar(ros::NodeHandle &node, ros::NodeHandle &private_nh);
  ~ContiRadar();

  bool Init();

  std::vector<std::string> StringSplit(const std::string &str,
                                       const std::string &delim);

 private:
  bool ConfigureRadar(const SetRadarParam &set_radar_config);

  bool CompareRadarConfig(const RadarConfig &receive_config,
                          const SetRadarParam &set_radar_config);

  void ConfigUpdate(uint8_t *datas, unsigned int length);

  void Receive60AMessages(const uint8_t *datas, int &obj_nums);

  void Receive60BMessages(const uint8_t *datas,
                          sweeper_msgs::RadarFormation &radar_formation);

  void Receive60CMessages(const uint8_t *datas,
                          sweeper_msgs::RadarFormation &radar_formation);

  void Receive60DMessages(const uint8_t *datas,
                          sweeper_msgs::RadarFormation &radar_formation);

  void RecvRadarDatas();

  void PublishRadarDatas(sweeper_msgs::RadarFormation &radar_formation);
  uint8_t SetPosByte(unsigned int pos);

  bool Stop();

  void ContiRadarToImg(const sweeper_msgs::RadarFormation &radar_formation);

  int can_channel_id_;
  int can_filter_nums_;
  int current_sensor_id_;

  int img_width_, img_height_;
  int car_img_position_x_, car_img_position_y_;

  int measures_counter_;

  double lon_max_, lat_max_;

  bool flag_img_show_;
  bool flag_set_;
  bool flag_extrinsic_;

  Eigen::Affine3d conti_to_lidar_extrinsic_;

  sweeper::common::SocketCanClientRaw can_client_;
  ros::Publisher pub_radar_, pub_diagnose_;

  std::string radar_config_file_;
  std::vector<unsigned int> receeive_can_frame_id_;

  SetRadarParam set_conti_radar_params_;
};

}  // namespace sensor
}  // namespace sweeper

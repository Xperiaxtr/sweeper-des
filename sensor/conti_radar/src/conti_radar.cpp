#include "../include/conti_radar.h"

#include <fstream>
#include <opencv2/opencv.hpp>

namespace sweeper {
namespace sensor {
ContiRadar::ContiRadar(ros::NodeHandle &node, ros::NodeHandle &private_nh)
    : flag_extrinsic_(false) {
  private_nh.param<int>("can_filter_nums", can_filter_nums_, 5);  //不能小于１
  private_nh.param<int>("can_channel_id", can_channel_id_, 1);
  private_nh.param<int>("current_sensor_id", current_sensor_id_, 1);
  private_nh.param<int>("img_width", img_width_, 1000);
  private_nh.param<int>("img_height", img_height_, 1000);
  private_nh.param<int>("car_immg_position_x", car_img_position_x_, 500);
  private_nh.param<int>("car_immg_position_y", car_img_position_y_, 0);
  private_nh.param<double>("lon_max", lon_max_, 10);
  private_nh.param<double>("lat_max", lat_max_, 5);
  private_nh.param<bool>("flag_img_show", flag_img_show_, false);
  private_nh.param<bool>("flag_set", flag_set_, false);

  std::string conti_to_lidar_extrinsic_path =
      "../sweeper_ws/src/sweeper_haide/calibration/data/"
      "conti_to_lidar.yaml";
  if (!sweeper::common::LoadExtrinsic(conti_to_lidar_extrinsic_path,
                                      &conti_to_lidar_extrinsic_)) {
    AERROR << "The extrinsic loaded is failure !";
    flag_extrinsic_ = false;
  } else {
    AINFO << "The extrinsic loaded is successful !.";
    flag_extrinsic_ = true;
  }

  radar_config_file_ =
      "../sweeper_ws/src/sweeper_haide/sensor/conti_radar/config/"
      "radar_config.txt";
  std::fstream radar_config(radar_config_file_, std::ios_base::in);
  if (radar_config) {
    std::string line_str;
    while (getline(radar_config, line_str, '\n')) {
      std::vector<std::string> path_str = StringSplit(line_str, ":");
      if (path_str.size() != 2) continue;
      if (path_str[0] == "max_distance_valid") {
        if (atoi(path_str[1].c_str()) == 1)
          set_conti_radar_params_.max_distance_valid = true;
        else if (atoi(path_str[1].c_str()) == 0)
          set_conti_radar_params_.max_distance_valid = false;
      }
      if (path_str[0] == "output_type_valid") {
        if (atoi(path_str[1].c_str()) == 1)
          set_conti_radar_params_.output_type_valid = true;
        else if (atoi(path_str[1].c_str()) == 0)
          set_conti_radar_params_.output_type_valid = false;
      }
      if (path_str[0] == "radar_power_valid") {
        if (atoi(path_str[1].c_str()) == 1)
          set_conti_radar_params_.radar_power_valid = true;
        else if (atoi(path_str[1].c_str()) == 0)
          set_conti_radar_params_.radar_power_valid = false;
      }
      if (path_str[0] == "rcs_threshold_valid") {
        if (atoi(path_str[1].c_str()) == 1)
          set_conti_radar_params_.rcs_threshold_valid = true;
        else if (atoi(path_str[1].c_str()) == 0)
          set_conti_radar_params_.rcs_threshold_valid = false;
      }
      if (path_str[0] == "send_ext_info_valid") {
        if (atoi(path_str[1].c_str()) == 1)
          set_conti_radar_params_.send_ext_info_valid = true;
        else if (atoi(path_str[1].c_str()) == 0)
          set_conti_radar_params_.send_ext_info_valid = false;
      }
      if (path_str[0] == "send_quality_valid") {
        if (atoi(path_str[1].c_str()) == 1)
          set_conti_radar_params_.send_quality_valid = true;
        else if (atoi(path_str[1].c_str()) == 0)
          set_conti_radar_params_.send_quality_valid = false;
      }
      if (path_str[0] == "sensor_id_valid") {
        if (atoi(path_str[1].c_str()) == 1)
          set_conti_radar_params_.sensor_id_valid = true;
        else if (atoi(path_str[1].c_str()) == 0)
          set_conti_radar_params_.sensor_id_valid = false;
      }
      if (path_str[0] == "sort_index_valid") {
        if (atoi(path_str[1].c_str()) == 1)
          set_conti_radar_params_.sort_index_valid = true;
        else if (atoi(path_str[1].c_str()) == 0)
          set_conti_radar_params_.sort_index_valid = false;
      }
      if (path_str[0] == "store_in_nvm_valid") {
        if (atoi(path_str[1].c_str()) == 1)
          set_conti_radar_params_.store_in_nvm_valid = true;
        else if (atoi(path_str[1].c_str()) == 0)
          set_conti_radar_params_.store_in_nvm_valid = false;
      }
      if (path_str[0] == "ctrl_relay_valid") {
        if (atoi(path_str[1].c_str()) == 1)
          set_conti_radar_params_.ctrl_relay_valid = true;
        else if (atoi(path_str[1].c_str()) == 0)
          set_conti_radar_params_.ctrl_relay_valid = false;
      }

      if (path_str[0] == "max_distance")
        set_conti_radar_params_.max_distance = atof(path_str[1].c_str());
      if (path_str[0] == "output_type")
        set_conti_radar_params_.output_type = atoi(path_str[1].c_str());
      if (path_str[0] == "radar_power")
        set_conti_radar_params_.radar_power = atoi(path_str[1].c_str());
      if (path_str[0] == "rcs_threshold")
        set_conti_radar_params_.rcs_threshold = atoi(path_str[1].c_str());
      if (path_str[0] == "send_ext_info")
        set_conti_radar_params_.send_ext_info = atoi(path_str[1].c_str());
      if (path_str[0] == "send_quality")
        set_conti_radar_params_.send_quality = atoi(path_str[1].c_str());
      if (path_str[0] == "sensor_id")
        set_conti_radar_params_.sensor_id = atoi(path_str[1].c_str());
      if (path_str[0] == "sort_index")
        set_conti_radar_params_.sort_index = atoi(path_str[1].c_str());
      if (path_str[0] == "store_in_nvm")
        set_conti_radar_params_.store_in_nvm = atoi(path_str[1].c_str());
      if (path_str[0] == "ctrl_relay")
        set_conti_radar_params_.ctrl_relay = atoi(path_str[1].c_str());
    }
    radar_config.close();
  } else {
    AERROR << "Error:the config of radar is not existed !";
  }

  // AINFO << "max_distance_valid:" <<
  // set_conti_radar_params_.max_distance_valid; AINFO << "output_type_valid:"
  // << set_conti_radar_params_.output_type_valid; AINFO << "radar_power_valid:"
  // << set_conti_radar_params_.radar_power_valid; AINFO <<
  // "rcs_threshold_valid:"
  //       << set_conti_radar_params_.rcs_threshold_valid;
  // AINFO << "send_ext_info_valid:" <<
  // set_conti_radar_params_.send_quality_valid; AINFO << "sensor_id_valid:" <<
  // set_conti_radar_params_.sensor_id_valid; AINFO << "store_in_nvm_valid:" <<
  // set_conti_radar_params_.store_in_nvm_valid; AINFO << "ctrl_relay_valid:" <<
  // set_conti_radar_params_.ctrl_relay_valid; AINFO << "max_distance:" <<
  // set_conti_radar_params_.max_distance; AINFO << "output_type:" <<
  // set_conti_radar_params_.output_type; AINFO << "radar_power:" <<
  // set_conti_radar_params_.radar_power; AINFO << "send_ext_info:" <<
  // set_conti_radar_params_.send_ext_info; AINFO << "send_quality:" <<
  // set_conti_radar_params_.send_quality; AINFO << "sensor_id:" <<
  // set_conti_radar_params_.sensor_id; AINFO << "sort_index:" <<
  // set_conti_radar_params_.sort_index; AINFO << "store_in_nvm:" <<
  // set_conti_radar_params_.store_in_nvm; AINFO << "ctrl_relay:" <<
  // set_conti_radar_params_.ctrl_relay;

  receeive_can_frame_id_ = {0x201, 0x60A, 0x60B, 0x60C, 0x60D};

  measures_counter_ = -1;

  pub_radar_ =
      node.advertise<sweeper_msgs::RadarFormation>("/sweeper/sensor/radar", 1);
  pub_diagnose_ = node.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/common/diagnose", 1);
  AINFO << " Radar node initialize success!";
}

ContiRadar::~ContiRadar() { can_client_.Stop(); }

bool ContiRadar::Init() {
  can_client_.Init(can_channel_id_);

  if (!can_client_.Start(can_filter_nums_, receeive_can_frame_id_)) {
    AERROR << "Error:socket can created failed !";
    return false;
  }

  std::thread recieve_thread(&ContiRadar::RecvRadarDatas, this);
  recieve_thread.detach();
  return true;
}

void ContiRadar::RecvRadarDatas() {
  AINFO << "Create the receive thread of radar datas !";
  bool flag_setted = false;
  sweeper_msgs::RadarFormation conti_radar_formation;
  int object_nums = 0;

  while (ros::ok()) {
    std::vector<sweeper::common::CanFrame> receive_can_frame;
    int receive_frame_num = 1;

    if (flag_set_ && !flag_setted) {
      uint32_t current_id = current_sensor_id_;
      uint32_t state_frame = 0x201 + current_id * 0x10;
      ConfigureRadar(set_conti_radar_params_);
      usleep(50000);
      flag_setted = true;
      continue;
    } else if (flag_set_ && flag_setted) {
      AINFO << "The parameters of radar is setted!";
    }

    if (!flag_set_ &&
        can_client_.Receive(&receive_can_frame, &receive_frame_num,
                            receeive_can_frame_id_)) {
      uint32_t frame_id = receive_can_frame.front().id;

      switch (frame_id) {
        case 0x60A:
          if (measures_counter_ != -1) {
            ContiRadarToImg(conti_radar_formation);
            PublishRadarDatas(conti_radar_formation);
          }
          Receive60AMessages(&receive_can_frame.front().data[0], object_nums);
          conti_radar_formation.nof_objects = object_nums;
          break;
        case 0x60B:
          Receive60BMessages(&receive_can_frame[0].data[0],
                             conti_radar_formation);
          break;
        case 0x60C:
          Receive60CMessages(&receive_can_frame[0].data[0],
                             conti_radar_formation);
          break;
        case 0x60D:
          Receive60DMessages(&receive_can_frame[0].data[0],
                             conti_radar_formation);
          break;
        case 0x201: {
          RadarConfig receive_radar_config;
          int rec_max_distance_high = receive_can_frame.front().data[1] * 4;
          int rec_max_distance_low = receive_can_frame.front().data[2] >> 6;
          receive_radar_config.max_distance =
              (rec_max_distance_high + rec_max_distance_low) * 2;

          receive_radar_config.output_type =
              (receive_can_frame.front().data[5] & 0x0C) >> 2;
          receive_radar_config.radar_power =
              (receive_can_frame.front().data[4] & 0x80) >> 7;
          receive_radar_config.rcs_threshold =
              (receive_can_frame.front().data[7] & 0x1C) * 4;
          receive_radar_config.send_ext_info =
              (receive_can_frame.front().data[5] & 0x20) >> 5;
          receive_radar_config.send_quality =
              (receive_can_frame.front().data[5] & 0x10) >> 4;
          receive_radar_config.sensor_id =
              (receive_can_frame.front().data[4] & 0x07);
          receive_radar_config.sort_index =
              (receive_can_frame.front().data[4] & 0x70) >> 4;
        } break;
        default:
          AERROR << "Error:the frame id can't identify!";
      }
    }
  }
}

bool ContiRadar::CompareRadarConfig(const RadarConfig &receive_config,
                                    const SetRadarParam &set_radar_config) {
  double distance_error =
      receive_config.max_distance - set_radar_config.max_distance;
  if (fabs(distance_error) < 0.1 &&
      receive_config.output_type == set_radar_config.output_type &&
      receive_config.radar_power == set_radar_config.radar_power &&
      receive_config.rcs_threshold == set_radar_config.rcs_threshold &&
      receive_config.send_ext_info == set_radar_config.send_ext_info &&
      receive_config.send_quality == set_radar_config.send_quality &&
      receive_config.sensor_id == set_radar_config.sensor_id &&
      receive_config.sort_index == set_radar_config.sort_index)
    return true;
  else
    return false;
}

bool ContiRadar::ConfigureRadar(const SetRadarParam &set_radar_config) {
  int send_frame_num = 1;
  std::vector<sweeper::common::CanFrame> send_can_frame;
  sweeper::common::CanFrame tmp_can_frame;

  tmp_can_frame.id = 0x200 + current_sensor_id_ * 0x10;
  AINFO << "current_sensor_id_:" << current_sensor_id_;
  tmp_can_frame.len = 8;
  ConfigUpdate(&tmp_can_frame.data[0], tmp_can_frame.len);
  send_can_frame.push_back(tmp_can_frame);
  for (unsigned int i = 0; i < 8; ++i) {
    AINFO << "i:" << i << " data:" << (int)tmp_can_frame.data[i];
  }
  if (can_client_.Send(send_can_frame, &send_frame_num)) {
    return true;
  } else {
    return false;
  }
}

void ContiRadar::ConfigUpdate(uint8_t *datas, unsigned int length) {
  for (unsigned int i = 0; i < length; ++i) {
    *(datas + i) = SetPosByte(i);
  }
}

uint8_t ContiRadar::SetPosByte(unsigned int pos) {
  uint8_t tmp_value = 0x00;
  sweeper::common::Byte byte(&tmp_value);
  switch (pos) {
    case 0:
      if (set_conti_radar_params_.max_distance_valid) byte.set_bit_1(0);
      if (set_conti_radar_params_.sensor_id_valid) byte.set_bit_1(1);
      if (set_conti_radar_params_.radar_power_valid) byte.set_bit_1(2);
      if (set_conti_radar_params_.output_type_valid) byte.set_bit_1(3);
      if (set_conti_radar_params_.send_quality_valid) byte.set_bit_1(4);
      if (set_conti_radar_params_.send_ext_info_valid) byte.set_bit_1(5);
      if (set_conti_radar_params_.sort_index_valid) byte.set_bit_1(6);
      if (set_conti_radar_params_.store_in_nvm_valid) byte.set_bit_1(7);
      break;
    case 1: {
      int tmp = (int)(set_conti_radar_params_.max_distance / 2);
      uint8_t value = tmp >> 2;
      byte.set_value(value);
      break;
    }
    case 2: {
      int tmp = (int)(set_conti_radar_params_.max_distance / 2);
      uint8_t value = tmp << 6;
      value &= 0xC0;
      byte.set_value(value);
      break;
    }
    case 4: {
      uint8_t value = set_conti_radar_params_.sensor_id;
      byte.set_value(value, 0, 3);

      value = set_conti_radar_params_.output_type;
      byte.set_value(value, 3, 2);

      value = set_conti_radar_params_.radar_power;
      byte.set_value(value, 5, 3);
      break;
    }
    case 5: {
      uint8_t value;
      if (set_conti_radar_params_.ctrl_relay_valid)
        value = 1;
      else
        value = 0;
      byte.set_value(value, 0, 1);

      value = set_conti_radar_params_.ctrl_relay;
      byte.set_value(value, 1, 1);

      value = set_conti_radar_params_.send_quality;
      byte.set_value(value, 2, 1);

      value = set_conti_radar_params_.send_ext_info;
      byte.set_value(value, 3, 1);

      value = set_conti_radar_params_.sort_index;
      byte.set_value(value, 4, 3);

      if (set_conti_radar_params_.store_in_nvm_valid)
        value = 1;
      else
        value = 0;
      byte.set_value(value, 7, 1);
      break;
    }
    case 6: {
      uint8_t value;
      if (set_conti_radar_params_.rcs_threshold_valid)
        value = 1;
      else
        value = 0;
      byte.set_value(value, 0, 1);

      value = set_conti_radar_params_.rcs_threshold;
      byte.set_value(value, 1, 3);
      break;
    }
    case 3:
    case 7:
      break;
    default:
      AERROR << "The pos of setted is error!";
  }
  return byte.get_byte();
}

void ContiRadar::Receive60AMessages(const uint8_t *datas, int &obj_nums) {
  obj_nums = *datas;
  int measures_counter_high = *(datas + 1) * 256;
  int measures_counter_low = *(datas + 2);
  measures_counter_ = measures_counter_high + measures_counter_low;
}

void ContiRadar::Receive60BMessages(
    const uint8_t *datas, sweeper_msgs::RadarFormation &radar_formation) {
  sweeper_msgs::RadarObject radar_obj;
  Eigen::Matrix<double, 4, 1> orign_conti_point;
  Eigen::Matrix<double, 4, 1> calib_conti_point;
  radar_obj.id = *datas;

  int long_dist_high = *(datas + 1) * 32;
  int long_dist_low = *(datas + 2) >> 3;
  radar_obj.lon_dist = (long_dist_high + long_dist_low) * 0.2 - 500;

  int lat_dist_high = (*(datas + 2) & 0x07) * 256;
  int lat_dist_low = *(datas + 3);
  radar_obj.lat_dist = (lat_dist_high + lat_dist_low) * 0.2 - 204.6;

  int lon_vel_high = *(datas + 4) * 4;
  int lon_vel_low = *(datas + 5) >> 6;
  radar_obj.lon_vel = (lon_vel_high + lon_vel_low) * 0.25 - 128;

  int lat_vel_high = (*(datas + 5) & 0x3f) * 8;
  int lat_vel_low = *(datas + 6) >> 5;
  radar_obj.lat_vel = (lat_vel_high + lat_vel_low) * 0.25 - 64;

  radar_obj.ob_dp = *(datas + 6) & 0x07;

  Eigen::Matrix4d transform_conti_to_lidar = conti_to_lidar_extrinsic_.matrix();
  orign_conti_point << radar_obj.lon_dist, radar_obj.lat_dist, 0, 1;
  calib_conti_point = transform_conti_to_lidar * orign_conti_point;

  radar_obj.lon_dist = calib_conti_point(0);
  radar_obj.lat_dist = calib_conti_point(1);

  radar_formation.object.push_back(radar_obj);
}
void ContiRadar::Receive60CMessages(
    const uint8_t *datas, sweeper_msgs::RadarFormation &radar_formation) {
  unsigned int i = 0;
  bool flag_find = false;
  for (; i < radar_formation.object.size(); ++i) {
    int tmp_id = *datas;
    if (radar_formation.object[i].id == tmp_id) {
      flag_find = true;
      break;
    }
  }
  if (!flag_find) return;

  radar_formation.object[i].lon_dist_rms = *(datas + 1) >> 3;

  int lat_dist_rms_high = (*(datas + 1) & 0x07) * 4;
  int lat_dist_rms_low = *(datas + 2) >> 6;
  radar_formation.object[i].lat_dist_rms = lat_dist_rms_high + lat_dist_rms_low;

  radar_formation.object[i].lon_vel_rms = (*(datas + 2) & 0x3f) >> 1;

  int lat_vel_rms_high = (*(datas + 2) & 0x01) * 16;
  int lat_vel_rms_low = *(datas + 3) >> 4;
  radar_formation.object[i].lat_vel_rms = lat_vel_rms_high + lat_vel_rms_low;

  int lon_accr_rms_high = (*(datas + 3) & 0x0f) * 16;
  int lon_accr_rms_low = *(datas + 4) >> 7;
  radar_formation.object[i].lon_accr_rms = lon_accr_rms_high + lon_accr_rms_low;

  radar_formation.object[i].lat_accr_rms = (*(datas + 4) & 0x7C) >> 2;

  int orientation_rms_high = (*(datas + 4) & 0x03) * 8;
  int orientation_rms_low = *(datas + 5) >> 5;
  radar_formation.object[i].orientation_rms =
      orientation_rms_high + orientation_rms_low;

  radar_formation.object[i].prob_exist = *(datas + 6) >> 5;
  radar_formation.object[i].meas_state = (*(datas + 6) & 0x1C) >> 2;
}
void ContiRadar::Receive60DMessages(
    const uint8_t *datas, sweeper_msgs::RadarFormation &radar_formation) {
  unsigned int i = 0;
  bool flag_find = false;
  for (; i < radar_formation.object.size(); ++i) {
    int tmp_id = *datas;
    if (radar_formation.object[i].id == tmp_id) {
      flag_find = true;
      break;
    }
  }
  if (!flag_find) return;

  radar_formation.object[i].ob_class = *(datas + 3) & 0x07;

  int acc_lon_high = (*(datas + 1)) * 8;
  int acc_lon_low = *(datas + 2) >> 5;
  radar_formation.object[i].acc_lon = (acc_lon_high + acc_lon_low) * 0.01 - 10;

  int acc_lat_high = (*(datas + 2) & 0x1F) * 16;
  int acc_lat_low = *(datas + 3) >> 4;
  radar_formation.object[i].acc_lat = (acc_lat_high + acc_lat_low) * 0.01 - 2.5;

  int orien_anle_high = (*(datas + 4)) * 4;
  int orien_anle_low = *(datas + 5) >> 6;
  radar_formation.object[i].orien_anle =
      (orien_anle_high + orien_anle_low) * 0.4 - 180;

  radar_formation.object[i].ob_len = *(datas + 6) * 0.2;
  radar_formation.object[i].ob_width = *(datas + 7) * 0.2;
}

void ContiRadar::PublishRadarDatas(
    sweeper_msgs::RadarFormation &radar_formation) {
  radar_formation.header.frame_id = "conti_radar";
  radar_formation.header.stamp = ros::Time::now();
  pub_radar_.publish(radar_formation);
  radar_formation = sweeper_msgs::RadarFormation();
}

void ContiRadar::ContiRadarToImg(
    const sweeper_msgs::RadarFormation &radar_formation) {
  if (flag_img_show_) {
    cv::Mat conti_img =
        cv::Mat(img_height_, img_width_, CV_8UC1, cv::Scalar(0));
    double lon_size = lon_max_ / img_height_;
    double lat_size = lat_max_ / img_width_;

    int num = radar_formation.nof_objects;

    for (unsigned int i = 0; i < num; ++i) {
      if (radar_formation.object[i].lon_dist <= lon_max_ &&
          fabs(radar_formation.object[i].lat_dist) <= lat_max_) {
        double temp_position_x = radar_formation.object[i].lon_dist / lon_size;
        double temp_position_y = radar_formation.object[i].lat_dist / lat_size;
        temp_position_x = img_height_ - temp_position_x;
        temp_position_y = temp_position_y + img_width_ / 2;

        // conti_img.at<uint8_t>(temp_position_x, temp_position_y) = 255;
        std::string x = std::to_string(radar_formation.object[i].lon_dist);
        x = x.erase(2, 5);

        std::string y = std::to_string(radar_formation.object[i].lat_dist);
        y = y.erase(2, 5);

        const std::string illustrate = x + "," + y;

        cv::putText(conti_img, illustrate,
                    cv::Point(temp_position_y, temp_position_x),
                    cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255), 1, 8);

        cv::circle(conti_img, cv::Point(temp_position_y, temp_position_x), 5,
                   cv::Scalar(255), -1, 8);
      }
    }
    imshow("conti_img", conti_img);
    cv::waitKey(1);
  }
}

std::vector<std::string> ContiRadar::StringSplit(const std::string &str,
                                                 const std::string &delim) {
  std::vector<std::string> res;
  if ("" == str) return res;
  char *strs = new char[str.length() + 1];
  strcpy(strs, str.c_str());

  char *d = new char[delim.length() + 1];
  strcpy(d, delim.c_str());

  char *p = strtok(strs, d);
  while (p) {
    std::string s = p;
    res.push_back(s);
    p = strtok(NULL, d);
  }
  return res;
}
}  // namespace sensor
}  // namespace sweeper
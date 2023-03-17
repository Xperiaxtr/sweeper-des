#ifndef MAPPING_SENSOR_DATA_KEY_FRAME_HPP_
#define MAPPING_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

#include "sensor_data/cloud_data.hpp"
#include "sensor_data/feature.hpp"

namespace mapping {
class KeyFrame {
 public:
  double time = 0.0;
  int index = 0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  int point_attribute = 0;

  bool optimized = false;

  // Feature feature;
  // CloudData::CLOUD key_cloud;

  // Eigen::Quaterniond quater = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  // Eigen::Vector3d position = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d gnss_data = Eigen::Vector3d(0.0, 0.0, 0.0);

 public:
  // Eigen::Quaterniond GetQuaternion();
  // void AddPoseToKeyFrame(const Eigen::Matrix4d &receive_pose);
  // void AddQuaterPoseToKeyFrame(const Eigen::Quaterniond &quat, const
  // Eigen::Vector3d &pos);

  void InputKeyFrame(const Eigen::Matrix4f &input_pose,
                     const Eigen::Vector3d &input_gnss, const int &input_index,
                     const int &input_point_attribute,
                     const double &input_time) {
    pose = input_pose;
    gnss_data = input_gnss;
    time = input_time;
    point_attribute = input_point_attribute;
    index = input_index;
  }
};
}  // namespace mapping

#endif
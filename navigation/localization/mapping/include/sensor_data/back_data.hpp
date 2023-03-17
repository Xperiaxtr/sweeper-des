#ifndef MAPPING_SENSOR_DATA_BACK_DATA_HPP_
#define MAPPING_SENSOR_DATA_BACK_DATA_HPP_

#include <Eigen/Dense>

namespace mapping {
class BackData {
 public:
  // flag
  bool flag_get_gps_data = false;
  bool flag_get_imu_data = false;
  bool flag_get_odom_data = false;
  bool flag_get_plane_data = false;

  // data
  Eigen::Vector3d gps_data;
  Eigen::Matrix4d odom_data;
  int index;

  // cov
  Eigen::Vector3d gps_cov;
  // Eigen::Vector6d odom_cov;
  double odom_cov;

 public:
  void InputOdomData(const Eigen::Matrix4d &input_data,
                     const double &input_cov, const double &time,
                     const int &input_index);
  void InputGpsData(const Eigen::Vector3d &input_data,
                    const Eigen::Vector3d &input_cov);
};
}  // namespace mapping

#endif
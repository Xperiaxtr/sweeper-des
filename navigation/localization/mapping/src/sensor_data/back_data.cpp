#include "sensor_data/back_data.hpp"

namespace mapping {
void BackData::InputOdomData(const Eigen::Matrix4d &input_data,
                             const double &input_cov,
                             const double &time, const int &input_index) {
  odom_data = input_data;
  odom_cov = input_cov;
  flag_get_odom_data = true;
  index = input_index;
}
void BackData::InputGpsData(const Eigen::Vector3d &input_data,
                            const Eigen::Vector3d &input_cov) {
  gps_data = input_data;
  gps_cov = input_cov;
  flag_get_gps_data = true;
}
}  // namespace mapping
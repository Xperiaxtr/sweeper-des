#ifndef MAPPING_SENSOR_DATA_GNSS_DATA_HPP_
#define MAPPING_SENSOR_DATA_GNSS_DATA_HPP_

#include <Eigen/Dense>
#include <deque>

#include "../../../../common/log.h"

namespace mapping {

class GNSSData {
 public:
  double time = 0;
  Eigen::Quaterniond gnss_quater;
  Eigen::Vector3d gnss_position;
  Eigen::Matrix4d gnss_pose;
  Eigen::Matrix<double, 6, 6> gnss_cov;

 public:
  Eigen::Matrix4d OrientationToMatrix();
  static bool SyncData(std::deque<GNSSData>& unsynced_data,
                       GNSSData& synced_data, double sync_time);
};
}  // namespace mapping

#endif
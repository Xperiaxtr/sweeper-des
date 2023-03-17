#ifndef MAPPING_SENSOR_DATA_ODOM_DATA_HPP
#define MAPPING_SENSOR_DATA_ODOM_DATA_HPP

#include <Eigen/Dense>
#include <deque>

#include "../../../../common/log.h"

namespace mapping {
class OdomData {
 public:
  Eigen::Quaterniond quater = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d pos = Eigen::Vector3d(0.0, 0.0, 0.0);
  double time;

 public:
  static bool SyncData(std::deque<OdomData>& unsynced_data,
                       OdomData& synced_data, double sync_time);
};
}  // namespace mapping

#endif
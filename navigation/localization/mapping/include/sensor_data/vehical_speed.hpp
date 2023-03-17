#ifndef MAPPING_SENSOR_DATA_VEHICAL_SPEED_HPP_
#define MAPPING_SENSOR_DATA_VEHICAL_SPEED_HPP_

#include <Eigen/Dense>
#include <deque>

#include "../../../../common/log.h"

namespace mapping {
class VehicalSpeed {
 public:
  double angular;
  double velocity;
  double time;

 public:
  static bool SyncData(std::deque<VehicalSpeed>& unsynced_data, VehicalSpeed& synced_data,
                       double sync_time);
};
}  // namespace mapping

#endif
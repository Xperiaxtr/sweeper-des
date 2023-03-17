#ifndef MAPPING_SENSOR_DATA_IMU_DATA_HPP_
#define MAPPING_SENSOR_DATA_IMU_DATA_HPP_

#include <Eigen/Dense>
#include <deque>

#include "../../../../common/log.h"

namespace mapping {
class IMUData {
 public:
  struct LinearAcc {
    double x;
    double y;
    double z;
  };
  struct AngularVelocaity {
    double x;
    double y;
    double z;
  };
  double time;
  Eigen::Quaterniond imu_quater;
  LinearAcc linear_acc;
  AngularVelocaity angular_velocity;

 public:
  static bool SyncData(std::deque<IMUData>& unsynced_data, IMUData& synced_data,
                       double sync_time);
};
}  // namespace mapping

#endif
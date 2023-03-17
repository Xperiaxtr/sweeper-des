#include "sensor_data/vehical_speed.hpp"

namespace mapping {
bool VehicalSpeed::SyncData(std::deque<VehicalSpeed>& unsynced_data, VehicalSpeed& synced_data,
                       double sync_time) {
  // imu只需要找到最近一帧即可
  while (unsynced_data.size() > 0) {
    if (fabs(unsynced_data.front().time - sync_time) < 0.1) {
      synced_data = unsynced_data.at(0);

    //   double imu_roll, imu_pitch, imu_yaw;
    //   tf::Quaternion imu_orientation(
    //       synced_data.imu_quater.x(), synced_data.imu_quater.y(),
    //       synced_data.imu_quater.z(), synced_data.imu_quater.w());
    //   tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

    //   AWARN << "imu yaw pitch roll : " << imu_yaw * 57.3<< " " << imu_pitch * 57.3<< " "
    //         << imu_roll * 57.3;
    
      return true;
    } else {
      unsynced_data.pop_front();
    }
  }
  return false;
}
}  // namespace mapping
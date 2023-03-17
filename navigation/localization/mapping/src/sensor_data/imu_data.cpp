#include "sensor_data/imu_data.hpp"

#include <tf/transform_broadcaster.h>
namespace mapping {
bool IMUData::SyncData(std::deque<IMUData>& unsynced_data, IMUData& synced_data,
                       double sync_time) {
  /*
  // imu只需要找到最近一帧即可
  while (unsynced_data.size() > 0) {
    if (fabs(unsynced_data.front().time - sync_time) < 0.05) {
      synced_data = unsynced_data.at(0);

      double imu_roll, imu_pitch, imu_yaw;
      tf::Quaternion imu_orientation(
          synced_data.imu_quater.x(), synced_data.imu_quater.y(),
          synced_data.imu_quater.z(), synced_data.imu_quater.w());
      tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

      AWARN << "imu yaw pitch roll : " << imu_yaw * 57.3<< " " << imu_pitch
  * 57.3<< " "
            << imu_roll * 57.3;
      return true;
    } else {
      unsynced_data.pop_front();
    }
  }
  return false;
  */
  while (unsynced_data.size() >= 2) {
    if (unsynced_data.front().time > sync_time) {
      synced_data = unsynced_data.front();
      if (synced_data.time - sync_time < 0.05)
        return true;
      else
        return false;
    }
    if (unsynced_data.at(1).time < sync_time) {
      unsynced_data.pop_front();
      continue;
    }
    if (sync_time - unsynced_data.front().time > 0.05) {
      unsynced_data.pop_front();
      continue;
    }
    if (unsynced_data.at(1).time - sync_time > 0.1) {
      return false;
    }
    break;
  }
  if (unsynced_data.size() < 2) {
    if (fabs(unsynced_data.front().time - sync_time) < 0.05) {
      synced_data = unsynced_data.front();
      return true;
    } else {
      return false;
    }
  }

  IMUData front_data = unsynced_data.at(0);
  IMUData back_data = unsynced_data.at(1);
  double front_scale =
      (back_data.time - sync_time) / (back_data.time - front_data.time);
  // double back_scale =
  //     (sync_time - front_data.time) / (back_data.time - front_data.time);

  synced_data.imu_quater = front_data.imu_quater.slerp(front_scale, back_data.imu_quater);
  return true;
}
}  // namespace mapping
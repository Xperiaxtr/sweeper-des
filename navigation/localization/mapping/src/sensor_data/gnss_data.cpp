
#include "sensor_data/gnss_data.hpp"

namespace mapping {
bool GNSSData::SyncData(std::deque<GNSSData>& unsynced_data,
                        GNSSData& synced_data, double sync_time) {
  while (unsynced_data.size() > 2) {
    if (unsynced_data.front().time > sync_time) {
      synced_data = unsynced_data.front();
      return false;
    }
    if (unsynced_data.at(1).time < sync_time && unsynced_data.size() > 2) {
      unsynced_data.pop_front();
      continue;
    }
    if (sync_time - unsynced_data.front().time > 0.2 &&
        unsynced_data.size() > 1) {
      unsynced_data.pop_front();
      break;
    }
    if (unsynced_data.at(1).time - sync_time > 0.2 &&
        unsynced_data.size() > 1) {
      unsynced_data.pop_front();
      break;
    }
    break;
  }
  if (unsynced_data.size() < 2) {
    if (unsynced_data.size() > 0) synced_data = unsynced_data.at(0);
    return false;
  }

  AWARN << "Gnss sync size: " << unsynced_data.size();

  GNSSData front_data = unsynced_data.at(0);
  GNSSData back_data = unsynced_data.at(1);
  synced_data.time = sync_time;

  AWARN << "gnss front cov: " << front_data.gnss_cov(0, 0) << " "
        << front_data.gnss_cov(0, 0) << " " << front_data.gnss_cov(1, 1) << " "
        << front_data.gnss_cov(2, 2) << " " << front_data.gnss_cov(3, 3) << " "
        << front_data.gnss_cov(4, 4) << " " << front_data.gnss_cov(5, 5);

  AWARN << "gnss back cov: " << back_data.gnss_cov(0, 0) << " "
        << back_data.gnss_cov(0, 0) << " " << back_data.gnss_cov(1, 1) << " "
        << back_data.gnss_cov(2, 2) << " " << back_data.gnss_cov(3, 3) << " "
        << back_data.gnss_cov(4, 4) << " " << back_data.gnss_cov(5, 5);

  bool gnss_cov_exceed = false;
  if (front_data.gnss_cov(0, 0) > 0.1 || front_data.gnss_cov(1, 1) > 0.1 ||
      front_data.gnss_cov(2, 2) > 0.1 || front_data.gnss_cov(3, 3) > 1.0 ||
      front_data.gnss_cov(4, 4) > 0.1 || front_data.gnss_cov(5, 5) > 1.0 ||
      back_data.gnss_cov(0, 0) > 0.1 || back_data.gnss_cov(1, 1) > 0.1 ||
      back_data.gnss_cov(2, 2) > 0.1 || back_data.gnss_cov(3, 3) > 1.0 ||
      back_data.gnss_cov(4, 4) > 0.1 || back_data.gnss_cov(5, 5) > 1.0) {
    AWARN << "GNSS cov limit exceeded";
    gnss_cov_exceed = true;
  }

  if (back_data.time < sync_time) {
    synced_data.gnss_quater = back_data.gnss_quater;
    synced_data.gnss_position = back_data.gnss_position;
    AWARN << "gnss time < cloud time ";
    AWARN << "time is : " << sync_time - back_data.time;
    if (!gnss_cov_exceed && sync_time - back_data.time < 0.1) {
      synced_data.gnss_cov = back_data.gnss_cov;
      return true;
    } else
      return false;
  }

  double front_scale =
      (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale =
      (sync_time - front_data.time) / (back_data.time - front_data.time);

  synced_data.gnss_position = front_scale * front_data.gnss_position +
                              back_scale * back_data.gnss_position;
  synced_data.gnss_quater =
      front_data.gnss_quater.slerp(front_scale, back_data.gnss_quater);
  synced_data.gnss_cov = (front_data.gnss_cov + back_data.gnss_cov) / 2.0;
  if (!gnss_cov_exceed)
    return true;
  else
    return false;
}

Eigen::Matrix4d GNSSData::OrientationToMatrix() {
  gnss_pose = Eigen::Matrix4d::Identity();
  gnss_pose.block(0, 0, 3, 3) = gnss_quater.matrix();
  gnss_pose.block(0, 3, 3, 1) = gnss_position;
  return gnss_pose;
}
}  // namespace mapping

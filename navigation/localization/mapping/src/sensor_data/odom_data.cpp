#include "sensor_data/odom_data.hpp"

namespace mapping {
bool OdomData::SyncData(std::deque<OdomData>& unsynced_data,
                        OdomData& synced_data, double sync_time) {
  //找到最近的一帧
  while (unsynced_data.size() > 0) {
    if (fabs(unsynced_data.front().time - sync_time) < 0.05) {
      synced_data = unsynced_data.at(0);
      return true;
    } else if (sync_time - unsynced_data.front().time > 0.05) {
      unsynced_data.pop_front();
    } else /*  if (unsynced_data.front().time - sync_time > 0.05)*/ {
      AWARN << "odom data time > sync_time";
      return false;
    }
  }
  return false;
}

}  // namespace mapping
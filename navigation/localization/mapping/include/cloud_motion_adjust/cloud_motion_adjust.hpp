#ifndef MAPPING_CLOUD_MOTION_ADJUST_CLOUD_MOTION_ADJUST_HPP_
#define MAPPING_CLOUD_MOTION_ADJUST_CLOUD_MOTION_ADJUST_HPP_

/*
点云运动畸变消除:
    包括大疆点云运动消除和16线运动畸变消除
    处理方式都相同，包括计算点云每个点的时间，接收车速信息和点云去畸变三个过程
 */
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/imu_data.hpp"

namespace mapping {
class CloudMotionAdjust {
 public:
 void ReceiveSweepInformation();
 void CalCloudPointTime();
 void CloudAdjustMotion();
};
}  // namespace mapping

#endif
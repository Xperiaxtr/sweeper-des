#ifdef MAPPING_SENSOR_DATA_SENSOR_PRETREAT_DATA_HPP_
#define MAPPING_SENSOR_DATA_SENSOR_PRETREAT_DATA_HPP_

#include <ros/ros.h>
// sensor_data
#include "sensor_data/feature.hpp"
#include "sensor_data/gnss_data.hpp"
#include "sensor_data/imu_data.hpp"

namespace mapping {
class SensorPretreatData {
 public:
 Feature feature;
 GNSSData gnss_data;
 ImuData imu_data;
};
}  // namespace mapping
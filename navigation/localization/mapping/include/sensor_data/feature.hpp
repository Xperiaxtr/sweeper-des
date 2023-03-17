#ifndef MAPPING_SENSOR_DATA_FEATURE_HPP_
#define MAPPING_SENSOR_DATA_FEATURE_HPP_

#include "sensor_data/cloud_data.hpp"

namespace mapping {
class Feature {
 public:
  CloudData::CLOUD_PTR corner_cloud;
  CloudData::CLOUD_PTR surf_cloud;
  CloudData::CLOUD_PTR intensity_cloud;
  CloudData::CLOUD_PTR cloud_raw;
  // CloudData::CLOUD_PTR cloud_map;
  double time;

 public:
  Feature()
      : corner_cloud(new CloudData::CLOUD()),
        surf_cloud(new CloudData::CLOUD()),
        intensity_cloud(new CloudData::CLOUD()),
        cloud_raw(new CloudData::CLOUD())/*,
        cloud_map(new CloudData::CLOUD())*/ {}
  void clear() {
    corner_cloud->clear();
    surf_cloud->clear();
    intensity_cloud->clear();
    cloud_raw->clear();
    // cloud_map->clear();
  }
};
}  // namespace mapping

#endif
#ifndef MAPPING_SENSOR_DATA_CLOUD_DATA_HPP_
#define MAPPING_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mapping {
class CloudData {
 public:
  using POINT = pcl::PointXYZI;
  using CLOUD = pcl::PointCloud<POINT>;
  using CLOUD_PTR = CLOUD::Ptr;

 public:
  CloudData() : cloud_ptr(new CLOUD()) {}

 public:
  double time = 0.0;
  CLOUD_PTR cloud_ptr;
};
}  // namespace mapping

#endif
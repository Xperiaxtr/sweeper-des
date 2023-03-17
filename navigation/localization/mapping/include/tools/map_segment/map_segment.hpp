//从地图中截取一个小地图
#ifndef MAPPING_TOOLS_MAP_SEGMENT_MAP_SEGMENT_HPP_
#define MAPPING_TOOLS_MAP_SEGMENT_MAP_SEGMENT_HPP_

#include <pcl/filters/crop_box.h>

#include "../../../../common/log.h"
#include "sensor_data/cloud_data.hpp"

namespace mapping {
class MapSegment {
 public:
  MapSegment();
  void SetSize(std::vector<double> size);
  bool SetOrigin(Eigen::Vector3d origin);
  void Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
              CloudData::CLOUD_PTR& filtered_cloud_ptr);
  void PrintEdge();

 public:
  bool frist_map_segment_;

 private:
  void CalculateEdge();

 private:
  pcl::CropBox<CloudData::POINT> pcl_box_filter_;

  Eigen::Vector3d origin_;
  std::vector<double> size_;
  std::vector<double> edge_;
};
}  // namespace mapping
#endif
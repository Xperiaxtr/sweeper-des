#include "tools/map_segment/map_segment.hpp"

namespace mapping {
MapSegment::MapSegment() {
  size_.resize(6);
  edge_.resize(6);
  origin_.resize(3);
  frist_map_segment_ = true;
}

void MapSegment::SetSize(std::vector<double> size) {
  size_ = size;
  AWARN << "Box Filter 的尺寸为：" << std::endl
        << "min_x: " << size.at(0) << ", "
        << "max_x: " << size.at(1) << ", "
        << "min_y: " << size.at(2) << ", "
        << "max_y: " << size.at(3) << ", "
        << "min_z: " << size.at(4) << ", "
        << "max_z: " << size.at(5) << std::endl
        << std::endl;
}

bool MapSegment::SetOrigin(Eigen::Vector3d origin) {
  if (fabs(origin.x() - origin_.x()) +
      fabs(origin.y() - origin_.y()) > 50 || frist_map_segment_) {
    origin_ = origin;
    CalculateEdge();
    frist_map_segment_ = false;
    return true;
  } else
  {
    AWARN << "distance : "<<fabs(origin.x() - origin_.x()) +
      fabs(origin.y() - origin_.y())<<" xyz: "<<origin_.x()<<" "<<origin.x()<<" "<<origin_.y()<<" "<<origin.y()<<" "<<origin_.z()<<" "<<origin.z();
    return false;
  }
}

void MapSegment::CalculateEdge() {
  edge_.at(0) = -size_.at(0) + origin_.x();
  edge_.at(1) = size_.at(1) + origin_.x();
  edge_.at(2) = -size_.at(2) + origin_.y();
  edge_.at(3) = size_.at(3) + origin_.y();
  edge_.at(4) = -size_.at(4) + origin_.z();
  edge_.at(5) = size_.at(5) + origin_.z();
}

void MapSegment::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
                        CloudData::CLOUD_PTR& output_cloud_ptr) {
  output_cloud_ptr->clear();
  pcl_box_filter_.setMin(
      Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
  pcl_box_filter_.setMax(
      Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
  pcl_box_filter_.setInputCloud(input_cloud_ptr);
  pcl_box_filter_.filter(*output_cloud_ptr);
}

void MapSegment::PrintEdge() {
  AWARN << "Box 的范围为：" << std::endl
        << "min_x: " << edge_.at(0) << ", "
        << "max_x: " << edge_.at(1) << ", "
        << "min_y: " << edge_.at(2) << ", "
        << "max_y: " << edge_.at(3) << ", "
        << "min_z: " << edge_.at(4) << ", "
        << "max_z: " << edge_.at(5) << std::endl
        << std::endl;
}

}  // namespace mapping
/*
得到点云的特征点， 包括:
强度
角点
平面
*/
#ifndef MAPPING_GET_FEATURE_GET_FEATURE_HPP_
#define MAPPING_GET_FEATURE_GET_FEATURE_HPP_

#include <pcl/filters/voxel_grid.h>
#include <time.h>

#include <deque>

#include "../../../../common/log.h"
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/feature.hpp"

namespace mapping {
class GetFeature {
 public:
  double livox_z_;
  double the_corner_min_curvature_;
  double the_surface_max_curvature_;
  double the_min_hight_;
  double view_max_distance_;
  double view_min_distance_;
  double outlier_min_distance_;
  double the_min_intensity_difference_;
  double the_max_intensity_hight_;
  double the_min_slope_difference_;
  double the_max_z_hight_;
  // Feature new_feature_data_;

 public:
  GetFeature(double livox_hight, double corner_min_curvature,
             double surf_max_curvature, double the_min_hight,
             double intensity_min_different, double min_slope_different,
             double max_z_hight, double corner_filter_size,
             double surf_filter_size, double intensity_filter_size,
             double cloud_filter_size);
  ~GetFeature();
  void GetLivoxScan(const CloudData::CLOUD_PTR &livox_cloud_ptr,
                    std::vector<std::vector<int>> &scans);
  // void CalculatePointFeature(const CloudData::CLOUD_PTR &livox_cloud_ptr,
  //                            std::vector<std::vector<int>> &scans);
  // void RemoveNeighborPicked(const CloudData::CLOUD_PTR &livox_cloud_ptr);
  // void GetCloudFeature(const CloudData &livox_cloud,
  //                      const std::vector<std::vector<int>> &scans,
  //                      Feature &new_feature_data);
  // void GetFeatureParam(double livox_hight, double corner_min_curvature,
  //                      double surf_max_curvature, double the_min_hight,
  //                      double intensity_min_different,
  //                      double min_slope_different, double max_z_hight,
  //                      double corner_filter_size, double surf_filter_size,
  //                      double intensity_filter_size, double cloud_filter_size);
  void CalculatePoint(const CloudData::CLOUD_PTR &livox_cloud_ptr,
                      std::vector<std::vector<int>> &scans,
                      Feature &new_feature_data);

 public:
  void GetLivoxFeature(const CloudData &livox_cloud, Feature &new_feature_data);
  // void ParseData(std::deque<Feature> &cloud_data_buff);

 private:
  float point_curvature_[30000];
  float point_intensity_[30000];
  // float point_slope_[30000];
  float point_distance_[30000];
  float point_max_hight_[30000];
  // float two_point_hight_[30000];
  // int point_neighbor_picked_[30000];
  // int point_segment_[30000];
  // int cloud_point_label_[30000];
  // double time;
  pcl::VoxelGrid<CloudData::POINT> corner_filter_;
  pcl::VoxelGrid<CloudData::POINT> surf_filter_;
  pcl::VoxelGrid<CloudData::POINT> intensity_filter_;
  pcl::VoxelGrid<CloudData::POINT> raw_cloud_filter_;
  pcl::VoxelGrid<CloudData::POINT> map_cloud_filter_;
  // std::deque<Feature> new_feature_data_;
};
}  // namespace mapping

#endif
//本类保存并显示全局地图

#ifndef MAPPING_SAVE_VIEW_MAP_SAVE_VIEW_MAP_HPP_
#define MAPPING_SAVE_VIEW_MAP_SAVE_VIEW_MAP_HPP_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <string.h>
#include <unistd.h>

#include <fstream>
#include <mutex>
// #include <opencv2/opencv.hpp>

#include "../../../../common/log.h"
#include "publisher/cloud_publisher.hpp"
#include "sensor_data/key_frame.hpp"
#include "tools/math_calculation.hpp"

namespace mapping {
class SaveAndViewMap {
 public:
  CloudData::CLOUD_PTR global_map_;

 public:
  SaveAndViewMap(double corner_size, double surf_size, double intensity_size);
  void PublishAndSaveMap(const std::vector<KeyFrame> &key_frames);
  void GetPcdAndLineName(std::string save_file_name);
  bool RenameAndSaveFile(std::string save_file_name);
  void SaveLineToFile(const std::vector<KeyFrame> &key_frames);
  bool ResetParam();
  int RemoveDir(const char *dir);
  void MapToImage(const CloudData::CLOUD_PTR &global_map,
                  const std::vector<KeyFrame> &key_frames);

 private:
  pcl::VoxelGrid<CloudData::POINT> corner_filter_;
  pcl::VoxelGrid<CloudData::POINT> surf_filter_;
  pcl::VoxelGrid<CloudData::POINT> intensity_filter_;
  pcl::VoxelGrid<CloudData::POINT> cloud_filter_;
  std::string save_dir_name_;
  std::string save_pcd_name_;
  std::string save_line_name_;
  std::string save_cloud_name_;
  std::mutex save_mutex_;
  cv::Mat map_jpg_;
};
}  // namespace mapping

#endif
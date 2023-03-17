
#ifndef MAPPING_MAPPING_LOOP_CLOSING_LOOP_CLOSING_HPP_
#define MAPPING_MAPPING_LOOP_CLOSING_LOOP_CLOSING_HPP_

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>

#include <mutex>

#include "../../../../common/log.h"
#include "mapping/back_end/back_end.hpp"
#include "publisher/cloud_publisher.hpp"
#include "sensor_data/back_data.hpp"
#include "sensor_data/key_frame.hpp"
#include "tools/math_calculation.hpp"
// #include

namespace mapping {
class LoopClosing {
 public:
  LoopClosing(int loop_history_search_num, double icp_loop_score,
              double cloud_size, double map_size);
  void ResetLoopParam();
  int PerformLoopClosure(const std::vector<KeyFrame> &key_frames,
                         const BackData &back_data);
  void DetectionLoopClosure(const std::vector<KeyFrame> &key_frames,
                            const BackData &back_data,
                            std::vector<int> &loop_points_,
                            std::vector<Eigen::Matrix4d> &loop_trans,
                            BackEnd &back_end);

 private:
  pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_history_key_poses_;
  int save_pcd_times_;
  int last_sucess_loop_id_;
  int history_frame_id_;
  int loop_history_search_num_;
  int now_key_frame_id_;
  double icp_loop_score_;
  pcl::VoxelGrid<CloudData::POINT> cloud_filter_;
  pcl::VoxelGrid<CloudData::POINT> map_cloud_filter_;
  std::mutex loop_mutex_;
};
}  // namespace mapping

#endif
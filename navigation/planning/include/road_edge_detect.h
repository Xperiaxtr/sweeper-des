#pragma once

#include <math.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "../../../common/log.h"
#include "../../../common/pose_util.h"

namespace sweeper {
namespace navigation {
class RoadEdgeDetect {
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr;

 public:
  RoadEdgeDetect(ros::NodeHandle &private_nh, const double &filter_min_dis,
                 const double &filter_slope, const cv::Point &center);
  ~RoadEdgeDetect(){};

  void CorrectionCloud(const int mode, const int &curb_sweep_mode,
                       const PointCloudPtr &point_cloud_in,
                       PointCloudPtr point_cloud_out, double *height);
  void PointcloudToImg(const PointCloudPtr &cloud_in,
                       const bool &only_trace_flag, cv::Mat *low_obstacle_img,
                       cv::Mat *high_obstacle_img, cv::Mat *road_edge_img,
                       bool *avoid_obstacle_flag);
  void GetRoadEdgeImg(cv::Mat src, cv::Mat dstImage);
  void DilateImg(cv::Mat src, cv::Mat &dst, int width, int hight, int flag = 0);
  Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f before,
                                     Eigen::Vector3f after);
  void RemoveDiscretePoints(cv::Mat src, cv::Mat &dst, int width, int hight);

 private:
  int GetAdaptiveThreshold(cv::Mat src);

  int mode_;
  int curb_sweep_mode_;
  float livox_groud_height_;
  double car_width_;
  double front_dis_;
  double resolution_;
  double road_edge_thre_;
  double low_obstacle_thre_;
  double high_obstacle_thre_;
  double planefit_thre_;
  double abs_ground_height_;
  double filter_min_dis_;
  double filter_slope_;
  double delta_ground_thre_;

  cv::Point center_;
  Eigen::Affine3d livox_car_extrinsic_;
};

}  // namespace navigation
}  // namespace sweeper

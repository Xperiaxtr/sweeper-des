#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <time.h>

#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "../../../common/log.h"
#include "../../../common/watch_dog.h"
#include "bspline.h"
#include "double_circle_method.h"
#include "road_edge_detect.h"
#include "sweeper_msgs/Light.h"
#include "sweeper_msgs/Obu.h"
#include "sweeper_msgs/SweepMission.h"
#include "sweeper_msgs/SweeperChassisDetail.h"
#include "tracker/object.h"
#include "tracker/tracker.h"

#define PI 3.1415926
#define RAD_TO_DEGREE (180.0 / PI)
#define DEGREE_TO_RAD (PI / 180.0)

using namespace std;

namespace sweeper {
namespace navigation {
enum MODE {
  NORMAL_TRACE_MODE,
  MANUAL_MODE,
  RIGHT_CURB_MODE,
  LEFT_CURB_MODE,
  U_TURN_MODE,
  LOW_SPEED_MODE,
  HIGH_SPEED_MODE,
  CROSSING_MODE,
  CROSSING_STRAIGHT_MODE,
  CROSSING_LEFT_MODE,
  CROSSING_RIGHT_MODE,
};
class Planning {
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr;

 public:
  Planning(ros::NodeHandle &node, ros::NodeHandle &private_nh);
  ~Planning();

  bool Init();
  void Run();
  void Stop();
  void Reset();

 private:
  void GetLocalPath();
  void ReceiveGlobalPath(nav_msgs::Path global_path);
  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void TimerCallback(const ros::TimerEvent &event);
  void ChassisCallback(
      const sweeper_msgs::SweeperChassisDetail &sweeprvchassis);
  void ObuCallback(const sweeper_msgs::Obu &obu);
  void SweepModeCallback(const sweeper_msgs::SweepMission &sweep_mission);
  void FilterLine(const std::vector<cv::Point> &line,
                  std::vector<cv::Point> *filter_line);
  void SparseLine(const std::vector<cv::Point> &line,
                  std::vector<cv::Point> *sparse_line);
  void ConvertPathToImg(std::vector<cv::Point> &input_line, int *img_path);
  void PolyFit(const int order, const std::vector<cv::Point> &line,
               std::vector<cv::Point> *line_fit);
  void ContoursTracker(const PointCloud &cloud_in,
                       const ContoursVector &contours);
  void GetRoadEdgePath(const cv::Mat &road_edge_img,
                       ContoursVector *object_contours);
  double GetCross(cv::Point2f *p1, cv::Point2f *p2, const cv::Point &p);
  bool IsPointInMatrix(cv::Point2f *rp, const cv::Point &p);
  void FindDilateContour(const std::vector<cv::Point> input,
                         std::vector<cv::Point> &output_dilate);
  void GetPlanParam(PlanParam *plan_param);
  bool FindStateCode(const int code);
  void GetImgGlobalPath(const std::vector<cv::Point> &path,
                        std::vector<cv::Point> *patch_line);

  bool estop_;                // 紧急停车标志位
  bool traffic_light_stop_;   // 红绿灯停车标志位
  bool initialized_;          //初始化标志位
  bool dis_mutation_flag_;    // 马路沿突变标志位
  bool avoid_obstacle_flag_;  // 马路沿模式下是否能够避障标志位
  bool find_road_edge_flag_;  // 是否找到马路沿标志
  bool lidar_trace_flag_;     // 判断是否为激光寻迹标志
  bool only_trace_flag_;      // 只寻迹不避障
  bool last_have_obstacle_flag_;
  bool trace_good_flag_;

  float livox_ground_height_;
  double speed_;
  double sweep_width_;
  double resolution_;
  double front_dis_;
  double last_time_;
  double obj_expand_y_;
  double straight_distance_;
  double road_edge_thre_;
  double low_obstacle_thre_;
  double high_obstacle_thre_;
  double planefit_thre_;
  double wj_height_;
  double road_edge_thickness_;
  double abs_ground_height_;
  double straight_distance_thre_;
  double slope_;
  double slope_angle_;
  double filter_slop_thre_;
  double last_dis_min_x_;
  double last_dis_min_y_;
  double delta_dis_min_x_;
  double delta_dis_min_y_;
  double e_stop_x_, e_stop_y_;
  double erode_x_, erode_y_;
  double car_width_, car_length_;
  double obstacle_x_, obstacle_y_;

  int default_x_;
  int pause_;
  int mode_;
  int last_mode_;
  int current_type_;
  int order_;
  int fit_mode_;
  int area_thre_;
  int sparse_factor_;

  int start_row_, end_row_;
  int height_img_dilate_;
  int show_img_debug_;
  int curb_sweep_mode_;
  int current_road_type_;
  int last_road_type_;
  int obstacle_expand_width_;
  int left_trans_thre_, right_trans_thre_;
  int height_, width_;

  int obstacle_count_;
  int no_obstacle_count_;
  std::string sweep_frame_id_;

  unsigned int pub_count_;

  ContoursVector road_edge_boundary_;

  cv::Point center_;
  cv::Mat result_img_;

  ros::Publisher path_pub_;
  ros::Publisher state_code_pub_;
  ros::Subscriber point_cloud_sub_, global_path_sub_;
  ros::Subscriber chassis_sub_;
  ros::Subscriber obu_sub_;
  ros::Subscriber sweep_mission_sub_;
  ros::Timer plan_timer;
  image_transport::Publisher planning_img_pub_;

  BSpline bspline_;
  std::vector<cv::Point> img_path_points_center_;
  std::vector<cv::Point> trace_patch_path_;
  std::vector<cv::Point> last_road_edge_points_;
  std::vector<cv::Point> img_road_edge_points_;
  std::vector<std::shared_ptr<Object>> track_objects_;

  RoadEdgeDetect *road_edge_detect_;
  ObjectTracker *object_tracker_;
  DoubleCircleMethod *double_circle_method_;

  sweeper::common::WatchDog watchdog_cloud_;
  sweeper::common::WatchDog watchdog_path_;
  sweeper::common::WatchDog watchdog_obu_;
  sweeper::common::WatchDog watchdog_sweep_mode_;

  sweeper_msgs::Obu obu_msg_;
  sweeper_msgs::SensorFaultInformation state_code_;
};
}  // namespace navigation
}  // namespace sweeper

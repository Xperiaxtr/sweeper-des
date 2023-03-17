#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

#include "../../../../common/log.h"
#include "../../../../common/polygon_util.h"
#include "../../../../common/watch_dog.h"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweeperChassisDetail.h"

#define PI 3.1415926

ros::Publisher fusiong_point_pub_, pub_radar_fusion_state_, path_pub_,filter_box_pub_;

unsigned int count_ = 0;
double left_width_, right_width_;
double front_length_, back_length_;
double publish_time_;

double steering_angle_current_ = 0;

double wj_height_;

double forward_x_;
double dust_filter_x_,dust_filter_y_;
sensor_msgs::PointCloud2 livox_lidar_cloud_;
sensor_msgs::PointCloud wj_radar_rignt_points_;
sensor_msgs::PointCloud wj_radar_left_points_;

sweeper::common::PolygonUtil polygon_util;

sweeper::common::WatchDog watch_dog_radar_right_, watch_dog_radar_left_;

void ReceiveSweeperChassisDatas(
    const sweeper_msgs::SweeperChassisDetail &sweeper_chassis) {
  steering_angle_current_ =
      ((sweeper_chassis.steering_angle_output / 180) * PI);
}

void ReceiveWjLeftDatas(
    const sensor_msgs::PointCloud::ConstPtr &radar_left_cloud) {
  wj_radar_left_points_ = *radar_left_cloud;
  watch_dog_radar_left_.UpdataNow();
}

void ReceiveWjRightDatas(
    const sensor_msgs::PointCloud::ConstPtr &radar_right_cloud) {
  wj_radar_rignt_points_ = *radar_right_cloud;
  watch_dog_radar_right_.UpdataNow();
}

void SelfDiagnose() {
  if (++count_ % UINT_MAX == 0) count_ = 0;
  sweeper_msgs::SensorFaultInformation state_lidar_fusion;

  if (!watch_dog_radar_left_.DogIsOk(5) || !watch_dog_radar_right_.DogIsOk(5)) {
    state_lidar_fusion.state_code.push_back(2304);
  }

  if (!watch_dog_radar_left_.DogIsOk(5)) {
    wj_radar_left_points_.points.clear();
  }

  if (!watch_dog_radar_right_.DogIsOk(5)) {
    wj_radar_rignt_points_.points.clear();
  }

  if (state_lidar_fusion.state_code.empty()) {
    state_lidar_fusion.state_code.push_back(2300);
    state_lidar_fusion.header.frame_id = "radar_fusion";
    state_lidar_fusion.header.stamp = ros::Time::now();
  } else {
    state_lidar_fusion.header.frame_id = "radar_fusion";
    state_lidar_fusion.header.stamp = ros::Time::now();
  }
  if (count_ % 2 == 0) pub_radar_fusion_state_.publish(state_lidar_fusion);
}

bool CropFilterPointCloud(const pcl::PointCloud<pcl::PointXYZI> & input_point_cloud,
                                                        pcl::PointCloud<pcl::PointXYZI> & output_point_cloud ) {
  if(input_point_cloud.empty()){
    AWARN << "input point cloud is empty!";
    pcl::copyPointCloud(input_point_cloud,output_point_cloud);
    return false;
  }
  output_point_cloud.clear();
  for(size_t i = 0; i < input_point_cloud.points.size(); ++i){
    if(abs(input_point_cloud.points[i].x) < dust_filter_x_ &&
        abs(input_point_cloud.points[i].y) < dust_filter_y_) continue;
    output_point_cloud.points.push_back(input_point_cloud.points[i]);
  }
  return true;
}


void LidarDatasFusion(const ros::TimerEvent &e) {
  sensor_msgs::PointCloud left_points;
  sensor_msgs::PointCloud right_points;
  sensor_msgs::PointCloud2 livox_lidar_cloud;
  pcl::PointCloud<pcl::PointXYZI> fusion_points;

  left_points = wj_radar_left_points_;
  right_points = wj_radar_rignt_points_;
  livox_lidar_cloud = livox_lidar_cloud_;

  int right_size = right_points.points.size();
  int left_size = left_points.points.size();

  sweeper::common::PolygonDType ploygon;
  double theta = -steering_angle_current_;
  ploygon.resize(6);
  ploygon[0].x = forward_x_;
  ploygon[0].y = left_width_;
  ploygon[1].x = -front_length_;
  ploygon[1].y = left_width_;
  ploygon[2].x = -front_length_ - back_length_ * cos(theta);
  ploygon[2].y = left_width_ + back_length_ * sin(theta);
  ploygon[3].x = -front_length_ - back_length_ * cos(theta);
  ploygon[3].y = -right_width_ + back_length_ * sin(theta);
  ploygon[4].x = -front_length_;
  ploygon[4].y = -right_width_;
  ploygon[5].x = forward_x_;
  ploygon[5].y = -right_width_;

  for (unsigned int i = 0; i < left_size; ++i) {
    pcl::PointXYZI point;
    point.x = left_points.points[i].x;
    point.y = left_points.points[i].y;
    point.z = -wj_height_;
    if (point.y < -0.01) continue;
    bool filter_flag = polygon_util.IsXyPointIn2dXyPolygon(point, ploygon);
    if (filter_flag) continue;
    fusion_points.push_back(point);
  }

  for (unsigned int i = 0; i < right_size; ++i) {
    pcl::PointXYZI point;
    point.x = right_points.points[i].x;
    point.y = right_points.points[i].y;
    point.z = -wj_height_;
    if (point.y > 0.01) continue;
    bool filter_flag = polygon_util.IsXyPointIn2dXyPolygon(point, ploygon);
    if (filter_flag) continue;
    fusion_points.push_back(point);
  }

  nav_msgs::Path path;
  path.header.frame_id = "livox_frame";
  path.header.stamp = ros::Time::now();

  for (int j = 0; j < ploygon.points.size(); j++) {
    geometry_msgs::PoseStamped poses;
    poses.pose.position.x = ploygon.points[j].x;
    poses.pose.position.y = ploygon.points[j].y;
    poses.pose.position.z = -wj_height_;
    path.poses.push_back(poses);
    if (j == ploygon.points.size() - 1) {
      geometry_msgs::PoseStamped poses1;
      poses1.pose.position.x = ploygon.points[0].x;
      poses1.pose.position.y = ploygon.points[0].y;
      poses1.pose.position.z = -wj_height_;
      path.poses.push_back(poses1);
    }
  }
  /*绘制灰尘过滤范围框*/
  nav_msgs::Path dust_line;
  dust_line.header.frame_id = "livox_frame";
  dust_line.header.stamp = ros::Time::now();
  double point_axi_x,point_axi_y;
  
  for (int j = 0; j < ploygon.points.size(); j++) {
    geometry_msgs::PoseStamped poses;
    poses.pose.position.x = ploygon.points[j].x;
    poses.pose.position.y = ploygon.points[j].y;
    poses.pose.position.z = -wj_height_;
    dust_line.poses.push_back(poses);
    if (j == ploygon.points.size() - 1) {
      geometry_msgs::PoseStamped poses1;
      poses1.pose.position.x = ploygon.points[0].x;
      poses1.pose.position.y = ploygon.points[0].y;
      poses1.pose.position.z = -wj_height_;
      dust_line.poses.push_back(poses1);
    }
  }
  SelfDiagnose();

  /*过滤扫帚周围的单线灰尘水雾*/
  pcl::PointCloud<pcl::PointXYZI> filter_fusion_points;
  CropFilterPointCloud(fusion_points,filter_fusion_points);
  sensor_msgs::PointCloud2 fusion_cloud;
  pcl::toROSMsg(filter_fusion_points, fusion_cloud);
  fusion_cloud.header.frame_id = "livox_frame";
  path_pub_.publish(path);
  fusiong_point_pub_.publish(fusion_cloud);
}

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_INFO;
  FLAGS_v = 1;
}

int main(int argv, char **argc) {
  ros::init(argv, argc, "radar_fusion");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  google::InitGoogleLogging(argc[0]);
  LogSetting();

  nh_private.param("left_width", left_width_, 0.6);
  nh_private.param("right_width", right_width_, 0.6);
  nh_private.param("front_length", front_length_, 1.0);
  nh_private.param("back_length", back_length_, 1.0);
  nh_private.param("publish_time", publish_time_, 0.1);
  nh_private.param("wj_height", wj_height_, 1.406);
  nh_private.param("forward_x", forward_x_, 1.0);

  nh_private.param("dust_filter_x", dust_filter_x_, 0.5);
  nh_private.param("dust_filter_y", dust_filter_y_, 1.0);

  ros::Subscriber sub_wj_radar_left =
      nh.subscribe("/sweeper/sensor/radar_left", 1, ReceiveWjLeftDatas);
  ros::Subscriber sub_wj_radar_right =
      nh.subscribe("/sweeper/sensor/radar_right", 1, ReceiveWjRightDatas);

  ros::Subscriber sub_sweeper_chassis =
      nh.subscribe("/sweeper/chassis/detail", 1, ReceiveSweeperChassisDatas);

  fusiong_point_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/sweeper/sensor/fusion/lidar_radar", 1);
  pub_radar_fusion_state_ = nh.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/common/diagnose", 1);
  path_pub_ = nh.advertise<nav_msgs::Path>("/sweeper/filter_area", 1);
  filter_box_pub_= nh.advertise<nav_msgs::Path>("/sweeper/filter_dust_area", 1);

  ros::Timer timer_syn =
      nh.createTimer(ros::Duration(publish_time_), LidarDatasFusion);

  AINFO << "Start to deal with datas!";

  ros::spin();

  return 0;
}

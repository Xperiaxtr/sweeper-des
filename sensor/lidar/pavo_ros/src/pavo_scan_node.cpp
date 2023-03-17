#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sweeper_msgs/SensorFaultInformation.h>

#include <fstream>
#include <thread>
#include <vector>

#include "../../../../common/log.h"
#include "../../../../common/pose_util.h"
#include "../../../../common/watch_dog.h"
#include "pavo_driver.h"
#include "std_srvs/Empty.h"

#define COMP_NODES (36000)
#define CIRCLE_ANGLE (27000.0)
#define START_ANGLE (4500)

#define DEG2RAD(x) ((x)*M_PI / 180.)

using namespace pavo;
int radar_index_;
pavo::pavo_driver *drv = NULL;
bool flag_load_extrinisic_;
Eigen::Affine3d radar_left_to_straight_extrinsic_,
    radar_right_to_straight_extrinsic_, radar_right_to_left_extrinsic_,
    radar_to_lidar_extrinsic_;

sweeper::common::WatchDog watch_dog_pavo_;
ros::Publisher pub_self_diagnose_;

void publish_msg(ros::Publisher *pub,
                 std::vector<pavo_response_scan_t> &nodes_vec, ros::Time start,
                 double scan_time, std::string frame_id, bool inverted,
                 double angle_min, double angle_max, double min_range,
                 double max_range, bool switch_active_mode, int method) {
  // if (!flag_load_extrinisic_) return;
  watch_dog_pavo_.UpdataNow();
  sensor_msgs::LaserScan scanMsg;
  size_t node_count = nodes_vec.size();
  int counts = node_count * ((angle_max - angle_min) / 270.0f);
  int angle_start = 135 + angle_min;
  int node_start = node_count * (angle_start / 270.0f);

  scanMsg.ranges.resize(counts);
  scanMsg.intensities.resize(counts);

  float range = 0.0;
  float intensity = 0.0;

  for (size_t i = 0; i < counts; i++) {
    range = nodes_vec[node_start].distance * 0.002;
    intensity = nodes_vec[node_start].intensity;
    if ((range > max_range) || (range < min_range)) {
      range = 0.0;
      intensity = 0.0;
    }
    if (!inverted) {
      scanMsg.ranges[i] = range;
      scanMsg.intensities[i] = intensity;
      node_start = node_start + 1;
    } else {
      scanMsg.ranges[counts - 1 - i] = range;
      scanMsg.intensities[counts - 1 - i] = intensity;
      node_start = node_start + 1;
    }
  }
  scanMsg.header.stamp = start;
  scanMsg.header.frame_id = frame_id;
  scanMsg.angle_min = Degree2Radians(angle_min);
  scanMsg.angle_max = Degree2Radians(angle_max);
  scanMsg.angle_increment =
      (scanMsg.angle_max - scanMsg.angle_min) / (double)counts;
  scanMsg.scan_time = scan_time;
  scanMsg.time_increment = scan_time / (double)node_count;
  scanMsg.range_min = min_range;
  scanMsg.range_max = max_range;

  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud cloud;
  projector.projectLaser(scanMsg, cloud);

  sensor_msgs::PointCloud cloud_correct;
  if (radar_index_ == 0) {
    Eigen::Matrix4d transform_radar_left_to_straight =
        radar_left_to_straight_extrinsic_.matrix();
    Eigen::Matrix4d transform_radar_to_lidar =
        radar_to_lidar_extrinsic_.matrix();

    int num_cloud = cloud.points.size();
    for (unsigned int j = 0; j < num_cloud; ++j) {
      geometry_msgs::Point32 cloud_correct_points;
      Eigen::Matrix<double, 4, 1> orign_cloud_point;
      Eigen::Matrix<double, 4, 1> calib_cloud_point;
      double x = cloud.points[j].x;
      double y = cloud.points[j].y;
      if (sqrt(pow(x, 2) + pow(y, 2) > 0.03)) {
        orign_cloud_point << x, y, 0, 1;
        calib_cloud_point =
            transform_radar_left_to_straight * orign_cloud_point;

        calib_cloud_point = transform_radar_to_lidar * calib_cloud_point;

        cloud_correct_points.x = calib_cloud_point(0);
        cloud_correct_points.y = calib_cloud_point(1);
        cloud_correct_points.z = -1.6;
        cloud_correct.points.push_back(cloud_correct_points);
      }
    }
  } else if (radar_index_ == 1) {
    Eigen::Matrix4d transform_radar_right_to_straight =
        radar_right_to_straight_extrinsic_.matrix();
    Eigen::Matrix4d transform_radar_right_to_left =
        radar_right_to_left_extrinsic_.matrix();
    Eigen::Matrix4d transform_radar_to_lidar =
        radar_to_lidar_extrinsic_.matrix();

    int num_cloud = cloud.points.size();

    for (unsigned int j = 0; j < num_cloud; ++j) {
      geometry_msgs::Point32 cloud_correct_points;
      Eigen::Matrix<double, 4, 1> orign_cloud_point;
      Eigen::Matrix<double, 4, 1> calib_cloud_point;
      double x = cloud.points[j].x;
      double y = cloud.points[j].y;
      if (sqrt(pow(x, 2) + pow(y, 2)) > 0.03) {
        orign_cloud_point << x, y, 0, 1;
        calib_cloud_point =
            transform_radar_right_to_straight * orign_cloud_point;

        calib_cloud_point = transform_radar_right_to_left * calib_cloud_point;
        calib_cloud_point = transform_radar_to_lidar * calib_cloud_point;

        cloud_correct_points.x = calib_cloud_point(0);
        cloud_correct_points.y = calib_cloud_point(1);

        cloud_correct_points.z = -1.6;
        cloud_correct.points.push_back(cloud_correct_points);
      }
    }
  }

  cloud_correct.header.stamp = ros::Time::now();

  cloud_correct.header.frame_id = "livox_frame";
  pub->publish(cloud_correct);
}

void SelfDiagnose() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    sweeper_msgs::SensorFaultInformation state_radar;
    std::vector<int> state_code;
    //外参加载失败
    if (!flag_load_extrinisic_) {
      if (radar_index_ == 0)
        state_radar.state_code.push_back(2106);
      else if (radar_index_ == 1)
        state_radar.state_code.push_back(2107);
    }

    //掉线
    if (!watch_dog_pavo_.DogIsOk(5) && flag_load_extrinisic_) {
      if (radar_index_ == 0)
        state_radar.state_code.push_back(2104);
      else if (radar_index_ == 1)
        state_radar.state_code.push_back(2105);
    }

    if (state_radar.state_code.empty()) {
      if (radar_index_ == 0)
        state_radar.state_code.push_back(2100);
      else if (radar_index_ == 1)
        state_radar.state_code.push_back(2101);
    }

    if (radar_index_ == 0) {
      state_radar.header.frame_id = "radar_left";
      state_radar.header.stamp = ros::Time::now();
      pub_self_diagnose_.publish(state_radar);
    } else if (radar_index_ == 1) {
      state_radar.header.frame_id = "radar_right";
      state_radar.header.stamp = ros::Time::now();
      pub_self_diagnose_.publish(state_radar);
    }
    loop_rate.sleep();
  }
}

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_WARNING;
  FLAGS_v = 1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pavo_scan_node");
  std::string frame_id, lidar_ip, host_ip, scan_topic;

  google::InitGoogleLogging(argv[0]);
  LogSetting();

  int lidar_port, host_port;
  bool inverted, enable_motor, switch_active_mode;
  int motor_speed, motor_speed_get;
  int merge_coef;
  double angle_min;
  double angle_max;
  double max_range, min_range;
  int method;

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh("~");

  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
  nh_private.param<std::string>("scan_topic", scan_topic, "scan");
  nh_private.param<double>("angle_max", angle_max, 135.00);
  nh_private.param<double>("angle_min", angle_min, -135.00);
  nh_private.param<double>("range_max", max_range, 20.0);
  nh_private.param<double>("range_min", min_range, 0.10);
  nh_private.param<bool>("inverted", inverted, false);
  nh_private.param<int>("motor_speed", motor_speed, 15);
  nh_private.param<int>("merge_coef", merge_coef, 2);
  nh_private.param<bool>("enable_motor", enable_motor, true);
  nh_private.param<std::string>("lidar_ip", lidar_ip, "10.10.10.101");
  nh_private.param<int>("lidar_port", lidar_port, 2368);
  nh_private.param<std::string>("host_ip", host_ip, "10.10.10.100");
  nh_private.param<int>("host_port", host_port, 2368);
  nh_private.param<int>("method", method, 0);
  nh_private.param<bool>("switch_active_mode", switch_active_mode, false);

  if (lidar_ip == "192.168.2.100") {
    radar_index_ = 0;  //左
  } else if (lidar_ip == "192.168.3.100") {
    radar_index_ = 1;  //右
  }

  std::cout << "radar_index:" << radar_index_ << std::endl;
  ros::Publisher scan_pub;

  if (radar_index_ == 0)
    scan_pub = nh_private.advertise<sensor_msgs::PointCloud>(
        "/sweeper/sensor/radar_left", 1000);
  else if (radar_index_ == 1)
    scan_pub = nh_private.advertise<sensor_msgs::PointCloud>(
        "/sweeper/sensor/radar_right", 1000);

  pub_self_diagnose_ =
      nh_private.advertise<sweeper_msgs::SensorFaultInformation>(
          "/sweeper/common/diagnose", 1);

  std::string radar_left_to_straight_extrinsic_path =
      "../sweeper_ws/src/sweeper_haide/calibration/data/"
      "left_wj_to_straight.yaml";
  std::string radar_right_to_straight_extrinsic_path =
      "../sweeper_ws/src/sweeper_haide/calibration/data/"
      "right_wj_to_straight.yaml";
  std::string radar_right_to_left_path =
      "../sweeper_ws/src/sweeper_haide/calibration/data/"
      "radar_right_to_left.yaml";
  std::string radar_to_lidar_extrinsic_path =
      "../sweeper_ws/src/sweeper_haide/calibration/data/radar_to_lidar.yaml";

  if (!sweeper::common::LoadExtrinsic(radar_left_to_straight_extrinsic_path,
                                      &radar_left_to_straight_extrinsic_)) {
    flag_load_extrinisic_ = false;
  } else {
    flag_load_extrinisic_ = true;
  }

  if (!sweeper::common::LoadExtrinsic(radar_right_to_straight_extrinsic_path,
                                      &radar_right_to_straight_extrinsic_)) {
    flag_load_extrinisic_ = false;
  } else {
    flag_load_extrinisic_ = true;
  }

  if (!sweeper::common::LoadExtrinsic(radar_right_to_left_path,
                                      &radar_right_to_left_extrinsic_)) {
    flag_load_extrinisic_ = false;
  } else {
    flag_load_extrinisic_ = true;
  }

  if (!sweeper::common::LoadExtrinsic(radar_to_lidar_extrinsic_path,
                                      &radar_to_lidar_extrinsic_)) {
    flag_load_extrinisic_ = false;
  } else {
    flag_load_extrinisic_ = true;
  }

  if (flag_load_extrinisic_)
    AINFO << "The pavo start success";
  else
    AERROR << "Failed to load radar to lidar extrinsic!";

  std::thread diagnose_thread(SelfDiagnose);
  diagnose_thread.detach();

  std::vector<pavo_response_scan_t> scan_vec;
  if (!switch_active_mode) {
    drv = new pavo::pavo_driver();
  } else {
    drv = new pavo::pavo_driver(host_ip, host_port);
  }
  drv->pavo_open(lidar_ip, lidar_port);
  ros::Time start_scan_time;
  ros::Time end_scan_time;
  double scan_duration;
  drv->enable_motor(enable_motor);

  if (!enable_motor) {
    pid_t kill_num;
    kill_num = getppid();
    kill(kill_num, 4);
    return 0;
  }

  if (method == 0 || method == 1 || method == 2 || method == 3) {
    drv->enable_tail_filter(method);
    if (method > 0)
      ROS_INFO("success to eliminate the tail by using method: %d",
               (int)method);
  } else {
    ROS_ERROR("false to set tail filter!");
    return 0;
  }

  if (drv->get_motor_speed(motor_speed_get)) {
    ROS_INFO("success to get speed!");
    ROS_INFO("the motor_speed: %d", motor_speed_get);
  } else {
    ROS_ERROR("false to get speed!");
    return -1;
  }

  if (motor_speed != motor_speed_get) {
    if (drv->set_motor_speed(motor_speed)) {
      ROS_INFO("success to set speed!");
      ROS_INFO("motor_speed: %d", motor_speed);
      sleep(15);
    } else {
      ROS_ERROR("false to set speed!");
      return -1;
    }
  } else {
    ROS_INFO("success to set speed!");
    ROS_INFO("motor_speed: %d", motor_speed);
  }

  if (drv->set_merge_coef(merge_coef)) {
    ROS_INFO("success to set merge!");
    ROS_INFO("merge_coef: %d", merge_coef);
  } else {
    ROS_ERROR("false to set merge!");
    return -1;
  }

  ros::Rate rate(motor_speed);
  int count = 0;

  while (ros::ok()) {
    start_scan_time = ros::Time::now();
    bool status = drv->get_scanned_data(scan_vec, 150);
    if (!status) {
      delete drv;
      if (!switch_active_mode) {
        drv = new pavo::pavo_driver();
      } else {
        drv = new pavo::pavo_driver(host_ip, host_port);
      }
      drv->pavo_open(lidar_ip, lidar_port);
    }
    count = scan_vec.size();
    end_scan_time = ros::Time::now();
    scan_duration = (end_scan_time - start_scan_time).toSec();
    nh.getParam("angle_min", angle_min);
    nh.getParam("angle_max", angle_max);
    nh.getParam("range_min", min_range);
    nh.getParam("range_max", max_range);
    nh.getParam("frame_id", frame_id);
    nh.getParam("inverted", inverted);
    nh.getParam("method", method);
    publish_msg(&scan_pub, scan_vec, start_scan_time, scan_duration, frame_id,
                inverted, angle_min, angle_max, min_range, max_range, method,
                switch_active_mode);
    ros::spinOnce();
    rate.sleep();
  }
  delete drv;
}

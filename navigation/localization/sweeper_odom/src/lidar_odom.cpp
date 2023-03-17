#include "../include/sweeper_odom/get_lidar_odom.h"

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_WARNING;
  FLAGS_v = 1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "get_lidar_odom");
  ROS_INFO("\033[1;32m---->\033[0m lidar odom is Started.");
  google::InitGoogleLogging(argv[0]);
  LogSetting();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  GetLidarOdom get_lidar_odom(nh, nh_private);
  ros::spin();
  return 0;
}
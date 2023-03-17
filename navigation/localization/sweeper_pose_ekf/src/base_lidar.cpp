// #include "../include/sweeper_pose_ekf/sweeper_pose_ekf_based_lidar.h"
#include "ekf_localizer.hpp"

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_INFO;
  FLAGS_v = 1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sweeper_pose_ekf");
  google::InitGoogleLogging(argv[0]);
  LogSetting();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  sweeper::navigation::localization::fusion_location::EKFLocalizer
      ekf_localizer(nh, nh_private);
  ros::spin();
  return 0;
}
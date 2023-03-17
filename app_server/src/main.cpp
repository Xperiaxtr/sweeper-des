#include <ros/ros.h>

#include "app_server.h"

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_INFO;
  FLAGS_v = 4;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "app_server");
  google::InitGoogleLogging(argv[0]);
  LogSetting();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  sweeper::app_server::AppServer app_server(nh, nh_private);
  if (!app_server.Init()) {
    AERROR << "app server init failure!";
  } else {
    AINFO << "app server init succuss.";
    ros::spin();
  }
  return 0;
}
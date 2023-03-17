#include "../include/controll/controll.h"

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_WARNING;
  FLAGS_v = 1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controll");

  google::InitGoogleLogging(argv[0]);
  LogSetting();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  sweeper::navigation::controll::Controller sweeper_controller(nh, nh_private);
  ros::spin();

  return 0;
}
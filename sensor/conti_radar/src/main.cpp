#include "conti_radar.h"

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_INFO;
  FLAGS_v = 1;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "gps");
  google::InitGoogleLogging(argv[0]);
  LogSetting();
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  sweeper::sensor::ContiRadar conti_radar(node, priv_node);
  if (conti_radar.Init()) {
    AINFO << "ContiRadar driver init success!";
  } else {
    AERROR << "ContiRadar driver init failed!";
  }
  ros::spin();
  return 0;
}

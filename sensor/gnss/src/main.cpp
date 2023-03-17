#include "gnss.h"

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
  sweeper::gnss::Gnss gnss(node, priv_node);
  if (gnss.Init()) {
    AINFO << "Gnss driver init success!";
  } else {
    AERROR << "Gnss driver init failed!";
  }

  ros::spin();

  return 0;
}

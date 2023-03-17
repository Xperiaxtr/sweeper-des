#include "v2x.h"

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_songlin/data/log/";
  FLAGS_stderrthreshold = google::GLOG_INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_INFO;
  FLAGS_v = 1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "v2x");
  google::InitGoogleLogging(argv[0]);
  LogSetting();
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  sweeper::v2x::V2x v2x(node, priv_node);
  if (v2x.Init()) {
    AINFO << "v2x driver init success!";
  } else {
    AERROR << "v2x driver init failed!";
  }
  v2x.CommunicateObu();

  ros::spin();
  return 0;
}

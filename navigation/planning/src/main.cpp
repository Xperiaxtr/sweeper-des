#include "planning.h"

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_WARNING;
  FLAGS_v = 4;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "planning");
  google::InitGoogleLogging(argv[0]);
  LogSetting();
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  sweeper::navigation::Planning planning(nh, private_nh);

  if (!planning.Init()) {
    AERROR << "Planning init failure!";
    return -1;
  } else {
    AINFO << "Planning init successfully!";
  }
  ros::spin();
  return 0;
}
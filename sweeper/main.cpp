#include <gflags/gflags.h>

#include "sweeper.h"

using namespace std;

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_INFO;
  FLAGS_v = 1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sweeper");

  google::InitGoogleLogging(argv[0]);
  LogSetting();

  ROS_INFO("Sweeper start...");

  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  sweeper::Sweeper sweeper;

  if (!sweeper.Init(node, private_nh)) {
    ROS_ERROR("Sweeper init failure!");
  }

  return 0;
}

#include "flow/matcher_flow.hpp"
#include <thread>
using namespace mapping;

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_WARNING;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_WARNING;
  FLAGS_v = 1;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "matcher");
  google::InitGoogleLogging(argv[0]);
  LogSetting();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  MatcherFlow matcher_flow(nh, nh_private);
  std::thread matcher_node(&MatcherFlow::MatcherNode, &matcher_flow);
  ros::spin();
  return 0;
}
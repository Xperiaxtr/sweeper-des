
#include "flow/mapping_flow.hpp"

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
  ros::init(argc, argv, "mapping_is_start");
  google::InitGoogleLogging(argv[0]);
  LogSetting();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  MappingFlow mapping_flow(nh, nh_private);
  std::thread back_end(&MappingFlow::BackEndFlow, &mapping_flow);
  std::thread view_map(&MappingFlow::ViewGlobalMap, &mapping_flow);
  std::thread pubish_state(&MappingFlow::PublishMappingState, &mapping_flow);
  std::thread mapping_front(&MappingFlow::FrontNode, &mapping_flow);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  mapping_front.detach();
  back_end.detach();
  view_map.detach();
  pubish_state.detach();
  return 0;
}
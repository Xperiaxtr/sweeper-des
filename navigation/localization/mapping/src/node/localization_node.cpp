#include "flow/localization_flow.hpp"

using namespace mapping;

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_WARNING;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = false;
  FLAGS_minloglevel = google::GLOG_WARNING;
  FLAGS_v = 1;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "localization");
  google::InitGoogleLogging(argv[0]);
  LogSetting();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  LocalizationFlow localization_flow(nh, nh_private);
  std::thread pub_information(&LocalizationFlow::PubInformationThread, &localization_flow);
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    localization_flow.Process();
    rate.sleep();
  }
  pub_information.join();
  return 0;
}
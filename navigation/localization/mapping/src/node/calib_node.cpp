#include "calib/save_calib_data.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapping_is_start");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  SaveCalibData save_calib_data(nh, nh_private);
  std::thread save_node(&SaveCalibData::MapStructNode, &save_calib_data);
  std::thread matcher_node(&SaveCalibData::MatcherNode, &save_calib_data);
  save_node.detach();
  ros::spin();

  // ros::Rate rate(100);
  // while (ros::ok()) {
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  return 0;
}
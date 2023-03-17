#include "subscriber/sweep_mode_subscriber.hpp"

namespace mapping {
SweepModeSubscriber::SweepModeSubscriber(ros::NodeHandle& nh,
                                         std::string topic_name,
                                         size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size,
                              &SweepModeSubscriber::MsgCallback, this);
}

void SweepModeSubscriber::MsgCallback(
    const sweeper_msgs::SweepMission::ConstPtr& sweep_msg_ptr) {
  sweep_mode.receive_mode = true;
  sweep_mode.mode = sweep_msg_ptr->mode;
  sweep_mode.start_cmd = sweep_msg_ptr->start_cmd;
  sweep_mode.save_file_name = sweep_msg_ptr->line;
  sweep_mode.read_pcd_name = sweep_msg_ptr->map;
  sweep_mode.init_position = Eigen::Vector3d(
      sweep_msg_ptr->init_pose.position.x, sweep_msg_ptr->init_pose.position.y,
      sweep_msg_ptr->init_pose.position.z);
  sweep_mode.init_quater =
      Eigen::Quaterniond(sweep_msg_ptr->init_pose.orientation.w,
                         sweep_msg_ptr->init_pose.orientation.x,
                         sweep_msg_ptr->init_pose.orientation.y,
                         sweep_msg_ptr->init_pose.orientation.z);
}

bool SweepModeSubscriber::SweepMode::NewSweepMode(SweepMode last_sweep_mode) {
  if (!receive_mode) return false;
  if (last_sweep_mode.mode == mode && start_cmd == last_sweep_mode.start_cmd &&
      save_file_name == last_sweep_mode.save_file_name &&
      read_pcd_name == last_sweep_mode.read_pcd_name)
    return false;
  return true;
}
// void SweepModeSubscriber::InitParam() {}
}  // namespace mapping
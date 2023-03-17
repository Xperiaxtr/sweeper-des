
#ifndef MAPPING_SUBSCRIBER_SWEEP_MODE_SUBSCRIBER_HPP_
#define MAPPING_SUBSCRIBER_SWEEP_MODE_SUBSCRIBER_HPP_

#include <ros/ros.h>

#include <Eigen/Dense>

#include "sweeper_msgs/SweepMission.h"

namespace mapping {
class SweepModeSubscriber {
 public:
  struct SweepMode {
   public:
    bool receive_mode;
    int mode;
    int start_cmd;
    std::string save_file_name;
    std::string read_pcd_name;
    Eigen::Vector3d init_position;
    Eigen::Quaterniond init_quater;

   public:
    SweepMode() : receive_mode(false), mode(0), start_cmd(0){}
    bool NewSweepMode(SweepMode last_sweep_mode);
    // SweepMode() = default;
  };
  SweepMode sweep_mode;

 public:
  SweepModeSubscriber(ros::NodeHandle& nh, std::string topic_name,
                      size_t buff_size);
  SweepModeSubscriber() = default;
  void MsgCallback(const sweeper_msgs::SweepMission::ConstPtr& sweep_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  //   std::mutex buff_mutex_;
};
}  // namespace mapping
#endif

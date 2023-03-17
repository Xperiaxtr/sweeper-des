#ifndef MAPPING_SUBSCRIBER_VEHICAL_SUBSCRIBER_HPP_
#define MAPPING_SUBSCRIBER_VEHICAL_SUBSCRIBER_HPP_

#include <ros/ros.h>

#include <deque>
#include <mutex>
#include "math.h"

#include "sweeper_msgs/SweeperChassisDetail.h"
#include "sensor_data/vehical_speed.hpp"

#define KMPSToMPS 1.0/3.6
#define ToRad M_PI / 180.0

namespace mapping {
class VehicalSubscriber {
 public:
  VehicalSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  VehicalSubscriber() = default;
  void ParseData(std::deque<VehicalSpeed>& vehical_data_buff);

 private:
  void msg_callback(const sweeper_msgs::SweeperChassisDetail::ConstPtr& vehical_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::mutex buff_mutex_;
  std::deque<VehicalSpeed> new_vehical_data_;
};
}  // namespace mapping

#endif
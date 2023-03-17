
#ifndef MAPPING_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define MAPPING_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "sensor_data/gnss_data.hpp"
namespace mapping {
class GNSSSubscriber {
 public:
  GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  GNSSSubscriber() = default;
  void ParseData(std::deque<GNSSData>& gnss_data_buff);

 private:
  void msg_callback(const nav_msgs::Odometry::ConstPtr& gnss_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<GNSSData> new_gnss_data_;
  std::mutex buff_mutex_;
};
}  // namespace mapping

#endif
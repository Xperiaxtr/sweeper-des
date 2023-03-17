#ifndef MAPPING_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define MAPPING_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <deque>
#include <mutex>

#include "sensor_data/imu_data.hpp"

namespace mapping {
class IMUSubscriber {
 public:
  IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  IMUSubscriber() = default;
  void ParseData(std::deque<IMUData>& imu_data_buff);

 private:
  void msg_callback(const sensor_msgs::Imu::ConstPtr& imu_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::mutex buff_mutex_;
  std::deque<IMUData> new_imu_data_;
};
}  // namespace mapping

#endif
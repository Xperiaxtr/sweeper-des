#include "subscriber/imu_subscriber.hpp"

namespace mapping {
IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name,
                             size_t buff_size)
    : nh_(nh) {
  subscriber_ =
      nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

void IMUSubscriber::msg_callback(
    const sensor_msgs::Imu::ConstPtr& imu_msg_ptr) {
  buff_mutex_.lock();
  IMUData imu_data;
  imu_data.time = imu_msg_ptr->header.stamp.toSec();
  imu_data.imu_quater = Eigen::Quaterniond(
      imu_msg_ptr->orientation.w, imu_msg_ptr->orientation.x,
      imu_msg_ptr->orientation.y, imu_msg_ptr->orientation.z);

  new_imu_data_.push_back(imu_data);
  buff_mutex_.unlock();
}

void IMUSubscriber::ParseData(std::deque<IMUData>& imu_data_buff) {
  buff_mutex_.lock();
  while (new_imu_data_.size() > 0) {
    imu_data_buff.push_back(new_imu_data_.front());
    new_imu_data_.pop_front();
  }
  buff_mutex_.unlock();
}
}  // namespace mapping
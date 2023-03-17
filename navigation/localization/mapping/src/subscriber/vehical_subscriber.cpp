#include "subscriber/vehical_subscriber.hpp"

namespace mapping {
VehicalSubscriber::VehicalSubscriber(ros::NodeHandle& nh,
                                     std::string topic_name, size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size,
                              &VehicalSubscriber::msg_callback, this);
}

void VehicalSubscriber::msg_callback(
    const sweeper_msgs::SweeperChassisDetail::ConstPtr& vehical_msg_ptr) {
  buff_mutex_.lock();
  VehicalSpeed vehical_data;
  vehical_data.time = vehical_msg_ptr->header.stamp.toSec();
  // double kmph_to_mps = 1000.0 / 3600.0;
  // double to_rad = M_PI / 180.0;
  vehical_data.velocity = vehical_msg_ptr->vehicle_speed_output * KMPSToMPS;
  vehical_data.angular = vehical_msg_ptr->steering_angle_output * ToRad;

  new_vehical_data_.push_back(vehical_data);
  buff_mutex_.unlock();
}

// void VehicalSubscriber::ParseData(std::deque<VehicalSpeed>& vehical_data_buff) {
//   buff_mutex_.lock();
//   while (new_vehical_data_.size() > 0) {
//     vehical_data_buff.push_back(new_vehical_data_.front());
//     new_vehical_data_.pop_front();
//   }
//   buff_mutex_.unlock();
// }
void VehicalSubscriber::ParseData(std::deque<VehicalSpeed>& vehical_data_buff) {
  buff_mutex_.lock();
  while (new_vehical_data_.size() > 0) {
    // vehical_data_buff.push_back(new_vehical_data_.front());
    // new_vehical_data_.pop_front();
        vehical_data_buff.insert(vehical_data_buff.end(), new_vehical_data_.begin(),
                           new_vehical_data_.end());
    new_vehical_data_.clear();
  }
  buff_mutex_.unlock();
}
}  // namespace mapping
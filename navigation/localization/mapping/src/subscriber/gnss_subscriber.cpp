#include "subscriber/gnss_subscriber.hpp"

// #include "glog/logging.h"

namespace mapping {
GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name,
                               size_t buff_size)
    : nh_(nh) {
  subscriber_ =
      nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
}

void GNSSSubscriber::msg_callback(
    const nav_msgs::Odometry::ConstPtr& gnss_msg_ptr) {
  buff_mutex_.lock();
  // if (gnss_msg_ptr->header.frame_id == "best") {
  GNSSData gnss_data;
  gnss_data.time = gnss_msg_ptr->header.stamp.toSec();
  gnss_data.gnss_quater =
      Eigen::Quaterniond(gnss_msg_ptr->pose.pose.orientation.w,
                         gnss_msg_ptr->pose.pose.orientation.x,
                         gnss_msg_ptr->pose.pose.orientation.y,
                         gnss_msg_ptr->pose.pose.orientation.z);
  // gnss角度旋转90度，将角度与位置转换到同一坐标系下
  // Eigen::Quaterniond tran_90_yaw =
  //     Eigen::AngleAxisd(3.1415926 / 2.0, Eigen::Vector3d::UnitZ()) *
  //     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
  //     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  // gnss_data.gnss_quater = tran_90_yaw * gnss_data.gnss_quater;
  gnss_data.gnss_position = Eigen::Vector3d(gnss_msg_ptr->pose.pose.position.x,
                                            gnss_msg_ptr->pose.pose.position.y,
                                            gnss_msg_ptr->pose.pose.position.z);
  gnss_data.gnss_cov(0, 0) = gnss_msg_ptr->pose.covariance[0];
  gnss_data.gnss_cov(1, 1) = gnss_msg_ptr->pose.covariance[7];
  gnss_data.gnss_cov(2, 2) = gnss_msg_ptr->pose.covariance[14];
  gnss_data.gnss_cov(3, 3) = gnss_msg_ptr->pose.covariance[21];
  gnss_data.gnss_cov(4, 4) = gnss_msg_ptr->pose.covariance[28];
  gnss_data.gnss_cov(5, 5) = gnss_msg_ptr->pose.covariance[35];

  new_gnss_data_.push_back(gnss_data);
  // }
  buff_mutex_.unlock();
}

void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff) {
  buff_mutex_.lock();

  while (new_gnss_data_.size() > 0) {
    gnss_data_buff.push_back(new_gnss_data_.front());
    new_gnss_data_.pop_front();
  }

  buff_mutex_.unlock();
}

}  // namespace mapping
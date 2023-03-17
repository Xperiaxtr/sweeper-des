/*
发布里程计信息
 */

#include "publisher/odometry_publisher.hpp"

namespace mapping {
OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh,
                                     std::string topic_name,
                                     std::string base_frame_id,
                                     std::string child_frame_id, int buff_size)
    : nh_(nh) {
  publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
  odometry_.header.frame_id = base_frame_id;
  odometry_.child_frame_id = child_frame_id;
  odom_information_ = Eigen::Matrix<double, 6, 6>::Identity();
}

void OdometryPublisher::Publish4d(const Eigen::Matrix4d& transform_matrix,
                                  ros::Time pub_time) {
  odometry_.header.stamp = pub_time;

  // set the position
  odometry_.pose.pose.position.x = transform_matrix(0, 3);
  odometry_.pose.pose.position.y = transform_matrix(1, 3);
  odometry_.pose.pose.position.z = transform_matrix(2, 3);

  Eigen::Quaterniond q;
  q = transform_matrix.block<3, 3>(0, 0);
  odometry_.pose.pose.orientation.x = q.x();
  odometry_.pose.pose.orientation.y = q.y();
  odometry_.pose.pose.orientation.z = q.z();
  odometry_.pose.pose.orientation.w = q.w();

  odometry_.pose.covariance[0] = odom_information_(0, 0);
  odometry_.pose.covariance[7] = odom_information_(1, 1);
  odometry_.pose.covariance[14] = odom_information_(2, 2);
  odometry_.pose.covariance[21] = odom_information_(3, 3);
  odometry_.pose.covariance[28] = odom_information_(4, 4);
  odometry_.pose.covariance[35] = odom_information_(5, 5);

  publisher_.publish(odometry_);
}
void OdometryPublisher::PublishQuat(const Eigen::Quaterniond quat,
                                    Eigen::Vector3d pose,
                                    ros::Time pub_time) {
  odometry_.header.stamp = pub_time;

  // set the position
  odometry_.pose.pose.position.x = pose.x();
  odometry_.pose.pose.position.y = pose.y();
  odometry_.pose.pose.position.z = pose.z();

  // Eigen::Quaternionf q;
  // q = transform_matrix.block<3,3>(0,0);
  odometry_.pose.pose.orientation.x = quat.x();
  odometry_.pose.pose.orientation.y = quat.y();
  odometry_.pose.pose.orientation.z = quat.z();
  odometry_.pose.pose.orientation.w = quat.w();

  odometry_.pose.covariance[0] = odom_information_(0, 0);
  odometry_.pose.covariance[7] = odom_information_(1, 1);
  odometry_.pose.covariance[14] = odom_information_(2, 2);
  odometry_.pose.covariance[21] = odom_information_(3, 3);
  odometry_.pose.covariance[28] = odom_information_(4, 4);
  odometry_.pose.covariance[35] = odom_information_(5, 5);

  publisher_.publish(odometry_);
}

void OdometryPublisher::SetInformation(
    Eigen::Matrix<double, 6, 6> odom_information) {
  odom_information_ = odom_information;
}
}  // namespace mapping
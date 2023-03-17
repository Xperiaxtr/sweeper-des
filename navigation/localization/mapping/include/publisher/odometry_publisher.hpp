/*
点云发布
 */
#ifndef MAPPING_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define MAPPING_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

namespace mapping {
class OdometryPublisher {
 public:
  OdometryPublisher(ros::NodeHandle& nh, std::string topic_name,
                    std::string base_frame_id, std::string child_frame_id,
                    int buff_size);
  OdometryPublisher() = default;

  void Publish4d(const Eigen::Matrix4d& transform_matrix,
                 ros::Time pub_time);
  void PublishQuat(const Eigen::Quaterniond quat, Eigen::Vector3d pose,
                   ros::Time pub_time);
  void SetInformation(Eigen::Matrix<double, 6, 6> odom_information);

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  nav_msgs::Odometry odometry_;
  Eigen::Matrix<double, 6, 6> odom_information_;
};
}  // namespace mapping

#endif
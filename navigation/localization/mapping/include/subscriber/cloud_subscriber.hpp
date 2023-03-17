/*
订阅激光数据
 */
#ifndef MAPPING_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define MAPPING_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <deque>
#include <mutex>
// #include "get_feature/get_feature.hpp"
#include "../../../../common/log.h"
#include "sensor_data/feature.hpp"
// #include "sensor_data/cloud_data.hpp"

namespace mapping {
class CloudSubscriber {
 public:
  CloudSubscriber(ros::NodeHandle& nh, std::string topic_name,
                  size_t buff_size);
  CloudSubscriber() = default;
  void ParseData(std::deque<CloudData>& cloud_data_buff);

 private:
  void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<CloudData> new_cloud_data_;
  std::mutex buff_mutex_;
};
}  // namespace mapping
#endif
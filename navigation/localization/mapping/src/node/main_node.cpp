#include <pcl/common/transforms.h>
#include <ros/ros.h>

#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "subscriber/cloud_subscriber.hpp"
// #include "subscriber/gnss_subscriber.hpp"
// #include "subscriber/imu_subscriber.hpp"
// #include "tf_listener/tf_listener.hpp"
// #include "mapping/front_end/front_end.hpp"
// #include
using namespace mapping;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "get_feature_test");
  ros::NodeHandle nh;
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr =
      std::make_shared<CloudSubscriber>(nh, "/livox/lidar", 100000);

  std::shared_ptr<CloudPublisher> corner_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "corner_cloud", 100, "/map");
  std::shared_ptr<CloudPublisher> surf_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "surf_cloud", 100, "/map");
  std::shared_ptr<CloudPublisher> intensity_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "intensity_cloud", 100, "/map");

  std::deque<Feature> cloud_data_buff;
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    // std::cout<<"feature size : "<<cloud_data_buff.size()<<std::endl;
    cloud_sub_ptr->ParseData(cloud_data_buff);
    // std::cout<<"feature size : "<<cloud_data_buff.size()<<std::endl;
    while (cloud_data_buff.size() > 0) {
        Feature feature_data = cloud_data_buff.front();
        corner_pub_ptr->Publish(feature_data.corner_cloud);
        surf_pub_ptr->Publish(feature_data.surf_cloud);
        intensity_pub_ptr->Publish(feature_data.intensity_cloud);
        cloud_data_buff.pop_front();
    }
  }
}
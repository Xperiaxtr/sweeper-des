#include <pcl/common/transforms.h>
#include <ros/ros.h>

#include "mapping/front_end/front_end.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "subscriber/cloud_subscriber.hpp"

using namespace mapping;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "get_feature_test");
  ros::NodeHandle nh;
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr =
      std::make_shared<CloudSubscriber>(nh, "/livox/lidar", 100000);
  std::shared_ptr<OdometryPublisher> odom_pub_ptr =
      std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar",
                                          100);

  std::shared_ptr<CloudPublisher> corner_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "corner_cloud", 100, "/map");
  std::shared_ptr<CloudPublisher> surf_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "surf_cloud", 100, "/map");
  std::shared_ptr<CloudPublisher> intensity_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "intensity_cloud", 100, "/map");

  FrontEnd front_end;

  // std::deque<Feature> cloud_data_buff;
  std::deque<CloudData> cloud_data_buff;

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    cloud_sub_ptr->ParseData(cloud_data_buff);
    // std::cout<<"size : "<<cloud_data_buff.size()<<std::endl;
    while (cloud_data_buff.size() > 0) {
      CloudData cloud_data = cloud_data_buff.front();
      GetFeature get_feature;
      get_feature.GetLivoxFeature(cloud_data);
      // corner_pub_ptr->Publish(get_feature.new_feature_data_.corner_cloud);
      // surf_pub_ptr->Publish(get_feature.new_feature_data_.surf_cloud);
      // intensity_pub_ptr->Publish(get_feature.new_feature_data_.intensity_cloud);

      Eigen::Quaterniond quater;
      Eigen::Vector3d pos;
      front_end.receive_feature_ = get_feature.new_feature_data_;
      std::cout << "cin lidar odom" << std::endl;
      front_end.Process();
      std::cout << "get odom" << std::endl;
      quater = front_end.now_quater_;
      pos = front_end.now_pos_;
      odom_pub_ptr->PublishQuat(quater, pos);
      cloud_data_buff.pop_front();

      corner_pub_ptr->Publish(front_end.corner_map_bag_);
      surf_pub_ptr->Publish(get_feature.new_feature_data_.surf_cloud);
      intensity_pub_ptr->Publish(get_feature.new_feature_data_.intensity_cloud);
    }
    // ros::spinOnce();
  }
}
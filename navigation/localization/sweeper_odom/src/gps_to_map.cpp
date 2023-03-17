//gps转换到地图坐标系中
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "sweeper_msgs/SweeperChassisDetail.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

Eigen::Quaterniond world_to_map_quater_;
Eigen::Vector3d world_to_map_position_;
Eigen::Matrix<double, 4, 4> world_to_map_;
ros::Publisher pub_map_odom_;

Eigen::Quaterniond lidar_to_gps_quater_(0.671785679895454, 0, 0, 0.740745570548621);
Eigen::Vector3d lidar_to_gps_position_(-0.580947643300626, 0.625673451701035, 0);
Eigen::Matrix<double, 4, 4> lidar_to_gps_;

void ParamInit(ros::NodeHandle nh)
{
    std::vector<double> tran_list;
    if (nh.getParam("translation", tran_list))
    {
        ROS_INFO("get translation param");
    }
    else
        ROS_WARN("didn't find parameter translation");

    world_to_map_position_ = Eigen::Vector3d(tran_list[0], tran_list[1], tran_list[2]);

    std::vector<double> rotation_list;
    if (nh.getParam("rotation", rotation_list))
    {
        ROS_INFO("get rotation param");
    }
    else
        ROS_WARN("didn't find parameter rotation");

    world_to_map_quater_ = Eigen::Quaterniond(rotation_list[3], rotation_list[0], rotation_list[1], rotation_list[2]);

    world_to_map_.block(0, 0, 3, 3) = world_to_map_quater_.matrix();
    world_to_map_.block(0, 3, 3, 1) = world_to_map_position_;
    world_to_map_(3, 3) = 1;
    world_to_map_(3, 0) = 0;
    world_to_map_(3, 1) = 0;
    world_to_map_(3, 2) = 0;

    lidar_to_gps_.block(0, 0, 3, 3) = lidar_to_gps_quater_.matrix();
    lidar_to_gps_.block(0, 3, 3, 1) = lidar_to_gps_position_;
    lidar_to_gps_(3, 3) = 1;
    lidar_to_gps_(3, 0) = 0;
    lidar_to_gps_(3, 1) = 0;
    lidar_to_gps_(3, 2) = 0;
}

void ReceiveGpsOdom(const nav_msgs::Odometry input_odom)
{
    //1.lidar to gps ,gps to world => lidar in world
    Eigen::Quaterniond intput_gps_quater(input_odom.pose.pose.orientation.w, input_odom.pose.pose.orientation.x, input_odom.pose.pose.orientation.y, input_odom.pose.pose.orientation.z);
    Eigen::Matrix<double, 4, 4> input_gps;
    input_gps.block(0, 0, 3, 3) = intput_gps_quater.matrix();
    input_gps(0, 3) = input_odom.pose.pose.position.x;
    input_gps(1, 3) = input_odom.pose.pose.position.y;
    input_gps(2, 3) = input_odom.pose.pose.position.z;
    input_gps(3, 3) = 1;
    input_gps(3, 0) = 0;
    input_gps(3, 1) = 0;
    input_gps(3, 2) = 0;
    Eigen::Matrix<double, 4, 4> lidar_in_world = input_gps * lidar_to_gps_;
    //2.lidar in map
    Eigen::Matrix<double, 4, 4> lidar_in_map = world_to_map_ * lidar_in_world;
    Eigen::Matrix<double, 3, 3> lidar_in_map_rotation = lidar_in_map.block(0, 0, 3, 3);
    Eigen::Quaterniond lidar_in_map_quater(lidar_in_map_rotation);

    nav_msgs::Odometry gps_in_map;
    gps_in_map.header.frame_id = "/camera_init";
    gps_in_map.header.stamp = input_odom.header.stamp;
    gps_in_map.pose.pose.orientation.x = lidar_in_map_quater.x();
    gps_in_map.pose.pose.orientation.y = lidar_in_map_quater.y();
    gps_in_map.pose.pose.orientation.z = lidar_in_map_quater.z();
    gps_in_map.pose.pose.orientation.w = lidar_in_map_quater.w();
    gps_in_map.pose.pose.position.x = lidar_in_map(0, 3);
    gps_in_map.pose.pose.position.y = lidar_in_map(1, 3);
    gps_in_map.pose.pose.position.z = 0.0;

    gps_in_map.pose.covariance[0] = input_odom.pose.covariance[0];
    gps_in_map.pose.covariance[7] = input_odom.pose.covariance[7];
    gps_in_map.pose.covariance[14] = input_odom.pose.covariance[14];
    gps_in_map.pose.covariance[21] = input_odom.pose.covariance[21];
    gps_in_map.pose.covariance[28] = input_odom.pose.covariance[28];
    gps_in_map.pose.covariance[35] = input_odom.pose.covariance[35];

    pub_map_odom_.publish(gps_in_map);

    std::cout<<"x y z :"<<gps_in_map.pose.pose.position.x<<" "<<gps_in_map.pose.pose.position.y<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps world to map");
    ros::NodeHandle nh;
    ParamInit(nh);
    ros::Subscriber sub_gps_odom;
    sub_gps_odom = nh.subscribe<nav_msgs::Odometry>("/sweeper/sensor/gnss", 1, ReceiveGpsOdom);
    pub_map_odom_ = nh.advertise<nav_msgs::Odometry>("/sweeper/localization/calibration_gps", 1);
    ros::spin();
    return 0;
}
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub_correct_radar_, pub_correct2_radar_;

void FrontRadarCallback(const sensor_msgs::PointCloud::ConstPtr &radar_datas)
{
    sensor_msgs::PointCloud radar_correct_points, radar_correct2_points;
    geometry_msgs::Point32 temp_point;
    int radar_datas_num = radar_datas->points.size();

    // Eigen::Matrix<double, 2, 2> radar_to_lidar_matrix_2;
    // radar_to_lidar_matrix_2 << cos(-0.0153304), -sin(-0.0153304), sin(-0.0153304), cos(-0.0153304);

    // std::cout << "radar_to_lidar_1: " << radar_to_lidar_matrix_2 << std::endl;

    Eigen::Matrix3d radar_to_lidar_matrix;

    geometry_msgs::Quaternion radar_to_lidar = tf::createQuaternionMsgFromYaw(-0.0153304);
    Eigen::Quaterniond radar_to_lidar_quat;

    radar_to_lidar_quat.x() = radar_to_lidar.x;
    radar_to_lidar_quat.y() = radar_to_lidar.y;
    radar_to_lidar_quat.z() = radar_to_lidar.z;
    radar_to_lidar_quat.w() = radar_to_lidar.w;

    radar_to_lidar_matrix = radar_to_lidar_quat.matrix();

    std::cout << "radar_to_lidar_2: " << radar_to_lidar_matrix << std::endl;

    for (unsigned int i = 0; i < radar_datas_num; ++i)
    {
        Eigen::Matrix<double, 3, 1> radar_points;
        radar_points << radar_datas->points[i].x, radar_datas->points[i].y,0.8;

        Eigen::Matrix<double, 3, 1> radar_to_lidar_points;
        radar_to_lidar_points = radar_to_lidar_matrix * radar_points;
        radar_to_lidar_points(0) = radar_to_lidar_points(0) + 0.298169;
        radar_to_lidar_points(1) = radar_to_lidar_points(1) + -0.655807;

        temp_point.x = radar_to_lidar_points(0);
        temp_point.y = radar_to_lidar_points(1);

        radar_correct_points.points.push_back(temp_point);
    }
    radar_correct_points.header.frame_id = "aft_mapped";
    radar_correct_points.header.stamp = ros::Time::now();

    pub_correct_radar_.publish(radar_correct_points);
}

int main(int argv, char **argc)
{
    ros::init(argv, argc, "verify_radar_to_lidar");
    ros::NodeHandle nh;
    ros::Subscriber sub_radar = nh.subscribe("/sweeper/laser_front", 1, &FrontRadarCallback);
    pub_correct_radar_ = nh.advertise<sensor_msgs::PointCloud>("/sweeper/laser_front/correct", 1);
    pub_correct2_radar_ = nh.advertise<sensor_msgs::PointCloud>("/sweeper/laser_front/correct2", 1);
    ros::spin();
    return 0;
}

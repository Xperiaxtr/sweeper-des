#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

ros::Publisher pub_lidar_;

void DJCloudToPcd(const sensor_msgs::PointCloud2::ConstPtr &lidar_cloud)
{
    ROS_INFO("Start transform sensor_msgs::PointCloud2 to pcd!");
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*lidar_cloud, pcl_cloud);
    Eigen::Vector3d euler_angle(0.0, 0.261, 0.0002);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion;
    quaternion = yawAngle * pitchAngle * rollAngle;
    Eigen::Translation3d translation(0, 0, 1.9921); //    (0, 0, 1.3);(0, 0, 1.32)
    Eigen::Affine3d affine = translation * quaternion;
    Eigen::Matrix4d matrix = affine.matrix();
    pcl::transformPointCloud(pcl_cloud, pcl_cloud, matrix);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pcl_cloud, output);
    pub_lidar_.publish(output);
    // pcl::io::savePCDFileASCII("dj_lidar.pcd", pcl_cloud);
}

void SingleCloudToPcd(const sensor_msgs::PointCloud::ConstPtr &lidar_datas)
{
    ROS_INFO("Start transform sensor_msgs::PointCloud to pcd!");
    int num_cloud = lidar_datas->points.size();
    pcl::PointCloud<pcl::PointXYZ> front_cloud;
    for (unsigned int i = 0; i < num_cloud; ++i)
    {
        pcl::PointXYZ temp_point;
        temp_point.x = lidar_datas->points[i].x;
        temp_point.y = lidar_datas->points[i].y;
        temp_point.z = 1.0;

        front_cloud.push_back(temp_point);
    }
    //pcl::io::savePCDFileASCII("front_cloud.pcd", front_cloud);
}

int main(int argv, char **argc)
{
    ros::init(argv, argc, "save_pcd");
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud = nh.subscribe("/livox/lidar", 1, &DJCloudToPcd);
    ros::Subscriber sub_single_cloud = nh.subscribe("/sweeper/laser_front", 1, &SingleCloudToPcd);
    pub_lidar_ = nh.advertise<sensor_msgs::PointCloud2>("/sweeper/sensor/dj_lidar", 1);
    ros::spin();
    return 0;
}
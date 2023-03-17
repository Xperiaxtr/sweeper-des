#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

double theta = 0.0;
int num_temp_theta = 0;

double CalculateDistance(double x, double y, double z)
{
    double output = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    return output;
}

void FrontLaserScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr &lidar_datas)
{
    double object_position_x = 0.0;
    double object_position_y = 0.0;
    double object_position_z = 0.0;
    int num_object_cloud = 0;
    laser_geometry::LaserProjection projector_front;
    sensor_msgs::PointCloud cloud;
    projector_front.projectLaser(*lidar_datas, cloud);
    int num_front_cloud = cloud.points.size();
    std::cout << "num_front_cloud: " << num_front_cloud << std::endl;
    for (unsigned int i = 0; i < num_front_cloud; ++i)
    {
        if (CalculateDistance(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z) > 2.0)
        {
            num_object_cloud = num_object_cloud + 1;
            object_position_x = object_position_x + cloud.points[i].x;
            object_position_y = object_position_y + cloud.points[i].y;
        }
    }
    object_position_x = object_position_x / num_object_cloud;
    object_position_y = object_position_y / num_object_cloud;

    std::cout << "x: " << object_position_x << std::endl;
    std::cout << "y: " << object_position_y << std::endl;

    double temp_theta = atan2(object_position_y, object_position_x);
    num_temp_theta = num_temp_theta + 1;
    theta = theta + temp_theta;
    std::cout << "num_temp_theta: " << num_temp_theta << std::endl;
    std::cout << "theta: " << theta << std::endl;
}

int main(int argv, char **argc)
{
    ros::init(argv, argc, "transfrom_to_car");
    ros::NodeHandle nh;
    ros::Subscriber sub_lidar_front = nh.subscribe("/sweeper/laser_front", 1, &FrontLaserScanToPointCloud);
    ROS_INFO("Start to transform laserscan to pointcloud!");
    ros::spin();
    return 0;
}
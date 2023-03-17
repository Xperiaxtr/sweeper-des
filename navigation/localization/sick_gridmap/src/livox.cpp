#include <ros/ros.h>
#include <pcl/point_cloud.h>
//msgs type and conversion
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//pcd io
#include <pcl/io/pcd_io.h>
//point types
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>


// struct Pt_info
// {
//   int idx;
//   int plane;
//   int 
// }
ros::Publisher cloud_pub;
void call_back(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    pcl::fromROSMsg (*input, cloud);
    sensor_msgs::PointCloud2 output;
    int s=1000000;
    for(size_t i=0;i<1;i++)
    {
        int k=1000000,m=1000000;
        while(k--);
        while(m--);
        cloud1.points.push_back(cloud.points[i]);
        pcl::toROSMsg(cloud1, output);
        output.header.frame_id = "/odom";
        cloud_pub.publish( output );
    }
    while(s--);
}



int
main(int argc,char** argv)
{
// Initialize ROS
  ros::init (argc, argv, "pcl_2_pcd");
  ros::NodeHandle nh;
 
// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/livox/lidar", 1, call_back);
 cloud_pub =nh.advertise<sensor_msgs::PointCloud2>( "/cloud_map", 1,true );
// Spin
  ros::spin ();
}
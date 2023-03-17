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



ros::Publisher cloud_pub;
void call_back(const sensor_msgs::PointCloud2ConstPtr& input)
{
    cv::Mat color_iamge = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg (*input, cloud);
    for(size_t i = 0; i < cloud.points.size(); i++)
    {
        if(cloud.points[i].x < 20.0 && cloud.points[i].y < 10.0 && cloud.points[i].y > -10.0)
        {
            // if(cloud.points[i].intensity > 150) continue;
            int color = cloud.points[i].intensity * 5;
            int m = 1000 - (cloud.points[i].x) / 0.02;
            int n = 1000/2 - (cloud.points[i].y)/0.02;
            if(color > 255) color = 255;
            if(color < 0) color = 0;
            color_iamge.at<cv::Vec3b>(m, n)[0] = 255 - color;
            color_iamge.at<cv::Vec3b>(m, n)[1] = color;
            color_iamge.at<cv::Vec3b>(m, n)[2] = 255 - color;
        }
    }
    cv::namedWindow("color_iamge", CV_WINDOW_NORMAL);
    cv::imshow("color_iamge", color_iamge);
    cv::waitKey(1);

}



int
main(int argc,char** argv)
{
// Initialize ROS
  ros::init (argc, argv, "pcl_2_pcd");
  ros::NodeHandle nh;
 
// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/livox/lidar", 1, call_back);
//  cloud_pub =nh.advertise<sensor_msgs::PointCloud2>( "/cloud_map", 1,true );
// Spin
  ros::spin ();
}
//将livox激光雷达标定成与地面平行，并得到与地面的高度
#include <ros/ros.h>
#include <time.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#define PI 3.1415926
using namespace std;


    ros::Subscriber sub_cloud;
    ros::Publisher pub_cloud;

/*
函数功能：  得到平面系数 */
Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f before, Eigen::Vector3f after)
{

  before.normalize();
  after.normalize();

  float angle = acos(before.dot(after));
  Eigen::Vector3f p_rotate = before.cross(after);
  p_rotate.normalize();

    Eigen::AngleAxisd rotation_vector(
      angle, Eigen::Vector3d(p_rotate.x(), p_rotate.y(), p_rotate.z()));


  // Eigen::Matrix3f rotationMatrix;
  // rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
  // rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle)); //这里跟公式比多了一个括号，但是看实验结果它是对的。
  // rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));

  // rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
  // rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
  // rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));

  // rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
  // rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
  // rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

  // Eigen::Vector3f ea = rotationMatrix.eulerAngles(2, 1, 0);
  // // ea[0]=3.1415;
  // // ea[2]=-3.1415;
  // if(ea[0] > 2.0) ea[0] = PI - ea[0];
  // if(ea[1] > 2.0) ea[1] = PI - ea[1];
  // if(ea[2] > 2.0) ea[2] = PI - ea[2];
  // std::cout<<"ea[0]  "<<ea[0]<<"  "<<"ea[1]  "<<ea[1]<<"  "<<"ea[2]  "<<ea[2]<<std::endl;
  // Eigen::Matrix3d rotation_matrix3;
  // rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
  //                    Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
  //                    Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
  // Eigen::Quaternionf quater(rotationMatrix);
  // std::cout<<"quater  : "<<quater.x()<<" "<<quater.y()<<" "<<quater.z()<<" "<<quater.w()<<std::endl;


  Eigen::Quaterniond quaternion1(rotation_vector);

  tf::Quaternion gnss_q(quaternion1.x(), quaternion1.y(), quaternion1.z(),
                        quaternion1.w());
  tf::Matrix3x3 gnss_m(gnss_q);
  double roll, pitch, yaw;
  gnss_m.getRPY(roll, pitch, yaw);

  std::cout << "rpy : " << roll << " " << pitch << " " << yaw << std::endl;

  Eigen::Matrix3d rotation_matrix1 = Eigen::Matrix3d::Identity();
  rotation_matrix1 = rotation_vector.matrix();
  // Eigen::Quaterniond quater(rotation_matrix1);
  std::cout<<"quater: "<<quaternion1.x()<<" "<<quaternion1.y()<<" "<<quaternion1.z()<<" "<<quaternion1.w()<<std::endl;

  Eigen::Matrix4f rotationMatrix1;
  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 3; j++)
    {
      rotationMatrix1(i, j) = rotation_matrix1(i, j);
    }
  return rotationMatrix1;
}

/*
函数功能：  矫正点云
输入：  代矫正点云
输出:  矫正后点云
 */
void Correction_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud)
{
//   Eigen::Vector3d euler_angle(livox_yaw_, livox_pitch_, livox_roll_);
//   Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitX()));
//   Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
//   Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitZ()));

//   Eigen::Quaterniond quaternion;
//   quaternion = yawAngle * pitchAngle * rollAngle;
//   Eigen::Translation3d translation(livox_x_, livox_y_, livox_z_);
//   Eigen::Affine3d affine = translation * quaternion;
//   Eigen::Matrix4d matrix = affine.matrix();

//   pcl::transformPointCloud(*point_cloud, *point_cloud, matrix);

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_seg(
      new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i < point_cloud->points.size(); i++)
  {
    if (point_cloud->points[i].x < 10.0 && point_cloud->points[i].y < 5.0 && point_cloud->points[i].y > -5.0)// && point_cloud->points[i].z < -1.0)
      point_seg->points.push_back(point_cloud->points[i]);
  }

  if (point_seg->points.size() > 100)
  {
    pcl::SACSegmentation<pcl::PointXYZI> plane_seg;
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    plane_seg.setOptimizeCoefficients(true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(0.03);
    plane_seg.setInputCloud(point_seg);
    plane_seg.segment(*plane_inliers, *plane_coefficients); //得到平面系数，进而得到平面法向量

    // std::cout<<"plane_coefficients::  "<<plane_coefficients->values[0]<<"  "<<plane_coefficients->values[1]<<"  "<<plane_coefficients->values[2]
    // <<"  "<<plane_coefficients->values[3]<<std::endl;
    std::cout<<"z  "<<plane_coefficients->values[3]<<std::endl;
    Eigen::Vector3f before, after;
    before[0] = plane_coefficients->values[0];
    before[1] = plane_coefficients->values[1];
    // before[0]=0.0;
    // before[1]=0.0;
    before[2] = plane_coefficients->values[2];

    after[0] = 0.0;
    after[1] = 0.0;
    after[2] = 1.0;
    Eigen::Matrix4f rotationMatrix = CreateRotateMatrix(before, after);
    rotationMatrix(2, 3) =  plane_coefficients->values[3];
    pcl::transformPointCloud(*point_cloud, *point_cloud, rotationMatrix);
  }
}


void cloudcallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud_1);
    Correction_Cloud(cloud_1);
    
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_1, output);
    output.header.frame_id = "aft_mapped";
    pub_cloud.publish(output);//发布数据;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "corivation");
    // ros::Subscriber sub_cloud;
    // ros::Publisher pub_cloud;
    ros::NodeHandle nh;
    sub_cloud = nh.subscribe("/livox/lidar", 1, cloudcallback);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/correction/cloud", 1);

    ros::spin();
    return 0;
}


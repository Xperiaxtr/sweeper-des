#include <pcl/point_cloud.h>
#include <ros/ros.h>
// msgs type and conversion
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
// pcd io
#include <pcl/io/pcd_io.h>
// point types
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>

#include <opencv2/opencv.hpp>

#include "../../../../common/watch_dog.h"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweepMission.h"

#define UN_RECEIVED_LIVOX 3108

using namespace std;
using namespace cv;
ros::Subscriber sub;
ros::Subscriber sub_sweeper_mode_;
ros::Publisher corner_pub, surf_pub, full_pub, intensity_pub;
ros::Publisher pub_state_information_;
ros::Timer timer_get_feature;
sweeper::common::WatchDog watch_dog_livox_;
double the_corner_min_curvature_ = 0.02;
double the_surface_max_curvature_ = 0.0005;
double the_min_hight_ = 0.06;
double view_max_distance_ = 50;
double view_min_distance_ = 0.2;
double outlier_min_distance_ = 0.05;
double the_min_intensity_difference_ = 70;
double the_max_intensity_hight_ = 1;
double the_min_slope_difference_ = 0.3;  // 27度
double livox_z_ = 1.9921;
bool allow_get_feature_ = false;

//点云坐标变换
void transformation_coordinate(pcl::PointCloud<pcl::PointXYZI> &raw_cloud,
                               pcl::PointCloud<pcl::PointXYZI> &point_cloud) {
  Eigen::Vector3d euler_angle(0.0, 0.0, 0.0000);
  Eigen::AngleAxisd rollAngle(
      Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(
      Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(
      Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond quaternion;
  quaternion = yawAngle * pitchAngle * rollAngle;
  Eigen::Translation3d translation(0, 0,
                                   2.13);  //    (0, 0, 1.3);(0, 0, 1.32)
  Eigen::Affine3d affine = translation * quaternion;
  Eigen::Matrix4d matrix = affine.matrix();
  pcl::transformPointCloud(raw_cloud, point_cloud, matrix);
}

void Get_Scan(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
  watch_dog_livox_.UpdataNow();
  if (!allow_get_feature_) return;
  //点云曲率, 40000为一帧点云中点的最大数量
  float cloudCurvature[40000] = {0};
  float cloudintensity[40000] = {0};
  float cloudslope[40000] = {0};
  //曲率点对应的序号
  // int cloudSortInd[40000];
  //点是否筛选过标志：0-未筛选过，1-筛选过
  int cloudNeighborPicked[40000] = {0};

  double cloud_distance[40000];

  pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
  // std::cout << "size  " << laserCloudIn.points.size() << std::endl;
  vector<vector<int>> scans;
  vector<int> scan;
  std::vector<int> indices;
  //移除空点
  // pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
  transformation_coordinate(laserCloudIn, laserCloudIn);
  // std::cout << "size  " << laserCloudIn.points.size() << std::endl;
  int cloudSize = laserCloudIn.points.size();

  // double t1 = clock();
  double angle_last = 0.0, dis_incre_last = 0.0, dis_incre_now = 0.0,
         angle = 0.0;
  for (int i = 0; i < cloudSize; i++) {
    // angle = atan((laserCloudIn.points[i].z - 1.9921) /
    // (sqrt(laserCloudIn.points[i].x * laserCloudIn.points[i].x +
    // laserCloudIn.points[i].y * laserCloudIn.points[i].y)));
    angle = atan((laserCloudIn.points[i].z - 2.13) /
                 (sqrt(laserCloudIn.points[i].x * laserCloudIn.points[i].x +
                       laserCloudIn.points[i].y * laserCloudIn.points[i].y)));
    // std::cout<<"angle "<<angle<<std::endl;
    if (i == 0) {
      scan.push_back(i);
      angle_last = angle;
      continue;
    }
    dis_incre_now = angle - angle_last;
    if ((dis_incre_now > 0 && dis_incre_last < 0) ||
        (dis_incre_now < 0 && dis_incre_last > 0)) {
      if (scan.size() > 30) scans.push_back(scan);
      scan.clear();
      scan.push_back(i);
    } else
      scan.push_back(i);

    // double
    // angle=atan(laserCloudIn.points[i].z/sqrt(laserCloudIn.points[i].x*laserCloudIn.points[i].x+laserCloudIn.points[i].y*laserCloudIn.points[i].y));

    angle_last = angle;
    dis_incre_last = dis_incre_now;

    // cloud_distance[i] = sqrt(laserCloudIn.points[i].x *
    // laserCloudIn.points[i].x + laserCloudIn.points[i].y *
    // laserCloudIn.points[i].y + laserCloudIn.points[i].z *
    // laserCloudIn.points[i].z );
  }
  // transformation_coordinate(laserCloudIn, laserCloudIn);

  // cv::Mat color_mat = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
  // for (size_t i = 0; i < scans.size(); i++)
  // {
  //     for (size_t j = 6; j < scans[i].size() - 6; j++)
  //     {
  //         if (laserCloudIn.points[scans[i][j]].x > 0 &&
  //         laserCloudIn.points[scans[i][j]].x < 9.9 &&
  //         laserCloudIn.points[scans[i][j]].y > -4.9 &&
  //         laserCloudIn.points[scans[i][j]].y < 4.9)
  //         {

  //             int rand_x = 1000 - laserCloudIn.points[scans[i][j]].x / 0.01;
  //             int rand_y = 500 - laserCloudIn.points[scans[i][j]].y / 0.01;
  //             color_mat.at<cv::Vec3b>(rand_x, rand_y)[0] = ((i + 1) % 3) *
  //             80; color_mat.at<cv::Vec3b>(rand_x, rand_y)[1] = i % 5 * 50;
  //             color_mat.at<cv::Vec3b>(rand_x, rand_y)[2] = ((i + 1) % 2) *
  //             100;
  //         }
  //     }
  // }
  // cv::imshow("color", color_mat);
  // cv::waitKey(1);

  //开始计算曲率
  for (size_t i = 0; i < scans.size(); i++)
    for (size_t j = 6; j < scans[i].size() - 6; j++) {
      float diffX = /* laserCloudIn.points[scans[i][j]-5].x +
              laserCloudIn.points[scans[i][j] - 4].x
              + laserCloudIn.points[scans[i][j] - 3].x + */
          laserCloudIn.points[scans[i][j] - 2].x +
          laserCloudIn.points[scans[i][j] - 1].x -
          4 * laserCloudIn.points[scans[i][j]].x +
          laserCloudIn.points[scans[i][j] + 1].x +
          laserCloudIn.points[scans[i][j] + 2].x
          /*+ laserCloudIn.points[scans[i][j] + 3].x +
             laserCloudIn.points[scans[i][j] + 4].x
              + laserCloudIn.points[scans[i][j] + 5].x*/
          ;
      float diffY = /*laserCloudIn.points[scans[i][j] - 5].y +
              laserCloudIn.points[scans[i][j] - 4].y
              + laserCloudIn.points[scans[i][j] - 3].y + */
          laserCloudIn.points[scans[i][j] - 2].y +
          laserCloudIn.points[scans[i][j] - 1].y -
          4 * laserCloudIn.points[scans[i][j]].y +
          laserCloudIn.points[scans[i][j] + 1].y +
          laserCloudIn.points[scans[i][j] + 2].y
          /*+ laserCloudIn.points[scans[i][j] + 3].y +
             laserCloudIn.points[scans[i][j] + 4].y
              + laserCloudIn.points[scans[i][j] + 5].y*/
          ;
      float diffZ = /*laserCloudIn.points[scans[i][j] - 5].z +
              laserCloudIn.points[scans[i][j] - 4].z
              + laserCloudIn.points[scans[i][j] - 3].z + */
          laserCloudIn.points[scans[i][j] - 2].z +
          laserCloudIn.points[scans[i][j] - 1].z -
          4 * laserCloudIn.points[scans[i][j]].z +
          laserCloudIn.points[scans[i][j] + 1].z +
          laserCloudIn.points[scans[i][j] + 2].z
          /*+ laserCloudIn.points[scans[i][j] + 3].z +
             laserCloudIn.points[scans[i][j] + 4].z
              + laserCloudIn.points[scans[i][j] + 5].z*/
          ;

      cloudCurvature[scans[i][j]] =
          diffX * diffX + diffY * diffY + diffZ * diffZ;
      //  else cloudCurvature[scans[i][j]] = 0.0;
      // cloudSortInd[scans[i][j]] = scans[i][j];

      float intensity_point = laserCloudIn.points[scans[i][j] - 2].intensity +
                              laserCloudIn.points[scans[i][j] - 1].intensity -
                              laserCloudIn.points[scans[i][j] + 1].intensity -
                              laserCloudIn.points[scans[i][j] + 2].intensity;
      // - 4* laserCloudIn.points[scans[i][j]].intensity;
      cloudintensity[scans[i][j]] = sqrt(intensity_point * intensity_point);
      // std::cout<<"intensity : "<<cloudintensity[scans[i][j]]<<std::endl;
      //与后一点的斜率 y与x方向
      cloudslope[scans[i][j]] = fabs(laserCloudIn.points[scans[i][j] + 2].y -
                                     laserCloudIn.points[scans[i][j] - 2].y) /
                                (laserCloudIn.points[scans[i][j] + 2].x -
                                 laserCloudIn.points[scans[i][j] - 2].x);
    }

  //挑选点，排除容易被斜面挡住的点以及离群点
  for (size_t i = 5; i < laserCloudIn.points.size() - 6; i++) {
    float diffX = laserCloudIn.points[i + 1].x - laserCloudIn.points[i].x;
    float diffY = laserCloudIn.points[i + 1].y - laserCloudIn.points[i].y;
    float diffZ = laserCloudIn.points[i + 1].z - laserCloudIn.points[i].z;
    //计算有效曲率点与后一个点之间的距离平方和
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    if (diff > 0.1) {
      //点的深度
      float depth1 = sqrt(laserCloudIn.points[i].x * laserCloudIn.points[i].x +
                          laserCloudIn.points[i].y * laserCloudIn.points[i].y +
                          laserCloudIn.points[i].z * laserCloudIn.points[i].z);

      //后一个点的深度
      float depth2 =
          sqrt(laserCloudIn.points[i + 1].x * laserCloudIn.points[i + 1].x +
               laserCloudIn.points[i + 1].y * laserCloudIn.points[i + 1].y +
               laserCloudIn.points[i + 1].z * laserCloudIn.points[i + 1].z);
      //按照两点的深度的比例，将深度较大的点拉回后计算距离
      if (depth1 > depth2) {
        diffX = laserCloudIn.points[i + 1].x -
                laserCloudIn.points[i].x * depth2 / depth1;
        diffY = laserCloudIn.points[i + 1].y -
                laserCloudIn.points[i].y * depth2 / depth1;
        diffZ = laserCloudIn.points[i + 1].z -
                laserCloudIn.points[i].z * depth2 / depth1;

        //边长比也即是弧度值，若小于0.1，说明夹角比较小，斜面比较陡峭,点深度变化比较剧烈,点处在近似与激光束平行的斜面上
        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 <
            0.15) {
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloudIn.points[i + 1].x * depth1 / depth2 -
                laserCloudIn.points[i].x;
        diffY = laserCloudIn.points[i + 1].y * depth1 / depth2 -
                laserCloudIn.points[i].y;
        diffZ = laserCloudIn.points[i + 1].z * depth1 / depth2 -
                laserCloudIn.points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 <
            0.15) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          // cloudNeighborPicked[i + 4] = 1;
          // cloudNeighborPicked[i + 5] = 1;
          // cloudNeighborPicked[i + 6] = 1;
        }
      }
    }
    float diffX2 = laserCloudIn.points[i].x - laserCloudIn.points[i - 1].x;
    float diffY2 = laserCloudIn.points[i].y - laserCloudIn.points[i - 1].y;
    float diffZ2 = laserCloudIn.points[i].z - laserCloudIn.points[i - 1].z;
    //与前一个点的距离平方和
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
    //点深度的平方和
    float dis = laserCloudIn.points[i].x * laserCloudIn.points[i].x +
                laserCloudIn.points[i].y * laserCloudIn.points[i].y;
    cloud_distance[i] = sqrt(dis);
    //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
    if (diff > outlier_min_distance_ * dis ||
        diff2 > outlier_min_distance_ * dis ||
        dis > view_max_distance_ * view_max_distance_ ||
        dis < view_min_distance_ * view_min_distance_) {
      cloudNeighborPicked[i] = 1;
    }
  }
  pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp;
  pcl::PointCloud<pcl::PointXYZI> surfPointsSharp;
  pcl::PointCloud<pcl::PointXYZI> intensityPointsSharp;
  for (size_t i = 0; i < scans.size(); i++) {
    for (size_t j = 20; j < scans[i].size() - 20; j++) {
      if (cloudCurvature[scans[i][j]] > the_corner_min_curvature_ &&
          cloudNeighborPicked[scans[i][j]] != 1 &&
          fabs(cloudslope[scans[i][j] - 1] - cloudslope[scans[i][j]]) >
              the_min_slope_difference_ &&
          (cloud_distance[scans[i][j] - 1] - cloud_distance[scans[i][j]] > 0) &&
          (cloud_distance[scans[i][j] + 1] > cloud_distance[scans[i][j]]) /*  ||
(cloud_distance[scans[i][j] + 1] - cloud_distance[scans[i][j]] > 0) &&
(cloud_distance[scans[i][j] - 1] > cloud_distance[scans[i][j]]))*/
          && laserCloudIn.points[scans[i][j]].z > the_min_hight_) {
        cornerPointsSharp.push_back(laserCloudIn.points[scans[i][j]]);
      } else if (cloudCurvature[scans[i][j]] < the_surface_max_curvature_ &&
                 cloudNeighborPicked[scans[i][j]] != 1) {
        surfPointsSharp.push_back(laserCloudIn.points[scans[i][j]]);
      }
      if (cloudintensity[scans[i][j]] > the_min_intensity_difference_ &&
          laserCloudIn.points[scans[i][j]].z < the_max_intensity_hight_ &&
          cloudCurvature[scans[i][j]] < the_corner_min_curvature_ &&
          cloudintensity[scans[i][j]] > cloudintensity[scans[i][j] - 1]) {
        intensityPointsSharp.push_back(laserCloudIn.points[scans[i][j]]);
      }
    }
  }

  std::cout << "cornerPointsLessSharp" << cornerPointsSharp.size() << std::endl;
  std::cout << "surfPointsSharp  " << surfPointsSharp.size() << std::endl;
  std::cout << "intensitySharp  " << intensityPointsSharp.size() << std::endl;

  ros::Time current_time = laserCloudMsg->header.stamp;
  sensor_msgs::PointCloud2 temp_out_msg;

  pcl::toROSMsg(cornerPointsSharp, temp_out_msg);
  temp_out_msg.header.stamp = current_time;
  temp_out_msg.header.frame_id = "/camera_init";
  corner_pub.publish(temp_out_msg);

  pcl::toROSMsg(surfPointsSharp, temp_out_msg);
  temp_out_msg.header.stamp = current_time;
  temp_out_msg.header.frame_id = "/camera_init";
  surf_pub.publish(temp_out_msg);

  pcl::toROSMsg(laserCloudIn, temp_out_msg);
  temp_out_msg.header.stamp = current_time;
  temp_out_msg.header.frame_id = "/camera_init";
  full_pub.publish(temp_out_msg);

  pcl::toROSMsg(intensityPointsSharp, temp_out_msg);
  temp_out_msg.header.stamp = current_time;
  temp_out_msg.header.frame_id = "/camera_init";
  intensity_pub.publish(temp_out_msg);
}

void GetSweeperMode(sweeper_msgs::SweepMission sweeper_mode_command) {
  if (sweeper_mode_command.mode == 3 && sweeper_mode_command.start_cmd == 1) {
    allow_get_feature_ = true;
  } else
    allow_get_feature_ = false;
}

void GetSweeperInformation(const ros::TimerEvent &e) {
  sweeper_msgs::SensorFaultInformation fusion_navigation_code;
  fusion_navigation_code.header.frame_id = "localization";
  fusion_navigation_code.header.stamp = ros::Time::now();
  if (!watch_dog_livox_.DogIsOk(3)) {
    fusion_navigation_code.state_code.push_back(UN_RECEIVED_LIVOX);
  }
  if (fusion_navigation_code.state_code.empty())
    return;
  else
    pub_state_information_.publish(fusion_navigation_code);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "get_feature");
  ROS_INFO("\033[1;32m---->\033[0m get feature is Started.");
  ros::NodeHandle nh;
  ros::NodeHandle nh_pravite("~");
  nh_pravite.param<double>("the_corner_min_curvature",
                           the_corner_min_curvature_, 0.02);
  nh_pravite.param<double>("the_surface_max_curvature",
                           the_surface_max_curvature_, 0.0005);
  nh_pravite.param<double>("the_min_hight", the_min_hight_, 0.06);
  nh_pravite.param<double>("view_max_distance", view_max_distance_, 50);
  nh_pravite.param<double>("view_min_distance", view_min_distance_, 0.2);
  nh_pravite.param<double>("outlier_min_distance", outlier_min_distance_, 0.05);
  nh_pravite.param<double>("the_min_intensity_difference",
                           the_min_intensity_difference_, 70.0);
  nh_pravite.param<double>("the_max_intensity_hight", the_max_intensity_hight_,
                           1.0);
  nh_pravite.param<double>("the_min_slope_difference",
                           the_min_slope_difference_, 0.3);
  nh_pravite.param<double>("livox_z", livox_z_, 1.9921);
  nh_pravite.param<bool>("allow_get_feature", allow_get_feature_, false);

  std::cout << "the_corner_min_curvature : " << the_corner_min_curvature_
            << std::endl;
  std::cout << "the_surface_max_curvature : " << the_surface_max_curvature_
            << std::endl;
  std::cout << "the_min_hight : " << the_min_hight_ << std::endl;
  std::cout << "view_max_distance : " << view_max_distance_ << std::endl;
  std::cout << "view_min_distance : " << view_min_distance_ << std::endl;
  std::cout << "outlier_min_distance : " << outlier_min_distance_ << std::endl;
  std::cout << "the_min_intensity_difference : "
            << the_min_intensity_difference_ << std::endl;
  std::cout << "the_max_intensity_hight : " << the_max_intensity_hight_
            << std::endl;
  std::cout << "the_min_slope_difference : " << the_min_slope_difference_
            << std::endl;
  std::cout << "livox_z : " << livox_z_ << std::endl;
  std::cout << "allow_get_feature : " << allow_get_feature_ << std::endl;

  sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1000, Get_Scan);
  sub_sweeper_mode_ = nh.subscribe<sweeper_msgs::SweepMission>(
      "/sweeper/sweep/mission", 1000, GetSweeperMode);
  corner_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc2_corners", 1, true);
  surf_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc2_surface", 1, true);
  full_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc2_full", 1, true);
  intensity_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/pc2_intensity", 1, true);
  timer_get_feature =
      nh.createTimer(ros::Duration(0.1), GetSweeperInformation);
  pub_state_information_ = nh.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/mapping/diagnose", 1);
  ros::spin();
}

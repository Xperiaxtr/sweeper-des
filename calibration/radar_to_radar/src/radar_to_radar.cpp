#include <cv.h>
#include <dirent.h>
#include <highgui.h>
#include <opencv/cv.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sys/types.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

#define PI 3.1415926

int pcd_left_nums_ = 0, pcd_right_nums_ = 0;
double radar_distance_threshold_;
double radar_x_threshold_;
double classify_distance_threshold_;
double yaw_radar_left_to_straight_, yaw_radar_right_to_straight_;
std::string pcd_destination_, pcd_converted_;
std::string calibration_yaml_address_;
bool flag_pcd_left_ = false, flag_pcd_right_ = false;

pcl::PointCloud<pcl::PointXYZ> pcd_cloud_left_;
pcl::PointCloud<pcl::PointXYZ> pcd_cloud_right_;

using namespace std;

struct point_with_index {
  pcl::PointXYZ point;
  int index;
};

struct distance_with_index {
  double distance;
  int index_1;
  int index_2;
};

bool MyCompare(const distance_with_index &a, const distance_with_index &b) {
  return a.distance > b.distance;
}

int FindPointsIndex(const distance_with_index &a,
                    const distance_with_index &b) {
  if (a.index_1 == b.index_1) {
    return a.index_1;
  } else if (a.index_1 == b.index_2) {
    return a.index_1;
  }

  if (a.index_2 == b.index_1) {
    return a.index_2;
  } else if (a.index_2 == b.index_2) {
    return a.index_2;
  }
}

double SloveDiffTheta(cv::Point2f des_point_0, cv::Point2f des_point_1,
                      cv::Point2f conver_point_0, cv::Point2f conver_point_1) {
  double dest_delt_x, dest_delt_y;
  double convert_delt_x, convert_delt_y;
  dest_delt_x = des_point_1.x - des_point_0.x;
  dest_delt_y = des_point_1.y - des_point_0.y;
  convert_delt_x = conver_point_1.x - conver_point_0.x;
  convert_delt_y = conver_point_1.y - conver_point_0.y;

  std::cout << "dest_delt_x: " << dest_delt_x << std::endl;
  std::cout << "dest_delt_y: " << dest_delt_y << std::endl;
  std::cout << "convert_delt_x: " << convert_delt_x << std::endl;
  std::cout << "convert_delt_y: " << convert_delt_y << std::endl;

  double dest_theta = atan2(dest_delt_y, dest_delt_x);
  double convert_theta = atan2(convert_delt_y, convert_delt_x);

  std::cout << "dest_theta:" << dest_theta << std::endl;
  std::cout << "convert_theta:" << convert_theta << std::endl << std::endl;

  double diff_theta =
      atan2(sin(dest_theta - convert_theta), cos(dest_theta - convert_theta));

  if (fabs(dest_theta - convert_theta) > PI) {
    ROS_ERROR("The delt theta over 180 degree!");
  }
  return diff_theta;
}

void ClassifyObjectPoints(pcl::PointCloud<pcl::PointXYZ> input_cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud,
                          double distance_threshold) {
  int object_index = 0;
  std::vector<point_with_index> cloud_index;
  for (unsigned int i = 0; i < input_cloud.points.size(); ++i) {
    point_with_index temp_point;
    double temp_x = input_cloud.points[i].x;
    double temp_y = input_cloud.points[i].y;
    temp_point.point = input_cloud.points[i];
    temp_point.index = 0;
    cloud_index.push_back(temp_point);
  }

  std::vector<point_with_index>::iterator ptr_cloud_index = cloud_index.begin();
  for (; ptr_cloud_index != cloud_index.end(); ++ptr_cloud_index) {
    if ((*ptr_cloud_index).index == 0) {
      ++object_index;
      (*ptr_cloud_index).index = object_index;
      std::stack<point_with_index> object_points;
      object_points.push(*ptr_cloud_index);
      while (!object_points.empty()) {
        point_with_index target_point = object_points.top();
        object_points.pop();
        std::vector<point_with_index>::iterator ptr_cloud_index_2 =
            cloud_index.begin();
        for (; ptr_cloud_index_2 != cloud_index.end(); ++ptr_cloud_index_2) {
          double target_point_x = target_point.point.x;
          double target_point_y = target_point.point.y;
          if ((*ptr_cloud_index_2).index == 0) {
            double detected_point_x = (*ptr_cloud_index_2).point.x;
            double detected_point_y = (*ptr_cloud_index_2).point.y;
            double diff_x = fabs(target_point_x - detected_point_x);
            double diff_y = fabs(target_point_y - detected_point_y);
            double distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
            if (distance <= distance_threshold) {
              (*ptr_cloud_index_2).index = object_index;
              object_points.push(*ptr_cloud_index_2);
            }
          }
        }
      }
    }
  }
  std::vector<int> points_index;
  std::vector<point_with_index>::iterator ptr_label_cloud = cloud_index.begin();
  for (; ptr_label_cloud != cloud_index.end(); ++ptr_label_cloud) {
    int tmp_index = (*ptr_label_cloud).index;
    points_index.push_back(tmp_index);
  }

  int min_index = *min_element(points_index.begin(), points_index.end());
  int max_index = *max_element(points_index.begin(), points_index.end());

  if (max_index > 3 || min_index <= 0) {
    ROS_INFO("Have others object in pcd!");
    exit(-1);
  }

  std::vector<point_with_index>::iterator ptr_cloud_classify =
      cloud_index.begin();
  std::vector<std::vector<point_with_index> > classify_point;
  classify_point.resize(max_index + 1);
  for (; ptr_cloud_classify != cloud_index.end(); ++ptr_cloud_classify) {
    int temp_index = (*ptr_cloud_classify).index;
    classify_point[temp_index].push_back(*ptr_cloud_classify);
  }

  for (unsigned int i = 0; i < classify_point.size(); ++i) {
    double temp_x = 0;
    double temp_y = 0;
    if (classify_point[i].empty()) {
      continue;
    } else {
      std::vector<point_with_index>::iterator tmp_ptr =
          classify_point[i].begin();
      for (; tmp_ptr != classify_point[i].end(); ++tmp_ptr) {
        temp_x = temp_x + (*tmp_ptr).point.x;
        temp_y = temp_y + (*tmp_ptr).point.y;
      }

      int num_points = classify_point[i].size();
      temp_x = temp_x / num_points;
      temp_y = temp_y / num_points;

      pcl::PointXYZ final_points;
      final_points.x = temp_x;
      final_points.y = temp_y;
      final_points.z = 0;
      out_cloud->points.push_back(final_points);
    }
  }
}

int GetpointsFromPcd(std::string pcd_address,
                     std::vector<cv::Point2f> &points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_address, *original_cloud);
  int orignal_nums = original_cloud->points.size();
  for (int j = 0; j < orignal_nums; ++j) {
    filter_cloud->points.push_back(original_cloud->points[j]);
  }

  std::cout << "filter_cloud_size: " << filter_cloud->points.size()
            << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  ClassifyObjectPoints(*filter_cloud, out_cloud, classify_distance_threshold_);

  for (unsigned int j = 0; j < out_cloud->points.size(); ++j) {
    cv::Point2f tmp_point;
    tmp_point.x = out_cloud->points[j].x;
    tmp_point.y = out_cloud->points[j].y;

    std::cout << "points x:" << tmp_point.x << std::endl;
    std::cout << "points_y:" << tmp_point.y << std::endl << std::endl;

    points.push_back(tmp_point);
  }
  std::cout << " point 1 is: " << points[0].x << "," << points[0].y
            << std::endl;
  std::cout << "point 2 is: " << points[1].x << "," << points[1].y << std::endl;
  std::cout << " point 3 is: " << points[2].x << "," << points[2].y
            << std::endl;
  return out_cloud->points.size();
}

void FindEquavilentPoints(std::vector<cv::Point2f> points_destination,
                          std::vector<cv::Point2f> points_converted,
                          std::vector<cv::Point2f> &points_destination_output,
                          std::vector<cv::Point2f> &points_converted_output) {
  std::vector<distance_with_index> input_destination, input_converted;
  std::cout << "points_destination_size:" << points_destination.size()
            << std::endl;
  for (unsigned int i = 0; i < (points_destination.size() - 1); ++i) {
    unsigned int j = i + 1;
    for (; j < points_destination.size(); ++j) {
      double diff_x = fabs(points_destination[i].x - points_destination[j].x);
      double diff_y = fabs(points_destination[i].y - points_destination[j].y);
      double tmp_distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
      distance_with_index tmp;
      tmp.distance = tmp_distance;
      tmp.index_1 = i;
      tmp.index_2 = j;
      input_destination.push_back(tmp);
    }
  }

  for (unsigned int m = 0; m < (points_converted.size() - 1); ++m) {
    unsigned int n = m + 1;
    for (; n < points_converted.size(); ++n) {
      double diff_x = fabs(points_converted[m].x - points_converted[n].x);
      double diff_y = fabs(points_converted[m].y - points_converted[n].y);
      double tmp_distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
      distance_with_index tmp;
      tmp.distance = tmp_distance;
      tmp.index_1 = m;
      tmp.index_2 = n;

      input_converted.push_back(tmp);
    }
  }

  std::stable_sort(input_destination.begin(), input_destination.end(),
                   MyCompare);
  std::stable_sort(input_converted.begin(), input_converted.end(), MyCompare);

  for (unsigned int i = 0; i < input_destination.size(); ++i) {
    std::cout << "dest_distance:" << input_destination[i].distance << std::endl;
    std::cout << "dest_index_1:" << input_destination[i].index_1
              << " index_2:" << input_destination[i].index_2 << std::endl
              << std::endl;
    ;
  }

  for (unsigned int i = 0; i < input_converted.size(); ++i) {
    std::cout << "convert_distance:" << input_converted[i].distance
              << std::endl;
    std::cout << "convert_index_1:" << input_converted[i].index_1
              << " index_2:" << input_converted[i].index_2 << std::endl
              << std::endl;
  }

  ROS_INFO("line:%d", __LINE__);
  for (unsigned int i = 0; i < (input_destination.size() - 1); ++i) {
    unsigned int j = i + 1;
    for (; j < input_destination.size(); ++j) {
      int tmp_index =
          FindPointsIndex(input_destination[i], input_destination[j]);
      std::cout << "dest_index:" << tmp_index << std::endl;
      points_destination_output.push_back(points_destination[tmp_index]);
    }
  }

  for (unsigned int i = 0; i < (input_converted.size() - 1); ++i) {
    unsigned int j = i + 1;
    for (; j < input_converted.size(); ++j) {
      int tmp_index = FindPointsIndex(input_converted[i], input_converted[j]);
      std::cout << "convert_index:" << tmp_index << std::endl;
      points_converted_output.push_back(points_converted[tmp_index]);
    }
  }

  std::cout << "destination point nums: " << points_destination_output.size()
            << std::endl;
  std::cout << "converted point nums: " << points_converted_output.size()
            << std::endl;
}

void SloveTransformation(const std::vector<cv::Point2f> &destination_points,
                         const std::vector<cv::Point2f> &converted_points) {
  cv::Point2f destination_point_1, destination_point_2, destination_point_3;
  cv::Point2f converted_point_1, converted_point_2, converted_point_3;
  destination_point_1 = destination_points[0];
  destination_point_2 = destination_points[1];
  destination_point_3 = destination_points[2];
  converted_point_1 = converted_points[0];
  converted_point_2 = converted_points[1];
  converted_point_3 = converted_points[2];

  std::cout << "destination point 1 is: " << destination_point_1.x << ","
            << destination_point_1.y << std::endl;
  std::cout << "destination point 2 is: " << destination_point_2.x << ","
            << destination_point_2.y << std::endl;
  std::cout << "destination point 3 is: " << destination_point_3.x << ","
            << destination_point_3.y << std::endl;

  std::cout << "converted point 1 is: " << converted_point_1.x << ","
            << converted_point_1.y << std::endl;
  std::cout << "converted point 2 is: " << converted_point_2.x << ","
            << converted_point_2.y << std::endl;
  std::cout << "converted point 3 is: " << converted_point_3.x << ","
            << converted_point_3.y << std::endl;

  double delt_theta_1 = SloveDiffTheta(destination_point_1, destination_point_2,
                                       converted_point_1, converted_point_2);
  double delt_theta_2 = SloveDiffTheta(destination_point_1, destination_point_3,
                                       converted_point_1, converted_point_3);
  double delt_theta_3 = SloveDiffTheta(destination_point_2, destination_point_3,
                                       converted_point_2, converted_point_3);

  cv::Point2f calib_point_1, calib_point_2, calib_point_3;

  calib_point_1.x = converted_point_1.x * cos(delt_theta_1) -
                    converted_point_1.y * sin(delt_theta_1);
  calib_point_1.y = converted_point_1.y * cos(delt_theta_1) +
                    converted_point_1.x * sin(delt_theta_1);
  double translate_x_1 = destination_point_1.x - calib_point_1.x;
  double translate_y_1 = destination_point_1.y - calib_point_1.y;
  std::cout << "calib_point_1_x: " << calib_point_1.x << std::endl;
  std::cout << "calib_point_1_y: " << calib_point_1.y << std::endl;
  std::cout << "dest_point_1_x:" << destination_point_1.x << std::endl;
  std::cout << "dest_point_1_y:" << destination_point_1.y << std::endl;
  std::cout << "translate_x_1:" << translate_x_1 << std::endl;
  std::cout << "translate_y_1:" << translate_y_1 << std::endl;
  std::cout << "delt_theta_1: " << delt_theta_1 << std::endl
            << std::endl
            << std::endl;

  calib_point_2.x = converted_point_2.x * cos(delt_theta_2) -
                    converted_point_2.y * sin(delt_theta_2);
  calib_point_2.y = converted_point_2.y * cos(delt_theta_2) +
                    converted_point_2.x * sin(delt_theta_2);
  double translate_x_2 = destination_point_2.x - calib_point_2.x;
  double translate_y_2 = destination_point_2.y - calib_point_2.y;
  std::cout << "calib_point_2_x: " << calib_point_2.x << std::endl;
  std::cout << "calib_point_2_y: " << calib_point_2.y << std::endl;
  std::cout << "dest_point_2_x:" << destination_point_2.x << std::endl;
  std::cout << "dest_point_2_y:" << destination_point_2.y << std::endl;
  std::cout << "translate_x_2:" << translate_x_2 << std::endl;
  std::cout << "translate_y_2:" << translate_y_2 << std::endl;
  std::cout << "delt_theta_2: " << delt_theta_2 << std::endl << std::endl;

  calib_point_3.x = converted_point_3.x * cos(delt_theta_3) -
                    converted_point_3.y * sin(delt_theta_3);
  calib_point_3.y = converted_point_3.y * cos(delt_theta_3) +
                    converted_point_3.x * sin(delt_theta_3);
  double translate_x_3 = destination_point_3.x - calib_point_3.x;
  double translate_y_3 = destination_point_3.y - calib_point_3.y;
  std::cout << "calib_point_3_x: " << calib_point_3.x << std::endl;
  std::cout << "calib_point_3_y: " << calib_point_3.y << std::endl;
  std::cout << "dest_point_3_x:" << destination_point_3.x << std::endl;
  std::cout << "dest_point_3_y:" << destination_point_3.y << std::endl;
  std::cout << "translate_x_3:" << translate_x_3 << std::endl;
  std::cout << "translate_y_3:" << translate_y_3 << std::endl;
  std::cout << "delt_theta_3: " << delt_theta_3 << std::endl << std::endl;

  double translate_x = (translate_x_1 + translate_x_2 + translate_x_3) / 3;
  double translate_y = (translate_y_1 + translate_y_2 + translate_y_3) / 3;
  double delt_theta = (delt_theta_1 + delt_theta_2 + delt_theta_3) / 3;

  Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(-delt_theta, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

  std::string radar_left_to_right_yaml =
      calibration_yaml_address_ + "/radar_right_to_left.yaml";

  std::ofstream radar_right_to_left(radar_left_to_right_yaml, std::ios::app);

  // std::ofstream radar_right_to_left("../sweeper_ws/radar_right_to_left.yaml",
  //                                   std::ios::app);

  radar_right_to_left << "transform:" << std::endl;
  radar_right_to_left << " translation:" << std::endl;
  radar_right_to_left << "   x:" << translate_x << std::endl;
  radar_right_to_left << "   y:" << translate_y << std::endl;
  radar_right_to_left << " rotation:" << std::endl;
  radar_right_to_left << "   x:" << q.x() << std::endl;
  radar_right_to_left << "   y:" << q.y() << std::endl;
  radar_right_to_left << "   z:" << q.z() << std::endl;
  radar_right_to_left << "   w:" << q.w() << std::endl;
  radar_right_to_left.close();

  std::cout << "final_translate_x: " << translate_x << std::endl;
  std::cout << "final_translate_y: " << translate_y << std::endl;
  std::cout << "final_delt_theta: " << delt_theta << std::endl << std::endl;

  std::cout << "q_x:" << q.x() << std::endl;
  std::cout << "q_y:" << q.y() << std::endl;
  std::cout << "q_z:" << q.z() << std::endl;
  std::cout << "q_w:" << q.w() << std::endl;
}

void CalibLeftCloud(const sensor_msgs::PointCloud &cloud_left) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_destination_, *pcd_cloud) != -1 &&
      pcd_left_nums_ >= 20) {
    if (pcd_cloud->points.size() >= 3) {
      flag_pcd_left_ = true;
      return;
    }
  }

  Eigen::Matrix<double, 2, 2> transform_matrix_to_straight;
  transform_matrix_to_straight << cos(yaw_radar_left_to_straight_),
      -sin(yaw_radar_left_to_straight_), sin(yaw_radar_left_to_straight_),
      cos(yaw_radar_left_to_straight_);

  int nums = cloud_left.points.size();

  for (unsigned int i = 0; i < nums; ++i) {
    Eigen::Matrix<double, 2, 1> object_laser_position;
    Eigen::Matrix<double, 2, 1> object_calib_position;
    object_laser_position << cloud_left.points[i].x, cloud_left.points[i].y;

    if (cloud_left.points[i].y < -1.0 && cloud_left.points[i].y > -5.0 &&
        cloud_left.points[i].x < 4.0 && cloud_left.points[i].x > 1.0) {
      object_calib_position =
          transform_matrix_to_straight * object_laser_position;

      pcl::PointXYZ point;
      point.x = object_calib_position(0);
      point.y = object_calib_position(1);
      pcd_cloud_left_.push_back(point);
    }
  }
  ++pcd_left_nums_;
  if (pcd_left_nums_ >= 20)
    pcl::io::savePCDFileASCII(pcd_destination_, pcd_cloud_left_);
}

void CalibRightCloud(const sensor_msgs::PointCloud &cloud_right) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_converted_, *pcd_cloud) != -1 &&
      pcd_right_nums_ >= 20) {
    if (pcd_cloud->points.size() >= 3) {
      flag_pcd_right_ = true;
      return;
    }
  }
  std::cout << "pcd_right_nums:" << pcd_right_nums_ << std::endl;
  Eigen::Matrix<double, 2, 2> transform_matrix_to_straight;
  transform_matrix_to_straight << cos(yaw_radar_right_to_straight_),
      -sin(yaw_radar_right_to_straight_), sin(yaw_radar_right_to_straight_),
      cos(yaw_radar_right_to_straight_);

  int nums = cloud_right.points.size();

  for (unsigned int i = 0; i < nums; ++i) {
    Eigen::Matrix<double, 2, 1> object_laser_position;
    Eigen::Matrix<double, 2, 1> object_calib_position;
    object_laser_position << cloud_right.points[i].x, cloud_right.points[i].y;

    if (cloud_right.points[i].y > 1.0 && cloud_right.points[i].y < 4.5 &&
        cloud_right.points[i].x > 1.5 && cloud_right.points[i].x < 4.0) {
      object_calib_position =
          transform_matrix_to_straight * object_laser_position;

      pcl::PointXYZ point;
      point.x = object_calib_position(0);
      point.y = object_calib_position(1);
      pcd_cloud_right_.push_back(point);
    }
  }
  ++pcd_right_nums_;
  if (pcd_right_nums_ >= 20)
    pcl::io::savePCDFileASCII(pcd_converted_, pcd_cloud_right_);
}

void RadarToRadar() {
  int nums = 0;
  while (ros::ok() && (!flag_pcd_left_ || !flag_pcd_right_)) {
    if (nums <= 1) std::cout << "waiting !" << std::endl;
    if (nums <= 10000) ++nums;
  }
  std::vector<cv::Point2f> points_destination, points_converted;

  if (GetpointsFromPcd(pcd_destination_, points_destination) != 3) {
    std::cout << "Left:the number of calibration objects is error !"
              << std::endl;
    return;
  }

  if (GetpointsFromPcd(pcd_converted_, points_converted) != 3) {
    std::cout << "Right:the number of calibration objects is error !"
              << std::endl;
    return;
  }

  std::vector<cv::Point2f> points_destination_sorted, points_converted_sorted;

  FindEquavilentPoints(points_destination, points_converted,
                       points_destination_sorted, points_converted_sorted);
  SloveTransformation(points_destination_sorted, points_converted_sorted);
}

int main(int argv, char **argc) {
  ros::init(argv, argc, "radar_to_radar");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.param<double>("radar_distance_threshold",
                           radar_distance_threshold_, 2.0);
  nh_private.param<double>("radar_x_threshold", radar_x_threshold_, 1.0);
  nh_private.param<double>("classify_distance_threshold",
                           classify_distance_threshold_, 0.3);

  nh_private.param<double>("yaw_radar_left_to_straight",
                           yaw_radar_left_to_straight_, 0.0);
  nh_private.param<double>("yaw_radar_right_to_straight",
                           yaw_radar_right_to_straight_, 0.0);
  nh_private.param<string>("pcd_destination", pcd_destination_,
                           "../catkin_ws/left.pcd");
  nh_private.param<string>("pcd_converted", pcd_converted_,
                           "../catkin_ws/right.pcd");
  nh_private.param<string>("calibration_yaml_address",
                           calibration_yaml_address_, "../catkin_ws");

  ros::Subscriber sub_radar_left =
      nh.subscribe("/sweeper/sensor/radar_left", 1, &CalibLeftCloud);
  ros::Subscriber sub_radar_right =
      nh.subscribe("/sweeper/sensor/radar_right", 1, &CalibRightCloud);

  std::thread radar_to_radar_thread(RadarToRadar);
  radar_to_radar_thread.detach();

  ros::spin();
}
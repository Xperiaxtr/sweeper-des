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
#include <tf/transform_broadcaster.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#define PI 3.1415926
using namespace std;

int pcd_radar_nums_ = 0, pcd_lidar_nums_ = 0;
double lidar_hegihts_;
double lidar_classify_distance_threshold_;
double yaw_radar_left_to_straight_, yaw_radar_right_to_straight_;
bool flag_radar_pcd_ = false, flag_lidar_pcd_ = false;
std::string pcd_destination_, pcd_converted_;
std::string calibration_yaml_address_;

pcl::PointCloud<pcl::PointXYZ> pcd_left_cloud_;
pcl::PointCloud<pcl::PointXYZ> pcl_filter_cloud_;

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
    exit(-1);
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

  std::vector<point_with_index>::iterator ptr_cloud_classify =
      cloud_index.begin();
  std::vector<std::vector<point_with_index> > classify_point;
  classify_point.resize(max_index + 1);
  for (; ptr_cloud_classify != cloud_index.end(); ++ptr_cloud_classify) {
    int temp_index = (*ptr_cloud_classify).index;
    classify_point[temp_index].push_back(*ptr_cloud_classify);
  }
  std::cout << "classify_point_size:" << classify_point.size();

  //根据点的数目来滤除大疆激光雷达的杂点
  if (classify_point.size() > 3) {
    for (unsigned int i = 0; i < classify_point.size(); ++i) {
      for (unsigned j = i + 1; j < classify_point.size(); ++j) {
        if (classify_point[j].size() > classify_point[i].size()) {
          std::vector<point_with_index> tmp;
          tmp = classify_point[i];
          classify_point[i] = classify_point[j];
          classify_point[j] = tmp;
        }
      }
    }
  }

  for (unsigned int i = 0; i < 3; ++i) {
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

void GetRadarPointsFromPcd(std::string radar_pcd_address,
                           std::vector<cv::Point2f> &radar_points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_radar_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ>(radar_pcd_address, *original_cloud);
  int orignal_nums = original_cloud->points.size();
  std::cout << "orignal_nums:" << orignal_nums << std::endl;
  for (int j = 0; j < orignal_nums; ++j) {
    double temp_x = original_cloud->points[j].x;
    double temp_y = original_cloud->points[j].y;
    double distance = sqrt(pow(temp_x, 2) + pow(temp_y, 2));
    if (temp_x > 0.5 && distance > 1.5) {
      filter_radar_cloud->points.push_back(original_cloud->points[j]);
    }
  }

  std::cout << "the nums of radar filter points: "
            << filter_radar_cloud->points.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  double radar_distance_threshold = 0.2;

  ClassifyObjectPoints(*filter_radar_cloud, out_cloud,
                       radar_distance_threshold);

  for (unsigned int j = 0; j < out_cloud->points.size(); ++j) {
    cv::Point2f tmp_point;
    tmp_point.x = out_cloud->points[j].x;
    tmp_point.y = out_cloud->points[j].y;

    std::cout << "radar points x:" << tmp_point.x << std::endl;
    std::cout << "radar points y:" << tmp_point.y << std::endl << std::endl;

    radar_points.push_back(tmp_point);
  }
  std::cout << "the nums points of qualified in radar pcd: "
            << radar_points.size() << std::endl;
}

void GetLidarPointsFromPcd(std::string lidar_pcd_address,
                           std::vector<cv::Point2f> &lidar_points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_lidar_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ>(lidar_pcd_address, *original_cloud);

  unsigned int nums_lidar_points = original_cloud->points.size();
  for (unsigned int i = 0; i < nums_lidar_points; ++i) {
    double temp_x = original_cloud->points[i].x;
    double temp_y = original_cloud->points[i].y;
    double temp_z = original_cloud->points[i].z;

    filter_lidar_cloud->points.push_back(original_cloud->points[i]);
  }

  std::cout << "the nums of lidar filter points: "
            << filter_lidar_cloud->points.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  ClassifyObjectPoints(*filter_lidar_cloud, out_cloud,
                       lidar_classify_distance_threshold_);
  for (unsigned int j = 0; j < out_cloud->points.size(); ++j) {
    cv::Point2f tmp_point;
    tmp_point.x = out_cloud->points[j].x;
    tmp_point.y = out_cloud->points[j].y;

    std::cout << "lidar points x:" << tmp_point.x << std::endl;
    std::cout << "lidar points y:" << tmp_point.y << std::endl << std::endl;

    lidar_points.push_back(tmp_point);
  }
  std::cout << "the nums points of qualified in lidar pcd: "
            << lidar_points.size() << std::endl;
}

void FindEquavilentPoints(std::vector<cv::Point2f> points_destination,
                          std::vector<cv::Point2f> points_converted,
                          std::vector<cv::Point2f> &points_destination_output,
                          std::vector<cv::Point2f> &points_converted_output) {
  std::vector<distance_with_index> input_destination, input_converted;
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

  for (unsigned int i = 0; i < (input_destination.size() - 1); ++i) {
    unsigned int j = i + 1;
    for (; j < input_destination.size(); ++j) {
      int tmp_index =
          FindPointsIndex(input_destination[i], input_destination[j]);
      points_destination_output.push_back(points_destination[tmp_index]);
    }
  }

  std::cout << "input_converted_size:" << input_converted.size() << std::endl;
  for (unsigned int i = 0; i < (input_converted.size() - 1); ++i) {
    unsigned int j = i + 1;
    for (; j < input_converted.size(); ++j) {
      int tmp_index = FindPointsIndex(input_converted[i], input_converted[j]);
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
            << destination_point_3.y << std::endl
            << std::endl;

  std::cout << "converted point 1 is: " << converted_point_1.x << ","
            << converted_point_1.y << std::endl;
  std::cout << "converted point 2 is: " << converted_point_2.x << ","
            << converted_point_2.y << std::endl;
  std::cout << "converted point 3 is: " << converted_point_3.x << ","
            << converted_point_3.y << std::endl
            << std::endl;

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
  std::cout << "translate_x_1: " << translate_x_1 << std::endl;
  std::cout << "translate_y_1: " << translate_y_1 << std::endl;
  std::cout << "delt_theta_1: " << delt_theta_1 << std::endl;

  calib_point_2.x = converted_point_2.x * cos(delt_theta_2) -
                    converted_point_2.y * sin(delt_theta_2);
  calib_point_2.y = converted_point_2.y * cos(delt_theta_2) +
                    converted_point_2.x * sin(delt_theta_2);
  double translate_x_2 = destination_point_2.x - calib_point_2.x;
  double translate_y_2 = destination_point_2.y - calib_point_2.y;
  std::cout << "translate_x_2: " << translate_x_2 << std::endl;
  std::cout << "translate_y_2: " << translate_y_2 << std::endl;
  std::cout << "delt_theta_2: " << delt_theta_2 << std::endl << std::endl;

  calib_point_3.x = converted_point_3.x * cos(delt_theta_3) -
                    converted_point_3.y * sin(delt_theta_3);
  calib_point_3.y = converted_point_3.y * cos(delt_theta_3) +
                    converted_point_3.x * sin(delt_theta_3);
  double translate_x_3 = destination_point_3.x - calib_point_3.x;
  double translate_y_3 = destination_point_3.y - calib_point_3.y;
  std::cout << "translate_x_3: " << translate_x_3 << std::endl;
  std::cout << "translate_y_3: " << translate_y_3 << std::endl;
  std::cout << "delt_theta_3: " << delt_theta_3 << std::endl << std::endl;

  double translate_x = (translate_x_1 + translate_x_2 + translate_x_3) / 3;
  double translate_y = (translate_y_1 + translate_y_2 + translate_y_3) / 3;
  double delt_theta = (delt_theta_1 + delt_theta_2 + delt_theta_3) / 3;

  std::cout << "final_translate_x: " << translate_x << std::endl;
  std::cout << "final_translate_y: " << translate_y << std::endl;
  std::cout << "final_delt_theta: " << delt_theta << std::endl << std::endl;

  Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(-delt_theta, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

  std::string radar_to_lidar_yaml =
      calibration_yaml_address_ + "/radar_to_lidar.yaml";

  // std::ofstream radar_to_lidar("../sweeper_ws/radar_to_lidar.yaml",
  //                              std::ios::app);

  std::ofstream radar_to_lidar(radar_to_lidar_yaml, std::ios::app);
  
  radar_to_lidar << "transform:" << std::endl;
  radar_to_lidar << " translation:" << std::endl;
  radar_to_lidar << "   x:" << translate_x << std::endl;
  radar_to_lidar << "   y:" << translate_y << std::endl;
  radar_to_lidar << " rotation:" << std::endl;
  radar_to_lidar << "   x:" << q.x() << std::endl;
  radar_to_lidar << "   y:" << q.y() << std::endl;
  radar_to_lidar << "   z:" << q.z() << std::endl;
  radar_to_lidar << "   w:" << q.w() << std::endl;
  radar_to_lidar.close();

  std::cout << "final_translate_x: " << translate_x << std::endl;
  std::cout << "final_translate_y: " << translate_y << std::endl;
  std::cout << "final_delt_theta: " << delt_theta << std::endl;

  std::cout << "q_x:" << q.x() << std::endl;
  std::cout << "q_y:" << q.y() << std::endl;
  std::cout << "q_z:" << q.z() << std::endl;
  std::cout << "q_w:" << q.w() << std::endl;
}

void CalibLeftCloud(const sensor_msgs::PointCloud &left_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_converted_, *pcd_cloud) != -1 &&
      pcd_radar_nums_ >= 20) {
    if (pcd_cloud->points.size() >= 3) {
      flag_radar_pcd_ = true;
      return;
    }
  }

  Eigen::Matrix<double, 2, 2> transform_matrix_to_straight;
  transform_matrix_to_straight << cos(yaw_radar_left_to_straight_),
      -sin(yaw_radar_left_to_straight_), sin(yaw_radar_left_to_straight_),
      cos(yaw_radar_left_to_straight_);

  int nums_left_cloud = left_cloud.points.size();

  for (unsigned int i = 0; i < nums_left_cloud; ++i) {
    Eigen::Matrix<double, 2, 1> object_laser_position;
    Eigen::Matrix<double, 2, 1> object_car_position;
    if (left_cloud.points[i].y < -1.0 && left_cloud.points[i].y > -5.0 &&
        left_cloud.points[i].x < 4.0 && left_cloud.points[i].x > 0.0) {
      object_laser_position << left_cloud.points[i].x, left_cloud.points[i].y;
      object_car_position =
          transform_matrix_to_straight * object_laser_position;

      pcl::PointXYZ temp_point;
      temp_point.x = object_car_position(0);
      temp_point.y = object_car_position(1);
      temp_point.z = 0.0;

      pcd_left_cloud_.push_back(temp_point);
    }
  }
  ++pcd_radar_nums_;
  if (pcd_radar_nums_ >= 20)
    pcl::io::savePCDFileASCII(pcd_converted_, pcd_left_cloud_);
}

void GetLivoxLidar(const sensor_msgs::PointCloud2::ConstPtr &lidar_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_destination_, *pcd_cloud) != -1 &&
      pcd_lidar_nums_ >= 20) {
    if (pcd_cloud->points.size() >= 3) {
      flag_lidar_pcd_ = true;
      return;
    }
  }
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*lidar_cloud, pcl_cloud);

  // //标定大疆激光雷达点云
  // Eigen::Vector3d euler_angle(0.0, 0.0, 0.0);
  // Eigen::AngleAxisd rollAngle(
  //     Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchAngle(
  //     Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
  // Eigen::AngleAxisd yawAngle(
  //     Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond quaternion;
  quaternion.x() = 0.0;
  quaternion.y() = 0.0;
  quaternion.z() = 0.0;
  quaternion.w() = 1.0;

  Eigen::Translation3d translation(0, 0, 0.0);  //    (0, 0, 1.3);(0, 0, 1.32)
  Eigen::Affine3d affine = translation * quaternion;
  Eigen::Matrix4d matrix = affine.matrix();
  pcl::transformPointCloud(pcl_cloud, pcl_cloud, matrix);

  // sensor_msgs::PointCloud2 output;
  // pcl::toROSMsg(pcl_cloud, output);
  // pub_lidar_.publish(output);

  pcl::PointXYZ lidar_filter_point;

  int lidar_nums = pcl_cloud.size();
  for (unsigned int i = 0; i < lidar_nums; ++i) {
    double point_x = pcl_cloud[i].x;
    double point_y = pcl_cloud[i].y;
    double point_z = pcl_cloud[i].z;

    // if ((point_y > -0.3 && point_y < 0.01 && point_x > 2.9 && point_x < 3.5
    // &&
    //      point_z > -1.5) ||
    //     (point_y > 0.01 && point_y < 0.3 && point_x > 3.5 && point_x < 4.0 &&
    //      point_z > -1.5) ||
    //     (point_y > 1.0 && point_y < 1.5 && point_x > 3.2 && point_x < 4.0 &&
    //      point_z > -1.5)) {
    if (point_x > 2.5 && point_x < 5.0 && point_y < 3.0 && point_y > -3.0 &&
        point_z > -1.5) {
      lidar_filter_point.x = point_x;
      lidar_filter_point.y = point_y;
      lidar_filter_point.z = point_z;
      pcl_filter_cloud_.push_back(lidar_filter_point);
      //  }
    }
  }

  ++pcd_lidar_nums_;
  if (pcd_lidar_nums_ >= 20)
    pcl::io::savePCDFileASCII(pcd_destination_, pcl_filter_cloud_);
}

void RadarToLidar() {
  int nums = 0;
  while (ros::ok() && (!flag_lidar_pcd_ || !flag_radar_pcd_)) {
    if (nums <= 1) {
      std::cout << "waiting !" << std::endl;
    }
    if (nums <= 10000) ++nums;
  }

  std::vector<cv::Point2f> radar_points, lidar_points;

  GetRadarPointsFromPcd(pcd_converted_, radar_points);

  GetLidarPointsFromPcd(pcd_destination_, lidar_points);

  std::vector<cv::Point2f> radar_points_sorted, lidar_points_sorted;

  FindEquavilentPoints(lidar_points, radar_points, lidar_points_sorted,
                       radar_points_sorted);
  SloveTransformation(lidar_points_sorted, radar_points_sorted);
}

int main(int argv, char **argc) {
  ros::init(argv, argc, "radar_to_lidar");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.param<double>("lidar_hegihts", lidar_hegihts_, 2.0);

  nh_private.param<double>("lidar_classify_distance_threshold",
                           lidar_classify_distance_threshold_, 0.3);
  nh_private.param<double>("yaw_radar_left_to_straight",
                           yaw_radar_left_to_straight_, 0.0);
  nh_private.param<double>("yaw_radar_right_to_straight",
                           yaw_radar_right_to_straight_, 0.0);
  nh_private.param<string>("pcd_destination", pcd_destination_,
                           "../catkin_ws/dj.pcd");
  nh_private.param<string>("pcd_converted", pcd_converted_,
                           "../catkin_ws/left.pcd");
  nh_private.param<string>("calibration_yaml_address",
                           calibration_yaml_address_, "../catkin_ws");

  ros::Subscriber sub_radar_left =
      nh.subscribe("/sweeper/sensor/radar_left", 1, &CalibLeftCloud);
  ros::Subscriber sub_lidar = nh.subscribe("/livox/lidar", 1, &GetLivoxLidar);

  std::thread radar_to_lidar(RadarToLidar);
  radar_to_lidar.detach();

  ros::spin();
}
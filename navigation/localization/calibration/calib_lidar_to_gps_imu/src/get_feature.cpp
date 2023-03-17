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

using namespace std;
using namespace cv;
ros::Subscriber sub;
ros::Publisher corner_pub, surf_pub, full_pub, intensity_pub;
double the_corner_min_curvature_ = 0.05;
double the_surface_max_curvature_ = 0.0005;
double the_min_hight_ = 0.06;
double view_max_distance_ = 50;
double view_min_distance_ = 0.2;
double outlier_min_distance_ = 0.05;
double the_min_intensity_difference_ = 150;
double the_max_intensity_hight_ = 0.5;
double the_min_slope_difference_ = 0.5;  // 27度
float the_max_z_hight_ = 0.08;
double livox_z_ = 1.80;

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
                                   1.9921);  //    (0, 0, 1.3);(0, 0, 1.32)
  Eigen::Affine3d affine = translation * quaternion;
  Eigen::Matrix4d matrix = affine.matrix();
  pcl::transformPointCloud(raw_cloud, point_cloud, matrix);
}

void Get_Scan(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
  //点云曲率, 40000为一帧点云中点的最大数量
  float cloudCurvature[40000] = {0};
  float cloudintensity[40000] = {0};
  float cloudslope[40000] = {0};
  float point_distance_[30000] = {0};
  float point_curvature_[30000] = {0};
  float point_max_hight_[30000] = {0};
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
  // transformation_coordinate(laserCloudIn, laserCloudIn);
  // std::cout << "size  " << laserCloudIn.points.size() << std::endl;
  int cloudSize = laserCloudIn.points.size();

  // double t1 = clock();
  double angle_last = 0.0, dis_incre_last = 0.0, dis_incre_now = 0.0,
         angle = 0.0;
  for (int i = 0; i < cloudSize; i++) {
    // angle = atan((laserCloudIn.points[i].z - 1.9921) /
    // (sqrt(laserCloudIn.points[i].x * laserCloudIn.points[i].x +
    // laserCloudIn.points[i].y * laserCloudIn.points[i].y)));
    angle = atan((laserCloudIn.points[i].z) /
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

  pcl::PointCloud<pcl::PointXYZI> corner_cloud;
  pcl::PointCloud<pcl::PointXYZI> surf_cloud;
  pcl::PointCloud<pcl::PointXYZI> intensity_cloud;

  std::vector<int> corner_feature;
  std::vector<int> intensity_feature;
  std::vector<int> surf_feaure;
  bool left_surf_flag, right_surf_flag;
  for (size_t i = 0; i < scans.size(); i++) {
    for (size_t j = 5; j < scans[i].size() - 5; j++) {
      //对强度进行判断，一个点的左侧点强度基本一致，右侧点强度基本一致，并且左右侧强度相差大，左右2个点
      float left_intensity = laserCloudIn.points[scans[i][j - 2]].intensity +
                             laserCloudIn.points[scans[i][j - 1]].intensity;
      float right_intensity = laserCloudIn.points[scans[i][j + 2]].intensity +
                              laserCloudIn.points[scans[i][j + 1]].intensity;

      float diff_intensity = fabs(left_intensity - right_intensity) / 2.0;

      if (diff_intensity > 25.0/* &&
          fabs(laserCloudIn.points[scans[i][j - 2]].intensity -
               laserCloudIn.points[scans[i][j - 1]].intensity) < 10.0 &&
          fabs(laserCloudIn.points[scans[i][j + 2]].intensity -
               laserCloudIn.points[scans[i][j + 1]].intensity) < 10.0*/) {
        intensity_feature.push_back(scans[i][j]);
      }

      // 1.计算当前点与前后点的距离
      float depth = sqrt(laserCloudIn.points[scans[i][j]].x *
                             laserCloudIn.points[scans[i][j]].x +
                         laserCloudIn.points[scans[i][j]].y *
                             laserCloudIn.points[scans[i][j]].y +
                         laserCloudIn.points[scans[i][j]].z *
                             laserCloudIn.points[scans[i][j]].z);
      point_distance_[scans[i][j]] = depth;
      float diff_x_right = laserCloudIn.points[scans[i][j]].x -
                           laserCloudIn.points[scans[i][j + 1]].x;
      float diff_y_right = laserCloudIn.points[scans[i][j]].y -
                           laserCloudIn.points[scans[i][j + 1]].y;
      float diff_z_right = laserCloudIn.points[scans[i][j]].z -
                           laserCloudIn.points[scans[i][j + 1]].z;
      float diff_right =
          sqrt(diff_x_right * diff_x_right + diff_y_right * diff_y_right +
               diff_z_right * diff_z_right);
      float diff_x_left = laserCloudIn.points[scans[i][j]].x -
                          laserCloudIn.points[scans[i][j - 1]].x;
      float diff_y_left = laserCloudIn.points[scans[i][j]].y -
                          laserCloudIn.points[scans[i][j - 1]].y;
      float diff_z_left = laserCloudIn.points[scans[i][j]].z -
                          laserCloudIn.points[scans[i][j - 1]].z;
      float diff_left =
          sqrt(diff_x_left * diff_x_left + diff_y_left * diff_y_left +
               diff_z_left * diff_z_left);

      //离散点
      if (diff_left > 0.1 * depth && diff_right > 0.1 * depth) {
        continue;
      }

      // 2.计算曲率
      float left_diff_x = laserCloudIn.points[scans[i][j] - 4].x +
                          laserCloudIn.points[scans[i][j] - 3].x -
                          4 * laserCloudIn.points[scans[i][j] - 2].x +
                          laserCloudIn.points[scans[i][j] - 1].x +
                          laserCloudIn.points[scans[i][j]].x;
      float left_diff_y = laserCloudIn.points[scans[i][j] - 4].y +
                          laserCloudIn.points[scans[i][j] - 3].y -
                          4 * laserCloudIn.points[scans[i][j] - 2].y +
                          laserCloudIn.points[scans[i][j] - 1].y +
                          laserCloudIn.points[scans[i][j]].y;
      float left_diff_z = laserCloudIn.points[scans[i][j] - 4].z +
                          laserCloudIn.points[scans[i][j] - 3].z -
                          4 * laserCloudIn.points[scans[i][j] - 2].z +
                          laserCloudIn.points[scans[i][j] - 1].z +
                          laserCloudIn.points[scans[i][j]].z;
      float left_curvate = left_diff_x * left_diff_x +
                           left_diff_y * left_diff_y +
                           left_diff_z * left_diff_z;

      point_curvature_[scans[i][j] - 2] = left_curvate;

      float right_diff_x = laserCloudIn.points[scans[i][j] + 4].x +
                           laserCloudIn.points[scans[i][j] + 3].x -
                           4 * laserCloudIn.points[scans[i][j] + 2].x +
                           laserCloudIn.points[scans[i][j] + 1].x +
                           laserCloudIn.points[scans[i][j]].x;
      float right_diff_y = laserCloudIn.points[scans[i][j] + 4].y +
                           laserCloudIn.points[scans[i][j] + 3].y -
                           4 * laserCloudIn.points[scans[i][j] + 2].y +
                           laserCloudIn.points[scans[i][j] + 1].y +
                           laserCloudIn.points[scans[i][j]].y;
      float right_diff_z = laserCloudIn.points[scans[i][j] + 4].z +
                           laserCloudIn.points[scans[i][j] + 3].z -
                           4 * laserCloudIn.points[scans[i][j] + 2].z +
                           laserCloudIn.points[scans[i][j] + 1].z +
                           laserCloudIn.points[scans[i][j]].z;
      float right_curvate = right_diff_x * right_diff_x +
                            right_diff_y * right_diff_y +
                            right_diff_z * right_diff_z;

      point_curvature_[scans[i][j] + 2] = right_curvate;

      if (left_curvate < 0.001) surf_feaure.push_back(scans[i][j]);

      if (left_curvate < 0.005) {
        left_surf_flag = true;
        //添加平面点
        // surf_cloud->points.push_back(laserCloudIn.points[scans[i][j] -
        // 2]);
      } else {
        left_surf_flag = false;
      }
      if (right_curvate < 0.005) {
        right_surf_flag = true;
      } else {
        right_surf_flag = false;
      }

      // if (left_surf_flag && right_surf_flag) {
      Eigen::Vector3d lidar_point(laserCloudIn.points[scans[i][j]].x,
                                  laserCloudIn.points[scans[i][j]].y,
                                  laserCloudIn.points[scans[i][j]].z);
      Eigen::Vector3d norm_left(0, 0, 0);
      Eigen::Vector3d norm_right(0, 0, 0);
      for (int k = 1; k < 5; k++) {
        //左侧
        Eigen::Vector3d tmp =
            Eigen::Vector3d(laserCloudIn.points[scans[i][j - k]].x -
                                laserCloudIn.points[scans[i][j]].x,
                            laserCloudIn.points[scans[i][j - k]].y -
                                laserCloudIn.points[scans[i][j]].y,
                            laserCloudIn.points[scans[i][j - k]].z -
                                laserCloudIn.points[scans[i][j]].z);
        tmp.normalize();
        norm_left += (k / 10.0) * tmp;

        //右侧
        tmp = Eigen::Vector3d(laserCloudIn.points[scans[i][j + k]].x -
                                  laserCloudIn.points[scans[i][j]].x,
                              laserCloudIn.points[scans[i][j + k]].y -
                                  laserCloudIn.points[scans[i][j]].y,
                              laserCloudIn.points[scans[i][j + k]].z -
                                  laserCloudIn.points[scans[i][j]].z);
        tmp.normalize();
        norm_right += (k / 10.0) * tmp;
      }

      // if (left_surf_flag && right_surf_flag) {
      //计算两平面的夹角
      double cos_left_right = fabs(norm_left.dot(norm_right) /
                                   (norm_left.norm() * norm_right.norm()));
      if (left_surf_flag && right_surf_flag) {
        //计算最大距离
        float max_diff_x_right = laserCloudIn.points[scans[i][j]].x -
                                 laserCloudIn.points[scans[i][j + 5]].x;
        float max_diff_y_right = laserCloudIn.points[scans[i][j]].y -
                                 laserCloudIn.points[scans[i][j + 5]].y;
        float max_diff_z_right = laserCloudIn.points[scans[i][j]].z -
                                 laserCloudIn.points[scans[i][j + 5]].z;
        float max_diff_right = sqrt(max_diff_x_right * max_diff_x_right +
                                    max_diff_y_right * max_diff_y_right +
                                    max_diff_z_right * max_diff_z_right);

        float max_diff_x_left = laserCloudIn.points[scans[i][j]].x -
                                laserCloudIn.points[scans[i][j - 5]].x;
        float max_diff_y_left = laserCloudIn.points[scans[i][j]].y -
                                laserCloudIn.points[scans[i][j - 5]].y;
        float max_diff_z_left = laserCloudIn.points[scans[i][j]].z -
                                laserCloudIn.points[scans[i][j - 5]].z;
        float max_diff_left = sqrt(max_diff_x_left * max_diff_x_left +
                                   max_diff_y_left * max_diff_y_left +
                                   max_diff_z_left * max_diff_z_left);

        //添加角点
        if (cos_left_right < 0.6 && max_diff_right > 0.05 &&
            max_diff_left > 0.05 && max_diff_right < 3.0 * max_diff_left &&
            max_diff_right > 0.3 * max_diff_left) {  //并且前后模长基本一致
          corner_feature.push_back(scans[i][j]);
        }
      }
      //中断点
      else if (diff_right - diff_left > 0.1 &&
               left_curvate < 0.001) {  //左侧平滑，右侧突变
        //计算中断点角度
        double cos_singma = fabs(norm_left.dot(lidar_point) /
                                 (norm_left.norm() * lidar_point.norm()));
        //要求左侧的点要比右侧的点近，（平滑侧的点要更接近雷达）
        float left_depth = sqrt(laserCloudIn.points[scans[i][j] - 4].x *
                                    laserCloudIn.points[scans[i][j] - 4].x +
                                laserCloudIn.points[scans[i][j] - 4].y *
                                    laserCloudIn.points[scans[i][j] - 4].y +
                                laserCloudIn.points[scans[i][j] - 4].z *
                                    laserCloudIn.points[scans[i][j] - 4].z);
        float right_depth = sqrt(laserCloudIn.points[scans[i][j] + 4].x *
                                     laserCloudIn.points[scans[i][j] + 4].x +
                                 laserCloudIn.points[scans[i][j] + 4].y *
                                     laserCloudIn.points[scans[i][j] + 4].y +
                                 laserCloudIn.points[scans[i][j] + 4].z *
                                     laserCloudIn.points[scans[i][j] + 4].z);
        //添加角点
        if (cos_singma < 0.5 && cos_left_right < 0.7 &&
            right_depth - left_depth > 0.1) {
          corner_feature.push_back(scans[i][j]);
        }
      } else if (diff_left - diff_right > 0.2 &&
                 left_curvate > 0.001)  //左侧突变， 右侧平滑
      {
        double cos_singma = fabs(norm_right.dot(lidar_point) /
                                 (norm_right.norm() * lidar_point.norm()));
        //要求左侧的点要比右侧的点近，（平滑侧的点要更接近雷达）
        float left_depth = sqrt(laserCloudIn.points[scans[i][j] - 4].x *
                                    laserCloudIn.points[scans[i][j] - 4].x +
                                laserCloudIn.points[scans[i][j] - 4].y *
                                    laserCloudIn.points[scans[i][j] - 4].y +
                                laserCloudIn.points[scans[i][j] - 4].z *
                                    laserCloudIn.points[scans[i][j] - 4].z);
        float right_depth = sqrt(laserCloudIn.points[scans[i][j] + 4].x *
                                     laserCloudIn.points[scans[i][j] + 4].x +
                                 laserCloudIn.points[scans[i][j] + 4].y *
                                     laserCloudIn.points[scans[i][j] + 4].y +
                                 laserCloudIn.points[scans[i][j] + 4].z *
                                     laserCloudIn.points[scans[i][j] + 4].z);
        //添加角点
        if (cos_singma < 0.5 && cos_left_right < 0.7 &&
            left_depth - right_depth > 0.1) {
          corner_feature.push_back(scans[i][j]);
        }
      }

      double min_z = 3.0, max_z = -3.0;
      for (int k = -7; k <= 7; k++) {
        if (laserCloudIn.points[scans[i][j] + k].z > max_z) {
          max_z = laserCloudIn.points[scans[i][j] + k].z;
        }
        if (laserCloudIn.points[scans[i][j] + k].z < min_z) {
          min_z = laserCloudIn.points[scans[i][j] + k].z;
        }
      }
      point_max_hight_[scans[i][j]] = max_z - min_z;
    }
  }

  //对角点的曲率，高度，进行判断
  for (size_t i = 0; i < corner_feature.size(); i++) {
    //如果曲率符合，高度符合，则加入特征中
    if (point_curvature_[corner_feature[i]] > 0.01 &&
        laserCloudIn.points[corner_feature[i]].z > the_min_hight_ - livox_z_ &&
        point_distance_[corner_feature[i]] > 1.0 &&
        laserCloudIn.points[corner_feature[i]].x < 50 &&
        point_max_hight_[corner_feature[i]] > the_max_z_hight_)
      corner_cloud.points.push_back(laserCloudIn.points[corner_feature[i]]);
    // new_feature_data.corner_cloud->points.push_back(
    //     laserCloudIn.points[corner_feature[i]]);
  }
  //对强度点的曲率，高度，进行判断
  for (size_t i = 0; i < intensity_feature.size(); i++) {
    //如果曲率符合，高度符合，则加入特征中
    if (point_curvature_[intensity_feature[i]] < 0.004 &&
        laserCloudIn.points[intensity_feature[i]].z <
            the_max_intensity_hight_ - livox_z_ &&
        point_distance_[intensity_feature[i]] > 1.0 &&
        laserCloudIn.points[intensity_feature[i]].x < 50)
      intensity_cloud.points.push_back(
          laserCloudIn.points[intensity_feature[i]]);
  }
  for (size_t i = 0; i < surf_feaure.size(); i++) {
    if (point_distance_[surf_feaure[i]] > 1.0)
      surf_cloud.points.push_back(laserCloudIn.points[surf_feaure[i]]);
  }

  // // transformation_coordinate(laserCloudIn, laserCloudIn);

  // // cv::Mat color_mat = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
  // // for (size_t i = 0; i < scans.size(); i++)
  // // {
  // //     for (size_t j = 6; j < scans[i].size() - 6; j++)
  // //     {
  // //         if (laserCloudIn.points[scans[i][j]].x > 0 &&
  // laserCloudIn.points[scans[i][j]].x < 9.9 &&
  // laserCloudIn.points[scans[i][j]].y > -4.9 &&
  // laserCloudIn.points[scans[i][j]].y < 4.9)
  // //         {

  // //             int rand_x = 1000 - laserCloudIn.points[scans[i][j]].x /
  // 0.01;
  // //             int rand_y = 500 - laserCloudIn.points[scans[i][j]].y /
  // 0.01;
  // //             color_mat.at<cv::Vec3b>(rand_x, rand_y)[0] = ((i + 1) % 3) *
  // 80;
  // //             color_mat.at<cv::Vec3b>(rand_x, rand_y)[1] = i % 5 * 50;
  // //             color_mat.at<cv::Vec3b>(rand_x, rand_y)[2] = ((i + 1) % 2) *
  // 100;
  // //         }
  // //     }
  // // }
  // // cv::imshow("color", color_mat);
  // // cv::waitKey(1);

  // //开始计算曲率
  // for (size_t i = 0; i < scans.size(); i++)
  //     for (size_t j = 6; j < scans[i].size() - 6; j++)
  //     {
  //         float diffX = /* laserCloudIn.points[scans[i][j]-5].x +
  //         laserCloudIn.points[scans[i][j] - 4].x
  //                 + laserCloudIn.points[scans[i][j] - 3].x + */
  //                       laserCloudIn.points[scans[i][j] - 2].x +
  //                       laserCloudIn.points[scans[i][j] - 1].x - 4 *
  //                       laserCloudIn.points[scans[i][j]].x +
  //                       laserCloudIn.points[scans[i][j] + 1].x +
  //                       laserCloudIn.points[scans[i][j] + 2].x
  //             /*+ laserCloudIn.points[scans[i][j] + 3].x +
  //             laserCloudIn.points[scans[i][j] + 4].x
  //                 + laserCloudIn.points[scans[i][j] + 5].x*/
  //             ;
  //         float diffY = /*laserCloudIn.points[scans[i][j] - 5].y +
  //         laserCloudIn.points[scans[i][j] - 4].y
  //                 + laserCloudIn.points[scans[i][j] - 3].y + */
  //                       laserCloudIn.points[scans[i][j] - 2].y +
  //                       laserCloudIn.points[scans[i][j] - 1].y - 4 *
  //                       laserCloudIn.points[scans[i][j]].y +
  //                       laserCloudIn.points[scans[i][j] + 1].y +
  //                       laserCloudIn.points[scans[i][j] + 2].y
  //             /*+ laserCloudIn.points[scans[i][j] + 3].y +
  //             laserCloudIn.points[scans[i][j] + 4].y
  //                 + laserCloudIn.points[scans[i][j] + 5].y*/
  //             ;
  //         float diffZ = /*laserCloudIn.points[scans[i][j] - 5].z +
  //         laserCloudIn.points[scans[i][j] - 4].z
  //                 + laserCloudIn.points[scans[i][j] - 3].z + */
  //                       laserCloudIn.points[scans[i][j] - 2].z +
  //                       laserCloudIn.points[scans[i][j] - 1].z - 4 *
  //                       laserCloudIn.points[scans[i][j]].z +
  //                       laserCloudIn.points[scans[i][j] + 1].z +
  //                       laserCloudIn.points[scans[i][j] + 2].z
  //             /*+ laserCloudIn.points[scans[i][j] + 3].z +
  //             laserCloudIn.points[scans[i][j] + 4].z
  //                 + laserCloudIn.points[scans[i][j] + 5].z*/
  //             ;

  //         cloudCurvature[scans[i][j]] = diffX * diffX + diffY * diffY + diffZ
  //         * diffZ;
  //         //  else cloudCurvature[scans[i][j]] = 0.0;
  //         // cloudSortInd[scans[i][j]] = scans[i][j];

  //         float intensity_point = laserCloudIn.points[scans[i][j] -
  //         4].intensity + laserCloudIn.points[scans[i][j] - 3].intensity +
  //         laserCloudIn.points[scans[i][j] - 2].intensity +
  //         laserCloudIn.points[scans[i][j] - 1].intensity -
  //                                 8 *
  //                                 laserCloudIn.points[scans[i][j]].intensity
  //                                 + laserCloudIn.points[scans[i][j] +
  //                                 1].intensity +
  //                                 laserCloudIn.points[scans[i][j] +
  //                                 2].intensity +
  //                                 laserCloudIn.points[scans[i][j] +
  //                                 3].intensity +
  //                                 laserCloudIn.points[scans[i][j] +
  //                                 4].intensity;
  //         cloudintensity[scans[i][j]] = sqrt(intensity_point *
  //         intensity_point);
  //         //与后一点的斜率 y与x方向
  //         cloudslope[scans[i][j]] = fabs(laserCloudIn.points[scans[i][j] +
  //         2].y - laserCloudIn.points[scans[i][j] - 2].y) /
  //         (laserCloudIn.points[scans[i][j] + 2].x -
  //         laserCloudIn.points[scans[i][j] - 2].x);
  //     }

  // //挑选点，排除容易被斜面挡住的点以及离群点
  // for (size_t i = 5; i < laserCloudIn.points.size() - 6; i++)
  // {
  //     float diffX = laserCloudIn.points[i + 1].x - laserCloudIn.points[i].x;
  //     float diffY = laserCloudIn.points[i + 1].y - laserCloudIn.points[i].y;
  //     float diffZ = laserCloudIn.points[i + 1].z - laserCloudIn.points[i].z;
  //     //计算有效曲率点与后一个点之间的距离平方和
  //     float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

  //     if (diff > 0.1)
  //     {
  //         //点的深度
  //         float depth1 = sqrt(laserCloudIn.points[i].x *
  //         laserCloudIn.points[i].x +
  //                             laserCloudIn.points[i].y *
  //                             laserCloudIn.points[i].y +
  //                             laserCloudIn.points[i].z *
  //                             laserCloudIn.points[i].z);

  //         //后一个点的深度
  //         float depth2 = sqrt(laserCloudIn.points[i + 1].x *
  //         laserCloudIn.points[i + 1].x +
  //                             laserCloudIn.points[i + 1].y *
  //                             laserCloudIn.points[i + 1].y +
  //                             laserCloudIn.points[i + 1].z *
  //                             laserCloudIn.points[i + 1].z);
  //         //按照两点的深度的比例，将深度较大的点拉回后计算距离
  //         if (depth1 > depth2)
  //         {
  //             diffX = laserCloudIn.points[i + 1].x - laserCloudIn.points[i].x
  //             * depth2 / depth1; diffY = laserCloudIn.points[i + 1].y -
  //             laserCloudIn.points[i].y * depth2 / depth1; diffZ =
  //             laserCloudIn.points[i + 1].z - laserCloudIn.points[i].z *
  //             depth2 / depth1;

  //             //边长比也即是弧度值，若小于0.1，说明夹角比较小，斜面比较陡峭,点深度变化比较剧烈,点处在近似与激光束平行的斜面上
  //             if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) /
  //             depth2 < 0.15)
  //             {
  //                 cloudNeighborPicked[i - 2] = 1;
  //                 cloudNeighborPicked[i - 1] = 1;
  //                 cloudNeighborPicked[i] = 1;
  //             }
  //         }
  //         else
  //         {
  //             diffX = laserCloudIn.points[i + 1].x * depth1 / depth2 -
  //             laserCloudIn.points[i].x; diffY = laserCloudIn.points[i + 1].y
  //             * depth1 / depth2 - laserCloudIn.points[i].y; diffZ =
  //             laserCloudIn.points[i + 1].z * depth1 / depth2 -
  //             laserCloudIn.points[i].z;

  //             if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) /
  //             depth1 < 0.15)
  //             {
  //                 cloudNeighborPicked[i + 1] = 1;
  //                 cloudNeighborPicked[i + 2] = 1;
  //                 cloudNeighborPicked[i + 3] = 1;
  //                 // cloudNeighborPicked[i + 4] = 1;
  //                 // cloudNeighborPicked[i + 5] = 1;
  //                 // cloudNeighborPicked[i + 6] = 1;
  //             }
  //         }
  //     }
  //     float diffX2 = laserCloudIn.points[i].x - laserCloudIn.points[i - 1].x;
  //     float diffY2 = laserCloudIn.points[i].y - laserCloudIn.points[i - 1].y;
  //     float diffZ2 = laserCloudIn.points[i].z - laserCloudIn.points[i - 1].z;
  //     //与前一个点的距离平方和
  //     float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
  //     //点深度的平方和
  //     float dis = laserCloudIn.points[i].x * laserCloudIn.points[i].x +
  //     laserCloudIn.points[i].y * laserCloudIn.points[i].y; cloud_distance[i]
  //     = sqrt(dis);
  //     //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
  //     if (diff > outlier_min_distance_ * dis || diff2 > outlier_min_distance_
  //     * dis || dis > view_max_distance_ * view_max_distance_ || dis <
  //     view_min_distance_ * view_min_distance_)
  //     {
  //         cloudNeighborPicked[i] = 1;
  //     }
  // }
  // pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp;
  // pcl::PointCloud<pcl::PointXYZI> surfPointsSharp;
  // pcl::PointCloud<pcl::PointXYZI> intensityPointsSharp;
  // for (size_t i = 0; i < scans.size(); i++)
  // {
  //     for (size_t j = 10; j < scans[i].size() - 10; j++)
  //     {
  //         if (cloudCurvature[scans[i][j]] > the_corner_min_curvature_ &&
  //         cloudNeighborPicked[scans[i][j]] != 1 &&
  //         fabs(cloudslope[scans[i][j] - 1] - cloudslope[scans[i][j]]) >
  //         the_min_slope_difference_ &&
  //             (cloud_distance[scans[i][j] - 1] - cloud_distance[scans[i][j]]
  //             > 0) && (cloud_distance[scans[i][j] + 1] >
  //             cloud_distance[scans[i][j]])/*  ||
  //              (cloud_distance[scans[i][j] + 1] - cloud_distance[scans[i][j]]
  //              > 0) && (cloud_distance[scans[i][j] - 1] >
  //              cloud_distance[scans[i][j]]))*/ &&
  //             laserCloudIn.points[scans[i][j]].z > the_min_hight_)
  //         {
  //             cornerPointsSharp.push_back(laserCloudIn.points[scans[i][j]]);
  //         }
  //         else if (cloudCurvature[scans[i][j]] < the_surface_max_curvature_
  //         && cloudNeighborPicked[scans[i][j]] != 1)
  //         {
  //             surfPointsSharp.push_back(laserCloudIn.points[scans[i][j]]);
  //         }
  //         if (cloudintensity[scans[i][j]] > the_min_intensity_difference_ &&
  //         laserCloudIn.points[scans[i][j]].z < the_max_intensity_hight_)
  //         {
  //             intensityPointsSharp.push_back(laserCloudIn.points[scans[i][j]]);
  //         }
  //     }
  // }

  std::cout << "cornerPointsLessSharp" << corner_cloud.size() << std::endl;
  std::cout << "surfPointsSharp  " << surf_cloud.size() << std::endl;
  std::cout << "intensityPointsSharp  " << intensity_cloud.size()
            << std::endl;

  ros::Time current_time = laserCloudMsg->header.stamp;
  sensor_msgs::PointCloud2 temp_out_msg;

  pcl::toROSMsg(corner_cloud, temp_out_msg);
  temp_out_msg.header.stamp = current_time;
  temp_out_msg.header.frame_id = "/camera_init";
  corner_pub.publish(temp_out_msg);

  pcl::toROSMsg(surf_cloud, temp_out_msg);
  temp_out_msg.header.stamp = current_time;
  temp_out_msg.header.frame_id = "/camera_init";
  surf_pub.publish(temp_out_msg);

  pcl::toROSMsg(intensity_cloud, temp_out_msg);
  temp_out_msg.header.stamp = current_time;
  temp_out_msg.header.frame_id = "/camera_init";
  intensity_pub.publish(temp_out_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "get_feature");
  ros::NodeHandle nh;
  nh.param<double>("the_corner_min_curvature", the_corner_min_curvature_, 0.05);
  nh.param<double>("the_surface_max_curvature", the_surface_max_curvature_,
                   0.0005);
//   nh.param<double>("the_min_hight", the_min_hight_, 0.06);
  nh.param<double>("view_max_distance", view_max_distance_, 50);
  nh.param<double>("view_min_distance", view_min_distance_, 0.2);
  nh.param<double>("outlier_min_distance", outlier_min_distance_, 0.05);
  nh.param<double>("the_min_intensity_difference",
                   the_min_intensity_difference_, 150.0);
  nh.param<double>("the_max_intensity_hight", the_max_intensity_hight_, 0.5);
  nh.param<double>("the_min_slope_difference", the_min_slope_difference_, 0.5);
//   nh.param<double>("livox_z", livox_z_, 1.9921);
  sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1000, Get_Scan);
  corner_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc2_corners", 1, true);
  surf_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc2_surface", 1, true);
  full_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc2_full", 1, true);
  intensity_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/pc2_intensity", 1, true);
  ros::spin();
}

/*
get feature
 */

#include "get_feature/get_feature.hpp"

namespace mapping {

GetFeature::GetFeature(double livox_hight, double corner_min_curvature,
                       double surf_max_curvature, double the_min_hight,
                       double intensity_min_different,
                       double min_slope_different, double max_z_hight,
                       double corner_filter_size, double surf_filter_size,
                       double intensity_filter_size, double cloud_filter_size) {
  livox_z_ = livox_hight;
  the_corner_min_curvature_ = corner_min_curvature;
  the_surface_max_curvature_ = surf_max_curvature;
  the_min_hight_ = the_min_hight;
  view_min_distance_ = 1.0;
  view_max_distance_ = 50;
  the_min_intensity_difference_ = intensity_min_different;
  the_max_intensity_hight_ = 1.0;
  the_min_slope_difference_ = min_slope_different;
  the_max_z_hight_ = max_z_hight;
  corner_filter_.setLeafSize(corner_filter_size, corner_filter_size,
                             corner_filter_size);
  surf_filter_.setLeafSize(surf_filter_size, surf_filter_size,
                           surf_filter_size);
  intensity_filter_.setLeafSize(intensity_filter_size, intensity_filter_size,
                                intensity_filter_size);
  raw_cloud_filter_.setLeafSize(cloud_filter_size, cloud_filter_size,
                                cloud_filter_size);
}

void GetFeature::GetLivoxFeature(const CloudData &livox_cloud,
                                 Feature &new_feature_data) {
  memset(point_curvature_, 0.0, sizeof(point_curvature_));
  memset(point_intensity_, 0.0, sizeof(point_intensity_));
  // memset(point_slope_, 0.0, sizeof(point_slope_));
  memset(point_distance_, 0.0, sizeof(point_distance_));
  // memset(point_neighbor_picked_, 0.0, sizeof(point_neighbor_picked_));
  memset(point_max_hight_, 0.0, sizeof(point_max_hight_));
  // memset(point_segment_, 0, sizeof(point_segment_));
  // memset(cloud_point_label_, 0, sizeof(cloud_point_label_));
  std::vector<std::vector<int>> scans;

  GetLivoxScan(livox_cloud.cloud_ptr, scans);
  // CalculatePointFeature(livox_cloud.cloud_ptr, scans);
  // RemoveNeighborPicked(livox_cloud.cloud_ptr);
  // GetCloudFeature(livox_cloud, scans, new_feature_data);
  // return new_feature_data;
  // std::cout << "feature ok" << std::endl;
  new_feature_data.time = livox_cloud.time;
  CalculatePoint(livox_cloud.cloud_ptr, scans, new_feature_data);
}

void GetFeature::GetLivoxScan(const CloudData::CLOUD_PTR &livox_cloud_ptr,
                              std::vector<std::vector<int>> &scans) {
  // std::cout << "get_feature " << std::endl;
  std::vector<int> scan;
  scans.clear();
  int livox_cloud_size = livox_cloud_ptr->points.size();
  double angle_last = 0.0;        //上一时刻的角度
  double angle_incre_last = 0.0;  //上一时刻角度的增量
  double angle_incre_now = 0.0;   //当前时刻的增量
  double angle_now = 0.0;         //当前时刻的角度
  for (int i = 0; i < livox_cloud_size; i++) {
    angle_now = atan(
        livox_cloud_ptr->points[i].z /
        (sqrt(livox_cloud_ptr->points[i].x * livox_cloud_ptr->points[i].x +
              livox_cloud_ptr->points[i].y * livox_cloud_ptr->points[i].y)));
    if (i == 0) {
      scan.push_back(i);
      angle_last = angle_now;
      continue;
    }
    angle_incre_now = angle_now - angle_last;
    if ((angle_incre_now > 0 && angle_incre_last < 0) ||
        (angle_incre_now < 0 && angle_incre_last > 0)) {
      if (scan.size() > 30) scans.push_back(scan);
      scan.clear();
      scan.push_back(i);
    } else {
      scan.push_back(i);
    }
    angle_last = angle_now;
    angle_incre_last = angle_incre_now;
  }
}

// void GetFeature::CalculatePointFeature(
//     const CloudData::CLOUD_PTR &livox_cloud_ptr,
//     std::vector<std::vector<int>> &scans) {
//   //计算特征系数
//   for (size_t i = 0; i < scans.size(); i++) {
//     for (size_t j = 6; j < scans[i].size() - 6; j++) {
//       float diff_x = livox_cloud_ptr->points[scans[i][j] - 2].x +
//                      livox_cloud_ptr->points[scans[i][j] - 1].x -
//                      4 * livox_cloud_ptr->points[scans[i][j]].x +
//                      livox_cloud_ptr->points[scans[i][j] + 1].x +
//                      livox_cloud_ptr->points[scans[i][j] + 2].x;
//       float diff_y = livox_cloud_ptr->points[scans[i][j] - 2].y +
//                      livox_cloud_ptr->points[scans[i][j] - 1].y -
//                      4 * livox_cloud_ptr->points[scans[i][j]].y +
//                      livox_cloud_ptr->points[scans[i][j] + 1].y +
//                      livox_cloud_ptr->points[scans[i][j] + 2].y;
//       float diff_z = livox_cloud_ptr->points[scans[i][j] - 2].z +
//                      livox_cloud_ptr->points[scans[i][j] - 1].z -
//                      4 * livox_cloud_ptr->points[scans[i][j]].z +
//                      livox_cloud_ptr->points[scans[i][j] + 1].z +
//                      livox_cloud_ptr->points[scans[i][j] + 2].z;

//       point_curvature_[scans[i][j]] =
//           diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;

//       float intensity_point =
//           livox_cloud_ptr->points[scans[i][j] - 2].intensity +
//           livox_cloud_ptr->points[scans[i][j] - 1].intensity -
//           livox_cloud_ptr->points[scans[i][j] + 1].intensity -
//           livox_cloud_ptr->points[scans[i][j] + 2].intensity;

//       point_intensity_[scans[i][j]] = sqrt(intensity_point *
//       intensity_point);

//       point_slope_[scans[i][j]] =
//           fabs(livox_cloud_ptr->points[scans[i][j] + 2].y -
//                livox_cloud_ptr->points[scans[i][j] - 2].y) /
//           (livox_cloud_ptr->points[scans[i][j] + 2].x -
//            livox_cloud_ptr->points[scans[i][j] - 2].x);

//       double min_z = 3.0, max_z = -3.0;
//       // double two_point_max_distance = 0.0;
//       for (int k = -7; k <= 7; k++) {
//         // double distance =
//         //     sqrt((livox_cloud_ptr->points[scans[i][j] + k].x -
//         //           livox_cloud_ptr->points[scans[i][j] - 1 + k].x) *
//         //              (livox_cloud_ptr->points[scans[i][j] + k].x -
//         //               livox_cloud_ptr->points[scans[i][j] - 1 + k].x) +
//         //          (livox_cloud_ptr->points[scans[i][j] + k].y -
//         //           livox_cloud_ptr->points[scans[i][j] - 1 + k].y) *
//         //              (livox_cloud_ptr->points[scans[i][j] + k].y -
//         //               livox_cloud_ptr->points[scans[i][j] - 1 + k].y) +
//         //          (livox_cloud_ptr->points[scans[i][j] + k].z -
//         //           livox_cloud_ptr->points[scans[i][j] - 1 + k].z) *
//         //              (livox_cloud_ptr->points[scans[i][j] + k].z -
//         //               livox_cloud_ptr->points[scans[i][j] - 1 + k].z));

//         // if (distance > two_point_max_distance) {
//         //   two_point_max_distance = distance;
//         // }

//         if (livox_cloud_ptr->points[scans[i][j] + k].z > max_z) {
//           max_z = livox_cloud_ptr->points[scans[i][j] + k].z;
//         }
//         if (livox_cloud_ptr->points[scans[i][j] + k].z < min_z) {
//           min_z = livox_cloud_ptr->points[scans[i][j] + k].z;
//         }
//         // point_max_hight_[scans[i][j]] = max_z - min_z;
//       }
//       // two_point_hight_[scans[i][j]] = two_point_max_distance;
//       point_max_hight_[scans[i][j]] = max_z - min_z;
//     }
//   }
// }

void GetFeature::CalculatePoint(const CloudData::CLOUD_PTR &livox_cloud_ptr,
                                std::vector<std::vector<int>> &scans,
                                Feature &new_feature_data) {
  // new_feature_data.time = livox_cloud_ptr.time;
  // *new_feature_data.cloud_raw = *livox_cloud_ptr;
  // new_feature_data.time = livox_cloud.time;
  CloudData::CLOUD_PTR corner_cloud(new CloudData::CLOUD());
  CloudData::CLOUD_PTR surf_cloud(new CloudData::CLOUD());
  CloudData::CLOUD_PTR intensity_cloud(new CloudData::CLOUD());
  // CloudData::CLOUD_PTR raw_cloud(new CloudData::CLOUD());
  // *raw_cloud = *livox_cloud_ptr;
  std::vector<int> corner_feature;
  std::vector<int> intensity_feature;
  std::vector<int> surf_feaure;
  bool left_surf_flag, right_surf_flag;
  for (size_t i = 0; i < scans.size(); i++) {
    for (size_t j = 5; j < scans[i].size() - 5; j++) {
      //对强度进行判断，一个点的左侧点强度基本一致，右侧点强度基本一致，并且左右侧强度相差大，左右2个点
      float left_intensity =
          livox_cloud_ptr->points[scans[i][j - 2]].intensity +
          livox_cloud_ptr->points[scans[i][j - 1]].intensity;
      float right_intensity =
          livox_cloud_ptr->points[scans[i][j + 2]].intensity +
          livox_cloud_ptr->points[scans[i][j + 1]].intensity;

      float diff_intensity = fabs(left_intensity - right_intensity) / 2.0;

      // 1.计算当前点与前后点的距离
      float depth = sqrt(livox_cloud_ptr->points[scans[i][j]].x *
                             livox_cloud_ptr->points[scans[i][j]].x +
                         livox_cloud_ptr->points[scans[i][j]].y *
                             livox_cloud_ptr->points[scans[i][j]].y +
                         livox_cloud_ptr->points[scans[i][j]].z *
                             livox_cloud_ptr->points[scans[i][j]].z);
      point_distance_[scans[i][j]] = depth;
      float diff_x_right = livox_cloud_ptr->points[scans[i][j]].x -
                           livox_cloud_ptr->points[scans[i][j + 1]].x;
      float diff_y_right = livox_cloud_ptr->points[scans[i][j]].y -
                           livox_cloud_ptr->points[scans[i][j + 1]].y;
      float diff_z_right = livox_cloud_ptr->points[scans[i][j]].z -
                           livox_cloud_ptr->points[scans[i][j + 1]].z;
      float diff_right =
          sqrt(diff_x_right * diff_x_right + diff_y_right * diff_y_right +
               diff_z_right * diff_z_right);
      float diff_x_left = livox_cloud_ptr->points[scans[i][j]].x -
                          livox_cloud_ptr->points[scans[i][j - 1]].x;
      float diff_y_left = livox_cloud_ptr->points[scans[i][j]].y -
                          livox_cloud_ptr->points[scans[i][j - 1]].y;
      float diff_z_left = livox_cloud_ptr->points[scans[i][j]].z -
                          livox_cloud_ptr->points[scans[i][j - 1]].z;
      float diff_left =
          sqrt(diff_x_left * diff_x_left + diff_y_left * diff_y_left +
               diff_z_left * diff_z_left);

      //离散点
      if (diff_left > 0.01 * depth && diff_right > 0.01 * depth) {
        continue;
      }

      if (diff_intensity > the_min_intensity_difference_) {
        intensity_feature.push_back(scans[i][j]);
      }

      // 2.计算曲率
      float left_diff_x = livox_cloud_ptr->points[scans[i][j] - 4].x +
                          livox_cloud_ptr->points[scans[i][j] - 3].x -
                          4 * livox_cloud_ptr->points[scans[i][j] - 2].x +
                          livox_cloud_ptr->points[scans[i][j] - 1].x +
                          livox_cloud_ptr->points[scans[i][j]].x;
      float left_diff_y = livox_cloud_ptr->points[scans[i][j] - 4].y +
                          livox_cloud_ptr->points[scans[i][j] - 3].y -
                          4 * livox_cloud_ptr->points[scans[i][j] - 2].y +
                          livox_cloud_ptr->points[scans[i][j] - 1].y +
                          livox_cloud_ptr->points[scans[i][j]].y;
      float left_diff_z = livox_cloud_ptr->points[scans[i][j] - 4].z +
                          livox_cloud_ptr->points[scans[i][j] - 3].z -
                          4 * livox_cloud_ptr->points[scans[i][j] - 2].z +
                          livox_cloud_ptr->points[scans[i][j] - 1].z +
                          livox_cloud_ptr->points[scans[i][j]].z;
      float left_curvate = left_diff_x * left_diff_x +
                           left_diff_y * left_diff_y +
                           left_diff_z * left_diff_z;

      point_curvature_[scans[i][j] - 2] = left_curvate;

      float right_diff_x = livox_cloud_ptr->points[scans[i][j] + 4].x +
                           livox_cloud_ptr->points[scans[i][j] + 3].x -
                           4 * livox_cloud_ptr->points[scans[i][j] + 2].x +
                           livox_cloud_ptr->points[scans[i][j] + 1].x +
                           livox_cloud_ptr->points[scans[i][j]].x;
      float right_diff_y = livox_cloud_ptr->points[scans[i][j] + 4].y +
                           livox_cloud_ptr->points[scans[i][j] + 3].y -
                           4 * livox_cloud_ptr->points[scans[i][j] + 2].y +
                           livox_cloud_ptr->points[scans[i][j] + 1].y +
                           livox_cloud_ptr->points[scans[i][j]].y;
      float right_diff_z = livox_cloud_ptr->points[scans[i][j] + 4].z +
                           livox_cloud_ptr->points[scans[i][j] + 3].z -
                           4 * livox_cloud_ptr->points[scans[i][j] + 2].z +
                           livox_cloud_ptr->points[scans[i][j] + 1].z +
                           livox_cloud_ptr->points[scans[i][j]].z;
      float right_curvate = right_diff_x * right_diff_x +
                            right_diff_y * right_diff_y +
                            right_diff_z * right_diff_z;

      point_curvature_[scans[i][j] + 2] = right_curvate;

      if (left_curvate < 0.005) surf_feaure.push_back(scans[i][j] - 2);

      if (left_curvate < 0.005) {
        left_surf_flag = true;
      } else {
        left_surf_flag = false;
      }
      if (right_curvate < 0.005) {
        right_surf_flag = true;
      } else {
        right_surf_flag = false;
      }

      // if (left_surf_flag && right_surf_flag) {
      Eigen::Vector3d lidar_point(livox_cloud_ptr->points[scans[i][j]].x,
                                  livox_cloud_ptr->points[scans[i][j]].y,
                                  livox_cloud_ptr->points[scans[i][j]].z);
      Eigen::Vector3d norm_left(0, 0, 0);
      Eigen::Vector3d norm_right(0, 0, 0);
      for (int k = 1; k < 5; k++) {
        //左侧
        Eigen::Vector3d tmp =
            Eigen::Vector3d(livox_cloud_ptr->points[scans[i][j - k]].x -
                                livox_cloud_ptr->points[scans[i][j]].x,
                            livox_cloud_ptr->points[scans[i][j - k]].y -
                                livox_cloud_ptr->points[scans[i][j]].y,
                            livox_cloud_ptr->points[scans[i][j - k]].z -
                                livox_cloud_ptr->points[scans[i][j]].z);
        tmp.normalize();
        norm_left += (k / 10.0) * tmp;

        //右侧
        tmp = Eigen::Vector3d(livox_cloud_ptr->points[scans[i][j + k]].x -
                                  livox_cloud_ptr->points[scans[i][j]].x,
                              livox_cloud_ptr->points[scans[i][j + k]].y -
                                  livox_cloud_ptr->points[scans[i][j]].y,
                              livox_cloud_ptr->points[scans[i][j + k]].z -
                                  livox_cloud_ptr->points[scans[i][j]].z);
        tmp.normalize();
        norm_right += (k / 10.0) * tmp;
      }

      // if (left_surf_flag && right_surf_flag) {
      //计算两平面的夹角
      double cos_left_right = fabs(norm_left.dot(norm_right) /
                                   (norm_left.norm() * norm_right.norm()));
      if (left_surf_flag && right_surf_flag) {
        //计算最大距离
        // float max_diff_x_right = livox_cloud_ptr->points[scans[i][j]].x -
        //                          livox_cloud_ptr->points[scans[i][j + 5]].x;
        // float max_diff_y_right = livox_cloud_ptr->points[scans[i][j]].y -
        //                          livox_cloud_ptr->points[scans[i][j + 5]].y;
        // float max_diff_z_right = livox_cloud_ptr->points[scans[i][j]].z -
        //                          livox_cloud_ptr->points[scans[i][j + 5]].z;
        // float max_diff_right = sqrt(max_diff_x_right * max_diff_x_right +
        //                             max_diff_y_right * max_diff_y_right +
        //                             max_diff_z_right * max_diff_z_right);

        // float max_diff_x_left = livox_cloud_ptr->points[scans[i][j]].x -
        //                         livox_cloud_ptr->points[scans[i][j - 5]].x;
        // float max_diff_y_left = livox_cloud_ptr->points[scans[i][j]].y -
        //                         livox_cloud_ptr->points[scans[i][j - 5]].y;
        // float max_diff_z_left = livox_cloud_ptr->points[scans[i][j]].z -
        //                         livox_cloud_ptr->points[scans[i][j - 5]].z;
        // float max_diff_left = sqrt(max_diff_x_left * max_diff_x_left +
        //                            max_diff_y_left * max_diff_y_left +
        //                            max_diff_z_left * max_diff_z_left);

        //添加角点
        if (cos_left_right < 0.8/* && max_diff_right > 0.05 &&
            max_diff_left > 0.05 && max_diff_right < 3.0 * max_diff_left &&
            max_diff_right > 0.3 * max_diff_left*/) {  //并且前后模长基本一致
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
        float left_depth = sqrt(livox_cloud_ptr->points[scans[i][j] - 4].x *
                                    livox_cloud_ptr->points[scans[i][j] - 4].x +
                                livox_cloud_ptr->points[scans[i][j] - 4].y *
                                    livox_cloud_ptr->points[scans[i][j] - 4].y +
                                livox_cloud_ptr->points[scans[i][j] - 4].z *
                                    livox_cloud_ptr->points[scans[i][j] - 4].z);
        float right_depth =
            sqrt(livox_cloud_ptr->points[scans[i][j] + 4].x *
                     livox_cloud_ptr->points[scans[i][j] + 4].x +
                 livox_cloud_ptr->points[scans[i][j] + 4].y *
                     livox_cloud_ptr->points[scans[i][j] + 4].y +
                 livox_cloud_ptr->points[scans[i][j] + 4].z *
                     livox_cloud_ptr->points[scans[i][j] + 4].z);
        //添加角点
        if (cos_singma < 0.6 && cos_left_right < 0.6 &&
            right_depth - left_depth > 0.04) {
          corner_feature.push_back(scans[i][j]);
        }
      } else if (diff_left - diff_right > 0.1 &&
                 left_curvate > 0.001)  //左侧突变， 右侧平滑
      {
        double cos_singma = fabs(norm_right.dot(lidar_point) /
                                 (norm_right.norm() * lidar_point.norm()));
        //要求左侧的点要比右侧的点近，（平滑侧的点要更接近雷达）
        float left_depth = sqrt(livox_cloud_ptr->points[scans[i][j] - 4].x *
                                    livox_cloud_ptr->points[scans[i][j] - 4].x +
                                livox_cloud_ptr->points[scans[i][j] - 4].y *
                                    livox_cloud_ptr->points[scans[i][j] - 4].y +
                                livox_cloud_ptr->points[scans[i][j] - 4].z *
                                    livox_cloud_ptr->points[scans[i][j] - 4].z);
        float right_depth =
            sqrt(livox_cloud_ptr->points[scans[i][j] + 4].x *
                     livox_cloud_ptr->points[scans[i][j] + 4].x +
                 livox_cloud_ptr->points[scans[i][j] + 4].y *
                     livox_cloud_ptr->points[scans[i][j] + 4].y +
                 livox_cloud_ptr->points[scans[i][j] + 4].z *
                     livox_cloud_ptr->points[scans[i][j] + 4].z);
        //添加角点
        if (cos_singma < 0.6 && cos_left_right < 0.6 &&
            left_depth - right_depth > 0.1) {
          corner_feature.push_back(scans[i][j]);
        }
      }

      double min_z = 3.0, max_z = -3.0;
      for (int k = -7; k <= 7; k++) {
        if (livox_cloud_ptr->points[scans[i][j] + k].z > max_z) {
          max_z = livox_cloud_ptr->points[scans[i][j] + k].z;
        }
        if (livox_cloud_ptr->points[scans[i][j] + k].z < min_z) {
          min_z = livox_cloud_ptr->points[scans[i][j] + k].z;
        }
      }
      point_max_hight_[scans[i][j]] = max_z - min_z;
    
    }
  }

  //对角点的曲率，高度，进行判断
  for (size_t i = 0; i < corner_feature.size(); i++) {
    //如果曲率符合，高度符合，则加入特征中
    if (point_curvature_[corner_feature[i]] > 0.01 &&
        // livox_cloud_ptr->points[corner_feature[i]].z >
        // the_min_hight_ - livox_z_ &&
        point_distance_[corner_feature[i]] > 5.0 &&
        livox_cloud_ptr->points[corner_feature[i]].x < 50 &&
        point_max_hight_[corner_feature[i]] > the_max_z_hight_)
      corner_cloud->points.push_back(
          livox_cloud_ptr->points[corner_feature[i]]);
  }
  //对强度点的曲率，高度，进行判断
  for (size_t i = 0; i < intensity_feature.size(); i++) {
    //如果曲率符合，高度符合，则加入特征中
    if (point_curvature_[intensity_feature[i]] < 0.005 &&
        livox_cloud_ptr->points[intensity_feature[i]].z <
            the_max_intensity_hight_ - livox_z_ &&
        point_max_hight_[intensity_feature[i]] < the_max_z_hight_ &&
        point_distance_[intensity_feature[i]] > 1.0 &&
        livox_cloud_ptr->points[intensity_feature[i]].x < 30)
      intensity_cloud->points.push_back(
          livox_cloud_ptr->points[intensity_feature[i]]);
  }
  for (size_t i = 0; i < surf_feaure.size(); i++) {
    if (point_distance_[surf_feaure[i]] > 1.0)
      surf_cloud->points.push_back(livox_cloud_ptr->points[surf_feaure[i]]);
  }

  corner_filter_.setInputCloud(corner_cloud);
  corner_filter_.filter(*new_feature_data.corner_cloud);
  surf_filter_.setInputCloud(surf_cloud);
  surf_filter_.filter(*new_feature_data.surf_cloud);
  intensity_filter_.setInputCloud(intensity_cloud);
  intensity_filter_.filter(*new_feature_data.intensity_cloud);
  raw_cloud_filter_.setInputCloud(livox_cloud_ptr);
  raw_cloud_filter_.filter(*new_feature_data.cloud_raw);
}

// void GetFeature::RemoveNeighborPicked(
//     const CloudData::CLOUD_PTR &livox_cloud_ptr) {
//   //挑选点，排除容易被斜面挡住的点以及离群点
//   for (size_t i = 5; i < livox_cloud_ptr->points.size() - 6; i++) {
//     float diffX =
//         livox_cloud_ptr->points[i + 1].x - livox_cloud_ptr->points[i].x;
//     float diffY =
//         livox_cloud_ptr->points[i + 1].y - livox_cloud_ptr->points[i].y;
//     float diffZ =
//         livox_cloud_ptr->points[i + 1].z - livox_cloud_ptr->points[i].z;
//     //计算有效曲率点与后一个点之间的距离平方和
//     float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

//     if (diff > 0.1) {
//       //点的深度
//       float depth1 =
//           sqrt(livox_cloud_ptr->points[i].x * livox_cloud_ptr->points[i].x +
//                livox_cloud_ptr->points[i].y * livox_cloud_ptr->points[i].y +
//                livox_cloud_ptr->points[i].z * livox_cloud_ptr->points[i].z);

//       //后一个点的深度
//       float depth2 = sqrt(
//           livox_cloud_ptr->points[i + 1].x * livox_cloud_ptr->points[i + 1].x
//           + livox_cloud_ptr->points[i + 1].y * livox_cloud_ptr->points[i +
//           1].y + livox_cloud_ptr->points[i + 1].z * livox_cloud_ptr->points[i
//           + 1].z);
//       //按照两点的深度的比例，将深度较大的点拉回后计算距离
//       if (depth1 > depth2) {
//         diffX = livox_cloud_ptr->points[i + 1].x -
//                 livox_cloud_ptr->points[i].x * depth2 / depth1;
//         diffY = livox_cloud_ptr->points[i + 1].y -
//                 livox_cloud_ptr->points[i].y * depth2 / depth1;
//         diffZ = livox_cloud_ptr->points[i + 1].z -
//                 livox_cloud_ptr->points[i].z * depth2 / depth1;

//         //边长比也即是弧度值，若小于0.1，说明夹角比较小，斜面比较陡峭,点深度变化比较剧烈,点处在近似与激光束平行的斜面上
//         if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 <
//             0.15) {
//           point_neighbor_picked_[i - 2] = 1;
//           point_neighbor_picked_[i - 1] = 1;
//           point_neighbor_picked_[i] = 1;
//         }
//       } else {
//         diffX = livox_cloud_ptr->points[i + 1].x * depth1 / depth2 -
//                 livox_cloud_ptr->points[i].x;
//         diffY = livox_cloud_ptr->points[i + 1].y * depth1 / depth2 -
//                 livox_cloud_ptr->points[i].y;
//         diffZ = livox_cloud_ptr->points[i + 1].z * depth1 / depth2 -
//                 livox_cloud_ptr->points[i].z;

//         if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 <
//             0.15) {
//           point_neighbor_picked_[i + 1] = 1;
//           point_neighbor_picked_[i + 2] = 1;
//           point_neighbor_picked_[i + 3] = 1;
//         }
//       }
//     }
//     float diffX2 =
//         livox_cloud_ptr->points[i].x - livox_cloud_ptr->points[i - 1].x;
//     float diffY2 =
//         livox_cloud_ptr->points[i].y - livox_cloud_ptr->points[i - 1].y;
//     float diffZ2 =
//         livox_cloud_ptr->points[i].z - livox_cloud_ptr->points[i - 1].z;
//     //与前一个点的距离平方和
//     float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
//     //点深度的平方和
//     float dis = livox_cloud_ptr->points[i].x * livox_cloud_ptr->points[i].x +
//                 livox_cloud_ptr->points[i].y * livox_cloud_ptr->points[i].y;
//     point_distance_[i] = sqrt(dis);
//     //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
//     if (diff > outlier_min_distance_ * dis ||
//         diff2 > outlier_min_distance_ * dis ||
//         dis > view_max_distance_ * view_max_distance_ ||
//         dis < view_min_distance_ * view_min_distance_) {
//       point_neighbor_picked_[i] = 1;
//     }
//   }
// }

// void GetFeature::GetCloudFeature(const CloudData &livox_cloud,
//                                  const std::vector<std::vector<int>> &scans,
//                                  Feature &new_feature_data) {
//   // std::cout << "get cloud" << std::endl;
//   // Feature now_feature;
//   new_feature_data.time = livox_cloud.time;
//   CloudData::CLOUD_PTR corner_cloud(new CloudData::CLOUD());
//   CloudData::CLOUD_PTR surf_cloud(new CloudData::CLOUD());
//   CloudData::CLOUD_PTR intensity_cloud(new CloudData::CLOUD());
//   CloudData::CLOUD_PTR raw_cloud(new CloudData::CLOUD());
//   CloudData::CLOUD_PTR map_cloud(new CloudData::CLOUD());
//   for (size_t i = 0; i < scans.size(); i++) {
//     for (size_t j = 20; j < scans[i].size() - 20; j++) {
//       if (
//           point_curvature_[scans[i][j]] > the_corner_min_curvature_ &&
//           point_max_hight_[scans[i][j]] > the_max_z_hight_ &&
//           point_neighbor_picked_[scans[i][j]] != 1 &&
//           fabs(point_slope_[scans[i][j] - 1] - point_slope_[scans[i][j]]) >
//               the_min_slope_difference_ /* &&
//           // (point_distance_[scans[i][j] - 1] - point_distance_[scans[i][j]]
//           >
//           //  0) &&
//           // (point_distance_[scans[i][j] + 1] >
//           point_distance_[scans[i][j]])*/
//           && livox_cloud.cloud_ptr->points[scans[i][j]].z >
//                  the_min_hight_ - livox_z_/*   &&
//           point_segment_[scans[i][j]] != 0*/) {
//         // new_feature_data.corner_cloud->points.push_back(
//         //     livox_cloud.cloud_ptr->points[scans[i][j]]);
//         corner_cloud->points.push_back(
//             livox_cloud.cloud_ptr->points[scans[i][j]]);
//       } else if (point_curvature_[scans[i][j]] < the_surface_max_curvature_
//       &&
//                  point_distance_[scans[i][j]] < 60 &&
//                  point_distance_[scans[i][j]] > view_min_distance_) {
//         // new_feature_data.surf_cloud->points.push_back(
//         //     livox_cloud.cloud_ptr->points[scans[i][j]]);
//         surf_cloud->points.push_back(
//             livox_cloud.cloud_ptr->points[scans[i][j]]);
//       }
//       if (point_intensity_[scans[i][j]] > the_min_intensity_difference_ &&
//           livox_cloud.cloud_ptr->points[scans[i][j]].z <
//               the_max_intensity_hight_ - livox_z_ &&
//           point_curvature_[scans[i][j]] < the_corner_min_curvature_ /* &&
//           point_intensity_[scans[i][j]] > point_intensity_[scans[i][j] - 1]*/
//           // &&point_neighbor_picked_[scans[i][j]] != 1
//           && point_distance_[scans[i][j]] < 40) {
//         // new_feature_data.intensity_cloud->points.push_back(
//         //     livox_cloud.cloud_ptr->points[scans[i][j]]);
//         intensity_cloud->points.push_back(
//             livox_cloud.cloud_ptr->points[scans[i][j]]);
//       }
//       // if (point_distance_[scans[i][j]] < 10 &&
//       //     point_distance_[scans[i][j]] > view_min_distance_ &&
//       //     point_curvature_[scans[i][j]] > the_corner_min_curvature_ &&
//       //     point_max_hight_[scans[i][j]] > the_max_z_hight_) {
//       //   // new_feature_data.cloud_map->points.push_back(
//       //   //     livox_cloud.cloud_ptr->points[scans[i][j]]);
//       //
//       map_cloud->points.push_back(livox_cloud.cloud_ptr->points[scans[i][j]]);
//       // }
//       if (point_distance_[scans[i][j]] < 50)
//         // new_feature_data.cloud_raw->points.push_back(
//         //     livox_cloud.cloud_ptr->points[scans[i][j]]);
//         raw_cloud->points.push_back(livox_cloud.cloud_ptr->points[scans[i][j]]);
//     }
//   }

//   corner_filter_.setInputCloud(corner_cloud);
//   corner_filter_.filter(*new_feature_data.corner_cloud);
//   surf_filter_.setInputCloud(surf_cloud);
//   surf_filter_.filter(*new_feature_data.surf_cloud);
//   intensity_filter_.setInputCloud(intensity_cloud);
//   intensity_filter_.filter(*new_feature_data.intensity_cloud);
//   raw_cloud_filter_.setInputCloud(raw_cloud);
//   raw_cloud_filter_.filter(*new_feature_data.cloud_raw);

//   AWARN << "get feature"
//         << "corner : " << new_feature_data.corner_cloud->points.size()
//         << "surf : " << new_feature_data.surf_cloud->points.size()
//         << "intensity : " << new_feature_data.intensity_cloud->points.size();
//   //<< "map size : " << new_feature_data.cloud_map->points.size();
//   // new_feature_data_.push_back(now_feature);
// }

// void GetFeature::ParseData(std::deque<Feature>& cloud_data_buff) {
//   std::cout<<"parse data"<<std::endl;
//   if (new_feature_data_.size() > 0) {
//     cloud_data_buff.insert(cloud_data_buff.end(),
//     new_feature_data_.begin(),
//                            new_feature_data_.end());
//     new_feature_data_.clear();
//   }
// }

GetFeature::~GetFeature() {}

}  // namespace mapping
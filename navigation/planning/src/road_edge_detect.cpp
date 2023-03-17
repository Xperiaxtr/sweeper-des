#include "road_edge_detect.h"

namespace sweeper {
namespace navigation {
RoadEdgeDetect::RoadEdgeDetect(ros::NodeHandle &private_nh,
                               const double &filter_min_dis,
                               const double &filter_slope,
                               const cv::Point &center)
    : mode_(0),
      low_obstacle_thre_(0.3),
      planefit_thre_(0.2),
      abs_ground_height_(1.9921) {
  private_nh.param<double>("car_width", car_width_, 1.2);
  private_nh.param<double>("resolution", resolution_, 0.02);
  private_nh.param<double>("front_dis", front_dis_, 10);
  private_nh.param<double>("road_edge_thre", road_edge_thre_, 0.3);
  private_nh.param<double>("low_obstacle_thre", low_obstacle_thre_, 0.3);
  private_nh.param<double>("high_obstacle_thre", high_obstacle_thre_, 0.3);
  private_nh.param<double>("planefit_thre", planefit_thre_, 0.2);
  private_nh.param<double>("abs_ground_height", abs_ground_height_, 1.32);
  private_nh.param<double>("delta_ground_thre", delta_ground_thre_, 0.1);

  filter_min_dis_ = filter_min_dis;
  filter_slope_ = filter_slope;
  center_ = center;

  std::string livox_car_extrinstic_path =
      "../sweeper_ws/src/sweeper_haide/calibration/data/"
      "livox_car_extrinsic.yaml";
  if (!sweeper::common::LoadExtrinsic(livox_car_extrinstic_path,
                                      &livox_car_extrinsic_)) {
    AERROR << "Failed to load livox to car extrinsic.";
  }
  Eigen::Matrix4d livox_to_car = livox_car_extrinsic_.matrix();
  AWARN << "livox_car_extrinsic_:\n" << livox_to_car;
}

/*
函数功能：  矫正点云
输入：  代矫正点云
输出:  矫正后点云
 */
void RoadEdgeDetect::CorrectionCloud(const int mode, const int &curb_sweep_mode,
                                     const PointCloudPtr &point_cloud_in,
                                     PointCloudPtr point_cloud_out,
                                     double *height) {
  mode_ = mode;
  curb_sweep_mode_ = curb_sweep_mode;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_seg(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filter_point(
      new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i < point_cloud_in->points.size(); i++) {
    // TODO:修改z的值
    if ((point_cloud_in->points[i].x < 6.0 &&
         point_cloud_in->points[i].z < 0.3 - abs_ground_height_) &&
        ((point_cloud_in->points[i].y < 3.0 &&
          point_cloud_in->points[i].y > -0.5 && curb_sweep_mode_ == 1 &&
          mode == 2) ||
         (point_cloud_in->points[i].y > -3.0 &&
          point_cloud_in->points[i].y < 0.5 && curb_sweep_mode_ == 0 &&
          mode == 2) ||
         (mode == 1 && fabs(point_cloud_in->points[i].y) < 1.6)))
      point_seg->points.push_back(point_cloud_in->points[i]);

    // TODO:修改z的值
    if ((mode_ == 1 && fabs(point_cloud_in->points[i].x) < 10 &&
         fabs(point_cloud_in->points[i].y) < 5.0) ||
        (mode_ == 2 && point_cloud_in->points[i].x < 10 &&
         point_cloud_in->points[i].x > 0.01 &&
         fabs(point_cloud_in->points[i].y) < 5.0) &&
            point_cloud_in->points[i].z < 0.3) {
      filter_point->points.push_back(point_cloud_in->points[i]);
    }
  }

  if (point_seg->points.size() > 100) {
    pcl::SACSegmentation<pcl::PointXYZI> plane_seg;
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    plane_seg.setOptimizeCoefficients(true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(0.03);
    plane_seg.setInputCloud(point_seg);
    //得到平面系数，进而得到平面法向量
    plane_seg.segment(*plane_inliers, *plane_coefficients);

    Eigen::Vector3f before, after;
    before[0] = plane_coefficients->values[0];
    before[1] = plane_coefficients->values[1];
    before[2] = plane_coefficients->values[2];
    livox_groud_height_ = plane_coefficients->values[3];
    // *height = plane_coefficients->values[3];
    // AWARN << "hight: " << plane_coefficients->values[3];

    if (plane_inliers->indices.size() < 2000 ||
        fabs(plane_coefficients->values[3] - 1.76) > 0.20) {
      *point_cloud_out = *filter_point;
      return;
    }
    after[0] = 0.0;
    after[1] = 0.0;
    after[2] = 1.0;
    pcl::transformPointCloud(*filter_point, *point_cloud_out,
                             CreateRotateMatrix(before, after));
  } else {
    *point_cloud_out = *filter_point;
  }
}

void RoadEdgeDetect::PointcloudToImg(const PointCloudPtr &cloud_in,
                                     const bool &only_trace_flag,
                                     cv::Mat *low_obstacle_img,
                                     cv::Mat *high_obstacle_img,
                                     cv::Mat *road_edge_img,
                                     bool *avoid_obstacle_flag) {
  double height_to_pixel_factor = 0.002;
  int avoid_obstacle_point_count = 0;
  int height = low_obstacle_img->rows;
  int width = low_obstacle_img->cols;

  double height_img_thre;
  if (mode_ == 1){
    if(only_trace_flag)
      height_img_thre = 0.35;
    else 
      height_img_thre = 0.30;
  } else if (mode_ == 2)
    height_img_thre = low_obstacle_thre_;
  double ground_height;
  if (fabs(livox_groud_height_ - abs_ground_height_) < delta_ground_thre_)
    ground_height = livox_groud_height_;
  else
    ground_height = abs_ground_height_;

  double road_edge_thre = road_edge_thre_ - ground_height;
  double low_obstacle_thre = height_img_thre - ground_height;
  double high_obstacle_thre = high_obstacle_thre_ - ground_height;
  for (int i = 0; i < cloud_in->points.size(); i++) {
    double x = cloud_in->points[i].x;
    double y = cloud_in->points[i].y;
    double z = cloud_in->points[i].z;

    if (curb_sweep_mode_ == 1 &&
        y > (filter_slope_ - 0.1) * (x - filter_min_dis_) && y > 0) {
      // if(y > filter_slope_ * (point.x - 1.4) && x < 0)
      continue;
    }
    if (curb_sweep_mode_ == 0 &&
        y < -(filter_slope_ - 0.1) * (x - filter_min_dis_) && y < 0) {
      continue;
    }

    int h = center_.y - round(x / resolution_);
    int w = center_.x - round(y / resolution_);

    // 修改ｚ>的值
    if (fabs(x) > 9.99 || fabs(y) > 9.99 || z > -0.31) continue;
    if (h <= 2 || h >= height - 2 || w <= 2 || w >= width - 2) continue;

    if (z < road_edge_thre) {
      int color =
          round((z + planefit_thre_ + ground_height) / height_to_pixel_factor);
      if (color > 255)
        color = 255;
      else if (color < 0)
        color = 0;
      road_edge_img->at<uchar>(h, w) = 255 - color;
    }

    if (z >= low_obstacle_thre && z <= high_obstacle_thre)
      low_obstacle_img->at<uchar>(h, w) = 255;
    if (z >= high_obstacle_thre) high_obstacle_img->at<uchar>(h, w) = 255;

    // 判断车左前方或者右前方是否有障碍物
    if (z > road_edge_thre) {
      if (curb_sweep_mode_ == 0) {
        if (x < 3.5 && y < -car_width_ / 2 && y > -2 * car_width_)
          avoid_obstacle_point_count++;
      } else if (curb_sweep_mode_ == 1) {
        if (x < 3.5 && y > car_width_ / 2 && y < 2 * car_width_)
          avoid_obstacle_point_count++;
      }
    }
  }
  if (avoid_obstacle_point_count > 5) {
    *avoid_obstacle_flag = false;
    // AWARN << "Not avoid obstacle.";
  } else {
    *avoid_obstacle_flag = true;
    // AWARN << "Can avoid obstacle.";
  }
}

/*
函数功能：  将图片分成上下两块，分别进行二值化处理，去除地面
输入：  灰度图
输出： 处理的二值化图
 */
void RoadEdgeDetect::GetRoadEdgeImg(cv::Mat src, cv::Mat dstImage) {
  int src1_height = src.rows * 0.4;
  int src2_height = src.rows - src1_height;
  cv::Mat src1 = cv::Mat(src1_height, src.cols, CV_8UC1, cv::Scalar(0));
  cv::Mat src2 = cv::Mat(src2_height, src.cols, CV_8UC1, cv::Scalar(0));

  for (int i = 0; i < src.rows; i++) {
    for (int j = 0; j < src.cols; j++) {
      if (i < src.rows * 4 / 10) {
        src1.at<uchar>(i, j) = src.at<uchar>(i, j);
      } else {
        src2.at<uchar>(i - src.rows * 4 / 10, j) = src.at<uchar>(i, j);
      }
    }
  }

  int threshold1, threshold2;
  threshold1 = GetAdaptiveThreshold(src1);
  threshold2 = GetAdaptiveThreshold(src2);
  // ADEBUG << "Threshold 1: " << threshold1;
  // ADEBUG << "Threshold 2: " << threshold2;

  if (threshold1 - threshold2 > 10) threshold1 = threshold2;
  if (threshold2 - threshold1 > 10) threshold2 = threshold1;
  cv::threshold(src1, src1, threshold1, 255, cv::THRESH_BINARY_INV);
  cv::threshold(src2, src2, threshold2, 255, cv::THRESH_BINARY_INV);

  for (int i = 0; i < src.rows; i++) {
    for (int j = 0; j < src.cols; j++) {
      if (i < src.rows * 4 / 10) {
        dstImage.at<uchar>(i, j) = src1.at<uchar>(i, j);
      } else {
        dstImage.at<uchar>(i, j) = src2.at<uchar>(i - src.rows * 4 / 10, j);
      }
    }
  }
}

/*
函数功能：  计算图片的阈值
输入： 灰度图图片
输出：  阈值
 */
int RoadEdgeDetect::GetAdaptiveThreshold(cv::Mat src) {
  const int step = 20;
  int threshold = 0;
  int means_object = 0;
  int object_num = 0;
  int means_back = 0;
  int back_num = 0;
  int threshold_last = 105;  // 115
  //计算直方图
  int height = src.rows;
  int width = src.cols;
  int pixelCount[256] = {0};
  int pixelCount_now[256] = {0};

  //统计灰度级中每个像素在整幅图像中的个数
  for (size_t i = 0; i < height; i++) {
    for (size_t j = 0; j < width; j++) {
      if (src.at<uchar>(i, j) != 255 && src.at<uchar>(i, j) != 0) {
        //将像素值作为计数数组的下标
        pixelCount[src.at<uchar>(i, j)]++;
      }
    }
  }

  int start = step - step / 2;
  int end = 256 - step / 2;
  double temp;
  //直方图优化
  bool frist = true;
  for (int i = start; i < end; i++) {
    if (frist) {
      for (int j = 0 - step / 2; j < step / 2; j++) {
        temp += pixelCount[(int)(i + j)];
      }
      frist = false;
    } else
      temp = temp - pixelCount[(int)(i - step / 2 - 1)] +
             pixelCount[(int)(i + step / 2)];
    pixelCount_now[i] = (int)(temp / step);
  }

  //   if (show_img_info_) {
  //     //显示直方图，确认无误
  //     cv::Mat dstHist = cv::Mat(801, 256, CV_8UC1, cv::Scalar(255));
  //     for (size_t i = 0; i < 256; i++) {
  //       cv::line(dstHist, cv::Point(i, 800),
  //                cv::Point(i, 800 - pixelCount_now[i] / 7), cv::Scalar(0), 1,
  //                1);
  //     }
  //     // cv::imshow("dstHist", dstHist);
  //     // cv::waitKey(1);
  //   }

  // int max_threshold = 255;
  // std::vector<int> mount;
  // for (size_t i = 30; i < 220; i++) {
  //   if (pixelCount_now[i] > 1000) {
  //     if (pixelCount_now[max_threshold] < pixelCount_now[i]) max_threshold =
  //     i; if (pixelCount_now[i] >= pixelCount_now[i - 1] &&
  //         pixelCount_now[i] >= pixelCount_now[i + 1] &&
  //         pixelCount_now[i] >= pixelCount_now[i - 2] &&
  //         pixelCount_now[i] >= pixelCount_now[i + 2]) {
  //       mount.push_back(i);
  //     }
  //   }
  // }

  // if (mount.size() > 1) {
  //   for (size_t i = 0; i < mount.size() - 1; i++) {
  //     if (mount[i + 1] - mount[i] < 20 ||
  //         (mount[i + 1] - mount[i] < 50 &&
  //          abs(pixelCount_now[mount[i + 1]] - pixelCount_now[mount[i]]) <
  //              500)) {
  //       mount[i] = (mount[i + 1] + mount[i]) / 2;
  //       mount.erase(mount.begin() + i + 1);
  //       i--;
  //     }
  //   }
  //   if (mount.size() == 2 && (mount[0] + mount[1]) / 2 < max_threshold) {
  //     ADEBUG << "using two mount ways";
  //     ADEBUG << "Mount size: " << mount.size()
  //            << "threshold: " << (mount[0] + mount[1]) / 2;
  //     return (mount[0] + mount[1]) / 2;
  //   } else {
  //     // ADEBUG << "Don't using two mount ways,mount size: " << mount.size();
  //     for (size_t i = 0; i < mount.size(); i++) {
  //       // ADEBUG << "mount threshold: " << mount[i];
  //     }
  //   }
  // } else {
  //   // ADEBUG << "don't using two mount ways, size<2.";
  // }

  while (abs(threshold - threshold_last) > 2) {
    means_back = 0;
    means_object = 0;
    back_num = 0;
    object_num = 0;
    threshold = threshold_last;
    for (size_t i = 1; i < 255; i++) {
      if (i < threshold) {
        means_back += pixelCount[i] * i;
        back_num += pixelCount[i];
      } else {
        means_object += pixelCount[i] * i;
        object_num += pixelCount[i];
      }
    }
    threshold_last = (means_back / (back_num + 0.001) +
                      means_object / (object_num + 0.001)) /
                     2.0;
  }
  return threshold;
}

void RoadEdgeDetect::RemoveDiscretePoints(cv::Mat src, cv::Mat &dst, int width,
                                          int hight) {
  int width_img = src.cols;
  int hight_img = src.rows;
  dst = cv::Mat(hight_img, width_img, CV_8UC1, cv::Scalar(0));
  int img[hight_img][width_img];
  int img_point = 0;
  for (size_t i = 0; i < hight_img; i++) {
    for (size_t j = 0; j < width_img; j++) {
      if (src.at<uchar>(i, j) == 255)
        img_point = 1;
      else
        img_point = 0;
      img[i][j] = 0;
      if (i == 0 || j == 0) continue;
      img[i][j] = img_point + img[i - 1][j] + img[i][j - 1] - img[i - 1][j - 1];
    }
  }
  int i_hight = 0;
  int j_width = 0;
  int i__hight = 0;
  int j__width = 0;

  for (size_t i = 0; i < hight_img; i++)
    for (size_t j = 0; j < width_img; j++) {
      if (src.at<uchar>(i, j) == 0) {
        continue;
      }
      i_hight = i + hight / 2;
      j_width = j + width / 2;
      i__hight = i - hight / 2;
      j__width = j - width / 2;

      if (i_hight > hight_img - 1) i_hight = hight_img - 1;
      if (j_width > width_img - 1) j_width = width_img - 1;
      if (i__hight < 0) i__hight = 0;
      if (j__width < 0) j__width = 0;
      if (img[i_hight][j_width] - img[i__hight][j_width] -
              img[i_hight][j__width] + img[i__hight][j__width] >
          5)
        dst.at<uchar>(i, j) = 255;
    }
}

void RoadEdgeDetect::DilateImg(cv::Mat src, cv::Mat &dst, int width, int hight,
                               int flag) {
  int width_img = src.cols;
  int hight_img = src.rows;
  dst = cv::Mat(hight_img, width_img, CV_8UC1, cv::Scalar(0));
  int img[hight_img][width_img];
  int img_point = 0;
  for (size_t i = 0; i < hight_img; i++) {
    for (size_t j = 0; j < width_img; j++) {
      if (src.at<uchar>(i, j) == 255)
        img_point = 1;
      else
        img_point = 0;
      img[i][j] = 0;
      if (i == 0 || j == 0) continue;
      img[i][j] = img_point + img[i - 1][j] + img[i][j - 1] - img[i - 1][j - 1];
    }
  }
  int i_hight = 0;
  int j_width = 0;
  int i__hight = 0;
  int j__width = 0;

  for (size_t i = 0; i < hight_img; i++)
    for (size_t j = 0; j < width_img; j++) {
      if (src.at<uchar>(i, j) == 255) {
        dst.at<uchar>(i, j) = 255;
        continue;
      }

      if (flag == 0) {
        i_hight = i + hight / 2;
        i__hight = i - hight / 2;
      } else if (flag == 1) {
        i_hight = i + hight / 2;
        i__hight = i;
      } else if (flag == 2) {
        i_hight = i;
        i__hight = i - hight / 2;
      }
      j_width = j + width / 2;
      j__width = j - width / 2;

      if (i_hight > hight_img - 1) i_hight = hight_img - 1;
      if (j_width > width_img - 1) j_width = width_img - 1;
      if (i__hight < 0) i__hight = 0;
      if (j__width < 0) j__width = 0;
      if (img[i_hight][j_width] - img[i__hight][j_width] -
              img[i_hight][j__width] + img[i__hight][j__width] >
          0)
        dst.at<uchar>(i, j) = 255;
    }
}

/*
函数功能：  得到平面系数
*/
Eigen::Matrix4f RoadEdgeDetect::CreateRotateMatrix(Eigen::Vector3f before,
                                                   Eigen::Vector3f after) {
  before.normalize();
  after.normalize();

  float angle = acos(before.dot(after));
  Eigen::Vector3f p_rotate = before.cross(after);
  p_rotate.normalize();

  Eigen::AngleAxisd rotation_vector(
      angle, Eigen::Vector3d(p_rotate.x(), p_rotate.y(), p_rotate.z()));

  Eigen::Matrix3d rotation_matrix1 = Eigen::Matrix3d::Identity();
  rotation_matrix1 = rotation_vector.matrix();

  Eigen::Matrix4f rotationMatrix1 = Eigen::Matrix4f::Identity();
  rotationMatrix1.block(0, 0, 3, 3) = rotation_matrix1.cast<float>();
  return rotationMatrix1;
}

}  // namespace navigation
}  // namespace sweeper
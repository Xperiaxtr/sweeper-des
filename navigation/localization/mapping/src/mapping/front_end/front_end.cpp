#include "mapping/front_end/front_end.hpp"

namespace mapping {

FrontEnd::FrontEnd(double corner_filter_size, double surf_filter_size,
                   double intensity_filter_size, int icp_times, int bag_frames_num) {
  corner_map_bag_.reset(new CloudData::CLOUD());
  surf_map_bag_.reset(new CloudData::CLOUD());
  intensity_map_bag_.reset(new CloudData::CLOUD());
  frame_counts_ = 0;
  lidar_match_cov_ = 1.0;
  bag_frames_num_ = bag_frames_num;
  // get_new_key_frame_ = true;
  now_quater_ = last_quater_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  now_pos_ = last_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  last_key_pos_ = Eigen::Vector3d(-10, -10, -10);
  map_corner_filter_.setLeafSize(corner_filter_size, corner_filter_size,
                                 corner_filter_size);
  map_surf_filter_.setLeafSize(surf_filter_size, surf_filter_size,
                               surf_filter_size);
  map_intensity_filter_.setLeafSize(
      intensity_filter_size, intensity_filter_size, intensity_filter_size);
  matcher = new SE3Matcher(icp_times);
}

void FrontEnd::ResetFrontEndParam() {
  corner_key_frames_.clear();
  surf_key_frames_.clear();
  intensity_key_frames_.clear();
  now_quater_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  now_pos_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  frame_counts_ = 0;
  last_key_pos_ = Eigen::Vector3d(-10, -10, -10);
  // get_new_key_frame_ = true;
}

void FrontEnd::Process(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &corner_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &intensity_cloud) {
  if (frame_counts_ < 2) {
    GetKeyFrame(corner_cloud, surf_cloud, intensity_cloud);
  } else {
    GetMapBag();
    GetKeyFrame(corner_cloud, surf_cloud, intensity_cloud);
  }
  if (frame_counts_ < 20) frame_counts_++;
}

void FrontEnd::GetKeyFrame(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &corner_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &intensity_cloud) {
  Eigen::Matrix4d now_pose = Eigen::Matrix4d::Identity();
  //进行匹配
  if (frame_counts_ >= 2) {
    double start_time = clock();
    AWARN << "size: " << corner_cloud->points.size() << " "
          << surf_cloud->points.size() << " " << intensity_cloud->points.size();
    lidar_match_cov_ = matcher->GetCloudToMapMacth(
        corner_cloud, surf_cloud, intensity_cloud, corner_map_bag_,
        surf_map_bag_, intensity_map_bag_);

    AWARN << "matcher time: "
          << (double)(clock() - start_time) / CLOCKS_PER_SEC;
    now_quater_ = matcher->m_q_w_curr_;
    now_pos_ = matcher->m_t_w_curr_;
    now_pose.block(0, 0, 3, 3) = now_quater_.matrix();
    now_pose.block(0, 3, 3, 1) = now_pos_;
  }

  bool angle_is_right = false;
  if ((float)last_quater_.angularDistance(now_quater_) * 57.3 > 3.0) {
    angle_is_right = true;
  }
  //如果是第一次并且距离大于阈值
  if (frame_counts_ < 10 || angle_is_right ||
      fabs(last_position_.x() - now_pos_.x()) > 0.3 ||
      fabs(last_position_.y() - now_pos_.y()) > 0.3 ||
      fabs(last_position_.z() - now_pos_.z()) > 0.1) {
    if (angle_is_right || fabs(last_position_.x() - now_pos_.x()) > 0.3 ||
        fabs(last_position_.y() - now_pos_.y()) > 0.3 ||
        fabs(last_position_.z() - now_pos_.z()) > 0.1)
      // get_new_key_frame_ = true;
    last_position_ = now_pos_;
    last_quater_ = now_quater_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr this_corner(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr this_surf(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr this_intensity(
        new pcl::PointCloud<pcl::PointXYZI>());

    pcl::transformPointCloud(*corner_cloud, *this_corner,
                             now_pose.cast<float>());
    pcl::transformPointCloud(*surf_cloud, *this_surf, now_pose.cast<float>());
    pcl::transformPointCloud(*intensity_cloud, *this_intensity,
                             now_pose.cast<float>());

    // pcl::PointCloud<pcl::PointXYZI>::Ptr this_corner(
    //     new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::PointCloud<pcl::PointXYZI>::Ptr this_surf(
    //     new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::PointCloud<pcl::PointXYZI>::Ptr this_intensity(
    //     new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::copyPointCloud(*corner_cloud, *this_corner);
    // pcl::copyPointCloud(*surf_cloud, *this_surf);
    // pcl::copyPointCloud(*intensity_cloud, *this_intensity);
    corner_key_frames_.push_back(this_corner);
    surf_key_frames_.push_back(this_surf);
    intensity_key_frames_.push_back(this_intensity);
  }
  while (corner_key_frames_.size() > bag_frames_num_) {
    corner_key_frames_.pop_front();
    surf_key_frames_.pop_front();
    intensity_key_frames_.pop_front();
  }
}

void FrontEnd::GetMapBag() {
  // std::cout << "get bag" << std::endl;
  CloudData::CLOUD_PTR corner_map_bag(new CloudData::CLOUD());
  CloudData::CLOUD_PTR surf_map_bag(new CloudData::CLOUD());
  CloudData::CLOUD_PTR intensity_map_bag(new CloudData::CLOUD());

  for (size_t i = 0; i < corner_key_frames_.size(); i++) {
    *corner_map_bag += *corner_key_frames_[i];
    *surf_map_bag += *surf_key_frames_[i];
    *intensity_map_bag += *intensity_key_frames_[i];
  }
  // std::cout << "before bag size : " << corner_map_bag->points.size() << " "
  //           << surf_map_bag->points.size() << " "
  //           << intensity_map_bag->points.size() << std::endl;

  // std::cout << "filter" << std::endl;
  map_corner_filter_.setInputCloud(corner_map_bag);
  map_corner_filter_.filter(*corner_map_bag_);
  map_surf_filter_.setInputCloud(surf_map_bag);
  map_surf_filter_.filter(*surf_map_bag_);
  map_intensity_filter_.setInputCloud(intensity_map_bag);
  map_intensity_filter_.filter(*intensity_map_bag_);
  AWARN << "bag_size : " << corner_map_bag_->points.size() << " "
        << surf_map_bag_->points.size() << " "
        << intensity_map_bag_->points.size();
  // std::cout << "filter ok" << std::endl;
}

void FrontEnd::PredictLidarPose(Eigen::Quaterniond predict_quater,
                                Eigen::Vector3d predict_pos) {
  matcher->PredictMatcherPose(predict_quater, predict_pos);
}

bool FrontEnd::OutputNewKeyFrame(Eigen::Matrix4d &odom_pose) {
  double distance = sqrt(
      (last_key_pos_.x() - now_pos_.x()) * (last_key_pos_.x() - now_pos_.x()) +
      (last_key_pos_.y() - now_pos_.y()) * (last_key_pos_.y() - now_pos_.y()) +
      (last_key_pos_.z() - now_pos_.z()) * (last_key_pos_.z() - now_pos_.z()));
  if (distance < 0.3) return false;

  odom_pose.block(0, 0, 3, 3) = now_quater_.matrix();
  odom_pose.block(0, 3, 3, 1) = now_pos_;
  last_key_pos_ = now_pos_;
  // new_key_frame.position = now_pos_;
  // new_key_frame.quater = now_quater_;
  // if (get_new_key_frame_) {
  //   get_new_key_frame_ = false;
  //   return true;
  // }
  // return false;
  return true;
}

}  // namespace mapping
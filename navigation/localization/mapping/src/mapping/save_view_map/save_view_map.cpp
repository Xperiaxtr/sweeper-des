
#include "mapping/save_view_map/save_view_map.hpp"

namespace mapping {
SaveAndViewMap::SaveAndViewMap(double corner_size, double surf_size,
                               double intensity_size)
    : global_map_(new CloudData::CLOUD()) {
  corner_filter_.setLeafSize(corner_size, corner_size, corner_size);
  surf_filter_.setLeafSize(surf_size, surf_size, surf_size);
  intensity_filter_.setLeafSize(intensity_size, intensity_size, intensity_size);
  cloud_filter_.setLeafSize(corner_size, corner_size, corner_size);
  save_dir_name_ = "../sweeper_ws/src/sweeper_haide/data/path/uname";
  save_line_name_ = "../sweeper_ws/src/sweeper_haide/data/path/uname.txt";
  save_pcd_name_ = "../sweeper_ws/src/sweeper_haide/data/path/uname/0.pcd";
  save_cloud_name_ =
      "../sweeper_ws/src/sweeper_haide/data/path/uname/0_cloud.pcd";
  // save_cloud_name_ = "../uname_cloud.pcd";
}

void SaveAndViewMap::PublishAndSaveMap(
    const std::vector<KeyFrame> &key_frames) {
  AWARN << "key_frames num : " << key_frames.size();
  if (key_frames.size() < 30) return;
  // global_map_->clear();

  CloudData::CLOUD_PTR global_map_corner(new CloudData::CLOUD());
  CloudData::CLOUD_PTR global_map_surf(new CloudData::CLOUD());
  CloudData::CLOUD_PTR global_map_intensity(new CloudData::CLOUD());
  CloudData::CLOUD_PTR global_map_cloud(new CloudData::CLOUD());
  CloudData::CLOUD_PTR global_map_jpg(new CloudData::CLOUD());
  save_mutex_.lock();
  for (size_t i = 0; i < key_frames.size(); i++) {
    if (!key_frames[i].optimized) continue;
    CloudData::CLOUD tran_cloud;
    Feature now_feature;

    CloudData::CLOUD full_cloud;

    std::string key_frame_path =
        "../sweeper_ws/src/sweeper_haide/data/map/slam_data/key_frames/";
    pcl::io::loadPCDFile(key_frame_path + std::to_string(i) + "_corner.pcd",
                         *now_feature.corner_cloud);
    pcl::io::loadPCDFile(key_frame_path + std::to_string(i) + "_surf.pcd",
                         *now_feature.surf_cloud);
    pcl::io::loadPCDFile(key_frame_path + std::to_string(i) + "_intensity.pcd",
                         *now_feature.intensity_cloud);
    pcl::io::loadPCDFile(key_frame_path + std::to_string(i) + "_cloud.pcd",
                         *now_feature.cloud_raw);

    for (size_t j = 0; j < now_feature.cloud_raw->points.size(); j++) {
      CloudData::POINT raw_point = now_feature.cloud_raw->points[j];
      if (raw_point.x > 30.0 || fabs(raw_point.y) > 30.0) continue;
      raw_point.intensity = (raw_point.z + 2.0) * 90.0;
      full_cloud.points.push_back(raw_point);
    }
    pcl::transformPointCloud(full_cloud, tran_cloud, key_frames[i].pose);
    *global_map_jpg += tran_cloud;

    pcl::transformPointCloud(*now_feature.corner_cloud, tran_cloud,
                             key_frames[i].pose);
    for (size_t j = 0; j < tran_cloud.points.size(); j++) {
      CloudData::POINT corner_point = tran_cloud.points[j];
      corner_point.intensity = 8.0;
      global_map_corner->points.push_back(corner_point);
    }
    pcl::transformPointCloud(*now_feature.surf_cloud, tran_cloud,
                             key_frames[i].pose);
    for (size_t j = 0; j < tran_cloud.points.size(); j++) {
      CloudData::POINT surf_point = tran_cloud.points[j];
      surf_point.intensity = 0.2;
      global_map_surf->points.push_back(surf_point);
    }
    pcl::transformPointCloud(*now_feature.intensity_cloud, tran_cloud,
                             key_frames[i].pose);
    for (size_t j = 0; j < tran_cloud.points.size(); j++) {
      CloudData::POINT intensity_point = tran_cloud.points[j];
      intensity_point.intensity = 20;
      global_map_intensity->points.push_back(intensity_point);
    }

    pcl::transformPointCloud(*now_feature.cloud_raw, tran_cloud,
                             key_frames[i].pose);
    for (size_t j = 0; j < tran_cloud.points.size(); j++) {
      CloudData::POINT cloud_point = tran_cloud.points[j];
      if (fabs(cloud_point.x - key_frames[i].pose(0, 3)) < 50.0 &&
          fabs(cloud_point.y - key_frames[i].pose(1, 3)) < 50.0 &&
          fabs(cloud_point.x - key_frames[i].pose(0, 3)) > 0.3)
        global_map_cloud->points.push_back(cloud_point);
    }

    // pcl::transformPointCloud(*now_feature.cloud_map, tran_cloud,
    //                          key_frames[i].pose);
    // for (size_t j = 0; j < tran_cloud.points.size(); j++) {
    //   CloudData::POINT cloud_point = tran_cloud.points[j];
    //   // cloud.intensity = 20;
    //   global_map_cloud_map->points.push_back(cloud_point);
    // }
  }
  // AWARN<<" 1111111";
  // CloudData::CLOUD_PTR global_map_jpg(new CloudData::CLOUD());
  // *global_map_jpg = *global_map_corner /*+ *global_map_intensity*/;

  MapToImage(global_map_jpg, key_frames);
  save_mutex_.unlock();

  CloudData::CLOUD_PTR down_global_map_corner(new CloudData::CLOUD());
  CloudData::CLOUD_PTR down_global_map_surf(new CloudData::CLOUD());
  CloudData::CLOUD_PTR down_global_map_intensity(new CloudData::CLOUD());
  CloudData::CLOUD_PTR down_global_map_cloud(new CloudData::CLOUD());
  // CloudData::CLOUD_PTR down_global_map_cloud_map(new CloudData::CLOUD());

  corner_filter_.setInputCloud(global_map_corner);
  corner_filter_.filter(*down_global_map_corner);
  surf_filter_.setInputCloud(global_map_surf);
  surf_filter_.filter(*down_global_map_surf);
  intensity_filter_.setInputCloud(global_map_intensity);
  intensity_filter_.filter(*down_global_map_intensity);
  cloud_filter_.setInputCloud(global_map_cloud);
  cloud_filter_.filter(*down_global_map_cloud);
  // cloud_filter_.setInputCloud(global_map_cloud_map);
  // cloud_filter_.filter(*down_global_map_cloud_map);

  for (size_t i = 0; i < down_global_map_corner->points.size(); i++) {
    global_map_->points.push_back(down_global_map_corner->points[i]);
  }
  for (size_t i = 0; i < down_global_map_surf->points.size(); i++) {
    global_map_->points.push_back(down_global_map_surf->points[i]);
  }
  for (size_t i = 0; i < down_global_map_intensity->points.size(); i++) {
    global_map_->points.push_back(down_global_map_intensity->points[i]);
  }

  // map_pub_ptr_->Publish(global_map);
  global_map_->width = 1;
  global_map_->height = global_map_->points.size();
  AWARN << "cloud size: " << global_map_->points.size();
  if (!pcl::io::savePCDFileBinary(save_pcd_name_.c_str(), *global_map_)) {
    AWARN << "save pcd ok!";
  } else {
    AWARN << "save pcd fail";
  }
  AWARN << "cloud size: " << down_global_map_cloud->points.size();
  down_global_map_cloud->width = 1;
  down_global_map_cloud->height = down_global_map_cloud->points.size();
  AWARN << "cloud size: " << down_global_map_cloud->points.size();
  if (!pcl::io::savePCDFileBinary(save_cloud_name_.c_str(),
                                  *down_global_map_cloud)) {
    AWARN << "save pcd ok!";
  } else {
    AWARN << "save pcd fail";
  }
  global_map_->clear();
}

void SaveAndViewMap::MapToImage(const CloudData::CLOUD_PTR &global_map,
                                const std::vector<KeyFrame> &key_frames) {
  CloudData::CLOUD filter_cloud = *global_map;
  // for (size_t i = 0; i < global_map->points.size(); i++) {
  //   if (global_map->points[i].intensity > 0.5)
  //     filter_cloud.points.push_back(global_map->points[i]);
  // }
  CloudData::POINT min, max;
  pcl::getMinMax3D(filter_cloud, min, max);

  //得到jpg图片
  int hight = (max.x - min.x) / 0.1;  // 5cm分辨率
  int width = (max.y - min.y) / 0.1;  // 5cm分辨率

  MathCalculation::SaveMapToimageYaml(save_dir_name_, max.x, max.y, 0.1);

  map_jpg_ = cv::Mat(hight + 1, width + 1, CV_8UC3, cv::Scalar(0, 0, 0));
  for (size_t i = 0; i < filter_cloud.points.size(); i++) {
    CloudData::POINT point_now = filter_cloud.points[i];
    int m = (-point_now.x + max.x) / 0.1;
    int n = (-point_now.y + max.y) / 0.1;
    int gray = point_now.intensity;
    if (gray > 255) gray = 255;
    else if(gray < 0)gray =0;
    if (m < hight + 1 && n < width + 1) {
      map_jpg_.at<cv::Vec3b>(m, n)[0] = gray;
      map_jpg_.at<cv::Vec3b>(m, n)[1] = gray;
      map_jpg_.at<cv::Vec3b>(m, n)[2] = gray;
    } else {
      AWARN << "image is out "
            << " raw row, clos: " << hight << " " << width
            << " now row, clos: " << m << " " << n;
    }
  }
  for (size_t i = 0; i < key_frames.size(); i++) {
    if (!key_frames[i].optimized) continue;
    int m = (-key_frames[i].pose(0, 3) + max.x) / 0.1;
    int n = (-key_frames[i].pose(1, 3) + max.y) / 0.1;
    if (m < hight + 1 && n < width + 1) {
      cv::circle(map_jpg_, cv::Point(n, m), 2, cv::Scalar(0, 255, 0),
                 -1);  // 画半径为1的圆(画点）
    } else {
      AWARN << "image is out "
            << " raw row, clos: " << hight << " " << width
            << " now row, clos: " << m << " " << n;
    }
  }
  std::string image_name = save_dir_name_ + "/map_image.jpg";
  cv::imwrite(image_name, map_jpg_);
  AWARN << "save image is ok";
}

void SaveAndViewMap::GetPcdAndLineName(std::string save_file_name) {
  //保存PCD的位置与name
  // std::string save_now_pcd_num = std::to_string(save_now_pcd_num_);
  save_pcd_name_ = save_dir_name_ + "/0.pcd";
  AWARN << "----- Save map to " << save_pcd_name_ << "------";
  //报错line的位置与name
  std::string path_file = "../sweeper_ws/src/sweeper_haide/data/path";
  save_line_name_ = path_file + "/" + save_file_name + ".txt";
  AWARN << "----- Save line to " << save_line_name_ << "------";
}

void SaveAndViewMap::SaveLineToFile(const std::vector<KeyFrame> &key_frames) {
  if (key_frames.size() < 10) return;
  std::ofstream myfile(save_line_name_.c_str(), std::ios::app);
  Eigen::Vector3d last_key_pose(0.0, 0.0, 0.0);
  AWARN << "save line to : " << save_line_name_ << "  start !";
  AWARN << "line size : " << key_frames.size();
  for (size_t i = 0; i < key_frames.size(); i++) {
    if (!key_frames[i].optimized) continue;
    if (fabs(last_key_pose.x() - key_frames[i].pose(0, 3)) > 0.2 ||
        fabs(last_key_pose.y() - key_frames[i].pose(1, 3)) > 0.2) {
      Eigen::Matrix3f key_frame_matrix = key_frames[i].pose.block(0, 0, 3, 3);
      Eigen::Quaternionf key_frame_quater(key_frame_matrix);
      myfile << std::to_string(0) << "|"
             << std::to_string(key_frames[i].gnss_data.x()) << "|"
             << std::to_string(key_frames[i].gnss_data.y()) << "|"
             << std::to_string(key_frames[i].pose(0, 3)) << "|"
             << std::to_string(key_frames[i].pose(1, 3)) << "|"
             << std::to_string(key_frames[i].pose(2, 3)) << "|"
             << std::to_string(key_frame_quater.x()) << "|"
             << std::to_string(key_frame_quater.y()) << "|"
             << std::to_string(key_frame_quater.z()) << "|"
             << std::to_string(key_frame_quater.w()) << "|"
             << std::to_string(key_frames[i].point_attribute) << "\n";
      last_key_pose.x() = key_frames[i].pose(0, 3);
      last_key_pose.y() = key_frames[i].pose(1, 3);
      last_key_pose.z() = key_frames[i].pose(2, 3);
    }
  }
  AWARN << "save line to : " << save_line_name_ << "  end !";
  myfile.close();
}

bool SaveAndViewMap::RenameAndSaveFile(std::string save_file_name) {
  // AWARN << "read line size : " << save_data_to_file_.size();
  std::string file_name = "../sweeper_ws/src/sweeper_haide/data/path";
  save_line_name_ = file_name + "/" + save_file_name + ".txt";
  std::string rename_file_name = file_name + "/" + save_file_name;
  if (!access(save_dir_name_.c_str(), F_OK)) {
    if (!rename(save_dir_name_.c_str(), rename_file_name.c_str())) {
      AWARN
          << "rename unname file to Specified name successful,  the file is : "
          << save_dir_name_ << " to : " << rename_file_name;
    } else {
      AWARN << "rename unname file to Specified name fail";
      return false;
    }
  }
  // AWARN << "read line size : " << save_data_to_file_.size();
  // SaveLineToFile(0);
  return true;
}

bool SaveAndViewMap::ResetParam() {
  bool make_file_ok = true;
  RemoveDir(save_dir_name_.c_str());
  if (mkdir(save_dir_name_.c_str(), 0777)) {
    AWARN << "make file errorly ! " << std::endl;
    make_file_ok = false;
  } else {
    AWARN << "make file sucess ! " << std::endl;
    make_file_ok = true;
  }
  // save_data_to_file_.clear();
  // save_now_pcd_num_ = 0;
  return make_file_ok;
}

int SaveAndViewMap::RemoveDir(const char *dir) {
  char cur_dir[] = ".";
  char up_dir[] = "..";
  char dir_name[128];
  DIR *dirp;
  struct dirent *dp;
  struct stat dir_stat;

  // 参数传递进来的目录不存在，直接返回
  if (0 != access(dir, F_OK)) {
    return 0;
  }

  // 获取目录属性失败，返回错误
  if (0 > stat(dir, &dir_stat)) {
    perror("get directory stat error");
    return -1;
  }

  if (S_ISREG(dir_stat.st_mode)) {  // 普通文件直接删除
    remove(dir);
  } else if (S_ISDIR(dir_stat.st_mode)) {  // 目录文件，递归删除目录中内容
    dirp = opendir(dir);
    while ((dp = readdir(dirp)) != NULL) {
      // 忽略 . 和 ..
      if ((0 == strcmp(cur_dir, dp->d_name)) ||
          (0 == strcmp(up_dir, dp->d_name))) {
        continue;
      }

      sprintf(dir_name, "%s/%s", dir, dp->d_name);
      RemoveDir(dir_name);  // 递归调用
    }
    closedir(dirp);

    rmdir(dir);  // 删除空目录
  } else {
    perror("unknow file type!");
  }

  return 0;
}

// void SaveAndViewMap::PubGlobalMap() {}
// void SaveAndViewMap::SaveGlobalMap() {}
}  // namespace mapping
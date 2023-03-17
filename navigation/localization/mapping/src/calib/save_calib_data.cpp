#include "calib/save_calib_data.hpp"

SaveCalibData::SaveCalibData(ros::NodeHandle nh, ros::NodeHandle nh_private) {
  predict_quater_ = now_quater_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  fixed_gnss_point_ = predict_pos_ = now_pos_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  lidar_time_ = 0.0;
  save_index_ = 0;
  flag_frist_gnss_ = 1;

  corner_feature_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, "sweeper/calib/corner_feature", 1000, "/camera_init");
  surf_feature_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, "sweeper/calib/surf_feature", 1000, "/camera_init");
  intensity_feature_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, "sweeper/calib/intensity_feature", 1000, "/camera_init");
  corner_bag_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, "sweeper/calib/corner_map", 1000, "/camera_init");
  map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "sweeper/calib/cloud_map",
                                                  1000, "/camera_init");
  odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "sweeper/calib/lidar_odom", "/camera_init", "lidar", 1000);
  vehical_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "sweeper/calib/vehical_odom", "/camera_init", "lidar", 1000);

  sub_gnss_data_ = nh.subscribe<nav_msgs::Odometry>(
      "/sweeper/localization/gnss", 1, &SaveCalibData::ReceiveGnssData, this);
  // sub_livox_data_ = nh.subscribe<sensor_msgs::PointCloud2>(
  //     "/livox/lidar", 1, &SaveCalibData::ReceiveLidarData, this);
  sub_sweeper_data_ = nh.subscribe /* <sweeper_msgs::SweeperChassisDetail>*/ (
      "/sweeper/chassis/detail", 1, &SaveCalibData::ReceiveSweeperInformation,
      this);
  sub_imu_data_ = nh.subscribe<sensor_msgs::Imu>(
      "/sweeper/sensor/imu", 1, &SaveCalibData::ReceiveImuData, this);

  //下采样参数
  double corner_filter_size, surf_filter_size, intensity_filter_size,
      corner_map_filter_size, surf_map_filter_size, intensity_map_filter_size;
  nh_private.param<double>("corner_map_filter", corner_map_filter_size, 0.3);
  nh_private.param<double>("surf_map_filter", surf_map_filter_size, 0.5);
  nh_private.param<double>("intensity_map_filter", intensity_map_filter_size,
                           0.3);
  nh_private.param<double>("corner_filter", corner_filter_size, 0.3);
  nh_private.param<double>("surf_filter", surf_filter_size, 0.5);
  nh_private.param<double>("intensity_filter", intensity_filter_size, 0.3);

  double map_filter_size = 0.3;
  nh_private.param<double>("map_filter", map_filter_size, 0.3);
  map_filter_.setLeafSize(map_filter_size, map_filter_size, map_filter_size);

  //特征提取参数
  double livox_hight, corner_min_curvature, surf_max_curvature, the_min_hight,
      intensity_min_different, min_slope_different, max_z_hight;

  nh_private.param<double>("livox_hight", livox_hight, 1.79);
  nh_private.param<double>("corner_min_curvature", corner_min_curvature, 0.01);
  nh_private.param<double>("surf_max_curvature", surf_max_curvature, 0.005);
  nh_private.param<double>("the_min_hight", the_min_hight, 0.06);
  nh_private.param<double>("intensity_min_different", intensity_min_different,
                           15.0);
  nh_private.param<double>("min_slope_different", min_slope_different, 0.1);
  nh_private.param<double>("max_z_hight", max_z_hight, 0.5);

  std::string key_frames_path;
  nh_private.param("key_frames_path", key_frames_path,
                   std::string("../sweeper_ws/src/sweeper_haide/navigation/"
                               "localization/calibration/calib_data"));
  key_frames_path.erase(key_frames_path.end() - 28, key_frames_path.end());

  front_end_ = new FrontEnd(corner_map_filter_size, surf_map_filter_size,
                            intensity_map_filter_size, 10, 500);
  get_feature_ = new GetFeature(
      livox_hight, corner_min_curvature, surf_max_curvature, the_min_hight,
      intensity_min_different, min_slope_different, max_z_hight,
      corner_filter_size, surf_filter_size, intensity_filter_size,
      corner_filter_size);

  key_frames_path_ = key_frames_path + "/calib_data";
  std::cout << "key_frames_path_: " << key_frames_path_ << std::endl;
  ResetParam();
}

SaveCalibData::~SaveCalibData() {
  delete get_feature_;
  delete front_end_;
}

void SaveCalibData::PredictLidarPose() {
  predict_mutex_.lock();
  while (sweeper_information_deque_.size() > 0) {
    // AWARN << "dis_time : " << sweeper_information_deque_[0].time -
    // lidar_time_
    //       << " " << imu_information_deque_.size();
    // ROS_ERROR("dis_time : %f",
    //           sweeper_information_deque_[0].time - lidar_time_);
    // std::cout << " size: " << sweeper_information_deque_.size() << " "
    //           << imu_information_deque_.size() << std::endl;

    if (sweeper_information_deque_[0].time > lidar_time_) {
      break;
    }
    if (lidar_time_ - sweeper_information_deque_[0].time > 0.12) {
      sweeper_information_deque_.pop_front();
      continue;
    }
    Eigen::Quaterniond lidar_quater = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    if (imu_information_deque_.size() > 0) {
      // ROS_ERROR("imu diff time %f", imu_information_deque_[0].time -
      //                                   sweeper_information_deque_[0].time);
      while (imu_information_deque_.size() > 0) {
        if (imu_information_deque_[0].time -
                sweeper_information_deque_[0].time <
            -0.01) {
          imu_information_deque_.pop_front();
        } else {
          // ROS_ERROR("imu diff time %f", imu_information_deque_[0].time -
          //                         sweeper_information_deque_[0].time);
          // double diff_imu_roll =
          //     imu_information_deque_[0].angular_velocity_x * 0.02;
          // double diff_imu_pitch =
          //     imu_information_deque_[0].angular_velocity_y * 0.02;
          double diff_imu_yaw =
              imu_information_deque_[0].angular_velocity_z * 0.02;
          Eigen::Quaterniond diff_quater =
              Eigen::AngleAxisd(diff_imu_yaw, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
          lidar_quater = diff_quater;
          break;
        }
      }
    } else {
      lidar_quater = sweeper_information_deque_[0].diff_quater;
    }
    Eigen::Vector3d lidar_pos =
        Eigen::Vector3d(sweeper_information_deque_[0].distance, 0.0, 0.0);
    now_pos_ = now_quater_ * lidar_pos + now_pos_;
    now_quater_ = now_quater_ * lidar_quater;
    predict_pos_ = predict_quater_ * lidar_pos + predict_pos_;
    predict_quater_ = predict_quater_ * lidar_quater;

    sweeper_information_deque_.pop_front();
  }
  predict_mutex_.unlock();
}

void SaveCalibData::CloudAdjustMotion(
    const Eigen::Matrix4d &tran_pose,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud_ptr) {
  Eigen::Matrix3d tran_rotation = tran_pose.block(0, 0, 3, 3);
  Eigen::Quaterniond tran_quater(tran_rotation);
  Eigen::Vector3d tran_pos(tran_pose(0, 3), tran_pose(1, 3), tran_pose(2, 3));

  int cloud_size = point_cloud_ptr->points.size();
  for (int i = 0; i < cloud_size; i++) {
    Eigen::Vector3d now_point(point_cloud_ptr->points[i].x,
                              point_cloud_ptr->points[i].y,
                              point_cloud_ptr->points[i].z);
    double rate = ((double)i) / ((double)(cloud_size));
    Eigen::Quaterniond adjust_quater =
        Eigen::Quaterniond::Identity().slerp(rate, tran_quater);
    Eigen::Vector3d adjust_pos = rate * tran_pos;
    Eigen::Vector3d tran_point = adjust_quater * now_point + adjust_pos;
    point_cloud_ptr->points[i].x = tran_point.x();
    point_cloud_ptr->points[i].y = tran_point.y();
    point_cloud_ptr->points[i].z = tran_point.z();
  }
}

void SaveCalibData::ReceiveSweeperInformation(
    const sweeper_msgs::SweeperChassisDetail &sweeper_chassis_detail) {
  double car_yaw = sweeper_chassis_detail.steering_angle_output / 57.3;
  double speed = sweeper_chassis_detail.vehicle_speed_output / 3.6;
  double time = sweeper_chassis_detail.header.stamp.toSec();
  // now_sweeper_information_.steering_angle = car_yaw;
  SweeperInformation now_sweeper_information;
  now_sweeper_information.distance = speed * 0.02;
  now_sweeper_information.time = time;
  double car_angle = 0.02 * speed * sin(car_yaw) /
                     (cos(car_yaw) * CAR_FRONT_DIS_ + CAR_BACK_DIS_);
  Eigen::Quaterniond diff_quater =
      Eigen::AngleAxisd(-car_angle, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  now_sweeper_information.diff_quater = diff_quater;
  sweeper_mutex_.lock();
  sweeper_information_deque_.push_back(now_sweeper_information);
  sweeper_mutex_.unlock();
}

void SaveCalibData::ReceiveImuData(
    const sensor_msgs::Imu::ConstPtr &imu_msg_ptr) {
  double time = imu_msg_ptr->header.stamp.toSec();
  static double last_imu_time = time;
  double diff_time = time - last_imu_time;
  if (diff_time <= 0 || diff_time > 0.5) {
    last_imu_time = time;
    return;
  }
  ImuInformation now_imu_information;
  now_imu_information.time = time;
  now_imu_information.angular_velocity_x = imu_msg_ptr->angular_velocity.x;
  now_imu_information.angular_velocity_y = imu_msg_ptr->angular_velocity.y;
  now_imu_information.angular_velocity_z = imu_msg_ptr->angular_velocity.z;
  imu_mutex_.lock();
  imu_information_deque_.push_back(now_imu_information);
  imu_mutex_.unlock();
}

void SaveCalibData::AddCloudToFile(const CloudData::CLOUD_PTR &cloud_ptr,
                                   const int &key_frame_index) {
  //存储到硬盘中
  std::string file_path =
      key_frames_path_ + "/cloud/" + std::to_string(key_frame_index) + ".pcd";
  // std::cout << "path: " << file_path << std::endl;
  if (cloud_ptr->points.size() > 5)
    pcl::io::savePCDFileBinary(file_path.c_str(), *cloud_ptr);
  std::cout << "save_pcd_ok" << std::endl;
}

void SaveCalibData::ReceiveLidarData(
    const sensor_msgs::PointCloud2::ConstPtr &input) {
  CloudData pcl_raw_cloud;
  lidar_time_ = input->header.stamp.toSec();
  pcl::fromROSMsg(*input, *pcl_raw_cloud.cloud_ptr);

  //数据预处理
  Eigen::Quaterniond prv_quater = predict_quater_;
  Eigen::Vector3d prv_pos = predict_pos_;
  PredictLidarPose();
  Eigen::Matrix4d prv_pose = Eigen::Matrix4d::Identity();
  prv_pose.block(0, 0, 3, 3) = prv_quater.matrix();
  prv_pose.block(0, 3, 3, 1) = prv_pos;
  Eigen::Matrix4d now_pose = Eigen::Matrix4d::Identity();
  now_pose.block(0, 0, 3, 3) = predict_quater_.matrix();
  now_pose.block(0, 3, 3, 1) = predict_pos_;
  Eigen::Matrix4d tran_pose = prv_pose.inverse() * now_pose;
  vehical_pub_ptr_->Publish4d(now_pose, ros::Time::now());

  //点云矫正
  CloudAdjustMotion(tran_pose, pcl_raw_cloud.cloud_ptr);
  Feature now_feature;
  get_feature_->GetLivoxFeature(pcl_raw_cloud, now_feature);

  front_end_->PredictLidarPose(now_quater_, now_pos_);
  front_end_->Process(now_feature.corner_cloud, now_feature.surf_cloud,
                      now_feature.intensity_cloud);

  corner_feature_pub_ptr_->Publish(now_feature.corner_cloud);
  surf_feature_pub_ptr_->Publish(now_feature.surf_cloud);
  // intensity_feature_pub_ptr_->Publish(now_feature.intensity_cloud);

  now_quater_ = front_end_->now_quater_;
  now_pos_ = front_end_->now_pos_;

  Eigen::Matrix4d key_frame_pose = Eigen::Matrix4d::Identity();
  if (!front_end_->OutputNewKeyFrame(key_frame_pose)) {
    AWARN << "can't become a key frame";
    return;
  }
  // lidar_pose_deque_.push_back(key_frame_pose.cast<float>());
  corner_bag_pub_ptr_->Publish(front_end_->corner_map_bag_);
  odom_pub_ptr_->Publish4d(key_frame_pose, ros::Time::now());

  // save
  // gnss_pose_mutex_.lock();
  Eigen::Quaterniond gnss_quater = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gnss_vector = Eigen::Vector3d::Zero();
  double min_deta_time = 0.1;
  int min_deta_time_index = 0;
  for (size_t i = 0; i < gnss_deque_data_.size(); i++) {
    if (fabs(gnss_deque_data_[i].gnss_time - lidar_time_) < min_deta_time) {
      min_deta_time = fabs(gnss_deque_data_[i].gnss_time - lidar_time_);
      min_deta_time_index = i;
      gnss_quater = gnss_deque_data_[i].gnss_quater;
      gnss_vector = gnss_deque_data_[i].gnss_vector;
    }
  }
  gnss_deque_data_.erase(gnss_deque_data_.begin(),
                         gnss_deque_data_.begin() + min_deta_time_index);
  // gnss_pose_mutex_.unlock();

  if (min_deta_time > 0.05) return;
  lidar_pose_deque_.push_back(key_frame_pose.cast<float>());
  // int index = 0;
  AddCloudToFile(pcl_raw_cloud.cloud_ptr, save_index_);
  SaveCalibPoseToFile(now_quater_, gnss_quater, now_pos_, gnss_vector);
  save_index_++;
  // ROS_ERROR("lidar_deque, %d, %d", lidar_pose_deque_.size(), save_index_);

  CloudData::CLOUD_PTR tran_key_cloud(new CloudData::CLOUD());
  pcl::transformPointCloud(*pcl_raw_cloud.cloud_ptr, *tran_key_cloud,
                           key_frame_pose.cast<float>());
  intensity_feature_pub_ptr_->Publish(tran_key_cloud);
}

void SaveCalibData::SaveCalibPoseToFile(const Eigen::Quaterniond &lidar_quater,
                                        const Eigen::Quaterniond &gnss_quater,
                                        const Eigen::Vector3d &lidar_pos,
                                        const Eigen::Vector3d &gnss_pos) {
  //   std::string gps_odom_path = "../lidar_gnss_pose.txt";
  std::string calib_pose_file = key_frames_path_ + "/lidar_gnss_pose.txt";

  // std::cout << "pose path: " << calib_pose_file << std::endl;
  std::ofstream myfile(calib_pose_file.c_str(), std::ios::app);
  myfile << to_string(lidar_pos.x()) << " " << to_string(lidar_pos.y()) << " "
         << to_string(lidar_pos.z()) << " " << to_string(lidar_quater.x())
         << " " << to_string(lidar_quater.y()) << " "
         << to_string(lidar_quater.z()) << " " << to_string(lidar_quater.w())
         << "\n"
         << to_string(gnss_pos.x()) << " " << to_string(gnss_pos.y()) << " "
         << to_string(gnss_pos.z()) << " " << to_string(gnss_quater.x()) << " "
         << to_string(gnss_quater.y()) << " " << to_string(gnss_quater.z())
         << " " << to_string(gnss_quater.w()) << "\n";
  myfile.close();

  std::cout << "save pose ok" << std::endl;
}

void SaveCalibData::ReceiveGnssData(
    const nav_msgs::OdometryConstPtr &receive_data) {
  double time = receive_data->header.stamp.toSec();
  Eigen::Quaterniond gnss_quater(receive_data->pose.pose.orientation.w,
                                 receive_data->pose.pose.orientation.x,
                                 receive_data->pose.pose.orientation.y,
                                 receive_data->pose.pose.orientation.z);
  Eigen::Vector3d gnss_vector(receive_data->pose.pose.position.x,
                              receive_data->pose.pose.position.y,
                              receive_data->pose.pose.position.z);
  if(flag_frist_gnss_){
    flag_frist_gnss_ = false;
    fixed_gnss_point_ = gnss_vector;
  }
  if (receive_data->pose.covariance[0] > 0.2 ||
      receive_data->pose.covariance[7] > 0.2 ||
      receive_data->pose.covariance[14] > 0.2 ||
      receive_data->pose.covariance[21] > 0.2 ||
      receive_data->pose.covariance[28] > 0.2/* ||
      receive_data->pose.covariance[35] > 0.2*/)
    return;
  // Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();
  // gnss_pose.block(0, 0, 3, 3) = gnss_quater.matrix();
  // gnss_pose.block(0, 3, 3, 1) = gnss_pos;

  GNSSData now_gnss_data;
  // now_gnss_data.gnss_pose = gnss_pose;
  now_gnss_data.gnss_time = time;
  now_gnss_data.gnss_quater = gnss_quater;
  now_gnss_data.gnss_vector = gnss_vector - fixed_gnss_point_;

  gnss_pose_mutex_.lock();
  gnss_deque_data_.push_back(now_gnss_data);
  gnss_pose_mutex_.unlock();
}

void SaveCalibData::SaveMapCloud() {
  save_mutex_.lock();
  std::vector<Eigen::Matrix4f> lidar_pose_deque(lidar_pose_deque_);
  save_mutex_.unlock();

  if (lidar_pose_deque.size() < 20) return;

  CloudData::CLOUD_PTR global_map(new CloudData::CLOUD());
  for (size_t i = 0; i < lidar_pose_deque.size(); i++) {
    // load cloud
    CloudData::CLOUD_PTR now_key_cloud(new CloudData::CLOUD());
    CloudData::CLOUD_PTR seg_key_cloud(new CloudData::CLOUD());
    CloudData::CLOUD_PTR tran_key_cloud(new CloudData::CLOUD());
    pcl::io::loadPCDFile(
        key_frames_path_ + "/cloud/" + std::to_string(i) + ".pcd",
        *now_key_cloud);

    for (size_t j = 0; j < now_key_cloud->points.size(); j++) {
      if (now_key_cloud->points[j].x < 20.0)
        seg_key_cloud->points.push_back(now_key_cloud->points[j]);
    }
    pcl::transformPointCloud(*seg_key_cloud, *tran_key_cloud,
                             lidar_pose_deque[i]);
    *global_map += *tran_key_cloud;
  }

  CloudData::CLOUD_PTR down_global_map(new CloudData::CLOUD());
  map_filter_.setInputCloud(global_map);
  map_filter_.filter(*down_global_map);

  // save
  std::string map_path = key_frames_path_ + "/map.pcd";
  std::cout << "save map size: " << down_global_map->points.size() << std::endl;
  if (!pcl::io::savePCDFileBinary(map_path.c_str(), *down_global_map)) {
    // AWARN << "save pcd ok!";
    std::cout << "save map ok" << std::endl;
  } else {
    // AWARN << "save pcd fail";
    std::cout << "save map fail" << std::endl;
  }
  // publish
  map_pub_ptr_->Publish(global_map);
}

void SaveCalibData::MapStructNode() {
  // ros::NodeHandle nh_matcher;
  // ros::CallbackQueue livox_callback_queue;
  // nh_matcher.setCallbackQueue(&livox_callback_queue);

  // ros::Subscriber livox_sub = nh_matcher.subscribe<sensor_msgs::PointCloud2>(
  //     "/livox/lidar", 1, &MatcherFlow::ReceiveLivoxData, this);
  ros::Rate rate(0.02);
  while (ros::ok()) {
    // livox_callback_queue.callAvailable(ros::WallDuration());
    SaveMapCloud();
    rate.sleep();
  }
  // return NULL;
  return;
}

void SaveCalibData::MatcherNode() {
  ros::NodeHandle nh_matcher;
  ros::CallbackQueue livox_callback_queue;
  nh_matcher.setCallbackQueue(&livox_callback_queue);

  ros::Subscriber livox_sub = nh_matcher.subscribe<sensor_msgs::PointCloud2>(
      "/livox/lidar", 1, &SaveCalibData::ReceiveLidarData, this);
  ros::Rate rate(15);
  while (nh_matcher.ok()) {
    livox_callback_queue.callAvailable(ros::WallDuration());
    rate.sleep();
  }
  // return NULL;
  return;
}

bool SaveCalibData::ResetParam() {
  bool make_file_ok = true;
  RemoveDir(key_frames_path_.c_str());
  if (mkdir(key_frames_path_.c_str(), 0777)) {
    AWARN << "make file errorly ! " << std::endl;
    make_file_ok = false;
  } else {
    AWARN << "make file sucess ! " << std::endl;
    make_file_ok = true;
  }
  std::string cloud_dir_name = key_frames_path_ + "/cloud";
  mkdir(cloud_dir_name.c_str(), 0777);
  // save_data_to_file_.clear();
  // save_now_pcd_num_ = 0;
  return make_file_ok;
}

int SaveCalibData::RemoveDir(const char *dir) {
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

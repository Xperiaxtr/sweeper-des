#include "flow/matcher_flow.hpp"

namespace mapping {
MatcherFlow::MatcherFlow(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT,
                                                     CloudData::POINT>()),
      local_map_ptr_(new CloudData::CLOUD()),
      global_map_corner_ptr_(new CloudData::CLOUD()),
      global_map_surf_ptr_(new CloudData::CLOUD()),
      global_map_intensity_ptr_(new CloudData::CLOUD()) {
  flag_get_fixed_point_ = flag_gnss_to_map_ = false;
  flag_livox_to_gnss_ = false;
  get_new_lidar_data_ = false;
  flag_receive_init_pose_ = false;
  anh_ndt_.setResolution(1.0);
  anh_ndt_.setStepSize(0.1);
  anh_ndt_.setTransformationEpsilon(0.01);
  anh_ndt_.setMaximumIterations(30);

  ndt_ptr_->setResolution(1.0);
  ndt_ptr_->setStepSize(0.1);
  ndt_ptr_->setTransformationEpsilon(0.01);
  ndt_ptr_->setMaximumIterations(30);

  register_.reset(new pclomp::NormalDistributionsTransform<CloudData::POINT,
                                                           CloudData::POINT>());

  kdtree_from_seg_map_ = pcl::KdTreeFLANN<CloudData::POINT>::Ptr(
      new pcl::KdTreeFLANN<CloudData::POINT>());
  register_->setResolution(1.0);
  int avalib_cpus = omp_get_max_threads();
  std::cout << "cpus: " << avalib_cpus << std::endl;
  register_->setNumThreads(1);
  register_->setNeighborhoodSearchMethod(pclomp::DIRECT7);

  register_->setTransformationEpsilon(0.01);
  register_->setMaximumIterations(30);
  register_->setResolution(1.0);

  cloud_filter_.setLeafSize(0.3, 0.3, 0.3);
  local_map_filter_.setLeafSize(0.3, 0.3, 0.3);

  corner_filter_.setLeafSize(0.5, 0.5, 0.5);
  surf_filter_.setLeafSize(0.5, 0.5, 0.5);
  intensity_filter_.setLeafSize(0.5, 0.5, 0.5);
  // map_corner_filter_.setLeafSize(0.5, 0.5, 0.5);
  // map_surf_filter_.setLeafSize(0.5, 0.5, 0.5);
  // map_intensity_filter_.setLeafSize(0.5, 0.5, 0.5);

  local_map_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "global_map", 100, "/camera_init");
  cloud_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "now_cloud", 100, "/camera_init");
  segment_map_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "segment_map", 100, "/camera_init");

  odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "sweeper/navigation/lidar_odom", "/camera_init", "lidar", 100);

  init_pose_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "sweeper/navigation/init_pose", "/camera_init", "lidar", 100);

  gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "sweeper/navigation/gnss_odom", "/camera_init", "gnss", 100);

  local_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "local_odom", "/camera_init", "lidar", 100);

  pred_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "sweeper/navigation/predict_odom", "/camera_init", "lidar", 100);

  pub_state_information_ = nh.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/localization/diagnose", 1);

  sub_gnss_data_ = nh.subscribe<nav_msgs::Odometry>(
      "/sweeper/localization/gnss", 1, &MatcherFlow::ReceiveGnssData, this);
  // sub_livox_data_ = nh.subscribe<sensor_msgs::PointCloud2>(
  //     "/livox/lidar", 1, &MatcherFlow::ReceiveLivoxData, this);
  sub_vehicle_data_ = nh.subscribe<sweeper_msgs::SweeperChassisDetail>(
      "/sweeper/chassis/detail", 1, &MatcherFlow::ReceiveVehicleData, this);
  sub_imu_data = nh.subscribe<sensor_msgs::Imu>(
      "/sweeper/sensor/imu", 1, &MatcherFlow::ReceiveImuData, this);
  sub_sweeper_mode_ = nh.subscribe<sweeper_msgs::SweepMission>(
      "/sweeper/sweep_mode", 1, &MatcherFlow::ReceiveSweeperMode, this);

  timer_matcher_state_ = nh.createTimer(
      ros::Duration(0.1), &MatcherFlow::PubLocalizationInformation, this);

  pub_cov_odom_ = nh.advertise<nav_msgs::Odometry>("/sweeper_cov", 1);

  InitMatcherParam();
  read_pcd_name_ = "../uname.pcd";
  last_pcd_name_ = "unname";

  double corner_filter_size, surf_filter_size, intensity_filter_size,
      corner_map_filter_size, surf_map_filter_size, intensity_map_filter_size;
  MathCalculation::GetYamlParam(nh_private, "localization/corner_map_filter",
                                corner_map_filter_size, 0.3);
  MathCalculation::GetYamlParam(nh_private, "localization/surf_map_filter",
                                surf_map_filter_size, 0.3);
  MathCalculation::GetYamlParam(nh_private, "localization/intensity_map_filter",
                                intensity_map_filter_size, 0.3);
  MathCalculation::GetYamlParam(nh_private, "localization/corner_filter",
                                corner_filter_size, 0.3);
  MathCalculation::GetYamlParam(nh_private, "localization/surf_filter",
                                surf_filter_size, 0.3);
  MathCalculation::GetYamlParam(nh_private, "localization/intensity_filter",
                                intensity_filter_size, 0.3);
  EnterFilterParam(corner_filter_size, surf_filter_size, intensity_filter_size,
                   corner_map_filter_size, surf_map_filter_size,
                   intensity_map_filter_size);

  //特征提取参数
  double livox_hight, corner_min_curvature, surf_max_curvature, the_min_hight,
      intensity_min_different, min_slope_different, max_z_hight;
  MathCalculation::GetYamlParam(nh_private, "feature/livox_hight", livox_hight,
                                -2.13);
  MathCalculation::GetYamlParam(nh_private, "feature/corner_min_curvature",
                                corner_min_curvature, 0.01);
  MathCalculation::GetYamlParam(nh_private, "feature/surf_max_curvature",
                                surf_max_curvature, 0.005);
  MathCalculation::GetYamlParam(nh_private, "feature/the_min_hight",
                                the_min_hight, 0.06);
  MathCalculation::GetYamlParam(nh_private, "feature/intensity_min_different",
                                intensity_min_different, 0.1);
  MathCalculation::GetYamlParam(nh_private, "feature/min_slope_different",
                                min_slope_different, 0.01);
  MathCalculation::GetYamlParam(nh_private, "feature/max_z_hight", max_z_hight,
                                0.01);

  // get_feature_.GetFeatureParam(livox_hight, corner_min_curvature,
  //                              surf_max_curvature, the_min_hight,
  //                              intensity_min_different, min_slope_different,
  //                              max_z_hight, 0.1, 0.1, 0.1, 0.1);
  get_feature_ = new GetFeature(
      livox_hight, corner_min_curvature, surf_max_curvature, the_min_hight,
      intensity_min_different, min_slope_different, max_z_hight,
      corner_filter_size, surf_filter_size, intensity_filter_size, 0.2);

  std::vector<double> map_segment_size(6, 100.0);
  map_segment_.SetSize(map_segment_size);

  // gnss参数
  std::string gnss_config_path =
      "../sweeper_ws/src/sweeper_haide/calibration/data/"
      "livox_to_gps.yaml";
  flag_livox_to_gnss_ = MathCalculation::GetCalibYamlParam(
      gnss_config_path, livox_to_gnss_quater_, livox_to_gnss_pos_);

  nh_private.param<bool>("use_ndt_mode", use_ndt_mode_, false);
  nh_private.param<bool>("use_imu_data", use_imu_data_, true);
  nh_private.param<bool>("use_odom_data", use_odom_data_, true);
  nh_private.param<bool>("use_gnss_data", use_gnss_data_, true);
  int max_iteration = 8;
  nh_private.param<int>("max_iteration", max_iteration, 8);
  matcher_ = new SE3Matcher(max_iteration);
}

void MatcherFlow::EnterFilterParam(double corner_filter_size,
                                   double surf_filter_size,
                                   double intensity_filter_size,
                                   double corner_map_filter_size,
                                   double surf_map_filter_size,
                                   double intensity_map_filter_size) {
  corner_filter_.setLeafSize(corner_filter_size, corner_filter_size,
                             corner_filter_size);
  surf_filter_.setLeafSize(surf_filter_size, surf_filter_size,
                           surf_filter_size);
  intensity_filter_.setLeafSize(intensity_filter_size, intensity_filter_size,
                                intensity_filter_size);
  map_corner_filter_.setLeafSize(corner_map_filter_size, corner_map_filter_size,
                                 corner_map_filter_size);
  map_surf_filter_.setLeafSize(surf_map_filter_size, surf_map_filter_size,
                               surf_map_filter_size);
  map_intensity_filter_.setLeafSize(intensity_map_filter_size,
                                    intensity_map_filter_size,
                                    intensity_map_filter_size);
}

void MatcherFlow::ReceiveSweeperMode(
    sweeper_msgs::SweepMission sweeper_mode_command) {
  //得到模式，地图，初始位置
  if (sweeper_mode_command.mode == 5) {
    switch (sweeper_mode_command.start_cmd) {
      case 1:  //开始
        read_pcd_name_ = sweeper_mode_command.map;
        if (read_pcd_name_.size() > 0)
          if (read_pcd_name_.compare(last_pcd_name_) || fitness_score_ > 5.0) {
            std::string path_name =
                "../sweeper_ws/src/sweeper_haide/data/path/";
            AWARN << "load pcd name : " << path_name + read_pcd_name_
                  << " start load map";
            if (fitness_score_ <= 15.0) {
              std::string gnss_to_map_path = path_name + read_pcd_name_;
              gnss_to_map_path.erase(gnss_to_map_path.end() - 5,
                                     gnss_to_map_path.end());

              AWARN << "yaml : " << gnss_to_map_path + "gps_to_map.yaml";
              flag_gnss_to_map_ = MathCalculation::GetCalibYamlParam(
                  gnss_to_map_path + "gps_to_map.yaml", gnss_to_map_quater_,
                  gnss_to_map_pos_);

              flag_get_fixed_point_ = MathCalculation::GetFixedGpsPoint(
                  gnss_to_map_path + "gps_to_map.yaml", gnss_fixed_pos_);

              CurbAndLoadPcd(path_name + read_pcd_name_);
              // SegmentMap(Eigen::Vector3d(current_pose.x, current_pose.y,
              //                            current_pose.z));

              // MCLInit(current_pose, 2.0);
              AWARN << "get pcd";
            }
            Eigen::Quaterniond init_quater(
                sweeper_mode_command.init_pose.orientation.w,
                sweeper_mode_command.init_pose.orientation.x,
                sweeper_mode_command.init_pose.orientation.y,
                sweeper_mode_command.init_pose.orientation.z);

            init_pose_ = Eigen::Matrix4d::Identity();
            init_pose_.block(0, 0, 3, 3) = init_quater.matrix();
            init_pose_.block(0, 3, 3, 1) =
                Eigen::Vector3d(sweeper_mode_command.init_pose.position.x,
                                sweeper_mode_command.init_pose.position.y,
                                sweeper_mode_command.init_pose.position.z);
            flag_receive_init_pose_ = true;
            // init_pose_pub_ptr_->SetInformation(odom_information);
            init_pose_pub_ptr_->Publish4d(init_pose_, ros::Time::now());
            if (MCLInit(init_pose_, 60,
                        Eigen::Matrix<double, 1, 4>(1.0, 1.0, 0.4, 0.2))) {
              init_pose_set_ = true;
            }
            // init_pose_set_ = true;

            last_pcd_name_ = read_pcd_name_;
            flag_start_lidar_matcher_ = true;
          }
        break;
      case 2:  //取消,重置所有参数
        map_loaded_ = false;
        init_pose_set_ = false;
        last_pcd_name_ = "uname";
        fitness_score_ = 0.0;
        map_segment_.frist_map_segment_ = true;
        flag_get_fixed_point_ = flag_gnss_to_map_ = false;
        flag_start_lidar_matcher_ = false;
        flag_receive_init_pose_ = false;
        break;
    }
  } else {
    map_loaded_ = false;
    init_pose_set_ = false;
    last_pcd_name_ = "uname";
    fitness_score_ = 0.0;
    map_segment_.frist_map_segment_ = true;
    flag_get_fixed_point_ = flag_gnss_to_map_ = false;
    flag_start_lidar_matcher_ = false;
    flag_receive_init_pose_ = false;
  }
}

void MatcherFlow::CurbAndLoadPcd(std::string load_pcd_name) {
  CloudData::CLOUD_PTR local_map(new CloudData::CLOUD());
  global_map_corner_ptr_->clear();
  global_map_surf_ptr_->clear();
  global_map_intensity_ptr_->clear();
  // if (use_ndt_mode_) {
  std::string delete_string = ".pcd";
  int length = strlen(load_pcd_name.c_str()) - strlen(delete_string.c_str());
  std::string cal_string = load_pcd_name.substr(0, length);
  std::string pcd_name = cal_string + "_cloud.pcd";
  AWARN << "cloud path : " << pcd_name;
  pcl::io::loadPCDFile(pcd_name.c_str(), *local_map);

  local_map_filter_.setInputCloud(local_map);
  local_map_filter_.filter(*local_map_ptr_);
  // anh_ndt_.setInputTarget(local_map_ptr_);
  // } else {
  // AWARN << "load pcd";
  CloudData::CLOUD_PTR corner_map(new CloudData::CLOUD());
  CloudData::CLOUD_PTR surf_map(new CloudData::CLOUD());
  CloudData::CLOUD_PTR intensity_map(new CloudData::CLOUD());
  pcl::io::loadPCDFile(load_pcd_name.c_str(), *local_map);
  for (size_t i = 0; i < local_map->points.size(); i++) {
    if (local_map->points[i].intensity < 1.0) {
      surf_map->points.push_back(local_map->points[i]);
    } else if (local_map->points[i].intensity < 10.0) {
      corner_map->points.push_back(local_map->points[i]);
    } else {
      intensity_map->points.push_back(local_map->points[i]);
    }
  }
  // }

  map_corner_filter_.setInputCloud(corner_map);
  map_corner_filter_.filter(*global_map_corner_ptr_);
  map_surf_filter_.setInputCloud(surf_map);
  map_surf_filter_.filter(*global_map_surf_ptr_);
  map_intensity_filter_.setInputCloud(intensity_map);
  map_intensity_filter_.filter(*global_map_intensity_ptr_);

  mcl_matcher_.GetMatcherMap(global_map_corner_ptr_, global_map_surf_ptr_,
                             global_map_intensity_ptr_);

  AWARN << "pcd load ok!";
  local_map_pub_ptr->Publish(local_map);
  map_loaded_ = true;
}

void MatcherFlow::InitMatcherParam() {
  map_loaded_ = false;
  init_pose_set_ = false;
  fitness_score_ = 0.0;
  sweeper_information_deque_.clear();
  imu_information_deque_.clear();
  predict_pose_ = init_pose_ = now_pose_ = Eigen::Matrix4d::Identity();
}

void MatcherFlow::ReceiveGnssData(const nav_msgs::Odometry::ConstPtr& input) {
  watch_dog_gnss_.UpdataNow();

  if (!(flag_gnss_to_map_ && flag_livox_to_gnss_ && flag_get_fixed_point_))
    return;

  std::string gnss_mode = input->child_frame_id;
  Eigen::Quaterniond gnss_quat(
      input->pose.pose.orientation.w, input->pose.pose.orientation.x,
      input->pose.pose.orientation.y, input->pose.pose.orientation.z);
  Eigen::Vector3d gnss_pos(input->pose.pose.position.x - gnss_fixed_pos_.x(),
                           input->pose.pose.position.y - gnss_fixed_pos_.y(),
                           input->pose.pose.position.z - gnss_fixed_pos_.z());

  //从gps转换到lidar T(g->w) *T(l->g)
  Eigen::Quaterniond lidar_gnss_quater = gnss_quat * livox_to_gnss_quater_;
  Eigen::Vector3d lidar_gnss_pos = gnss_quat * livox_to_gnss_pos_ + gnss_pos;
  // //转换到map
  Eigen::Quaterniond map_gnss_quater = gnss_to_map_quater_ * lidar_gnss_quater;
  Eigen::Vector3d map_gnss_pos =
      gnss_to_map_quater_ * lidar_gnss_pos + gnss_to_map_pos_;

  Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();
  gnss_pose.block(0, 0, 3, 3) = map_gnss_quater.matrix();
  gnss_pose.block(0, 3, 3, 1) = map_gnss_pos;

  if (gnss_mode == "bad" || input->pose.covariance[0] > 0.15 ||
      input->pose.covariance[7] > 0.15 || input->pose.covariance[14] > 0.3 ||
      input->pose.covariance[35] > 1.0)
    return;
  gnss_pub_ptr_->Publish4d(gnss_pose, ros::Time::now());

  if (!init_pose_set_ || fitness_score_ >= 5.0) {
    if (input->pose.covariance[0] < 0.3 || input->pose.covariance[7] < 0.3 ||
        input->pose.covariance[14] < 0.3 || input->pose.covariance[35] < 0.3) {
      now_pose_ = gnss_pose;
      init_pose_set_ = true;
    } else if (get_new_lidar_data_ && map_loaded_) {
      Eigen::Matrix<double, 1, 4> gnss_cov(
          sqrt(input->pose.covariance[0]), sqrt(input->pose.covariance[7]),
          sqrt(input->pose.covariance[14]), sqrt(input->pose.covariance[21]));
      if (MCLInit(gnss_pose, 30, gnss_cov)) init_pose_set_ = true;
    }
  }
}

bool MatcherFlow::SegmentMap(Eigen::Vector3d pose) {
  if (map_loaded_ &&
      map_segment_.SetOrigin(Eigen::Vector3d(pose.x(), pose.y(), pose.z()))) {
    if (!use_ndt_mode_) {
      std::mutex matcher_mutex;
      matcher_mutex.lock();
      CloudData::CLOUD_PTR corner_map_ptr_(new CloudData::CLOUD());
      CloudData::CLOUD_PTR surf_map_ptr_(new CloudData::CLOUD());
      CloudData::CLOUD_PTR intensity_map_ptr_(new CloudData::CLOUD());
      map_segment_.Filter(global_map_corner_ptr_, corner_map_ptr_);
      map_segment_.Filter(global_map_surf_ptr_, surf_map_ptr_);
      map_segment_.Filter(global_map_intensity_ptr_, intensity_map_ptr_);
      map_segment_.PrintEdge();

      CloudData::CLOUD_PTR down_size_corner(new CloudData::CLOUD());
      CloudData::CLOUD_PTR down_size_surf(new CloudData::CLOUD());
      CloudData::CLOUD_PTR down_size_intensity(new CloudData::CLOUD());
      map_corner_filter_.setInputCloud(corner_map_ptr_);
      map_corner_filter_.filter(*down_size_corner);
      map_surf_filter_.setInputCloud(surf_map_ptr_);
      map_surf_filter_.filter(*down_size_surf);
      map_intensity_filter_.setInputCloud(intensity_map_ptr_);
      map_intensity_filter_.filter(*down_size_intensity);
      AWARN << "size: " << down_size_corner->points.size() << " "
            << down_size_surf->points.size() << " "
            << down_size_intensity->points.size();
      matcher_->CopyMapToMacther(down_size_corner, down_size_surf,
                                 down_size_intensity);
      mcl_matcher_.GetMatcherMap(down_size_corner, down_size_surf,
                                 down_size_intensity);
      matcher_mutex.unlock();

      segment_map_pub_ptr_->Publish(corner_map_ptr_);
      AWARN << "get map ok"
            << "  size: " << corner_map_ptr_->points.size();
      return true;
    } else {
      CloudData::CLOUD_PTR local_map(new CloudData::CLOUD());
      map_segment_.Filter(local_map_ptr_, local_map);
      map_segment_.PrintEdge();
      std::mutex matcher_mutex;
      matcher_mutex.lock();

      // ndt_ptr_->setInputTarget(local_map);
      register_->setInputTarget(local_map);
      kdtree_from_seg_map_->setInputCloud(local_map);
      matcher_mutex.unlock();

      segment_map_pub_ptr_->Publish(local_map);
      AWARN << "get ndt map ok"
            << "  size: " << local_map->points.size();
      return true;
    }
  }

  return false;
}

bool MatcherFlow::GetMaxValue(Point2 i, Point2 j) {
  return (i.score < j.score);
}

bool MatcherFlow::MCLInit(Eigen::Matrix4d init_pose, int particle_num,
                          Eigen::Matrix<double, 1, 4> pose_cov) {
  Eigen::Matrix3d init_rotation = init_pose.block(0, 0, 3, 3);
  Eigen::Quaterniond init_quater(init_rotation);
  tf::Quaternion tf_quater(init_quater.x(), init_quater.y(), init_quater.z(),
                           init_quater.w());
  double roll, pitch, yaw;
  tf::Matrix3x3(tf_quater).getRPY(roll, pitch, yaw);

  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(init_pose(0, 3), pose_cov(0, 0));
  std::normal_distribution<double> dist_y(init_pose(1, 3), pose_cov(0, 1));
  std::normal_distribution<double> dist_z(init_pose(2, 3), pose_cov(0, 2));
  std::normal_distribution<double> dist_yaw(yaw, pose_cov(0, 3));
  std::vector<Pose> mcl_poses;
  std::vector<Point2> weights;

  if (!get_new_lidar_data_) return false;
  get_new_lidar_data_ = false;
  //计算特征
  // Feature now_feature;
  // get_vlp16_feature_.GetFeature(pcl_raw_cloud_, now_feature);

  // pcl::transformPointCloud(*now_feature.corner_cloud,
  // *now_feature.corner_cloud,
  //                          rslidar_to_ins_matrix_.cast<float>());
  // pcl::transformPointCloud(*now_feature.surf_cloud,
  // *now_feature.surf_cloud,
  //                          rslidar_to_ins_matrix_.cast<float>());
  // pcl::transformPointCloud(*now_feature.intensity_cloud,
  //                          *now_feature.intensity_cloud,
  //                          rslidar_to_ins_matrix_.cast<float>());

  for (int i = 0; i < particle_num; i++) {
    Pose pose;
    pose.x = dist_x(gen);
    pose.y = dist_y(gen);
    pose.z = dist_z(gen);
    pose.yaw = dist_yaw(gen);
    pose.roll = roll;
    pose.pitch = pitch;
    double score = GetMCLScore(pose, now_feature_);
    mcl_poses.push_back(pose);
    Point2 weight;
    weight.ind = i;
    weight.score = score;
    weights.push_back(weight);
  }
  std::sort(weights.begin(), weights.end(), GetMaxValue);  //降序排列
  double score = weights[0].score;
  Pose mcl_pose = mcl_poses[weights[0].ind];
  geometry_msgs::Quaternion target_q;
  target_q = tf::createQuaternionMsgFromYaw(mcl_pose.yaw);

  // matcher_.InitMatcherPose(
  //     Eigen::Quaterniond(target_q.w, target_q.x, target_q.y, target_q.z),
  //     Eigen::Vector3d(mcl_pose.x, mcl_pose.y, mcl_pose.z), 4);
  // double matcher_before_time = clock();
  // matcher_.GetCloudToMapMacth(now_feature.corner_cloud,
  // now_feature.surf_cloud,
  //                             now_feature.intensity_cloud);
  // AWARN << "mcl matcher time: "
  //       << (double)(clock() - matcher_before_time) / CLOCKS_PER_SEC;

  // Eigen::Quaterniond now_quater = matcher_.m_q_w_curr_;
  // Eigen::Vector3d now_pos = matcher_.m_t_w_curr_;
  // double score = matcher_.GetMatcherScore(10, now_feature.surf_cloud,
  //                                         now_feature.corner_cloud);

  AWARN << "mcl score : " << score;
  if (score > 2.0) return false;

  // odom_pub_ptr_->PublishQuat(now_quater, now_pos, ros::Time::now());
  Eigen::Quaterniond lidar_quater =
      Eigen::Quaterniond(target_q.w, target_q.x, target_q.y, target_q.z);
  now_pose_.block(0, 0, 3, 3) = lidar_quater.matrix();
  now_pose_.block(0, 3, 3, 1) =
      Eigen::Vector3d(mcl_pose.x, mcl_pose.y, mcl_pose.z);

  Eigen::Matrix<double, 6, 6> odom_information =
      Eigen::Matrix<double, 6, 6>::Identity();
  odom_information(0, 0) = odom_information(1, 1) = odom_information(2, 2) =
      odom_information(3, 3) = odom_information(4, 4) = odom_information(5, 5) =
          weights[0].score;
  init_pose_pub_ptr_->SetInformation(odom_information);
  init_pose_pub_ptr_->Publish4d(now_pose_, ros::Time::now());
  return true;
}

double MatcherFlow::GetMCLScore(Pose& pose, Feature& now_feature) {
  //转换为四元数位置
  Eigen::Quaterniond predict_quater =
      Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pose.pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(pose.roll, Eigen::Vector3d::UnitX());
  mcl_matcher_.PredictMatcherPose(predict_quater,
                                  Eigen::Vector3d(pose.x, pose.y, pose.z));

  double score = mcl_matcher_.GetCloudToMapMacth(now_feature.corner_cloud,
                                                 now_feature.surf_cloud,
                                                 now_feature.intensity_cloud);
  //再对四元数转换为位置
  tf::Quaternion tf_quater(
      mcl_matcher_.m_q_w_curr_.x(), mcl_matcher_.m_q_w_curr_.y(),
      mcl_matcher_.m_q_w_curr_.z(), mcl_matcher_.m_q_w_curr_.w());
  // Pose localizer_pose;
  pose.x = mcl_matcher_.m_t_w_curr_.x();
  pose.y = mcl_matcher_.m_t_w_curr_.y();
  pose.z = mcl_matcher_.m_t_w_curr_.z();
  tf::Matrix3x3 tf_matrix(tf_quater);
  tf_matrix.getRPY(pose.roll, pose.pitch, pose.yaw);

  return score;
}

void MatcherFlow::MatcherNode() {
  ros::NodeHandle nh_matcher;
  ros::CallbackQueue livox_callback_queue;
  nh_matcher.setCallbackQueue(&livox_callback_queue);

  ros::Subscriber livox_sub = nh_matcher.subscribe<sensor_msgs::PointCloud2>(
      "/livox/lidar", 1, &MatcherFlow::ReceiveLivoxData, this);
  ros::Rate rate(15);
  while (nh_matcher.ok()) {
    livox_callback_queue.callAvailable(ros::WallDuration());
    rate.sleep();
  }
  // return NULL;
  return;
}

void MatcherFlow::CloudAdjustMotion(
    const Eigen::Matrix4d& tran_pose,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud_ptr) {
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

//采用ndt进行匹配
void MatcherFlow::ReceiveLivoxData(
    const sensor_msgs::PointCloud2::ConstPtr& input) {
  // AERROR << "111" << std::endl;
  watch_dog_livox_.UpdataNow();
  // AERROR << "111" << std::endl;
  CloudData pcl_raw_cloud;
  pcl::fromROSMsg(*input, *pcl_raw_cloud.cloud_ptr);
  double lidar_time = input->header.stamp.toSec();
  // AERROR << "111 " <<pcl_raw_cloud.cloud_ptr->points.size()<< std::endl;
  Eigen::Matrix4d tran_pose = PredictLidarPose(lidar_time);
  // AERROR << "111" << std::endl;
  CloudAdjustMotion(tran_pose, pcl_raw_cloud.cloud_ptr);
  // AERROR << "111" << std::endl;
  get_feature_->GetLivoxFeature(pcl_raw_cloud, now_feature_);
  get_new_lidar_data_ = true;

  AWARN << "map_loaded_ : " << map_loaded_ << " flag_receive_init_pose_ "
        << flag_receive_init_pose_ << " init_pose_set_ " << init_pose_set_;

  // if (map_loaded_ && flag_receive_init_pose_) {
  //   flag_receive_init_pose_ = false;
  //   if (MCLInit(init_pose_, 60,
  //               Eigen::Matrix<double, 1, 4>(1.0, 1.0, 0.4, 0.2))) {
  //     init_pose_set_ = true;
  //   } else
  //     init_pose_set_ = false;
  // }

  // init_pose_set_ = true;
  if (map_loaded_ && init_pose_set_) {
    // AERROR << "111" << std::endl;
    SegmentMap(
        Eigen::Vector3d(now_pose_(0, 3), now_pose_(1, 3), now_pose_(2, 3)));
    // AERROR << "111" << std::endl;

    tf::Quaternion predict_quat, matcher_quat, localizer_quat;
    ros::Time current_scan_time = input->header.stamp;
    CloudData::CLOUD_PTR pcl_filter_cloud_ptr(new CloudData::CLOUD());
    cloud_filter_.setInputCloud(pcl_raw_cloud.cloud_ptr);
    cloud_filter_.filter(*pcl_filter_cloud_ptr);

    predict_pose_ = predict_pose_ * tran_pose;
    Eigen::Matrix4d predict_pose = now_pose_ * tran_pose;
    pred_pub_ptr_->Publish4d(predict_pose_, ros::Time::now());

    AWARN << "predict_pose : " << predict_pose;

    nav_msgs::Odometry odom_now;
    odom_now.header.frame_id = "/camera_init";
    odom_now.header.stamp = input->header.stamp;
    Eigen::Matrix4d tran_matrix4d = Eigen::Matrix4d::Identity();

    if (use_ndt_mode_) {
      double matcher_before_time = clock();
      CloudData::CLOUD_PTR output_cloud(new CloudData::CLOUD());
      // ndt_ptr_->setInputSource(pcl_filter_cloud_ptr);
      // ndt_ptr_->align(*output_cloud, predict_pose.cast<float>());
      // cloud_pub_ptr->Publish(output_cloud);

      register_->setInputSource(pcl_filter_cloud_ptr);
      register_->align(*output_cloud, predict_pose.cast<float>());

      Eigen::Matrix4f tran = register_->getFinalTransformation();
      bool has_converged = register_->hasConverged();
      int iteration = register_->getFinalNumIteration();
      // fitness_score_ = register_->getFitnessScore(30.0);
      // CloudData::CLOUD_PTR score_cloud(new CloudData::CLOUD());
      // *score_cloud += *now_feature_.corner_cloud;
      // *score_cloud += *now_feature_.intensity_cloud;
      // *score_cloud += *now_feature_.surf_cloud;

      fitness_score_ = GetMatcherScore(pcl_filter_cloud_ptr, tran, 30, -1.6);
      double trans_probability = register_->getTransformationProbability();

      tran_matrix4d = tran.cast<double>();

      AWARN << "Print ndt matcher message !";
      AWARN << "has_converged : " << has_converged;
      AWARN << "iteration num : " << iteration;
      AWARN << "matcher score : " << fitness_score_;
      AWARN << "trans probability : " << trans_probability;
      AWARN << "matcher time: "
            << (double)(clock() - matcher_before_time) / CLOCKS_PER_SEC;

      // odom_now.pose.pose.position.x = iteration;
      // odom_now.pose.pose.position.y = fitness_score;
      // odom_now.pose.pose.position.z = trans_probability;
    } else  //其他采用特征匹配模式
    {
      //加载预测位置，先从pose转换为四元数和位置
      Eigen::Matrix3d predict_matrix = predict_pose.block(0, 0, 3, 3);
      // Eigen::Quaternionf predict_quaterf(predict_matrix);
      Eigen::Quaterniond predict_quaterd(predict_matrix);

      Eigen::Vector3d predict_position(predict_pose(0, 3), predict_pose(1, 3),
                                       predict_pose(2, 3));

      AWARN << "before : " << predict_position;
      AWARN << "quater: " << predict_quaterd.w() << " " << predict_quaterd.x()
            << " " << predict_quaterd.y() << " " << predict_quaterd.z()
            << " num : "
            << sqrt(predict_quaterd.w() * predict_quaterd.w() +
                    predict_quaterd.x() * predict_quaterd.x() +
                    predict_quaterd.y() * predict_quaterd.y() +
                    predict_quaterd.z() * predict_quaterd.z());

      matcher_->PredictMatcherPose(predict_quaterd, predict_position);
      double matcher_before_time = clock();

      fitness_score_ = matcher_->CloudToMapMacth(now_feature_.corner_cloud,
                                                 now_feature_.surf_cloud,
                                                 now_feature_.intensity_cloud);
      AWARN << "matcher time: "
            << (double)(clock() - matcher_before_time) / CLOCKS_PER_SEC;

      Eigen::Quaterniond now_quater = matcher_->m_q_w_curr_;
      Eigen::Vector3d now_pos = matcher_->m_t_w_curr_;

      tran_matrix4d.block(0, 0, 3, 3) = now_quater.matrix();
      tran_matrix4d.block(0, 3, 3, 1) = now_pos;

      AWARN << "Print plicp matcher message !";
      AWARN << "lidar_match_cov_ : " << fitness_score_;
      AWARN << "afetr matcher : " << now_pos;

      odom_now.pose.pose.position.x = fitness_score_;
      // odom_now.pose.pose.position.y = fitness_score;
      // odom_now.pose.pose.position.z = trans_probability;
    }
    // fitness_score_ *= fitness_score_;

    // Eigen::Matrix<double, 6, 6> odom_information =
    //     Eigen::Matrix<double, 6, 6>::Identity();
    // odom_information(0, 0) = odom_information(1, 1) = odom_information(2,
    // 2)
    // =
    //     odom_information(3, 3) = odom_information(4, 4) =
    //         odom_information(5, 5) = fitness_score;

    // odom_pub_ptr_->SetInformation(odom_information);
    // odom_pub_ptr_->Publish4d(tran_matrix4d, current_scan_time);
    CloudData::CLOUD_PTR cloud_raw_tran(new CloudData::CLOUD());
    pcl::transformPointCloud(*pcl_filter_cloud_ptr, *cloud_raw_tran,
                             tran_matrix4d);
    cloud_pub_ptr->Publish(cloud_raw_tran);

    tf::Matrix3x3 mat_l;
    mat_l.setValue(static_cast<double>(tran_matrix4d(0, 0)),
                   static_cast<double>(tran_matrix4d(0, 1)),
                   static_cast<double>(tran_matrix4d(0, 2)),
                   static_cast<double>(tran_matrix4d(1, 0)),
                   static_cast<double>(tran_matrix4d(1, 1)),
                   static_cast<double>(tran_matrix4d(1, 2)),
                   static_cast<double>(tran_matrix4d(2, 0)),
                   static_cast<double>(tran_matrix4d(2, 1)),
                   static_cast<double>(tran_matrix4d(2, 2)));

    Pose localizer_pose;
    localizer_pose.x = tran_matrix4d(0, 3);
    localizer_pose.y = tran_matrix4d(1, 3);
    localizer_pose.z = tran_matrix4d(2, 3);

    AWARN << "pose : " << localizer_pose.x << " " << localizer_pose.y << " "
          << localizer_pose.z;
    mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw,
                 1);

    double predict_pose_error =
        sqrt((localizer_pose.x - predict_pose(0, 3)) *
                 (localizer_pose.x - predict_pose(0, 3)) +
             (localizer_pose.y - predict_pose(1, 3)) *
                 (localizer_pose.y - predict_pose(1, 3)) +
             (localizer_pose.z - predict_pose(2, 3)) *
                 (localizer_pose.z - predict_pose(2, 3)));
    bool use_predict_pose = false;
    if (predict_pose_error <= 0.4 && fitness_score_ < 5.0) {
      use_predict_pose = false;
    } else {
      use_predict_pose = true;
    }
    use_predict_pose = false;
    // Pose current_pose;
    if (!use_predict_pose) {
      now_pose_ = tran_matrix4d;
    } else {
      now_pose_ = predict_pose;
    }

    Eigen::Matrix<double, 6, 6> odom_information =
        Eigen::Matrix<double, 6, 6>::Identity();
    odom_information(0, 0) = odom_information(1, 1) = odom_information(2, 2) =
        odom_information(3, 3) = odom_information(4, 4) =
            odom_information(5, 5) = fitness_score_;

    odom_pub_ptr_->SetInformation(odom_information);
    odom_pub_ptr_->Publish4d(now_pose_, input->header.stamp);

  } else
    AWARN << "init error";
}

void MatcherFlow::ReceiveImuData(
    const sensor_msgs::Imu::ConstPtr& imu_msg_ptr) {
  // AWARN << "receive imu";
  watch_dog_imu_.UpdataNow();
  double time = imu_msg_ptr->header.stamp.toSec();
  static double last_imu_time = time;
  double diff_time = time - last_imu_time;
  if (diff_time <= 0 || diff_time > 0.1) {
    last_imu_time = time;
    return;
  }
  ImuInformation now_imu_information;
  now_imu_information.time = time;
  now_imu_information.angular_velocity_x = imu_msg_ptr->angular_velocity.x;
  now_imu_information.angular_velocity_y = imu_msg_ptr->angular_velocity.y;
  now_imu_information.angular_velocity_z = imu_msg_ptr->angular_velocity.z;
  if (init_pose_set_) imu_information_deque_.push_back(now_imu_information);
}

void MatcherFlow::ReceiveVehicleData(
    const sweeper_msgs::SweeperChassisDetail sweeper_chassis_detail) {
  watch_dog_vehical_.UpdataNow();
  // AWARN << "receive sweeper";
  double car_yaw = sweeper_chassis_detail.steering_angle_output / 57.3;
  double speed = sweeper_chassis_detail.vehicle_speed_output / 3.6;
  double time = sweeper_chassis_detail.header.stamp.toSec();
  SweeperInformation now_sweeper_information;
  now_sweeper_information.distance = speed * 0.02;
  now_sweeper_information.time = time;
  double car_angle = 0.02 * speed * sin(car_yaw) /
                     (cos(car_yaw) * CAR_FRONT_DIS_ + CAR_BACK_DIS_);
  Eigen::Quaterniond diff_quater =
      Eigen::AngleAxisd(-0.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  now_sweeper_information.diff_quater = diff_quater;
  if (init_pose_set_)
    sweeper_information_deque_.push_back(now_sweeper_information);
}

Eigen::Matrix4d MatcherFlow::PredictLidarPose(double lidar_time) {
  Eigen::Vector3d tran_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Quaterniond tran_quater = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  Eigen::Matrix4d tran_pose = Eigen::Matrix4d::Identity();
  predict_mutex_.lock();
  while (sweeper_information_deque_.size() > 0) {
    // AWARN << "dis_time : " << sweeper_information_deque_[0].time -
    // lidar_time
    //       << " " << imu_information_deque_.size();
    if (sweeper_information_deque_[0].time > lidar_time) {
      break;
    }
    if (lidar_time - sweeper_information_deque_[0].time > 0.1) {
      sweeper_information_deque_.pop_front();
      continue;
    }
    Eigen::Quaterniond lidar_quater = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    if (imu_information_deque_.size() > 0) {
      while (imu_information_deque_.size() > 0) {
        if (imu_information_deque_[0].time -
                sweeper_information_deque_[0].time <
            -0.01) {
          imu_information_deque_.pop_front();
        } else {
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
    tran_pos = tran_quater * lidar_pos + tran_pos;
    tran_quater = tran_quater * lidar_quater;
    // predict_pos_ = predict_quater_ * lidar_pos + predict_pos_;
    // predict_quater_ = predict_quater_ * lidar_quater;

    sweeper_information_deque_.pop_front();
  }
  predict_mutex_.unlock();
  tran_pose.block(0, 0, 3, 3) = tran_quater.matrix();
  tran_pose.block(0, 3, 3, 1) = tran_pos;
  return tran_pose;
}

double MatcherFlow::GetMatcherScore(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& score_cloud,
    const Eigen::Matrix4f& tran_pose, const double& range,
    const double& height) {
  pcl::PointCloud<pcl::PointXYZI> height_cloud;
  for (size_t i = 0; i < score_cloud->points.size(); i++) {
    if (score_cloud->points[i].z > height)
      height_cloud.push_back(score_cloud->points[i]);
  }
  pcl::transformPointCloud(height_cloud, height_cloud, tran_pose);
  float distance_range = 0;
  int nr = 0;
  for (size_t i = 0; i < height_cloud.points.size(); i++) {
    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);
    kdtree_from_seg_map_->nearestKSearch(height_cloud.points[i], 1, nn_indices,
                                         nn_dists);

    if (nn_dists.size() > 0 && nn_dists[0] < range) {
      distance_range += nn_dists[0];
      nr++;
    }
  }
  if (nr < 10) return 500;
  return distance_range / nr;
}

void MatcherFlow::PubLocalizationInformation(const ros::TimerEvent& e) {
  sweeper_msgs::SensorFaultInformation sweep_information_code;
  sweep_information_code.header.frame_id = "localization";
  sweep_information_code.header.stamp = ros::Time::now();
  if (!watch_dog_livox_.DogIsOk(5)) {  //没有大疆数据
    sweep_information_code.state_code.push_back(UN_RECEIVED_LIVOX_DATA_);
  }
  if (!map_loaded_ && flag_start_lidar_matcher_)  //没有载入地图
    sweep_information_code.state_code.push_back(LOAD_MAP_FAIL_);
  if (!init_pose_set_ && flag_start_lidar_matcher_)  //没有接收到初始位置
    sweep_information_code.state_code.push_back(UN_RECEIVE_INIT_POSE_);
  if (!flag_start_lidar_matcher_ &&
      !(flag_gnss_to_map_ && flag_livox_to_gnss_ && flag_get_fixed_point_)) {
    sweep_information_code.state_code.push_back(3119);
  }
  if (map_loaded_ && init_pose_set_)
    sweep_information_code.state_code.push_back(ENTER_LOCALIZATION_FUCTION_);
  if (!sweep_information_code.state_code.empty()) {
    pub_state_information_.publish(sweep_information_code);
    // for (size_t i = 0; i < sweep_information_code.state_code.size(); i++)
    //   AWARN << "state information : " <<
    //   sweep_information_code.state_code[i];
  }
}

}  // namespace mapping
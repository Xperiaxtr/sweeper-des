#include "flow/localization_flow.hpp"

namespace mapping {
LocalizationFlow::LocalizationFlow(ros::NodeHandle nh,
                                   ros::NodeHandle nh_private) {
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/livox/lidar", 1000);
  gnss_sub_ptr_ =
      std::make_shared<GNSSSubscriber>(nh, "/sweeper/sensor/gnss", 1000);
  sub_sweeper_mode_ = nh.subscribe<sweeper_msgs::SweepMission>(
      "/sweeper/sweep_mode", 1, &LocalizationFlow::ReceiveSweeperMode, this);

  corner_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "corner_map_cloud",
                                                     100, "/camera_init");
  odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/sweeper/navigation/lidar_odom", "/camera_init", "lidar", 100);

  pub_state_information_ = nh.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/localization/diagnose", 1);

  corner_feature_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, "corner_features", 100, "/camera_init");
  surf_feature_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "surf_features",
                                                           100, "/camera_init");
  intensity_feature_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, "intensity_features", 100, "/camera_init");

  init_pose_ = Eigen::Matrix4d::Identity();
  now_pos_ = Eigen::Vector3d(0, 0, 0);
  now_quater_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  corner_filter_.setLeafSize(0.3, 0.3, 0.3);
  surf_filter_.setLeafSize(0.5, 0.5, 0.5);
  intensity_filter_.setLeafSize(0.3, 0.3, 0.3);
  map_corner_filter_.setLeafSize(0.3, 0.3, 0.3);
  map_surf_filter_.setLeafSize(0.5, 0.5, 0.5);
  map_intensity_filter_.setLeafSize(0.1, 0.1, 0.1);
  // matcher_.icp_max_iterations_ = 4;
  matcher_.mapping = false;
  pcd_name_ = "../global_map.pcd";
  last_pcd_name_ = "unname";
  load_map_success_ = true;
  init_pose_sucess_ = true;
  allow_localization_mode_ = false;

  nh_private.param<bool>("allow_localization_mode", allow_localization_mode_,
                         0);
  nh_private.param<int>("icp_times", icp_times_, 6);
  AWARN << "allow_localization_mode " << allow_localization_mode_ << std::endl;
  AWARN << "icp_times " << icp_times_ << std::endl;
  matcher_.icp_max_iterations_ = icp_times_;

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

  get_feature_.GetFeatureParam(
      livox_hight, corner_min_curvature, surf_max_curvature, the_min_hight,
      intensity_min_different, min_slope_different, max_z_hight);

  AWARN << "filter: "
        << "corner_map " << corner_map_filter_size << " surf_map"
        << surf_map_filter_size << " intensity_map" << intensity_map_filter_size
        << " corner : " << corner_filter_size << " surf " << surf_filter_size
        << " intensity " << intensity_filter_size << std::endl;
}

void LocalizationFlow::EnterFilterParam(double corner_filter_size,
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

void LocalizationFlow::ReceiveSweeperMode(
    sweeper_msgs::SweepMission sweeper_mode_command) {
  //得到模式，地图，初始位置
  //激光巡迹模式
  if (sweeper_mode_command.mode == 5) {
    switch (sweeper_mode_command.start_cmd) {
      case 1:  //开始
        pcd_name_ = sweeper_mode_command.map;

        if (pcd_name_.compare(last_pcd_name_) && pcd_name_.size() > 0) {
          allow_localization_mode_ = false;
          Eigen::Quaterniond init_quater =
              Eigen::Quaterniond(sweeper_mode_command.init_pose.orientation.w,
                                 sweeper_mode_command.init_pose.orientation.x,
                                 sweeper_mode_command.init_pose.orientation.y,
                                 sweeper_mode_command.init_pose.orientation.z);
          Eigen::Vector3d init_position =
              Eigen::Vector3d(sweeper_mode_command.init_pose.position.x,
                              sweeper_mode_command.init_pose.position.y,
                              sweeper_mode_command.init_pose.position.z);
          AWARN << "load pcd name : " << pcd_name_;
          AWARN << "start load map";
          if (ReadAndLoadMap(pcd_name_)) {
            last_pcd_name_ = pcd_name_;
            load_map_success_ = true;
          } else {
            load_map_success_ = false;
            return;
          }
          AWARN << "start init pose";
          if (GetInitPose(init_quater, init_position)) {
            init_pose_sucess_ = true;
          } else {
            init_pose_sucess_ = false;
            return;
          }
          allow_localization_mode_ = true;
        }
        break;
      case 2:  //取消,重置所有参数
        ResetLidarOdomParam();
        break;
    }
  } else {
    ResetLidarOdomParam();
  }
}

void LocalizationFlow::ResetLidarOdomParam() {
  allow_localization_mode_ = false;
  init_pose_sucess_ = true;
  load_map_success_ = true;
  last_pcd_name_ = "uname";
  
}

void LocalizationFlow::PubInformationThread() {
  ros::Rate rate(10);
  while (ros::ok()) {
    PubLocalizationInformation();
    rate.sleep();
  }
}

void LocalizationFlow::PubLocalizationInformation() {
  sweeper_msgs::SensorFaultInformation sweep_information_code;
  sweep_information_code.header.frame_id = "localization";
  sweep_information_code.header.stamp = ros::Time::now();
  if (!watch_dog_livox_.DogIsOk(5)) {  //没有大疆数据
    sweep_information_code.state_code.push_back(UN_RECEIVED_LIVOX_DATA_);
  }
  if (!load_map_success_)  //没有载入地图
    sweep_information_code.state_code.push_back(LOAD_MAP_FAIL_);
  if (!init_pose_sucess_)  //没有接收到初始位置
    sweep_information_code.state_code.push_back(UN_RECEIVE_INIT_POSE_);
  // //初始位置匹配失败
  // if (frist_matcher_score_ > 5.0) {
  //   ROS_ERROR("frist matcher error");
  //   AERROR << "frist matcher error";
  //   sweep_information_code.state_code.push_back(frist_matcher_error_);
  // }
  if (allow_localization_mode_)
    sweep_information_code.state_code.push_back(ENTER_LOCALIZATION_FUCTION_);
  if (!sweep_information_code.state_code.empty()) {
    pub_state_information_.publish(sweep_information_code);
    for (size_t i = 0; i < sweep_information_code.state_code.size(); i++)
      AWARN << "state information : " << sweep_information_code.state_code[i];
  }
}

void LocalizationFlow::Process() {
  std::deque<CloudData> cloud_data_buff;
  cloud_sub_ptr_->ParseData(cloud_data_buff);
  gnss_sub_ptr_->ParseData(unsynced_gnss_);
  // if (!ReadAndLoadMap(pcd_name_)) return;
  // if (!GetInitPose(init_pose_)) return;
  if (cloud_data_buff.size() > 0) watch_dog_livox_.UpdataNow();
  if (allow_localization_mode_) {
    while (cloud_data_buff.size() > 0) {
      CloudData cloud_data = cloud_data_buff.front();
      cloud_data_buff.pop_front();
      double cloud_time = cloud_data.time;
      // GetFeature get_feature;
      Feature new_feature;
      get_feature_.GetLivoxFeature(cloud_data, new_feature);
      GNSSData synced_gnss;

      CloudData::CLOUD_PTR down_size_corner(new CloudData::CLOUD());
      CloudData::CLOUD_PTR down_size_surf(new CloudData::CLOUD());
      CloudData::CLOUD_PTR down_size_intensity(new CloudData::CLOUD());

      map_corner_filter_.setInputCloud(new_feature.corner_cloud);
      map_corner_filter_.filter(*down_size_corner);

      map_surf_filter_.setInputCloud(new_feature.surf_cloud);
      map_surf_filter_.filter(*down_size_surf);

      map_intensity_filter_.setInputCloud(new_feature.intensity_cloud);
      map_intensity_filter_.filter(*down_size_intensity);

      corner_feature_pub_ptr_->Publish(down_size_corner);
      surf_feature_pub_ptr_->Publish(down_size_surf);
      intensity_feature_pub_ptr_->Publish(down_size_intensity);

      matcher_.GetCloudToMapMacth(down_size_corner, down_size_surf,
                                  down_size_intensity);
      now_quater_ = matcher_.m_q_w_curr_;
      now_pos_ = matcher_.m_t_w_curr_;
      lidar_match_cov_ = matcher_.lidar_match_score_;

      double lidar_odom_cov = MathCalculation::GetMatcherCov(lidar_match_cov_);
      AWARN << "lidar_odom_cov : " << lidar_odom_cov;
      Eigen::Matrix<double, 6, 6> odom_information =
          Eigen::Matrix<double, 6, 6>::Identity();
      odom_information(0, 0) = lidar_odom_cov * 0.1;
      odom_information(1, 1) = lidar_odom_cov * 0.1;
      odom_information(2, 2) = lidar_odom_cov * 0.1;
      odom_information(3, 3) = lidar_odom_cov * 0.1;
      odom_information(4, 4) = lidar_odom_cov * 0.1;
      odom_information(5, 5) = lidar_odom_cov * 0.1;
      odom_pub_ptr_->SetInformation(odom_information);
      odom_pub_ptr_->PublishQuat(now_quater_, now_pos_,
                                 ros::Time().fromSec(cloud_time));

      AWARN << "matcher pose : " << now_pos_ << std::endl;
    }
    corner_pub_ptr_->Publish(matcher_.laser_cloud_corner_map_);
  }
}
// bool LocalizationFlow::GetInitPose(Eigen::Matrix4d pose) {
//   if (init_pose_sucess_) return true;
//   Eigen::Matrix3d init_rotation = pose.block(0, 0, 3, 3);
//   Eigen::Quaterniond init_quater;
//   init_quater = init_rotation;
//   Eigen::Vector3d init_position(pose(0, 3), pose(1, 3), pose(2, 3));
//   matcher_.InitMatcherPose(init_quater, init_position, 6);
//   init_pose_sucess_ = true;
// }

bool LocalizationFlow::GetInitPose(Eigen::Quaterniond init_quater,
                                   Eigen::Vector3d init_position) {
  matcher_.InitMatcherPose(init_quater, init_position, 6);
  return true;
}

bool LocalizationFlow::ReadAndLoadMap(std::string map_name) {
  std::string full_pcd_name = "../sweeper_ws/src/sweeper_haide/data/path/" + map_name;
  // if (load_map_success_) return true;
  CloudData::CLOUD global_map;
  if (pcl::io::loadPCDFile(full_pcd_name.c_str(), global_map) == -1) {
    return false;
  } else {
    CloudData::CLOUD global_map_corner;
    CloudData::CLOUD global_map_surf;
    CloudData::CLOUD global_map_intensity;
    for (size_t i = 0; i < global_map.points.size(); i++) {
      if (global_map.points[i].intensity < 1.0) {
        global_map_surf.points.push_back(global_map.points[i]);
      } else if (global_map.points[i].intensity < 10.0) {
        global_map_corner.points.push_back(global_map.points[i]);
      } else {
        global_map_intensity.points.push_back(global_map.points[i]);
      }
    }
    *matcher_.laser_cloud_corner_map_ = global_map_corner;
    *matcher_.laser_cloud_surf_map_ = global_map_surf;
    *matcher_.laser_cloud_intensity_map_ = global_map_intensity;
    // load_map_success_ = true;
    return true;
  }
}

}  // namespace mapping
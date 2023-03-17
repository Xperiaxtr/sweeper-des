#include "flow/mapping_flow.hpp"

#include <tf/transform_broadcaster.h>

namespace mapping {
MappingFlow::MappingFlow(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : make_file_sucess_(true),
      save_data_sucess_(true),
      enter_mapping_mode_(false),
      // flag_gps_to_map_(false),
      point_attribute_(0),
      lidar_time_(0.0),
      key_frame_index_(0) {
  flag_gps_to_map_ = false;
  predict_quater_ = now_quater_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  predict_pos_ = now_pos_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  last_gps_pos_ = Eigen::Vector3d(-10, -10, -10);

  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, "/livox/lidar", 100000);
  gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(
      nh, "/sweeper/localization/gnss", 100000);

  sub_imu_information_ = nh.subscribe<sensor_msgs::Imu>(
      "/sweeper/sensor/imu", 1, &MappingFlow::ReceiveImuData, this);
  sub_vehical_information_ =
      nh.subscribe("/sweeper/chassis/detail", 1,
                   &MappingFlow::ReceiveSweeperInformation, this);
  sub_mapping_information_ = nh.subscribe<sweeper_msgs::SweepMission>(
      "/sweeper/sweep_mode", 1, &MappingFlow::ReceiveMappingInformation, this);

  global_map_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "global_map", 100, "/camera_init");
  corner_bag_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "corner_cloud", 100, "/camera_init");
  corner_feature_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, "corner_feature", 100, "/camera_init");
  surf_feature_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "surf_feature", 100, "/camera_init");
  intensity_feature_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, "intensity_feature", 100, "/camera_init");
  odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "lidar_odom", "/camera_init", "lidar", 100);
  gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "gnss_odom", "/camera_init", "lidar", 100);
  vehical_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "vehical_odom", "/camera_init", "lidar", 100);
  map_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "map_odom", "/camera_init", "lidar", 100);
  pub_state_information_ = nh.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/common/diagnose", 1);
  pub_optimize_path_ = nh.advertise<nav_msgs::Path>("/loam/path", 1);
  pub_loop_constraints_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/sweeper/mapping/loop_closure_constraints", 1);
  image_transport::ImageTransport it(nh);
  pub_map_image_ = it.advertise("/sweeper/mapping/image", 1);

  key_frames_.clear();
  last_mapping_state_ = 0;

  nh_private.param<bool>("using_loop", using_loop_, 0);
  nh_private.param<bool>("using_gps_data", using_gps_data_, 0);
  nh_private.param<bool>("using_imu_data", using_imu_data_, 0);
  nh_private.param<bool>("pub_global_map", pub_global_map_, 0);
  nh_private.param<bool>("using_debug_mode", allow_mapping_mode_, 0);
  nh_private.param<int>("icp_times", icp_times_, 8);
  nh_private.param<bool>("use_loop_closure", use_loop_closure_, 0);
  nh_private.param<bool>("need_init_mapping", need_init_mapping_, 0);
  nh_private.param<bool>("init_mapping", init_mapping_, 0);

  AWARN << "using_loop " << using_loop_ << std::endl;
  AWARN << "using_gps_data " << using_gps_data_ << std::endl;
  AWARN << "using_imu_data " << using_imu_data_ << std::endl;
  AWARN << "pub_global_map " << pub_global_map_ << std::endl;
  AWARN << "using_debug_mode " << allow_mapping_mode_ << std::endl;
  AWARN << "icp_times " << icp_times_ << std::endl;

  double corner_filter_size, surf_filter_size, intensity_filter_size,
      corner_map_filter_size, surf_map_filter_size, intensity_map_filter_size;
  nh_private.param<double>("corner_map_filter", corner_map_filter_size, 0.3);
  nh_private.param<double>("surf_map_filter", surf_map_filter_size, 0.5);
  nh_private.param<double>("intensity_map_filter", intensity_map_filter_size,
                           0.3);
  nh_private.param<double>("corner_filter", corner_filter_size, 0.3);
  nh_private.param<double>("surf_filter", surf_filter_size, 0.5);
  nh_private.param<double>("intensity_filter", intensity_filter_size, 0.3);

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

  //读取保存地图参数
  double corner_size, surf_size, intensity_size, cloud_size;
  nh_private.param<double>("save_corner_size", corner_size, 0.3);
  nh_private.param<double>("save_surf_size", surf_size, 0.5);
  nh_private.param<double>("save_intensity_size", intensity_size, 0.3);
  nh_private.param<double>("save_intensity_size", cloud_size, 0.3);

  // gnss参数
  std::string gnss_config_path =
      "../sweeper_ws/src/sweeper_haide/calibration/data/"
      "livox_to_gps.yaml";

  lidar_to_gps_ = Eigen::Matrix4d::Identity();
  MathCalculation::GetCalibYamlParam(gnss_config_path, gnss_to_lidar_quater_,
                                     gnss_to_lidar_pos_);
  lidar_to_gps_.block(0, 0, 3, 3) = gnss_to_lidar_quater_.matrix();
  lidar_to_gps_.block(0, 3, 3, 1) = gnss_to_lidar_pos_;
  std::cout << "tran param: " << lidar_to_gps_ << std::endl;

  front_end_ = new FrontEnd(corner_map_filter_size, surf_map_filter_size,
                            intensity_map_filter_size, icp_times_, 50);
  save_and_view_map_ =
      new SaveAndViewMap(corner_size, surf_size, intensity_size);
  get_feature_ = new GetFeature(
      livox_hight, corner_min_curvature, surf_max_curvature, the_min_hight,
      intensity_min_different, min_slope_different, max_z_hight,
      corner_filter_size, surf_filter_size, intensity_filter_size,
      corner_filter_size);
  loop_closing_ = new LoopClosing(15, 0.25, 0.25, 0.25);
  InitMapppingParam();
  prev_lidar_odom_ = odom_to_map_ = Eigen::Matrix4d::Identity();
}

void MappingFlow::ReceiveSweeperInformation(
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
  if (allow_mapping_mode_)
    sweeper_information_deque_.push_back(now_sweeper_information);
}

void MappingFlow::ReceiveImuData(
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
  if (allow_mapping_mode_)
    imu_information_deque_.push_back(now_imu_information);
}

void MappingFlow::PredictLidarPose() {
  predict_mutex_.lock();
  while (sweeper_information_deque_.size() > 0) {
    // AWARN << "dis_time : " << sweeper_information_deque_[0].time -
    // lidar_time_
    //       << " " << imu_information_deque_.size();
    if (sweeper_information_deque_[0].time > lidar_time_) {
      break;
    }
    if (lidar_time_ - sweeper_information_deque_[0].time > 0.1) {
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
          double diff_imu_roll =
              imu_information_deque_[0].angular_velocity_x * 0.02;
          double diff_imu_pitch =
              imu_information_deque_[0].angular_velocity_y * 0.02;
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

void MappingFlow::GetMappingStateCode() {
  sweeper_msgs::SensorFaultInformation mapping_code;
  mapping_code.header.frame_id = "mapping";
  mapping_code.header.stamp = ros::Time::now();

  if (!watch_dog_lidar_.DogIsOk(3)) {
    mapping_code.state_code.push_back(UN_RECEIVE_LIVOX_DATA_);
  }
  if (!watch_dog_gnss_.DogIsOk(3)) {
    mapping_code.state_code.push_back(UN_RECEIVE_GNSS_DATA_);
  }

  if (!make_file_sucess_) mapping_code.state_code.push_back(MAKE_FILE_FAIL);
  if (!save_data_sucess_) mapping_code.state_code.push_back(SAVE_DATA_FAIL_);
  if (mapping_code.state_code.empty())
    mapping_code.state_code.push_back(MAPPING_FUCTION_NORMAL_);
  if (enter_mapping_mode_)
    mapping_code.state_code.push_back(ENTER_MAPPING_FUCTION_);
  pub_state_information_.publish(mapping_code);
}

void MappingFlow::ReceiveMappingInformation(
    sweeper_msgs::SweepMission sweeper_mode_command) {
  static int last_mode = sweeper_mode_command.mode;
  AWARN << "last mode: " << last_mode;
  if (sweeper_mode_command.mode == 3) {
    point_attribute_ = sweeper_mode_command.point_attribute;
    enter_mapping_mode_ = true;
    if (last_mapping_state_ != sweeper_mode_command.start_cmd) {
      //建图开始和关闭采用是否读取数据来进行，如果有数据则建图，没有数据则不建图
      switch (sweeper_mode_command.start_cmd) {
        // AWARN << "save name : " << sweeper_mode_command.line;
        case 0:  //取消： 1.所有参数初始化，不保存数据，在回调函数直接进行
          AWARN << "取消建图";
          allow_mapping_mode_ = false;
          ResetMappingParam();
          save_data_sucess_ = true;
          ResetMappingParam();

          break;
        case 1:
          AWARN << "start mapping";
          allow_mapping_mode_ = true;
          break;
        case 3:  //保存： 1.将数据报存到vector并将当前vector保存，并初始化所有参数
          if (sweeper_mode_command.line.size() > 0) {
            AWARN << "save mapping";
            back_end_.BackEndOptimize(true);
            back_end_.OutputOptimizedPose(key_frames_);
            // save_and_view_map_->GetPcdAndLineName(sweeper_mode_command.line);
            //保存地图
            save_and_view_map_->PublishAndSaveMap(key_frames_);
            save_data_sucess_ = save_and_view_map_->RenameAndSaveFile(
                sweeper_mode_command.line);
            save_and_view_map_->SaveLineToFile(key_frames_);
            ResetMappingParam();
            allow_mapping_mode_ = false;
          }
          break;
        default:
          break;
      }
    }
    last_mapping_state_ = sweeper_mode_command.start_cmd;
    last_mode = sweeper_mode_command.mode;
  } else if (last_mode != sweeper_mode_command.mode) {
    enter_mapping_mode_ = false;
    ResetMappingParam();
    last_mode = sweeper_mode_command.mode;
    last_mapping_state_ = 0;
  }
}

void MappingFlow::ResetMappingParam() {
  allow_mapping_mode_ = false;
  now_pos_ = predict_pos_ = Eigen::Vector3d(0, 0, 0);
  now_quater_ = predict_quater_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  //前端数据清空
  front_end_->ResetFrontEndParam();
  //后端数据清空
  // back_end_.ResetBackEndParam();
  //回环数据清空
  loop_closing_->ResetLoopParam();
  // flow清空
  make_file_sucess_ = save_and_view_map_->ResetParam();

  // save_data_sucess_ = true;
  flag_gps_to_map_ = false;
  InitMapppingParam();
  last_gps_pos_ = Eigen::Vector3d(-10, -10, -10);
}

void MappingFlow::FrontNode() {
  ros::Rate rate(10);
  while (ros::ok()) {
    Run();
    rate.sleep();
  }
}

void MappingFlow::CloudAdjustMotion(
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

void MappingFlow::Run() {
  std::deque<CloudData> cloud_data_buff;
  cloud_sub_ptr_->ParseData(cloud_data_buff);
  gnss_sub_ptr_->ParseData(unsynced_gnss_);

  if (cloud_data_buff.size() > 0) watch_dog_lidar_.UpdataNow();
  if (unsynced_gnss_.size() > 0) watch_dog_gnss_.UpdataNow();

  if (!allow_mapping_mode_) {
    unsynced_gnss_.clear();
    return;
  }

  while (cloud_data_buff.size() > 0) {
    AWARN << "mapping mode: " << allow_mapping_mode_ << " "
          << cloud_data_buff.size();
    if (!allow_mapping_mode_) {
      cloud_data_buff.clear();
      break;
    }
    //数据预处理，时间同步与提取特征点
    CloudData cloud_data = cloud_data_buff.front();
    cloud_data_buff.pop_front();
    lidar_time_ = cloud_data.time;
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

    //点云矫正
    CloudAdjustMotion(tran_pose, cloud_data.cloud_ptr);

    double start_time = clock();
    Feature now_feature;
    get_feature_->GetLivoxFeature(cloud_data, now_feature);
    double feature_time = clock();
    AWARN << "feature time: "
          << (double)(feature_time - start_time) / CLOCKS_PER_SEC;
    corner_feature_pub_ptr_->Publish(now_feature.corner_cloud);
    surf_feature_pub_ptr_->Publish(now_feature.surf_cloud);
    intensity_feature_pub_ptr_->Publish(now_feature.intensity_cloud);

    GNSSData synced_gnss;
    bool valid_gnss =
        GNSSData::SyncData(unsynced_gnss_, synced_gnss, lidar_time_);
    Eigen::Matrix4d key_frame_pose = Eigen::Matrix4d::Identity();
    // PredictLidarPose();
    front_end_->PredictLidarPose(now_quater_, now_pos_);
    double before_front = clock();
    front_end_->Process(now_feature.corner_cloud, now_feature.surf_cloud,
                        now_feature.intensity_cloud);
    AWARN << "front time: "
          << (double)(clock() - before_front) / CLOCKS_PER_SEC;
    now_quater_ = front_end_->now_quater_;
    now_pos_ = front_end_->now_pos_;

    if (!front_end_->OutputNewKeyFrame(key_frame_pose)) {
      AWARN << "can't become a key frame";
      continue;
    }

    double odom_cov =
        front_end_->lidar_match_cov_ * front_end_->lidar_match_cov_;

    //加入gps与地图的转换
    if (!flag_gps_to_map_ && valid_gnss && synced_gnss.gnss_cov(5, 5) < 0.5) {
      std::string yaml_path = "../sweeper_ws/src/sweeper_haide/data/path/uname";

      fixed_pose_ = Eigen::Matrix4d::Zero();
      fixed_pose_.block(0, 3, 3, 1) = synced_gnss.gnss_position;
      Eigen::Matrix4d gps_point =
          synced_gnss.OrientationToMatrix() - fixed_pose_;
      Eigen::Matrix4d gps_pose = gps_point * lidar_to_gps_;

      gps_to_map_ = key_frame_pose * gps_pose.inverse();
      // AWARN << "gps_pose : " << gps_pose;
      // AWARN << "gps_pose inverse: " << gps_pose.inverse();
      // AWARN << "key_frame_pose " << key_frame_pose.block(0, 0, 4, 4);
      // std::cout << std::setprecision(15) << "gps_to_map_ "
      //           << gps_to_map_.block(0, 0, 4, 4) << std::endl;
      // AWARN << "lidar_to_gps " << lidar_to_gps_.block(0, 0, 4, 4);
      // AWARN << "lidar_to_gps "
      //       << synced_gnss.OrientationToMatrix().block(0, 0, 4, 4);

      Eigen::Matrix3d gps_to_lidar_rotation = gps_to_map_.block(0, 0, 3, 3);
      Eigen::Quaterniond gps_to_lidar_quater(gps_to_lidar_rotation);
      Eigen::Vector3d gps_to_lidar_pos = gps_to_map_.block(0, 3, 3, 1);
      MathCalculation::SaveGpsToMapYaml(yaml_path, gps_to_lidar_pos,
                                        gps_to_lidar_quater,
                                        synced_gnss.gnss_position);
      gnss_pub_ptr_->Publish4d(key_frame_pose, ros::Time::now());
      flag_gps_to_map_ = true;
    }
    AWARN << "matcer time: "
          << (double)(clock() - feature_time) / CLOCKS_PER_SEC;

    AddCloudToFile(now_feature, key_frame_index_);

    BackData back_data;
    back_data.InputOdomData(key_frame_pose, odom_cov, lidar_time_,
                            key_frame_index_);
    //将gps转到地图
    if (flag_gps_to_map_) {
      Eigen::Matrix4d gnss_pose =
          gps_to_map_ * (synced_gnss.OrientationToMatrix() - fixed_pose_) * lidar_to_gps_;
      if (valid_gnss) {
        back_data.InputGpsData(
            Eigen::Vector3d(gnss_pose(0, 3), gnss_pose(1, 3), gnss_pose(2, 3)),
            Eigen::Vector3d(synced_gnss.gnss_cov(0, 0),
                            synced_gnss.gnss_cov(1, 1),
                            synced_gnss.gnss_cov(2, 2)));
      }
      gnss_pub_ptr_->Publish4d(gnss_pose, ros::Time::now());
    }

    KeyFrame key_frame;
    key_frame.InputKeyFrame(key_frame_pose.cast<float>(),
                            synced_gnss.gnss_position, key_frame_index_,
                            point_attribute_, lidar_time_);
    key_frames_.push_back(key_frame);

    back_datas_.push_back(back_data);
    key_frame_index_++;

    //发布数据
    vehical_odom_pub_ptr_->PublishQuat(predict_quater_, predict_pos_,
                                       ros::Time::now());
    corner_bag_pub_ptr_->Publish(front_end_->corner_map_bag_);

    // double lidar_odom_cov = front_end_->lidar_match_cov_;
    Eigen::Matrix<double, 6, 6> odom_information =
        Eigen::Matrix<double, 6, 6>::Identity();
    odom_information(0, 0) = odom_cov;
    odom_information(1, 1) = odom_cov;
    odom_information(2, 2) = odom_cov;
    odom_pub_ptr_->SetInformation(odom_information);
    odom_pub_ptr_->Publish4d(key_frame_pose, ros::Time().fromSec(lidar_time_));
  }
}

void MappingFlow::BackEndFlow() {
  ros::Rate rate(5);
  while (ros::ok()) {
    // static int time = 0;
    if (!allow_mapping_mode_) {
      key_frame_index_ = 0;
      back_datas_.clear();
      key_frames_.clear();
      gps_load_points_.clear();
      loop_points_.clear();
      loop_trans_.clear();
      back_end_.ResetBackEndParam();
      prev_lidar_odom_ = odom_to_map_ = Eigen::Matrix4d::Identity();
    }
    //将数据转换
    std::vector<BackData> back_datas;
    // back_datas.insert(back_datas.end(), back_datas_.begin(),
    // back_datas_.end());
    back_mutex_.lock();
    back_datas.swap(back_datas_);
    back_mutex_.unlock();
    if (back_datas.size() > 0) {
      AERROR << "size: " << back_datas_.size() << "  " << back_datas.size();
      int time = 0;
      for (size_t i = 0; i < back_datas.size(); i++) {
        BackData back_data = back_datas[i];
        // back_datas.pop_front();

        // 1.将lidar_odom数据加入
        Eigen::Matrix4d now_lidar_odom = odom_to_map_ * back_data.odom_data;
        back_end_.AddVertexToG2O(now_lidar_odom);

        map_odom_pub_ptr_->Publish4d(now_lidar_odom, ros::Time::now());

        AWARN << "index: " << back_data.index;
        //将odom变化量数据加入到后端
        back_end_.AddLidarOdomToG2O(prev_lidar_odom_, back_data.odom_data,
                                    back_data.odom_cov);
        prev_lidar_odom_ = back_data.odom_data;
        key_frames_[back_data.index].pose = now_lidar_odom.cast<float>();

        // 2.判断回环检测
        back_data.odom_data = now_lidar_odom;
        if (time % 5 == 0)
          loop_closing_->DetectionLoopClosure(
              key_frames_, back_data, loop_points_, loop_trans_, back_end_);
        time++;

        // 3.gps数据
        if (back_data.flag_get_gps_data) {
          //判断距离与协方差
          if (CalDistance(back_data.gps_data, last_gps_pos_) > 5.0) {
            last_gps_pos_ = back_data.gps_data;
            back_end_.AddGNSSToG2O(back_data.gps_data, back_data.index,
                                   back_data.gps_cov);
            gps_load_points_.push_back(back_data.index);
          }
        }
      }
      Eigen::Matrix4d prev_map_pose = Eigen::Matrix4d::Identity();
      //优化并得到最后一个点的姿态
      if (back_end_.BackEndOptimize(false)) {
        prev_map_pose = back_end_.OutputOptimizedPose(key_frames_);
        // if (time % 5 == 0)
        PublishBackPath();
        VisualizeLoopClosure();
      }
      //得到odom的转换
      Eigen::Matrix4d last_point_pose =
          back_datas[back_datas.size() - 1].odom_data;
      odom_to_map_ = prev_map_pose * last_point_pose.inverse();
      back_datas.clear();
      // time++;
    }
    rate.sleep();
  }
}

void MappingFlow::VisualizeLoopClosure() {
  visualization_msgs::MarkerArray marker_array;
  // loop nodes
  visualization_msgs::Marker marker_node;
  marker_node.header.frame_id = "/camera_init";
  marker_node.header.stamp = ros::Time::now();
  marker_node.action = visualization_msgs::Marker::ADD;
  marker_node.type = visualization_msgs::Marker::SPHERE_LIST;
  marker_node.ns = "loop_nodes";
  marker_node.id = 0;
  marker_node.pose.orientation.w = 1;
  marker_node.scale.x = 0.3;
  marker_node.scale.y = 0.3;
  marker_node.scale.z = 0.3;
  marker_node.color.r = 0;
  marker_node.color.g = 0.8;
  marker_node.color.b = 1;
  marker_node.color.a = 1;

  // gps_nodes
  visualization_msgs::Marker marker_gps_node;
  marker_gps_node.header.frame_id = "/camera_init";
  marker_gps_node.header.stamp = ros::Time::now();
  marker_gps_node.action = visualization_msgs::Marker::ADD;
  marker_gps_node.type = visualization_msgs::Marker::SPHERE_LIST;
  marker_gps_node.ns = "loop_nodes";
  marker_gps_node.id = 0;
  marker_gps_node.pose.orientation.w = 1;
  marker_gps_node.scale.x = 0.3;
  marker_gps_node.scale.y = 0.3;
  marker_gps_node.scale.z = 0.3;
  marker_gps_node.color.r = 0.3;
  marker_gps_node.color.g = 0.9;
  marker_gps_node.color.b = 0.3;
  marker_gps_node.color.a = 1;
  // loop edges
  visualization_msgs::Marker marker_edge;
  marker_edge.header.frame_id = "/camera_init";
  marker_edge.header.stamp = ros::Time::now();
  marker_edge.action = visualization_msgs::Marker::ADD;
  marker_edge.type = visualization_msgs::Marker::LINE_LIST;
  marker_edge.ns = "loop_edges";
  marker_edge.id = 1;
  marker_edge.pose.orientation.w = 1;
  marker_edge.scale.x = 0.1;
  marker_edge.scale.y = 0.1;
  marker_edge.scale.z = 0.1;
  marker_edge.color.r = 0.9;
  marker_edge.color.g = 0.9;
  marker_edge.color.b = 0;
  marker_edge.color.a = 1;

  for (size_t i = 0; i < loop_points_.size();) {
    int key_cur = loop_points_[i];
    int key_pre = loop_points_[i + 1];
    geometry_msgs::Point p;
    p.x = key_frames_[key_cur].pose(0, 3);
    p.y = key_frames_[key_cur].pose(1, 3);
    p.z = key_frames_[key_cur].pose(2, 3);
    marker_node.points.push_back(p);
    marker_edge.points.push_back(p);
    p.x = key_frames_[key_pre].pose(0, 3);
    p.y = key_frames_[key_pre].pose(1, 3);
    p.z = key_frames_[key_pre].pose(2, 3);
    marker_node.points.push_back(p);
    marker_edge.points.push_back(p);

    // geometry_msgs::Point p;
    p.x = key_frames_[key_cur].pose(0, 3);
    p.y = key_frames_[key_cur].pose(1, 3);
    p.z = key_frames_[key_cur].pose(2, 3);
    marker_node.points.push_back(p);
    marker_edge.points.push_back(p);
    Eigen::Matrix4d loop_pose =
        key_frames_[key_cur].pose.cast<double>() * loop_trans_[i / 2];
    p.x = loop_pose(0, 3);
    p.y = loop_pose(1, 3);
    p.z = loop_pose(2, 3);
    marker_node.points.push_back(p);
    marker_edge.points.push_back(p);
    i += 2;
  }
  for (size_t i = 0; i < gps_load_points_.size(); i++) {
    int key_cur = gps_load_points_[i];
    geometry_msgs::Point p;
    p.x = key_frames_[key_cur].pose(0, 3);
    p.y = key_frames_[key_cur].pose(1, 3);
    p.z = key_frames_[key_cur].pose(2, 3);
    // marker_gps_node.points.push_back(p);
    marker_node.points.push_back(p);
  }

  marker_array.markers.push_back(marker_gps_node);
  marker_array.markers.push_back(marker_node);
  marker_array.markers.push_back(marker_edge);
  pub_loop_constraints_.publish(marker_array);
}

void MappingFlow::PublishBackPath() {
  nav_msgs::Path map_path;
  map_path.header.stamp = ros::Time::now();
  map_path.header.frame_id = "/camera_init";
  for (size_t i = 0; i < key_frames_.size(); i++) {
    if (!key_frames_[i].optimized) continue;
    geometry_msgs::PoseStamped g2o_optimize_pose;

    g2o_optimize_pose.pose.position.x = key_frames_[i].pose(0, 3);
    g2o_optimize_pose.pose.position.y = key_frames_[i].pose(1, 3);
    g2o_optimize_pose.pose.position.z = key_frames_[i].pose(2, 3);
    map_path.poses.push_back(g2o_optimize_pose);
  }
  pub_optimize_path_.publish(map_path);
}

bool MappingFlow::InitMapppingParam() {
  std::string data_path = "../sweeper_ws/src/sweeper_haide/data/map";
  key_frames_path_ = data_path + "/slam_data/key_frames";
  if (!FileManager::CreateDirectory(data_path + "/slam_data", "文件夹"))
    return false;
  if (!FileManager::InitDirectory(key_frames_path_, "关键帧点云"))
    return false;
  else
    return true;
}

void MappingFlow::AddCloudToFile(const Feature &now_feature,
                                 const int &key_frame_index) {
  //存储到硬盘中
  std::string file_path =
      key_frames_path_ + "/" + std::to_string(key_frame_index);
  std::string corner_path = "_corner.pcd";
  std::string surf_path = "_surf.pcd";
  std::string intensity_path = "_intensity.pcd";
  std::string cloud_path = "_cloud.pcd";
  std::string cloud_map_path = "_cloud_map.pcd";
  AWARN << "start corner cloud!"
        << " size: " << now_feature.corner_cloud->points.size();
  if (now_feature.corner_cloud->points.size() > 5)
    pcl::io::savePCDFileBinary(file_path + corner_path,
                               *now_feature.corner_cloud);
  AWARN << "save surf cloud ok!"
        << " size: " << now_feature.surf_cloud->points.size();
  pcl::io::savePCDFileBinary(file_path + surf_path, *now_feature.surf_cloud);
  AWARN << "save intensity cloud ok!"
        << " size: " << now_feature.intensity_cloud->points.size();
  if (now_feature.intensity_cloud->points.size() > 5)
    pcl::io::savePCDFileBinary(file_path + intensity_path,
                               *now_feature.intensity_cloud);
  AWARN << "save  cloud ok!"
        << " size: " << now_feature.cloud_raw->points.size();
  if (now_feature.cloud_raw->points.size() > 5)
    pcl::io::savePCDFileBinary(file_path + cloud_path, *now_feature.cloud_raw);
  AWARN << "save cloud ok!";
}

double MappingFlow::CalDistance(const Eigen::Vector3d &pos1,
                                const Eigen::Vector3d &pos2) {
  return sqrt((pos1.x() - pos2.x()) * (pos1.x() - pos2.x()) +
              (pos1.y() - pos2.y()) * (pos1.y() - pos2.y()) +
              (pos1.z() - pos2.z()) * (pos1.z() - pos2.z()));
}

void MappingFlow::ViewGlobalMap() {
  ros::Rate rate(0.2);
  while (ros::ok()) {
    if (pub_global_map_ && allow_mapping_mode_) {
      CloudData::CLOUD_PTR corner_cloud(new CloudData::CLOUD());
      CloudData::CLOUD_PTR global_map_corner(new CloudData::CLOUD());
      CloudData::CLOUD tran_cloud;
      view_mutex_.lock();
      std::vector<KeyFrame> key_frames(key_frames_);
      view_mutex_.unlock();
      for (size_t i = 0; i < key_frames.size(); i++) {
        if (!key_frames[i].optimized) continue;
        if (!pcl::io::loadPCDFile("../sweeper_ws/src/sweeper_haide/data/map/"
                                  "slam_data/key_frames/" +
                                      std::to_string(i) + "_cloud.pcd",
                                  *corner_cloud)) {
          // pcl::transformPointCloud(*corner_cloud, tran_cloud,
          //                          key_frames[i].pose);
          CloudData::CLOUD full_cloud;
          for (size_t j = 0; j < corner_cloud->points.size(); j++) {
            CloudData::POINT corner_point = corner_cloud->points[j];
            if (fabs(corner_point.x) > 30 || fabs(corner_point.y) > 30)
              continue;
            corner_point.intensity = (corner_point.z + 2.0) * 80.0;
            full_cloud.points.push_back(corner_point);
            // global_map_corner->points.push_back(corner_point);
          }
          pcl::transformPointCloud(full_cloud, tran_cloud, key_frames[i].pose);
          *global_map_corner += tran_cloud;
        }
      }
      CloudData::POINT min, max;
      pcl::getMinMax3D(*global_map_corner, min, max);
      int hight = (max.x - min.x) / 0.1;  // 10cm分辨率
      int width = (max.y - min.y) / 0.1;  // 10cm分辨率
      if (hight > 10 && width > 10) {
        jpg_map_pub_ =
            cv::Mat(hight + 1, width + 1, CV_8UC3, cv::Scalar(0, 0, 0));
        //写入地图
        for (size_t i = 0; i < global_map_corner->points.size(); i++) {
          CloudData::POINT point_now = global_map_corner->points[i];
          int m = (-point_now.x + max.x) / 0.1;
          int n = (-point_now.y + max.y) / 0.1;
          int gray = point_now.intensity;
          if (gray > 255)
            gray = 255;
          else if (gray < 0)
            gray = 0;
          if (m < hight + 1 && n < width + 1) {
            if (jpg_map_pub_.at<cv::Vec3b>(m, n)[0] > gray) continue;
            jpg_map_pub_.at<cv::Vec3b>(m, n)[0] = gray;
            jpg_map_pub_.at<cv::Vec3b>(m, n)[1] = gray;
            jpg_map_pub_.at<cv::Vec3b>(m, n)[2] = gray;
          } else {
            AWARN << "image is out "
                  << " raw row, clos: " << hight << " " << width
                  << " now row, clos: " << m << " " << n;
          }
        }
        //对点进行绘制
        int now_pose_m = 0, now_pose_n = 0;
        for (size_t i = 0; i < key_frames.size(); i++) {
          if (!key_frames[i].optimized) continue;
          double point_x = key_frames[i].pose(0, 3);
          double point_y = key_frames[i].pose(1, 3);
          int m = (-point_x + max.x) / 0.1;
          int n = (-point_y + max.y) / 0.1;
          now_pose_m = m;
          now_pose_n = n;
          if (m < hight + 1 && n < width + 1) {
            cv::circle(jpg_map_pub_, cv::Point(n, m), 2, cv::Scalar(0, 255, 0),
                       -1);  // 画半径为1的圆(画点）
          } else {
            AWARN << "image is out "
                  << " raw row, clos: " << hight << " " << width
                  << " now row, clos: " << m << " " << n;
          }
        }
        cv::circle(jpg_map_pub_, cv::Point(now_pose_n, now_pose_m), 13,
                   cv::Scalar(0, 255, 0), -1);

        for (size_t i = 0; i < loop_points_.size();) {
          int key_cur = loop_points_[i];
          int key_pre = loop_points_[i + 1];
          int frist_y = (-key_frames[key_cur].pose(0, 3) + max.x) / 0.1;
          int frist_x = (-key_frames[key_cur].pose(1, 3) + max.y) / 0.1;
          int second_y = (-key_frames[key_pre].pose(0, 3) + max.x) / 0.1;
          int second_x = (-key_frames[key_pre].pose(1, 3) + max.y) / 0.1;
          if (frist_y < hight + 1 && frist_x < width + 1 &&
              second_y < hight + 1 && second_x < width + 1) {
            cv::line(jpg_map_pub_, cv::Point(frist_x, frist_y),
                     cv::Point(second_x, second_y), cv::Scalar(0, 255, 255),
                     12);
          }
          i += 2;
        }
        for (size_t i = 0; i < gps_load_points_.size(); i++) {
          int key_cur = gps_load_points_[i];
          int frist_y = (-key_frames[key_cur].pose(0, 3) + max.x) / 0.1;
          int frist_x = (-key_frames[key_cur].pose(1, 3) + max.y) / 0.1;
          if (frist_y < hight + 1 && frist_x < width + 1) {
            cv::circle(jpg_map_pub_, cv::Point(frist_x, frist_y), 8,
                       cv::Scalar(0, 0, 255),
                       -1);  // 画半径为1的圆(画点）
          }
        }

        // cv::circle(jpg_map_pub_, cv::Point(frist_x, frist_y), 8,
        //            cv::Scalar(0, 0, 255), -1);

        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", jpg_map_pub_)
                .toImageMsg();
        msg->header.frame_id = "mapping";
        pub_map_image_.publish(msg);
      }
    }
    rate.sleep();
  }
}

void MappingFlow::PublishMappingState() {
  ros::Rate rate(10);
  while (ros::ok()) {
    GetMappingStateCode();
    rate.sleep();
  }
}

}  // namespace mapping

#include "../include/sweeper_odom/get_lidar_odom.h"

GetLidarOdom::GetLidarOdom(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : lidar_odom_error_times_(0),
      receive_new_fusion_odom_(false),
      showed_map_(false),
      allow_open_lidar_odom_(false),
      init_lidar_odom_(false),
      load_map_success_(true),
      receive_frist_pose_(true),
      cruise_trace_mode_(true),
      receive_pcd_name_sucessed_(false),
      frist_matcher_(true) {
  // int max_iteration = 5;
  // plicp_.m_para_icp_max_iterations = max_iteration;
  nh_private.param<int>("receive_sweeper_mode", receive_sweeper_mode_, 0);
  nh_private.param<int>("max_iteration", icp_max_times_, 5);
  nh_private.param<bool>("allow_open_lidar_odom", allow_open_lidar_odom_,
                         false);

  std::cout << "receive_sweeper_mode : " << receive_sweeper_mode_ << std::endl;
  std::cout << "max_iteration : " << icp_max_times_ << std::endl;
  std::cout << "allow_open_lidar_odom : " << allow_open_lidar_odom_
            << std::endl;

  sub_livox_lidar_ = nh.subscribe<sensor_msgs::PointCloud2>(
      "/livox/lidar", 1, &GetLidarOdom::MatcherCloud, this);
  sub_frist_odom_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1, &GetLidarOdom::FristOdomHander, this);
  sub_fusion_odom_ = nh.subscribe<nav_msgs::Odometry>(
      "/sweeper/localization/fusion_position", 1,
      &GetLidarOdom::ReceiveFusionOdom, this);
  sub_sweeper_mode_ = nh.subscribe<sweeper_msgs::SweepMission>(
      "/sweeper/sweep_mode", 1, &GetLidarOdom::ReceiveSweeperMode, this);
  sub_gps_position_ = nh.subscribe<nav_msgs::Odometry>(
      "/sweeper/sensor/gnss", 1, &GetLidarOdom::ReceiveGpsPosition, this);

  lidar_odom_pub_ =
      nh.advertise<nav_msgs::Odometry>("/sweeper/navigation/lidarodom", 1);
  pub_frist_odom_ =
      nh.advertise<nav_msgs::Odometry>("/sweeper/navigation/frist_odom", 1);
  cloud_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/sweeper/navigation/map", 1, true);
  pub_state_information_ = nh.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/localization/diagnose", 1);

  timer_lidar_odom_ = nh.createTimer(
      ros::Duration(0.1), &GetLidarOdom::GetSweeperInformation, this);

  down_size_filter_corner_.setLeafSize(0.4, 0.4, 0.4);
  down_size_filter_surf_.setLeafSize(0.4, 0.4, 0.4);
  down_size_filter_intensity_.setLeafSize(1.0, 1.0, 1.0);
  ResetLidarOdomParam();
  last_pcd_name_ = "unname";
}

//初始化匹配参数,包括初始位置，初始匹配参数，初始地图载入
void GetLidarOdom::InitLidarOdom() {
  if (get_pcd_name_.size() > 0) {
    if (!init_lidar_odom_) {
      ParamInit();
      plicp_.ReceivePosition(init_quater_, init_pose_);
      init_lidar_odom_ = true;
      last_pcd_name_ = get_pcd_name_;

      AINFO << "load pcd name : " << get_pcd_name_
            << "  pose: " << init_pose_.x() << " " << init_pose_.y() << " "
            << init_pose_.z() << " "
            << " quater: " << init_quater_.x() << " " << init_quater_.y() << " "
            << init_quater_.z() << " " << init_quater_.w();
    }
    receive_pcd_name_sucessed_ = true;
  } else {
    receive_pcd_name_sucessed_ = false;
  }
}

void GetLidarOdom::GetSweeperInformation(const ros::TimerEvent &e) {
  sweeper_msgs::SensorFaultInformation fusion_navigation_code;
  fusion_navigation_code.header.frame_id = "localization";
  fusion_navigation_code.header.stamp = ros::Time::now();
  if (!watch_dog_livox_.DogIsOk(3)) {  //没有大疆数据
    fusion_navigation_code.state_code.push_back(un_received_livox_data_);
  }
  if (!load_map_success_)  //没有载入地图
    fusion_navigation_code.state_code.push_back(un_load_map_);
  if (!receive_frist_pose_ && receive_pcd_name_sucessed_)  //没有接收到初始位置
    fusion_navigation_code.state_code.push_back(un_received_frist_pose_);
  //初始位置匹配失败
  if (frist_matcher_score_ > 5.0) {
    ROS_ERROR("frist matcher error");
    AERROR << "frist matcher error";
    fusion_navigation_code.state_code.push_back(frist_matcher_error_);
  }
  if (lidar_odom_error_times_ > 50)  // 5s匹配错误
  {
    fusion_navigation_code.state_code.push_back(long_time_matcher_error_);
  }

  //   if (fusion_navigation_code.state_code.empty())
  //     fusion_navigation_code.state_code.push_back(cruise_trace_mode_sucess_);
  //   if (cruise_trace_mode_ && receive_pcd_name_sucessed_)
  //     fusion_navigation_code.state_code.push_back(cin_cruise_trace_mode_);
  if (!fusion_navigation_code.state_code.empty())
    pub_state_information_.publish(fusion_navigation_code);
}

//重置参数
void GetLidarOdom::ResetLidarOdomParam() {
  //匹配参数清0
  init_lidar_odom_ = false;
  // get_pcd_name_.clear();
  // plicp参数清除
  plicp_.ResetAllParam();
  //地图清除
  plicp_.m_laser_cloud_intensity_from_map->clear();
  plicp_.m_laser_cloud_corner_from_map->clear();
  plicp_.m_laser_cloud_surf_from_map->clear();
  frist_matcher_ = true;
  frist_matcher_score_ = 0.0;
}

void GetLidarOdom::ReceiveSweeperMode(
    sweeper_msgs::SweepMission sweeper_mode_command) {
  //得到模式，地图，初始位置
  //激光巡迹模式
  // ROS_ERROR("cin mode");
  if (sweeper_mode_command.mode == 5) {
    cruise_trace_mode_ = true;
    switch (sweeper_mode_command.start_cmd) {
      case 1:  //开始
        get_pcd_name_ = sweeper_mode_command.map;
        if (get_pcd_name_.compare(last_pcd_name_) || !init_lidar_odom_) {
          AINFO << "receive mode: " << sweeper_mode_command.mode << " "
                << sweeper_mode_command.start_cmd << " "
                << sweeper_mode_command.map << " "
                << sweeper_mode_command.init_pose.position.x << " "
                << sweeper_mode_command.init_pose.position.y << " "
                << sweeper_mode_command.init_pose.position.z;
          AINFO << "quater " << sweeper_mode_command.init_pose.orientation.x
                << " " << sweeper_mode_command.init_pose.orientation.y << " "
                << sweeper_mode_command.init_pose.orientation.z << " "
                << sweeper_mode_command.init_pose.orientation.w;
          ResetLidarOdomParam();  //先初始化所有参数在载入参数
          get_pcd_name_ = sweeper_mode_command.map;
          init_quater_ =
              Eigen::Quaterniond(sweeper_mode_command.init_pose.orientation.w,
                                 sweeper_mode_command.init_pose.orientation.x,
                                 sweeper_mode_command.init_pose.orientation.y,
                                 sweeper_mode_command.init_pose.orientation.z);
          init_pose_ =
              Eigen::Vector3d(sweeper_mode_command.init_pose.position.x,
                              sweeper_mode_command.init_pose.position.y,
                              sweeper_mode_command.init_pose.position.z);
          if (fabs(init_pose_.x()) < 0.001 && fabs(init_pose_.y()) < 0.001 &&
              fabs(init_pose_.z()) < 0.001) {
            receive_frist_pose_ = false;
          } else {
            receive_frist_pose_ = true;
          }
          InitLidarOdom();
        }
        break;
      case 2:  //取消
        //重置所有参数
        ResetLidarOdomParam();
        last_pcd_name_ = "unname";
        break;
      default:
        break;
    }
  } else {
    ResetLidarOdomParam();
    last_pcd_name_ = "unname";
    cruise_trace_mode_ = false;
  }
}

void GetLidarOdom::ReceiveGpsPosition(const nav_msgs::Odometry input_odom) {
  std::cout << "receive gps data" << std::endl;
  //   double point_x = input_odom.pose.pose.position.x;
  //   double point_y = input_odom.pose.pose.position.y;
  //   double min_distance = 10;
  //   int pcd_position;
  //   for (size_t i = 0; i < maps_name_.size(); i++) {
  //     double distance =
  //         sqrt((point_x - gps_points_x_[i]) * (point_x - gps_points_x_[i]) +
  //              (point_y - gps_points_y_[i]) * (point_y - gps_points_y_[i]));
  //     if (distance < min_distance) {
  //       min_distance = distance;
  //       pcd_position = i;
  //     }
  //   }
  //   //载入地图1
  //   if (min_distance < 5 && receive_sweeper_mode_ == 4 &&
  //       !plicp_.recevied_cloud_)  // lidar寻线与混合模式14
  //   {
  //     pcd_name_ = maps_name_[pcd_position];
  //     sprintf(read_pcd_name_, pcd_name_.c_str());
  //     // read_pcd_name_ = pcd_name_.c_str();
  //     // ReadPcdName();
  //     // ParamInit();
  //     allow_open_lidar_odom_ = true;
  //     std::cout << "load cloud" << std::endl;
  //   }
}

void GetLidarOdom::TransformationCoordinate(
    pcl::PointCloud<pcl::PointXYZI> &raw_cloud) {
  pcl::PointCloud<pcl::PointXYZI> points_cloud_out;
  points_cloud_out.clear();
  for (size_t i = 0; i < raw_cloud.size(); i++) {
    pcl::PointXYZI point = raw_cloud.points[i];
    point.z += 2.13;
    points_cloud_out.push_back(point);
  }
  raw_cloud = points_cloud_out;
}

// bool GetLidarOdom::AdjustLidarOdom(double odom_time)
// {
//     if (fabs(odom_time - gps_data.time_gps) > 0.1)
//         return false;
//     if (fabs(gps_data.position_gps.x() - plicp_.m_t_w_curr.x()) > 2.0 ||
//         fabs(gps_data.position_gps.y() - plicp_.m_t_w_curr.y()) > 2.0 ||
//         fabs(gps_data.position_gps.z() - plicp_.m_t_w_curr.z()) > 2.0)
//         return true;
//     else
//         return false;
// }

// bool GetLidarOdom::InitializeLidarPosition()
// {
//     if (frist_odom_pose_.received_odom_pose)
//         return true;
//     if (gps_data.GpsDataIsOk())
//     {
//         Eigen::Vector3d position;
//         Eigen::Quaterniond quater;
//         quater = gps_data.quaternion_gps;
//         position.x() = gps_data.position_gps.x();
//         position.y() = gps_data.position_gps.y();
//         double min_distance = 200.0;
//         for (size_t i = 0; i < odom_position_path.size(); i++)
//         {
//             double distance = fabs(sqrt(odom_position_path[i].x() *
//             odom_position_path[i].x() + odom_position_path[i].y() *
//             odom_position_path[i].y()) - sqrt(gps_data.position_gps.x() *
//             gps_data.position_gps.x() + gps_data.position_gps.y() *
//             gps_data.position_gps.y())); if (distance < min_distance)
//             {
//                 position.z() = gps_data.position_gps.z();
//                 min_distance = distance;
//             }
//         }

//         plicp_.ReceivePosition(quater, position);
//         frist_odom_pose_.received_odom_pose = true;
//         std::cout << "Initialize Succeed ! GPS data is right, using gps data
//         to Initialize" << std::endl; return true;
//     }
//     else
//         std::cout << "Initialize Faile ! GPS data is error, please receive
//         odom data to Initialize in map" << std::endl;
//     if (frist_odom_pose_.receive_odom_pose)
//     {

//         Eigen::Vector3d position;
//         Eigen::Quaterniond quater;
//         quater = frist_odom_pose_.frist_pose_quater;
//         position.x() = gps_data.position_gps.x();
//         position.y() = gps_data.position_gps.y();
//         double min_distance = 200.0;
//         for (size_t i = 0; i < odom_position_path.size(); i++)
//         {
//             double distance = fabs(sqrt(odom_position_path[i].x() *
//             odom_position_path[i].x() + odom_position_path[i].y() *
//             odom_position_path[i].y()) -
//             sqrt(frist_odom_pose_.frist_pose_position.x() *
//             frist_odom_pose_.frist_pose_position.x() +
//             frist_odom_pose_.frist_pose_position.y() *
//             frist_odom_pose_.frist_pose_position.y())); if (distance <
//             min_distance)
//             {
//                 position.z() = gps_data.position_gps.z();
//                 min_distance = distance;
//             }
//         }

//         plicp_.ReceivePosition(quater, position);
//         frist_odom_pose_.received_odom_pose = true;
//         std::cout << "Initialize Succeed ! using gps odom data to Initialize
//         in map" << std::endl; return true;
//     }
//     else
//         std::cout << "Initialize Faile !  Please receive odom data to
//         Initialize in map" << std::endl;
//     return false;
// }

void GetLidarOdom::MatcherCloud(
    const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
  watch_dog_livox_.UpdataNow();
  if (!init_lidar_odom_ || !plicp_.recevied_cloud_) {
    if (!init_lidar_odom_)
      // ROS_ERROR("unreceive init");
      AERROR << "unreceive init";
    else
      // ROS_ERROR("unload pcd");
      AERROR << "unload pcd";
    // ROS_ERROR("init error");
    AERROR << "init error";
    return;
  }
  // ROS_ERROR("cin matcher cloud");
  float cloudCurvature[40000] = {0};
  float cloudintensity[40000] = {0};
  float cloudslope[40000] = {0};
  int cloudNeighborPicked[40000] = {0};
  double cloud_distance[40000];
  pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
  vector<vector<int>> scans;
  vector<int> scan;
  std::vector<int> indices;

  TransformationCoordinate(laserCloudIn);
  int cloudSize = laserCloudIn.points.size();

  double t1 = clock();
  double angle_last = 0.0, dis_incre_last = 0.0, dis_incre_now = 0.0,
         angle = 0.0;
  for (int i = 0; i < cloudSize; i++) {
    angle = atan((laserCloudIn.points[i].z - 2.13) /
                 (sqrt(laserCloudIn.points[i].x * laserCloudIn.points[i].x +
                       laserCloudIn.points[i].y * laserCloudIn.points[i].y)));
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

  //开始计算曲率
  for (size_t i = 0; i < scans.size(); i++) {
    for (size_t j = 6; j < scans[i].size() - 6; j++) {
      float diffX = laserCloudIn.points[scans[i][j] - 2].x +
                    laserCloudIn.points[scans[i][j] - 1].x -
                    4 * laserCloudIn.points[scans[i][j]].x +
                    laserCloudIn.points[scans[i][j] + 1].x +
                    laserCloudIn.points[scans[i][j] + 2].x;
      float diffY = laserCloudIn.points[scans[i][j] - 2].y +
                    laserCloudIn.points[scans[i][j] - 1].y -
                    4 * laserCloudIn.points[scans[i][j]].y +
                    laserCloudIn.points[scans[i][j] + 1].y +
                    laserCloudIn.points[scans[i][j] + 2].y;
      float diffZ = laserCloudIn.points[scans[i][j] - 2].z +
                    laserCloudIn.points[scans[i][j] - 1].z -
                    4 * laserCloudIn.points[scans[i][j]].z +
                    laserCloudIn.points[scans[i][j] + 1].z +
                    laserCloudIn.points[scans[i][j] + 2].z;

      cloudCurvature[scans[i][j]] =
          diffX * diffX + diffY * diffY + diffZ * diffZ;
      float intensity_point = laserCloudIn.points[scans[i][j] - 2].intensity +
                              laserCloudIn.points[scans[i][j] - 1].intensity -
                              laserCloudIn.points[scans[i][j] + 1].intensity -
                              laserCloudIn.points[scans[i][j] + 2].intensity;

      cloudintensity[scans[i][j]] = sqrt(intensity_point * intensity_point);
      cloudslope[scans[i][j]] = fabs(laserCloudIn.points[scans[i][j] + 2].y -
                                     laserCloudIn.points[scans[i][j] - 2].y) /
                                (laserCloudIn.points[scans[i][j] + 2].x -
                                 laserCloudIn.points[scans[i][j] - 2].x);
    }
  }
  //挑选点，排除容易被斜面挡住的点以及离群点
  for (size_t i = 5; i < laserCloudIn.points.size() - 6; i++) {
    float diffX = laserCloudIn.points[i + 1].x - laserCloudIn.points[i].x;
    float diffY = laserCloudIn.points[i + 1].y - laserCloudIn.points[i].y;
    float diffZ = laserCloudIn.points[i + 1].z - laserCloudIn.points[i].z;
    //计算有效曲率点与后一个点之间的距离平方和
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    if (diff > 0.1) {  //前提:两个点之间距离要大于0.1

      //点的深度
      float depth1 = sqrt(laserCloudIn.points[i].x * laserCloudIn.points[i].x +
                          laserCloudIn.points[i].y * laserCloudIn.points[i].y +
                          laserCloudIn.points[i].z * laserCloudIn.points[i].z);

      //后一个点的深度
      float depth2 =
          sqrt(laserCloudIn.points[i + 1].x * laserCloudIn.points[i + 1].x +
               laserCloudIn.points[i + 1].y * laserCloudIn.points[i + 1].y +
               laserCloudIn.points[i + 1].z * laserCloudIn.points[i + 1].z);

      //按照两点的深度的比例，将深度较大的点拉回后计算距离
      if (depth1 > depth2) {
        diffX = laserCloudIn.points[i + 1].x -
                laserCloudIn.points[i].x * depth2 / depth1;
        diffY = laserCloudIn.points[i + 1].y -
                laserCloudIn.points[i].y * depth2 / depth1;
        diffZ = laserCloudIn.points[i + 1].z -
                laserCloudIn.points[i].z * depth2 / depth1;

        //边长比也即是弧度值，若小于0.1，说明夹角比较小，斜面比较陡峭,点深度变化比较剧烈,点处在近似与激光束平行的斜面上
        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 <
            0.15) {  //排除容易被斜面挡住的点
          //该点及前面五个点（大致都在斜面上）全部置为筛选过
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloudIn.points[i + 1].x * depth1 / depth2 -
                laserCloudIn.points[i].x;
        diffY = laserCloudIn.points[i + 1].y * depth1 / depth2 -
                laserCloudIn.points[i].y;
        diffZ = laserCloudIn.points[i + 1].z * depth1 / depth2 -
                laserCloudIn.points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 <
            0.15) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
        }
      }
    }

    float diffX2 = laserCloudIn.points[i].x - laserCloudIn.points[i - 1].x;
    float diffY2 = laserCloudIn.points[i].y - laserCloudIn.points[i - 1].y;
    float diffZ2 = laserCloudIn.points[i].z - laserCloudIn.points[i - 1].z;
    //与前一个点的距离平方和
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    //点深度的平方和
    float dis = laserCloudIn.points[i].x * laserCloudIn.points[i].x +
                laserCloudIn.points[i].y * laserCloudIn.points[i].y +
                laserCloudIn.points[i].z * laserCloudIn.points[i].z;

    cloud_distance[i] = sqrt(dis);
    //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
    if (diff > 0.05 * dis || diff2 > 0.05 * dis || dis > 2500 || dis < 0.04) {
      cloudNeighborPicked[i] = 1;
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsSharp(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr intensityPointsSharp(
      new pcl::PointCloud<pcl::PointXYZI>());

  for (size_t i = 0; i < scans.size(); i++) {
    for (size_t j = 20; j < scans[i].size() - 20; j++) {
      if (cloudCurvature[scans[i][j]] > 0.02 &&
          cloudNeighborPicked[scans[i][j]] != 1 &&
          fabs(cloudslope[scans[i][j] - 1] - cloudslope[scans[i][j]]) > 0.3 &&
          (cloud_distance[scans[i][j] - 1] - cloud_distance[scans[i][j]] > 0) &&
          (cloud_distance[scans[i][j] + 1] > cloud_distance[scans[i][j]]) &&
          laserCloudIn.points[scans[i][j]].z > 0.06) {
        cornerPointsSharp->points.push_back(laserCloudIn.points[scans[i][j]]);
      } else if (cloudCurvature[scans[i][j]] < 0.0005 &&
                 cloudNeighborPicked[scans[i][j]] != 1) {
        surfPointsSharp->points.push_back(laserCloudIn.points[scans[i][j]]);
      }
      if (cloudintensity[scans[i][j]] > 70.0 &&
          laserCloudIn.points[scans[i][j]].z < 1.0 &&
          cloudCurvature[scans[i][j]] < 0.02 &&
          cloudintensity[scans[i][j]] > cloudintensity[scans[i][j] - 1]) {
        intensityPointsSharp->points.push_back(
            laserCloudIn.points[scans[i][j]]);
      }
    }
  }

  // std::cout << "cornerPointsLessSharp" << cornerPointsSharp->points.size()
  //           << std::endl;
  // std::cout << "surfPointsSharp  " << surfPointsSharp->points.size()
  //           << std::endl;

  AINFO << "coner surf intensity size : " << cornerPointsSharp->points.size()
        << " " << surfPointsSharp->points.size() << " "
        << intensityPointsSharp->points.size();
  double time_feature = clock();
  std::cout << "feature time : " << (double)(time_feature - t1) / CLOCKS_PER_SEC
            << endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp_ds(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsSharp_ds(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr intensityPointsSharp_ds(
      new pcl::PointCloud<pcl::PointXYZI>());

  down_size_filter_corner_.setInputCloud(cornerPointsSharp);
  down_size_filter_corner_.filter(*cornerPointsSharp_ds);
  down_size_filter_surf_.setInputCloud(surfPointsSharp);
  down_size_filter_surf_.filter(*surfPointsSharp_ds);
  down_size_filter_intensity_.setInputCloud(intensityPointsSharp);
  down_size_filter_intensity_.filter(*intensityPointsSharp_ds);
  plicp_.GetPL_ICP(cornerPointsSharp_ds, surfPointsSharp_ds,
                   intensityPointsSharp_ds);  //载入当前特征点
  if (!plicp_.recevied_cloud_)  //跑出地图，重置所有参数
  {
    ResetLidarOdomParam();
    return;
  }

  std::cout << "icp time : "
            << (double)(clock() - time_feature) / CLOCKS_PER_SEC << endl;
  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.header.stamp = laserCloudMsg->header.stamp;
  odomAftMapped.pose.pose.orientation.x = plicp_.m_q_w_curr.x();  //得到位姿
  odomAftMapped.pose.pose.orientation.y = plicp_.m_q_w_curr.y();
  odomAftMapped.pose.pose.orientation.z = plicp_.m_q_w_curr.z();
  odomAftMapped.pose.pose.orientation.w = plicp_.m_q_w_curr.w();

  odomAftMapped.pose.pose.position.x = plicp_.m_t_w_curr.x();
  odomAftMapped.pose.pose.position.y = plicp_.m_t_w_curr.y();
  odomAftMapped.pose.pose.position.z = plicp_.m_t_w_curr.z();

  if (plicp_.odom_cov_ < 0.007) {
    odomAftMapped.pose.covariance[0] = 0.15;
    // ROS_ERROR("the lidar cov %f , %f： ", odomAftMapped.pose.covariance[0],
    // plicp_.odom_cov_);
    odomAftMapped.pose.covariance[7] = 0.15;
    odomAftMapped.pose.covariance[14] = 0.15;
  } else {
    odomAftMapped.pose.covariance[0] = fabs(plicp_.odom_cov_ - 0.006) * 800;
    // ROS_ERROR("the lidar cov %f , %f： ", odomAftMapped.pose.covariance[0],
    // plicp_.odom_cov_);
    odomAftMapped.pose.covariance[7] = fabs(plicp_.odom_cov_ - 0.006) * 800;
    odomAftMapped.pose.covariance[14] = fabs(plicp_.odom_cov_ - 0.006) * 800;
  }
  lidar_odom_pub_.publish(odomAftMapped);
  if (frist_matcher_) {
    frist_matcher_score_ = odomAftMapped.pose.covariance[0];
    frist_matcher_ = false;
  }

  if (1) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr surf_map(
        new pcl::PointCloud<pcl::PointXYZI>());
    surf_map = plicp_.m_laser_cloud_corner_from_map;
    std::cout << "map cloud points sizes: " << surf_map->points.size()
              << std::endl;
    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*surf_map, cloudMsgTemp);
    cloudMsgTemp.header.frame_id = "camera_init";
    cloudMsgTemp.header.stamp = laserCloudMsg->header.stamp;
    cloud_map_pub_.publish(cloudMsgTemp);
    showed_map_ = true;
  }
  if (odomAftMapped.pose.covariance[0] > 5) {
    if (/* lidar_odom_error_times_ > 0 &&*/ receive_new_fusion_odom_) {
      fusion_odom_.z() = odomAftMapped.pose.pose.position.z;
      plicp_.ReceivePosition(fusion_quater_, fusion_odom_);
    }
    lidar_odom_error_times_++;
  } else
    lidar_odom_error_times_ = 0;

  cout << "lidar odom all time" << (double)(clock() - t1) / CLOCKS_PER_SEC
       << endl;
}

void GetLidarOdom::FristOdomHander(
    const geometry_msgs::PoseWithCovarianceStamped receivepose) {
  frist_odom_pose_.GetFristPose(receivepose);
  nav_msgs::Odometry odomAftMapped_frist;
  odomAftMapped_frist.header.frame_id = "camera_init";
  odomAftMapped_frist.header.stamp = receivepose.header.stamp;
  odomAftMapped_frist.pose.pose.orientation.x =
      frist_odom_pose_.frist_pose_quater.x();  //得到位姿
  odomAftMapped_frist.pose.pose.orientation.y =
      frist_odom_pose_.frist_pose_quater.y();
  odomAftMapped_frist.pose.pose.orientation.z =
      frist_odom_pose_.frist_pose_quater.z();
  odomAftMapped_frist.pose.pose.orientation.w =
      frist_odom_pose_.frist_pose_quater.w();

  odomAftMapped_frist.pose.pose.position.x =
      frist_odom_pose_.frist_pose_position.x();
  odomAftMapped_frist.pose.pose.position.y =
      frist_odom_pose_.frist_pose_position.y();
  odomAftMapped_frist.pose.pose.position.z =
      frist_odom_pose_.frist_pose_position.x();
  pub_frist_odom_.publish(odomAftMapped_frist);
}

void GetLidarOdom::ReceiveFusionOdom(const nav_msgs::Odometry input_odom) {
  fusion_odom_ = Eigen::Vector3d(input_odom.pose.pose.position.x,
                                 input_odom.pose.pose.position.y, 0.0);
  fusion_quater_ = Eigen::Quaterniond(
      input_odom.pose.pose.orientation.w, input_odom.pose.pose.orientation.x,
      input_odom.pose.pose.orientation.y, input_odom.pose.pose.orientation.z);
  fusion_time_ = input_odom.header.stamp;
  receive_new_fusion_odom_ = true;
}

void GetLidarOdom::ParamInit() {
  pcl::PointCloud<pcl::PointXYZI> cloud_tgt;
  std::string get_pcd_name =
      "../sweeper_ws/src/sweeper_haide/data/path/" + get_pcd_name_;
  if (!pcl::io::loadPCDFile(get_pcd_name.c_str(), cloud_tgt)) {
    load_map_success_ = true;
    // std::cout << "cloud load ok  size: " << cloud_tgt.points.size()
    //           << std::endl;

    AINFO << "cloud load size : " << cloud_tgt.points.size();
  } else
    load_map_success_ = false;
  plicp_.GetCornerSurfMap(cloud_tgt);
  plicp_.m_para_icp_max_iterations = icp_max_times_;
}
GetLidarOdom::~GetLidarOdom() {}

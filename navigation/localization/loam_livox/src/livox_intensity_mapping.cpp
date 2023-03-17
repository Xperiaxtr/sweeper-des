// #include <ceres/ceres.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <mutex>
#include <queue>
// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>

#include <string>
#include <thread>
#include <vector>

#include "edge_se3_priorxyz.hpp"
#include "point_cloud_matcher.hpp"
// #include "tools/common.h"
// #include "tools/logger.hpp"
// #include "tools/pcl_tools.hpp"
#include <pcl/registration/icp.h>
#include <time.h>

#include <fstream>
#include <mutex>

#include "../../../../common/log.h"
#include "../../../../common/watch_dog.h"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweepMission.h"

// g2o模块
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/se3quat.h>
// #include <g2o/types/slam3d/types_slam3d.h>
// #include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#define no_receive_feature_ 3109
#define no_receive_fusion_navigation_ 3110
#define make_file_error_ 3111
#define save_line_error_ 3112
#define cin_record_line_mode_ 3103
#define mapping_mode_normal_ 3101
struct PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  double q_w;
  double q_x;
  double q_y;
  double q_z;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef PointXYZIRPYT PointTypePose;

struct Data_pair {
  pcl::PointCloud<PointType> laser_cloud_corner;
  pcl::PointCloud<PointType> laser_cloud_surf;
  pcl::PointCloud<PointType> laser_cloud_intensity;
  // pcl::PointCloud<PointType> laser_cloud_full;

  double laser_time_corner;
  double laser_time_surf;
  double laser_time_intensity;
  // double laser_time_full;

  bool m_has_pc_corner = 0;
  bool m_has_pc_surf = 0;
  bool m_has_pc_intensity = 0;
  // bool m_has_pc_full = 0;

  void add_pc_corner(sensor_msgs::PointCloud2ConstPtr ros_pc) {
    // m_pc_corner = ros_pc;
    laser_time_corner = ros_pc->header.stamp.toSec();
    pcl::fromROSMsg(*ros_pc, laser_cloud_corner);
    m_has_pc_corner = true;
  }

  void add_pc_surf(sensor_msgs::PointCloud2ConstPtr ros_pc) {
    laser_time_surf = ros_pc->header.stamp.toSec();
    pcl::fromROSMsg(*ros_pc, laser_cloud_surf);
    m_has_pc_surf = true;
  }

  void add_pc_intensity(sensor_msgs::PointCloud2ConstPtr ros_pc) {
    laser_time_intensity = ros_pc->header.stamp.toSec();
    pcl::fromROSMsg(*ros_pc, laser_cloud_intensity);
    m_has_pc_intensity = true;
  }

  // void add_pc_full(sensor_msgs::PointCloud2ConstPtr ros_pc) {
  //   laser_time_full = ros_pc->header.stamp.toSec();
  //   pcl::fromROSMsg(*ros_pc, laser_cloud_full);
  //   m_has_pc_full = true;
  // }

  bool is_completed() {
    if (m_has_pc_corner && m_has_pc_surf && m_has_pc_intensity) {
      std::cout << "cloud data receive ok ！";
    }
    return (m_has_pc_corner && m_has_pc_surf && m_has_pc_intensity);
  }
  void reset_param() {
    m_has_pc_corner = false;
    m_has_pc_surf = false;
    m_has_pc_intensity = false;
    // m_has_pc_full = false;
  }
};

struct SaveDatas {
  int road_name;
  double gnss_x;
  double gnss_y;
  Eigen::Vector3d lidar_position;
  Eigen::Quaterniond lidar_quater;
};

struct Gps_data {
  double time_gps;
  Eigen::Quaterniond quaternion_gps;
  Eigen::Vector3d position_gps;
  double covariance_gps[6];
  bool receive_data = false;
  void get_datas(const nav_msgs::OdometryConstPtr input_odom) {
    std::cout << "receive gps data !" << std::endl;
    time_gps = input_odom->header.stamp.toSec();
    quaternion_gps = Eigen::Quaterniond(input_odom->pose.pose.orientation.w,
                                        input_odom->pose.pose.orientation.x,
                                        input_odom->pose.pose.orientation.y,
                                        input_odom->pose.pose.orientation.z);
    position_gps = Eigen::Vector3d(input_odom->pose.pose.position.x,
                                   input_odom->pose.pose.position.y,
                                   input_odom->pose.pose.position.z);
    // CorrectionGps(quaternion_gps, position_gps);
    covariance_gps[0] = input_odom->pose.covariance[0];
    covariance_gps[1] = input_odom->pose.covariance[7];
    covariance_gps[2] = input_odom->pose.covariance[14];
    covariance_gps[3] = input_odom->pose.covariance[21];
    covariance_gps[4] = input_odom->pose.covariance[28];
    covariance_gps[5] = input_odom->pose.covariance[35];
  }
};

class LaserMappingNew {
 public:
  std::queue<Data_pair> receive_clouds_;
  std::vector<Gps_data> receive_gpss;
  std::vector<SaveDatas> all_need_save_data_;
  Data_pair receive_cloud_now_;
  double timeLaserCloudCornerLast;
  double timeLaserCloudSurfLast;
  double time_laser_cloud_intensity_last_;

  double cloudrealtime;

  bool potentialLoopFlag;

  int closestHistoryFrameID;
  int latestFrameIDLoopCloure;
  size_t cloud_cin_size_ = 0;
  int loop_size_ = 0;
  int cloud_loop_frequency = 0;

  double plicp_search_radius = 50.0;
  bool using_loop_ = false;
  bool add_gps_edge = false;
  bool using_gps_data = false;
  bool save_odom_path = false;
  bool save_gps_path = false;
  bool save_gps_correspond_odom_data = false;
  double gps_max_cov = 0.12;
  double publish_range_map = 500.0;
  double icp_loop_history_search_num = 10;
  double icp_loop_radius = 8.0;
  double icp_loop_score = 0.2;

  std::mutex mtx;

  g2o::SparseOptimizer optimizer;

  PointCloudMatcher plicp;
  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D_100;
  std::vector<Eigen::Vector3d> gps_key_pose_3d_;

  std::vector<PointTypePose> cloudKeyPoses6D;
  std::vector<PointTypePose> cloudKeyPoses6D_100;
  std::vector<PointTypePose> g2o_key_poses_6d_;

  vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
  vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
  vector<pcl::PointCloud<PointType>::Ptr> intensity_cloud_key_frames_;

  //车开刚刚准备开时，保存一个局部地图
  vector<pcl::PointCloud<PointType>::Ptr> cornerCloudFrame_100;
  vector<pcl::PointCloud<PointType>::Ptr> surfCloudFrame_100;
  vector<pcl::PointCloud<PointType>::Ptr> intensity_cloud_frame_100_;

  pcl::PointCloud<PointType>::Ptr ds_cornerCloudKeyFrame_now;
  pcl::PointCloud<PointType>::Ptr ds_surfCloudKeyFrame_now;
  pcl::PointCloud<PointType>::Ptr ds_intensity_cloud_key_frame_now_;

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> down_size_filter_intensity_;

  pcl::VoxelGrid<PointType> saveFilterCorner;
  pcl::VoxelGrid<PointType> saveFilterSurf;
  pcl::VoxelGrid<PointType> save_filter_intensity_;

  pcl::VoxelGrid<PointType> downSizeFilterHistoryKeyFrames;
  pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;
  pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;
  pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_intensity_from_map_;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_history_key_poses_;

  //局部地图
  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
  pcl::PointCloud<PointType>::Ptr laser_cloud_intensity_from_map_;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;
  pcl::PointCloud<PointType>::Ptr laser_cloud_intensity_from_map_ds_;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laser_cloud_intensity_last_;
  // pcl::PointCloud<PointType>::Ptr laser_cloud_full_last_;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;

  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloudDS;

  pcl::PointCloud<PointType>::Ptr laser_cloud_corner_map_ds;
  pcl::PointCloud<PointType>::Ptr last_gps_position_cloud_;
  int frame_counts = 0;

  PointType currentRobotPosPoint;
  Eigen::Quaterniond m_q_w_curr;
  Eigen::Vector3d m_t_w_curr;

  Eigen::Isometry3d T_last = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d frist_gps_3d_ = Eigen::Isometry3d::Identity();

  Eigen::Vector3d euler_angle_last_mapping;

  int odom_edge_num_;
  int gps_edge_num_;
  double dis_edge_;
  Eigen::Vector3d last_loop_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  int last_gps_position_in_vector_ = 0;
  double lidar_odom_cov_ = 0.0;

  Eigen::Isometry3d last_gps_position_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d last_odom_position_in_gps_;
  Eigen::Quaterniond last_odom_quater_in_gps_;

  int last_mapping_state_;
  // char save_pcd_name_[5000];
  std::string save_pcd_name_;
  std::string save_line_name_;
  std::string save_file_name_;
  std::string save_dir_name_;
  int save_now_pcd_name_;
  bool allow_mapping_mode_;
  bool allow_mapping_last_mode_;
  bool is_frist_key_frame_point_ = true;
  size_t last_gps_in_key_frame_ = 0;
  int frist_gps_corresponding_lidar_vertex_;
  bool save_pcd_line_ok_;
  bool make_file_ok_;
  bool record_line_mode_;
  int g2o_vertex_num_;

  Eigen::Isometry3d gps_to_map_tran_;

  sweeper::common::WatchDog watch_dog_feature_;
  sweeper::common::WatchDog watch_dog_integrated_navigation_;
  sweeper_msgs::SensorFaultInformation::_state_code_type
      receive_mapping_state_code_;

  ros::Subscriber sub_lasercloud_corner;
  // ros::Subscriber sub_laser_full;
  ros::Subscriber sub_lasercloud_surf;
  ros::Subscriber sub_gps;
  ros::Subscriber sub_laser_cloud_intensity;
  ros::Subscriber sub_mapping_information_;
  ros::Subscriber sub_state_mapping_information_;

  ros::Publisher pub_path;
  ros::Publisher pub_odom;
  ros::Publisher pubLaserCloudSurround;
  ros::Publisher pubRecentKeyFrames;
  ros::Publisher pubKeyPoses;
  ros::Publisher pubFramesnow;
  ros::Publisher pubIcpKeyFrames;
  ros::Publisher pub_gps_position;
  ros::Publisher pub_correct_odom_, pub_gps_cloud_;
  ros::Publisher pub_state_information_;
  // ros::Publisher pub_full_cloud_;

  LaserMappingNew(ros::NodeHandle nh, ros::NodeHandle nh_private)
      : allow_mapping_mode_(false),
        allow_mapping_last_mode_(false),
        frist_gps_corresponding_lidar_vertex_(0),
        save_pcd_line_ok_(true),
        make_file_ok_(true),
        // record_line_mode_(false),
        // last_mapping_state_(100),
        g2o_vertex_num_(0) {
    last_mapping_state_ = 100;
    record_line_mode_ = false;
    ParamInit(nh_private);
    // sub_laser_full = nh.subscribe<sensor_msgs::PointCloud2>(
    //     "/pc2_full", 10000, &LaserMappingNew::LaserCloudFullLastHandler,
    //     this);
    sub_lasercloud_corner = nh.subscribe<sensor_msgs::PointCloud2>(
        "/pc2_corners", 10000, &LaserMappingNew::LaserCloudCornerLastHandler,
        this);
    sub_lasercloud_surf = nh.subscribe<sensor_msgs::PointCloud2>(
        "/pc2_surface", 10000, &LaserMappingNew::LaserCloudSurfLastHandler,
        this);
    sub_laser_cloud_intensity = nh.subscribe<sensor_msgs::PointCloud2>(
        "/pc2_intensity", 10000,
        &LaserMappingNew::LaserCloudIntensityLastHandler, this);
    sub_gps = nh.subscribe<nav_msgs::Odometry>(
        "/sweeper/localization/gnss", 10000, &LaserMappingNew::LaserGpsHandler,
        this);
    sub_mapping_information_ = nh.subscribe<sweeper_msgs::SweepMission>(
        "/sweeper/sweep_mode", 1, &LaserMappingNew::ReceiveMappingInformation,
        this);
    sub_state_mapping_information_ =
        nh.subscribe<sweeper_msgs::SensorFaultInformation>(
            "/sweeper/mapping/diagnose", 1,
            &LaserMappingNew::ReceiveMappingDiagnose, this);

    pub_state_information_ = nh.advertise<sweeper_msgs::SensorFaultInformation>(
        "/sweeper/common/diagnose", 1);
    pub_path = nh.advertise<nav_msgs::Path>("/loam/path", 1);
    pubLaserCloudSurround =
        nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
    pubRecentKeyFrames =
        nh.advertise<sensor_msgs::PointCloud2>("/recent_cloud", 2);
    pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
    pubFramesnow = nh.advertise<sensor_msgs::PointCloud2>("/now_cloud", 2);
    // pub_full_cloud_ =
    //     nh.advertise<sensor_msgs::PointCloud2>("/full_tran_cloud", 2);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/pub_odom", 1);
    pub_gps_position = nh.advertise<nav_msgs::Odometry>("/gps_position", 1);
    pub_correct_odom_ = nh.advertise<nav_msgs::Odometry>("/odom_correct", 1);
    // pub_map_position_ = nh.advertise<nav_msgs::Odometry>("/map_pos", 1);

    pubIcpKeyFrames =
        nh.advertise<sensor_msgs::PointCloud2>("/corrected_cloud", 2);

    pub_gps_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/gps_cloud", 1);

    downSizeFilterCorner.setLeafSize(0.4, 0.4, 0.4);
    down_size_filter_intensity_.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterSurf.setLeafSize(1.0, 1.0, 1.0);

    saveFilterSurf.setLeafSize(0.4, 0.4, 0.4);
    saveFilterCorner.setLeafSize(0.4, 0.4, 0.4);
    save_filter_intensity_.setLeafSize(0.4, 0.4, 0.4);

    downSizeFilterHistoryKeyFrames.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurroundingKeyPoses.setLeafSize(0.01, 0.01, 0.01);

    downSizeFilterGlobalMapKeyPoses.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterGlobalMapKeyFrames.setLeafSize(0.5, 0.5, 0.5);

    kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
    globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
    globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
    globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
    globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());

    cloudKeyPoses3D_100.reset(new pcl::PointCloud<PointType>());
    last_gps_position_cloud_.reset(new pcl::PointCloud<PointType>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
    laser_cloud_intensity_last_.reset(new pcl::PointCloud<PointType>());
    // laser_cloud_full_last_.reset(new pcl::PointCloud<PointType>());

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
    laser_cloud_intensity_from_map_.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());
    laser_cloud_intensity_from_map_ds_.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_intensity_from_map_.reset(new pcl::KdTreeFLANN<PointType>());

    kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
    globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
    globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
    globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
    globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());

    ds_cornerCloudKeyFrame_now.reset(new pcl::PointCloud<PointType>());
    ds_surfCloudKeyFrame_now.reset(new pcl::PointCloud<PointType>());
    ds_intensity_cloud_key_frame_now_.reset(new pcl::PointCloud<PointType>());

    latestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

    nearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    nearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

    laser_cloud_corner_map_ds.reset(new pcl::PointCloud<PointType>());
    potentialLoopFlag = false;

    //构建图优化
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    std::unique_ptr<Block::LinearSolverType> linearSolver(
        new g2o::LinearSolverCholmod<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
    //选择梯度下降方法
    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);
    plicp.m_para_icp_max_iterations = 8;

    timeLaserCloudCornerLast = 0;
    timeLaserCloudSurfLast = 0;
    time_laser_cloud_intensity_last_ = 0;
    ResetAllParam();
  }
  void ResetMappingParame() {
    gps_to_map_tran_ = Eigen::Isometry3d::Identity();
    m_q_w_curr.w() = 1.0;
    m_q_w_curr.x() = 0.0;
    m_q_w_curr.y() = 0.0;
    m_q_w_curr.z() = 0.0;
    m_t_w_curr = Eigen::Vector3d(0.0, 0.0, 0.0);
    euler_angle_last_mapping = Eigen::Vector3d(0.0, 0.0, 0.0);
    // receive_clouds_.clear();
    std::queue<Data_pair> empty;
    swap(empty, receive_clouds_);
    receive_gpss.clear();
    // kdtreeGlobalMap.clear();
    globalMapKeyPoses->clear();
    globalMapKeyPosesDS->clear();
    cloudKeyPoses3D->clear();
    cloudKeyPoses3D_100->clear();
    last_gps_position_cloud_->clear();
    // kdtreeSurroundingKeyPoses.clear();
    // kdtreeHistoryKeyPoses.clear();
    laserCloudCornerLast->clear();
    laserCloudSurfLast->clear();
    laser_cloud_intensity_last_->clear();
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    laser_cloud_intensity_from_map_->clear();
    laserCloudCornerFromMapDS->clear();
    laserCloudSurfFromMapDS->clear();
    laser_cloud_intensity_from_map_ds_->clear();
    // kdtreeCornerFromMap->clear();
    // kdtreeSurfFromMap->clear();
    // kdtree_intensity_from_map_->clear();
    // kdtreeGlobalMap->clear();
    globalMapKeyPoses->clear();
    globalMapKeyPosesDS->clear();
    globalMapKeyFrames->clear();
    globalMapKeyFramesDS->clear();
    ds_cornerCloudKeyFrame_now->clear();
    ds_surfCloudKeyFrame_now->clear();
    ds_intensity_cloud_key_frame_now_->clear();
    latestSurfKeyFrameCloud->clear();
    latestSurfKeyFrameCloudDS->clear();
    nearHistorySurfKeyFrameCloud->clear();
    nearHistorySurfKeyFrameCloudDS->clear();
    laser_cloud_corner_map_ds->clear();
    g2o_key_poses_6d_.clear();
    gps_key_pose_3d_.clear();
    cloudKeyPoses6D.clear();
    cloudKeyPoses6D_100.clear();
    cornerCloudKeyFrames.clear();
    surfCloudKeyFrames.clear();
    intensity_cloud_key_frames_.clear();
    cornerCloudFrame_100.clear();
    surfCloudFrame_100.clear();
    intensity_cloud_frame_100_.clear();
    timeLaserCloudCornerLast = 0;
    timeLaserCloudSurfLast = 0;
    time_laser_cloud_intensity_last_ = 0;
    cloud_loop_frequency = 0;
    save_pcd_line_ok_ = true;
    make_file_ok_ = true;
    record_line_mode_ = false;

    frame_counts = 0;
    T_last = Eigen::Isometry3d::Identity();

    plicp.m_q_w_curr = plicp.m_q_w_last =
        Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    plicp.m_t_w_curr = plicp.m_t_w_last = Eigen::Vector3d(0.0, 0.0, 0.0);
    T_last = Eigen::Isometry3d::Identity();
    frist_gps_3d_ = Eigen::Isometry3d::Identity();
    last_loop_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    last_gps_position_ = Eigen::Isometry3d::Identity();
    last_odom_position_in_gps_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    last_odom_quater_in_gps_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    last_gps_position_in_vector_ = 0;
    lidar_odom_cov_ = 0.0;
    odom_edge_num_ = 0;
    gps_edge_num_ = 0;
    dis_edge_ = 0.0;
    is_frist_key_frame_point_ = true;
    last_gps_in_key_frame_ = 0;
    frist_gps_corresponding_lidar_vertex_ = 0;
    optimizer.clear();
  }
  void ParamInit(ros::NodeHandle nh) {
    nh.param<bool>("using_loop", using_loop_, 0);
    nh.param<bool>("add_gps_edge", add_gps_edge, 0);
    nh.param<bool>("using_gps_data", using_gps_data, 0);
    nh.param<bool>("save_odom_path", save_odom_path, 0);
    nh.param<bool>("save_gps_path", save_gps_path, 0);
    nh.param<bool>("save_gps_correspond_odom_data",
                   save_gps_correspond_odom_data, 0);
    nh.param<double>("gps_max_cov", gps_max_cov, 0.12);
    nh.param<double>("publish_range_map", publish_range_map, 500.0);
    nh.param<double>("plicp_search_radius", plicp_search_radius, 50.0);
    nh.param<double>("icp_loop_history_search_num", icp_loop_history_search_num,
                     10.0);
    nh.param<double>("icp_loop_radius", icp_loop_radius, 8.0);
    nh.param<double>("icp_loop_score", icp_loop_score, 0.2);
    nh.param<bool>("allow_mapping_mode", allow_mapping_mode_, false);

    // std::cout << "receive param" << std::endl;
    // std::cout << "using_loop " << using_loop_ << std::endl;
    // std::cout << "add_gps_edge " << add_gps_edge << std::endl;
    // std::cout << "using_gps_data " << using_gps_data << std::endl;
    // std::cout << "save_odom_path " << save_odom_path << std::endl;
    // std::cout << "save_gps_path " << save_gps_path << std::endl;
    // std::cout << "save_gps_correspond_odom_data "
    //           << save_gps_correspond_odom_data << std::endl;
    // std::cout << "gps_max_cov " << gps_max_cov << std::endl;
    // std::cout << "publish_range_map " << publish_range_map << std::endl;
    // std::cout << "plicp_search_radius " << plicp_search_radius << std::endl;
    // std::cout << "icp_loop_history_search_num " <<
    // icp_loop_history_search_num
    //           << std::endl;
    // std::cout << "icp_loop_radius " << icp_loop_radius << std::endl;
    // std::cout << "icp_loop_score " << icp_loop_score << std::endl;
    // std::cout << "allow_mapping_mode : " << allow_mapping_mode_ << std::endl;
    AINFO << "receive param";
    AINFO << "using_loop " << using_loop_;
    AINFO << "add_gps_edge " << add_gps_edge;
    AINFO << "using_gps_data " << using_gps_data;
    AINFO << "save_odom_path " << save_odom_path;
    AINFO << "save_gps_path " << save_gps_path;
    AINFO << "save_gps_correspond_odom_data " << save_gps_correspond_odom_data;
    AINFO << "gps_max_cov " << gps_max_cov;
    AINFO << "publish_range_map " << publish_range_map;
    AINFO << "plicp_search_radius " << plicp_search_radius;
    AINFO << "icp_loop_history_search_num " << icp_loop_history_search_num;
    AINFO << "icp_loop_radius " << icp_loop_radius;
    AINFO << "icp_loop_score " << icp_loop_score;
    AINFO << "allow_mapping_mode : " << allow_mapping_mode_;
  }

  int remove_dir(const char *dir) {
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
        remove_dir(dir_name);  // 递归调用
      }
      closedir(dirp);

      rmdir(dir);  // 删除空目录
    } else {
      perror("unknow file type!");
    }
    return 0;
  }

  void ResetAllParam() {
    //建图参数初始化
    ResetMappingParame();
    //文件初始化
    save_dir_name_ = "../sweeper_ws/src/sweeper_haide/data/path/uname";
    remove_dir(save_dir_name_.c_str());
    if (mkdir(save_dir_name_.c_str(), 0777)) {
      std::cout << "make file errorly ! " << std::endl;
      make_file_ok_ = false;
    } else {
      std::cout << "make file successfully ! " << std::endl;
      make_file_ok_ = true;
    }
    //轨迹初始化
    all_need_save_data_.clear();
    save_now_pcd_name_ = 0;
  }

  void ReceiveMappingInformation(
      sweeper_msgs::SweepMission sweeper_mode_command) {
    // ROS_ERROR("map information");
    if (sweeper_mode_command.mode == 3) {
      record_line_mode_ = true;
      if (last_mapping_state_ != sweeper_mode_command.start_cmd) {
        //建图开始和关闭采用是否读取数据来进行，如果有数据则建图，没有数据则不建图
        switch (sweeper_mode_command.start_cmd) {
          case 0:  //取消： 1.所有参数初始化，不保存数据，在回调函数直接进行
            ResetAllParam();
            break;
          case 1:  //开始： 1.允许读取数据，标志位置为1
            allow_mapping_mode_ = true;
            break;
          case 2:  //暂停： 1.将数据保存到vector中，并初始化部分参数，允许读取数据置为false,保存pcd
            allow_mapping_mode_ = false;
            SaveLineToVector();
            SavePcdName();
            PublishAndSaveMap();
            ResetMappingParame();
            break;
          case 4:  //删除： 1.不将当前当前数据保存到vector中，初始化部分参数，
            allow_mapping_mode_ = false;
            ResetMappingParame();
            break;
          case 3:  //保存： 1.将数据报存到vector并将当前vector保存，并初始化所有参数
            allow_mapping_mode_ = false;
            SaveLineToVector();
            save_file_name_ = sweeper_mode_command.line;
            SavePcdName();
            SaveLineName();
            // SaveDataToFile(save_line_name_, 0);
            //保存地图
            PublishAndSaveMap();
            RenameFile();
            ResetAllParam();
            break;
          default:
            break;
        }
      }
      last_mapping_state_ = sweeper_mode_command.start_cmd;
    } else
      record_line_mode_ = false;
  }

  void SavePcdName() {
    std::string save_now_pcd_name = to_string(save_now_pcd_name_);
    // sprintf(save_pcd_name_, "%s/%s.pcd", save_dir_name_.c_str(),
    //         save_now_pcd_name.c_str());
    save_pcd_name_ = save_dir_name_ + "/" + save_now_pcd_name + ".pcd";
    std::cout << "----- Save cloud to " << save_pcd_name_ << " -----"
              << std::endl;

    // std::string path_file = "../sweeper_ws/src/sweeper_haide";
    // sprintf(save_line_name_, "%s/%s/%s.txt", path_file.c_str(),
    //         save_file_name_.c_str() save_file_name_.c_str());
    // std::cout << "----- Save line to " << save_line_name_ << "------"
    //           << std::endl;
    save_now_pcd_name_++;
  }
  void SaveLineName() {
    std::string path_file = "../sweeper_ws/src/sweeper_haide/data/path";
    // sprintf(save_line_name_, "%s/%s.txt", save_dir_name_.c_str(),
    //         save_file_name_.c_str());
    // std::cout << "----- Save line to " << save_line_name_ << "------"
    //           << std::endl;

    save_line_name_ = path_file + "/" + save_file_name_ + ".txt";
    std::cout << "----- Save line to " << save_line_name_ << "------"
              << std::endl;
  }

  void LaserCloudCornerLastHandler(
      const sensor_msgs::PointCloud2ConstPtr &msg) {
    receive_cloud_now_.add_pc_corner(msg);
    if (receive_cloud_now_.is_completed()) {
      watch_dog_feature_.UpdataNow();
      if (allow_mapping_mode_) receive_clouds_.push(receive_cloud_now_);
      receive_cloud_now_.reset_param();
    }
  }

  // void LaserCloudFullLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
  // {
  //   receive_cloud_now_.add_pc_full(msg);
  //   if (receive_cloud_now_.is_completed()) {
  //     watch_dog_feature_.UpdataNow();
  //     if (allow_mapping_mode_) receive_clouds_.push(receive_cloud_now_);
  //     receive_cloud_now_.reset_param();
  //   }
  // }

  void LaserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg) {
    receive_cloud_now_.add_pc_surf(msg);
    if (receive_cloud_now_.is_completed()) {
      watch_dog_feature_.UpdataNow();
      if (allow_mapping_mode_) receive_clouds_.push(receive_cloud_now_);
      receive_cloud_now_.reset_param();
      std::cout << "receive surf intensity" << std::endl;
    }
  }

  void LaserCloudIntensityLastHandler(
      const sensor_msgs::PointCloud2ConstPtr &msg) {
    receive_cloud_now_.add_pc_intensity(msg);
    if (receive_cloud_now_.is_completed()) {
      watch_dog_feature_.UpdataNow();
      if (allow_mapping_mode_) receive_clouds_.push(receive_cloud_now_);
      receive_cloud_now_.reset_param();
      std::cout << "receive intensity cloud" << std::endl;
    }
  }

  void LaserGpsHandler(const nav_msgs::OdometryConstPtr &input_odom) {
    watch_dog_integrated_navigation_.UpdataNow();
    if (!using_gps_data) return;
    if (allow_mapping_mode_) {
      Gps_data gps_data;
      gps_data.get_datas(input_odom);
      receive_gpss.push_back(gps_data);
    }
  }

  //对点云中的点进行修正
  void pointAssociateToMap(PointType const *const pi, PointType *const po,
                           Eigen::Quaterniond m_q_quater,
                           Eigen::Vector3d m_t_position) {
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w;

    point_w = m_q_quater * point_curr + m_t_position;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
  }

  //将点云修正转换到地图中
  void pointcloudAssociateToMap(pcl::PointCloud<PointType> const &pc_in,
                                pcl::PointCloud<PointType> &pt_out,
                                PointTypePose pose_6d) {
    Eigen::Quaterniond m_q_quater(pose_6d.q_w, pose_6d.q_x, pose_6d.q_y,
                                  pose_6d.q_z);
    Eigen::Vector3d m_t_position(pose_6d.x, pose_6d.y, pose_6d.z);

    unsigned int points_size = pc_in.points.size();
    pt_out.points.resize(points_size);

    for (unsigned int i = 0; i < points_size; i++) {
      pointAssociateToMap(&pc_in.points[i], &pt_out.points[i], m_q_quater,
                          m_t_position);
    }
  }

  //将点云修正转换到地图中
  void pointcloudAssociateToMapEigen(pcl::PointCloud<PointType> &pc_in,
                                     pcl::PointCloud<PointType> &pt_out,
                                     Eigen::Quaterniond m_q_quater,
                                     Eigen::Vector3d m_t_position) {
    unsigned int points_size = pc_in.points.size();
    pt_out.points.resize(points_size);

    for (unsigned int i = 0; i < points_size; i++) {
      pointAssociateToMap(&pc_in.points[i], &pt_out.points[i], m_q_quater,
                          m_t_position);
    }
  }

  void DownSampleCloud() {
    ds_cornerCloudKeyFrame_now->clear();
    saveFilterCorner.setInputCloud(laserCloudCornerLast);
    saveFilterCorner.filter(*ds_cornerCloudKeyFrame_now);

    ds_surfCloudKeyFrame_now->clear();
    saveFilterSurf.setInputCloud(laserCloudSurfLast);
    saveFilterSurf.filter(*ds_surfCloudKeyFrame_now);

    ds_intensity_cloud_key_frame_now_->clear();
    save_filter_intensity_.setInputCloud(laser_cloud_intensity_last_);
    save_filter_intensity_.filter(*ds_intensity_cloud_key_frame_now_);
  }

  //进行匹配
  void GetPLIcp() {
    // std::cout << "plicp running" << std::endl;
    *plicp.m_laser_cloud_corner_from_map =
        *laserCloudCornerFromMapDS;  //载入局部地图
    *plicp.m_laser_cloud_surf_from_map = *laserCloudSurfFromMapDS;
    *plicp.m_laser_cloud_intensity_from_map =
        *laser_cloud_intensity_from_map_ds_;

    plicp.GetPL_ICP(laserCloudCornerLast, laserCloudSurfLast,
                    laser_cloud_intensity_last_);  //载入当前特征点

    m_q_w_curr = plicp.m_q_w_curr;  //得到位姿
    m_t_w_curr = plicp.m_t_w_curr;

    lidar_odom_cov_ = plicp.odom_cov_;
  }

  //拼接地图,得到地图包
  void GetMapBag() {
    double t_start = clock();
    // std::cout << "576" << std::endl;
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    laser_cloud_intensity_from_map_->clear();
    // surroundingKeyPoses->clear();

    //建立局部地图
    int max_key_frame_size = cloudKeyPoses3D->points.size();
    int max_key_frame_size_100 = cloudKeyPoses3D_100->points.size();
    int last_15_frame;
    bool use_100_key_frame = false;
    if (max_key_frame_size < 10) {
      use_100_key_frame = true;
      last_15_frame = max_key_frame_size_100 - 25;

    } else {
      last_15_frame = max_key_frame_size - 25;
      // if (last_15_frame < 0) last_15_frame = 0;
    }
    if (last_15_frame < 0) last_15_frame = 0;

    if (use_100_key_frame) {
      for (int i = last_15_frame; i < max_key_frame_size_100; i++) {
        pcl::PointCloud<PointType> surrounding_corner;
        pcl::PointCloud<PointType> surrounding_surf;
        pcl::PointCloud<PointType> surrounding_intensity;

        pointcloudAssociateToMap(*cornerCloudFrame_100[i], surrounding_corner,
                                 cloudKeyPoses6D_100[i]);
        pointcloudAssociateToMap(*surfCloudFrame_100[i], surrounding_surf,
                                 cloudKeyPoses6D_100[i]);
        pointcloudAssociateToMap(*intensity_cloud_frame_100_[i],
                                 surrounding_intensity, cloudKeyPoses6D_100[i]);
        *laserCloudCornerFromMap += surrounding_corner;
        *laserCloudSurfFromMap += surrounding_surf;
        *laser_cloud_intensity_from_map_ += surrounding_intensity;
      }
    } else {
      for (int i = last_15_frame; i < max_key_frame_size; i++) {
        pcl::PointCloud<PointType> surrounding_corner;
        pcl::PointCloud<PointType> surrounding_surf;
        pcl::PointCloud<PointType> surrounding_intensity;

        pointcloudAssociateToMap(*cornerCloudKeyFrames[i], surrounding_corner,
                                 cloudKeyPoses6D[i]);
        pointcloudAssociateToMap(*surfCloudKeyFrames[i], surrounding_surf,
                                 cloudKeyPoses6D[i]);
        pointcloudAssociateToMap(*intensity_cloud_key_frames_[i],
                                 surrounding_intensity, cloudKeyPoses6D[i]);
        *laserCloudCornerFromMap += surrounding_corner;
        *laserCloudSurfFromMap += surrounding_surf;
        *laser_cloud_intensity_from_map_ += surrounding_intensity;
      }
    }

    std::cout << "time : " << ((double)clock() - t_start) / CLOCKS_PER_SEC
              << std::endl;

    std::cout << "size corner surf intensity : "
              << laserCloudCornerFromMap->points.size() << " "
              << laserCloudSurfFromMap->points.size() << " "
              << laser_cloud_intensity_from_map_->points.size() << std::endl;
    //下采样
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    // std::co
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);

    down_size_filter_intensity_.setInputCloud(laser_cloud_intensity_from_map_);
    down_size_filter_intensity_.filter(*laser_cloud_intensity_from_map_ds_);
    *laser_cloud_corner_map_ds = *laserCloudCornerFromMapDS;
    std::cout << "corner map: " << laser_cloud_corner_map_ds->points.size()
              << std::endl;
  }

  void SavePoseAndCloud() {
    if (cloudKeyPoses3D->points.size() == 0 ||
        (fabs(cloudKeyPoses3D->points[cloudKeyPoses3D->points.size() - 1].x -
              m_t_w_curr.x()) +
         fabs(cloudKeyPoses3D->points[cloudKeyPoses3D->points.size() - 1].y -
              m_t_w_curr.y()) +
         fabs(cloudKeyPoses3D->points[cloudKeyPoses3D->points.size() - 1].z -
              m_t_w_curr.z())) > 0.3 ||
        fabs(euler_angle_last_mapping(0) -
             m_q_w_curr.matrix().eulerAngles(2, 1, 0)(0)) > 0.1 ||
        fabs(euler_angle_last_mapping(1) -
             m_q_w_curr.matrix().eulerAngles(2, 1, 0)(1)) > 0.1 ||
        fabs(euler_angle_last_mapping(2) -
             m_q_w_curr.matrix().eulerAngles(2, 1, 0)(2)) > 0.1) {
      double min_delta_time = 1.0;
      size_t gps_in_key_frame = 0;
      for (size_t i = last_gps_in_key_frame_; i < receive_gpss.size(); i++) {
        if (fabs(receive_gpss[i].time_gps - cloudrealtime) < min_delta_time) {
          gps_in_key_frame = i;
          min_delta_time = fabs(receive_gpss[i].time_gps - cloudrealtime);
        }
        if (receive_gpss[i].time_gps > cloudrealtime) break;
      }
      last_gps_in_key_frame_ = gps_in_key_frame;
      std::cout << "gps size : " << receive_gpss.size() << std::endl;
      // PointType gps_3d_pose;
      gps_key_pose_3d_.push_back(receive_gpss[gps_in_key_frame].position_gps);

      PointType thisPose3D;
      PointTypePose thisPose6D;
      thisPose3D.x = m_t_w_curr.x();
      thisPose3D.y = m_t_w_curr.y();
      thisPose3D.z = m_t_w_curr.z();
      thisPose3D.intensity = cloudKeyPoses3D->points.size();
      cloudKeyPoses3D->push_back(thisPose3D);

      thisPose6D.x = thisPose3D.x;
      thisPose6D.y = thisPose3D.y;
      thisPose6D.z = thisPose3D.z;
      thisPose6D.intensity = thisPose3D.intensity;
      thisPose6D.q_w = m_q_w_curr.w();
      thisPose6D.q_x = m_q_w_curr.x();
      thisPose6D.q_y = m_q_w_curr.y();
      thisPose6D.q_z = m_q_w_curr.z();
      thisPose6D.time =
          (timeLaserCloudCornerLast + timeLaserCloudSurfLast) / 2.0;
      cloudKeyPoses6D.push_back(thisPose6D);
      euler_angle_last_mapping = m_q_w_curr.matrix().eulerAngles(2, 1, 0);

      // std::cout<<"thisPose3D  "<<thisPose3D.x<<" "<<thisPose3D.y<<"
      // "<<thisPose3D.z
      // <<" "<<thisPose3D.intensity<<std::endl;
      // std::cout<<"thisPose6D  "<<thisPose6D.q_w<<" "<<thisPose6D.q_x<<"
      // "<<thisPose6D.q_y<<" "<< thisPose6D.q_z<<std::endl;

      pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(
          new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(
          new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr this_intensity_key_frame(
          new pcl::PointCloud<PointType>());

      pcl::copyPointCloud(*ds_cornerCloudKeyFrame_now, *thisCornerKeyFrame);
      pcl::copyPointCloud(*ds_surfCloudKeyFrame_now, *thisSurfKeyFrame);
      pcl::copyPointCloud(*ds_intensity_cloud_key_frame_now_,
                          *this_intensity_key_frame);

      cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
      surfCloudKeyFrames.push_back(thisSurfKeyFrame);
      intensity_cloud_key_frames_.push_back(this_intensity_key_frame);

      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d T1toT2 = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond quaternion(thisPose6D.q_w, thisPose6D.q_x,
                                    thisPose6D.q_y, thisPose6D.q_z);
      Eigen::Matrix3d rotation_matrix = quaternion.matrix();
      Eigen::Vector3d v_position(thisPose3D.x, thisPose3D.y, thisPose3D.z);
      T.rotate(rotation_matrix);
      T.pretranslate(v_position);

      g2o::VertexSE3 *v = new g2o::VertexSE3();
      v->setId(thisPose3D.intensity);
      g2o_vertex_num_ = thisPose3D.intensity;
      // v->setEstimate(T);
      std::cout << "add vertex" << std::endl;
      // v->setMarginalized(true);//bianyuan
      if (thisPose3D.intensity == 0) {
        v->setEstimate(T);
        v->setFixed(true);
        optimizer.addVertex(v);
      } else {
        v->setEstimate(T);
        optimizer.addVertex(v);
        g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
        edge->vertices()[0] = optimizer.vertex(thisPose3D.intensity - 1);
        edge->vertices()[1] = optimizer.vertex(thisPose3D.intensity);
        std::cout << "intensity : " << thisPose3D.intensity << std::endl;
        double odom_cov_information;

        if (lidar_odom_cov_ < 0.007) {
          odom_cov_information = 0.02;
        } else {
          odom_cov_information = fabs(lidar_odom_cov_ - 0.006) * 10;
        }
        std::cout << "cov : " << odom_cov_information << std::endl;
        Eigen::Matrix<double, 6, 6> information =
            Eigen::Matrix<double, 6, 6>::Identity();

        information(0, 0) = 1.0 / (odom_cov_information * odom_cov_information);
        information(1, 1) = 1.0 / (odom_cov_information * odom_cov_information);
        information(2, 2) = 1.0 / (odom_cov_information * odom_cov_information);
        information(3, 3) = 1.0 / (odom_cov_information * odom_cov_information);
        information(4, 4) = 1.0 / (odom_cov_information * odom_cov_information);
        information(5, 5) = 1.0 / (odom_cov_information * odom_cov_information);
        edge->setInformation(information);

        AINFO<< "odom information : " << information(0, 0);

        T1toT2 = T_last.inverse() * T;
        edge->setMeasurement(T1toT2);
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
      }
      loop_size_ = cloudKeyPoses3D->points.size() - 1;
      T_last = T;
      odom_edge_num_++;

      if (pub_odom.getNumSubscribers() != 0) {
        nav_msgs::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = "camera_init";
        odomAftMapped.pose.pose.orientation.x = m_q_w_curr.x();
        odomAftMapped.pose.pose.orientation.y = m_q_w_curr.y();
        odomAftMapped.pose.pose.orientation.z = m_q_w_curr.z();
        odomAftMapped.pose.pose.orientation.w = m_q_w_curr.w();
        odomAftMapped.pose.pose.position.x = T(0, 3);
        odomAftMapped.pose.pose.position.y = T(1, 3);
        odomAftMapped.pose.pose.position.z = T(2, 3);
        pub_odom.publish(odomAftMapped);
      }
    }
    if (frame_counts < 100) {
      PointType thisPose3D;
      PointTypePose thisPose6D;
      thisPose3D.x = m_t_w_curr.x();
      thisPose3D.y = m_t_w_curr.y();
      thisPose3D.z = m_t_w_curr.z();
      thisPose3D.intensity = cloudKeyPoses3D_100->points.size();  //代表当前帧
      cloudKeyPoses3D_100->push_back(thisPose3D);
      thisPose6D.x = thisPose3D.x;
      thisPose6D.y = thisPose3D.y;
      thisPose6D.z = thisPose3D.z;
      thisPose6D.intensity = thisPose3D.intensity;
      thisPose6D.q_w = m_q_w_curr.w();
      thisPose6D.q_x = m_q_w_curr.x();
      thisPose6D.q_y = m_q_w_curr.y();
      thisPose6D.q_z = m_q_w_curr.z();
      thisPose6D.time = cloudrealtime;
      cloudKeyPoses6D_100.push_back(thisPose6D);

      pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(
          new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(
          new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr this_intensity_key_frame(
          new pcl::PointCloud<PointType>());
      pcl::copyPointCloud(*ds_cornerCloudKeyFrame_now, *thisCornerKeyFrame);
      pcl::copyPointCloud(*ds_surfCloudKeyFrame_now, *thisSurfKeyFrame);
      pcl::copyPointCloud(*ds_intensity_cloud_key_frame_now_,
                          *this_intensity_key_frame);

      cornerCloudFrame_100.push_back(thisCornerKeyFrame);
      surfCloudFrame_100.push_back(thisSurfKeyFrame);
      intensity_cloud_frame_100_.push_back(this_intensity_key_frame);
    }
    currentRobotPosPoint.x = m_t_w_curr.x();
    currentRobotPosPoint.y = m_t_w_curr.y();
    currentRobotPosPoint.z = m_t_w_curr.z();
  }

  void CorrectPoses() {
    g2o_key_poses_6d_.clear();
    std::cout << "g2o optimizer" << std::endl;
    nav_msgs::Path map_path;
    map_path.header.stamp = ros::Time().fromSec(cloudrealtime);
    map_path.header.frame_id = "/camera_init";
    std::cout << "loop size : " << loop_size_ << std::endl;
    for (int i = 0; i < g2o_vertex_num_; ++i) {
      geometry_msgs::PoseStamped CarPose;

      g2o::VertexSE3 *vertex =
          dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
      Eigen::Isometry3d pose = vertex->estimate();  //该帧优化后的位姿
      Eigen::Matrix3d rotation_matrix;              // = pose.block(0, 0, 3, 3);
      for (size_t k = 0; k < 3; k++) {
        for (size_t m = 0; m < 3; m++) {
          rotation_matrix(k, m) = pose(k, m);
        }
      }
      Eigen::Quaterniond quaternion = Eigen::Quaterniond(rotation_matrix);

      Eigen::Vector3d v_position(pose(0, 3), pose(1, 3), pose(2, 3));

      // cloudKeyPoses3D->points[i].x = pose(0, 3);
      // cloudKeyPoses3D->points[i].y = pose(1, 3);
      // cloudKeyPoses3D->points[i].z = pose(2, 3);
      PointTypePose g2o_key_poses_6d;
      g2o_key_poses_6d.x = pose(0, 3);
      g2o_key_poses_6d.y = pose(1, 3);
      g2o_key_poses_6d.z = pose(2, 3);
      g2o_key_poses_6d.q_w = quaternion.w();
      g2o_key_poses_6d.q_x = quaternion.x();
      g2o_key_poses_6d.q_y = quaternion.y();
      g2o_key_poses_6d.q_z = quaternion.z();

      CarPose.pose.position.x = pose(0, 3);
      CarPose.pose.position.y = pose(1, 3);
      CarPose.pose.position.z = pose(2, 3);
      map_path.poses.push_back(CarPose);

      g2o_key_poses_6d_.push_back(g2o_key_poses_6d);
      // last_loop_position_.x() = pose(0, 3);
      // last_loop_position_.y() = pose(1, 3);
      // last_loop_position_.z() = pose(2, 3);
      // if (pub_correct_odom_.getNumSubscribers() != 0) {
      //   nav_msgs::Odometry odomAftMapped;
      //   odomAftMapped.header.frame_id = "camera_init";
      //   odomAftMapped.pose.pose.orientation.x = quaternion.x();
      //   odomAftMapped.pose.pose.orientation.y = quaternion.y();
      //   odomAftMapped.pose.pose.orientation.z = quaternion.z();
      //   odomAftMapped.pose.pose.orientation.w = quaternion.w();
      //   odomAftMapped.pose.pose.position.x = v_position.x();
      //   odomAftMapped.pose.pose.position.y = v_position.y();
      //   odomAftMapped.pose.pose.position.z = v_position.z();
      //   pub_correct_odom_.publish(odomAftMapped);
      // }
    }

    // last_loop_position_ = cloudKeyPoses3D->points[loop_size_- 1];
    pub_path.publish(map_path);
  }

  void ClearCloud() {
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    laser_cloud_intensity_from_map_->clear();

    laserCloudCornerFromMapDS->clear();
    laserCloudSurfFromMapDS->clear();
    laser_cloud_intensity_from_map_ds_->clear();

    laserCloudSurfLast->clear();
    laserCloudCornerLast->clear();
    laser_cloud_intensity_last_->clear();

    ds_surfCloudKeyFrame_now->clear();
    ds_cornerCloudKeyFrame_now->clear();
    ds_intensity_cloud_key_frame_now_->clear();
  }

  // void SaveLineInTxt() {
  //   std::ofstream myfile(save_line_name_, std::ios::app);
  //   Eigen::Vector3d last_key_pose(0.0, 0.0, 0.0);
  //   for (size_t i = 0; i < cloudKeyPoses6D.size(); i++) {
  //     if (sqrt((cloudKeyPoses6D[i].x - last_key_pose.x()) *
  //                  (cloudKeyPoses6D[i].x - last_key_pose.x()) +
  //              (cloudKeyPoses6D[i].y - last_key_pose.y()) *
  //                  (cloudKeyPoses6D[i].y - last_key_pose.y()) +
  //              (cloudKeyPoses6D[i].z - last_key_pose.z()) *
  //                  (cloudKeyPoses6D[i].z - last_key_pose.z())) < 0.2) {
  //       myfile << to_string(cloudKeyPoses6D[i].x) << " "
  //              << to_string(cloudKeyPoses6D[i].y) << " "
  //              << to_string(cloudKeyPoses6D[i].z) << " "
  //              << to_string(cloudKeyPoses6D[i].q_x) << " "
  //              << to_string(cloudKeyPoses6D[i].q_y) << " "
  //              << to_string(cloudKeyPoses6D[i].q_z) << " "
  //              << to_string(cloudKeyPoses6D[i].q_w) << "\n";
  //       last_key_pose.x() = cloudKeyPoses6D[i].x;
  //       last_key_pose.y() = cloudKeyPoses6D[i].y;
  //       last_key_pose.z() = cloudKeyPoses6D[i].z;
  //     }
  //   }
  //   myfile.close();

  //   std::ofstream myfile_maps(
  //       "../sweeper_ws/src/sweeper_haide/data/map/map_collections.txt",
  //       std::ios::app);
  //   myfile_maps << to_string(frist_gps_point_.x) << " "
  //               << to_string(frist_gps_point_.y) << " " << save_pcd_name_
  //               << " "
  //               << save_line_name_ << std::endl;
  //   myfile_maps.close();
  // }

  void SaveLineToVector() {
    for (size_t i = 0; i < g2o_key_poses_6d_.size(); i++) {
      SaveDatas datas_now;
      datas_now.road_name = save_now_pcd_name_;
      datas_now.gnss_x = gps_key_pose_3d_[i].x();
      datas_now.gnss_y = gps_key_pose_3d_[i].y();
      datas_now.lidar_position.x() = g2o_key_poses_6d_[i].x;
      datas_now.lidar_position.y() = g2o_key_poses_6d_[i].y;
      datas_now.lidar_position.z() = g2o_key_poses_6d_[i].z;
      datas_now.lidar_quater.x() = g2o_key_poses_6d_[i].q_x;
      datas_now.lidar_quater.y() = g2o_key_poses_6d_[i].q_y;
      datas_now.lidar_quater.z() = g2o_key_poses_6d_[i].q_z;
      datas_now.lidar_quater.w() = g2o_key_poses_6d_[i].q_w;
      all_need_save_data_.push_back(datas_now);
    }
  }

  void Run() {
    if (receive_clouds_.empty()) return;
    if (receive_gpss.empty()) return;
    mtx.lock();
    Data_pair receive_cloud_last = receive_clouds_.front();
    receive_clouds_.pop();
    mtx.unlock();

    *laserCloudSurfLast = receive_cloud_last.laser_cloud_surf;
    *laserCloudCornerLast = receive_cloud_last.laser_cloud_corner;
    *laser_cloud_intensity_last_ = receive_cloud_last.laser_cloud_intensity;
    // *laser_cloud_full_last_ = receive_cloud_last.laser_cloud_full;

    timeLaserCloudSurfLast = receive_cloud_last.laser_time_surf;
    timeLaserCloudCornerLast = receive_cloud_last.laser_time_corner;
    time_laser_cloud_intensity_last_ = receive_cloud_last.laser_time_intensity;

    if (std::abs(timeLaserCloudCornerLast - timeLaserCloudSurfLast) < 0.005) {
      // std::cout << "1238" << std::endl;
      cloudrealtime = (timeLaserCloudCornerLast + timeLaserCloudSurfLast) / 2.0;
      if (frame_counts <= 10) {
        DownSampleCloud();
        SavePoseAndCloud();
      } else {
        GetMapBag();
        GetPLIcp();
        DownSampleCloud();
        SavePoseAndCloud();
        // CorrectPoses();
        PublishKeyPosesAndFrames();
      }
      ClearCloud();
      frame_counts++;
      // mtx.unlock();
    }
  }

  void VisualizeGlobalMapThread() {
    ros::Rate rate(0.01);
    while (ros::ok()) {
      PublishAndSaveMap();
      rate.sleep();
    }
  }

  void SaveDataToFile(std::string save_line_name, int raw_num_size) {
    std::ofstream myfile(save_line_name.c_str(), std::ios::app);
    Eigen::Vector3d last_key_pose(0.0, 0.0, 0.0);
    for (size_t i = 0; i < all_need_save_data_.size(); i++) {
      if (fabs(last_key_pose.x() - all_need_save_data_[i].lidar_position.x()) +
              fabs(last_key_pose.y() -
                   all_need_save_data_[i].lidar_position.y()) +
              fabs(last_key_pose.z() -
                   all_need_save_data_[i].lidar_position.z()) >
          0.01) {
        std::cout << "path : " << to_string(all_need_save_data_[0].road_name)
                  << std::endl;
        myfile << to_string(all_need_save_data_[i].road_name + raw_num_size)
               << "|" << to_string(all_need_save_data_[i].gnss_x) << "|"
               << to_string(all_need_save_data_[i].gnss_y) << "|"
               << to_string(all_need_save_data_[i].lidar_position.x()) << "|"
               << to_string(all_need_save_data_[i].lidar_position.y()) << "|"
               << to_string(all_need_save_data_[i].lidar_position.z()) << "|"
               << to_string(all_need_save_data_[i].lidar_quater.x()) << "|"
               << to_string(all_need_save_data_[i].lidar_quater.y()) << "|"
               << to_string(all_need_save_data_[i].lidar_quater.z()) << "|"
               << to_string(all_need_save_data_[i].lidar_quater.w()) << "\n";
        last_key_pose.x() = all_need_save_data_[i].lidar_position.x();
        last_key_pose.y() = all_need_save_data_[i].lidar_position.y();
        last_key_pose.z() = all_need_save_data_[i].lidar_position.z();
      }
    }
    myfile.close();
  }

  std::vector<std::string> StringSplit(const std::string &str,
                                       const std::string &delim) {
    std::vector<std::string> res;
    if ("" == str) return res;
    char *strs = new char[str.length() + 1];
    strcpy(strs, str.c_str());

    char *d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());

    char *p = strtok(strs, d);
    while (p) {
      std::string s = p;
      res.push_back(s);
      p = strtok(NULL, d);
    }

    return res;
  }

  void GetLastSaveRoadNum(std::string path_file, int &rode_num) {
    std::fstream fs(path_file, std::ios_base::in);
    std::string line_str;
    while (getline(fs, line_str, '\n')) {
      std::vector<std::string> path_str = StringSplit(line_str, "|");
      rode_num = atoi(path_str[0].c_str());
    }
    rode_num++;  //从0开始，所以++表示大小
  }

  void RenameFile() {
    char rename_file[5000];
    std::string file_name = "../sweeper_ws/src/sweeper_haide/data/path";
    sprintf(rename_file, "%s/%s", file_name.c_str(), save_file_name_.c_str());
    if (!access(rename_file, F_OK)) {
      //读取txt文件，得到参数
      int rode_num_in_txt = 0;
      std::string path_name = file_name + "/" + save_file_name_ + ".txt";

      GetLastSaveRoadNum(path_name, rode_num_in_txt);
      for (int i = 0; i < rode_num_in_txt; i++) {
        std::string old_pcd_name = save_dir_name_ + "/" + to_string(i) + ".pcd";
        std::string new_pcd_name = file_name + "/" + save_file_name_ + "/" +
                                   to_string(rode_num_in_txt + i) + ".pcd";
        if (!rename(old_pcd_name.c_str(), new_pcd_name.c_str()))
          std::cout << "rename ok" << std::endl;
        else
          std::cout << "rename pcd error !" << std::endl;
      }

      SaveDataToFile(path_name, rode_num_in_txt);
      return;
    }
    std::cout << "dir name : " << save_dir_name_ << " " << rename_file
              << std::endl;
    std::cout << "size : " << all_need_save_data_.size() << std::endl;
    ROS_ERROR("1477");
    // std::fstream file;
    // file.open(save_dir_name_.c_str());
    if (!access(save_dir_name_.c_str(), F_OK)) {
      rename(save_dir_name_.c_str(), rename_file);
      std::cout << "rename file ok!" << std::endl;
      // file.close();
    } else {
      std::cout << "rename file error!" << std::endl;
      // file.close();
    }
    SaveDataToFile(save_line_name_, 0);
  }

  void ReceiveMappingDiagnose(
      const sweeper_msgs::SensorFaultInformation mapping_state) {
    // receive_mapping_state_code_.assign(mapping_state.state_code.begin(),
    //                                    mapping_state.state_code.end());
    for (size_t i = 0; i < mapping_state.state_code.size(); i++) {
      receive_mapping_state_code_.push_back(mapping_state.state_code[i]);
    }
  }

  void GetMappingStateCode() {
    sweeper_msgs::SensorFaultInformation mapping_code;
    mapping_code.header.frame_id = "mapping";
    mapping_code.header.stamp = ros::Time::now();

    for (size_t i = 0; i < receive_mapping_state_code_.size(); i++) {
      mapping_code.state_code.push_back(receive_mapping_state_code_[i]);
    }
    receive_mapping_state_code_.clear();

    if (!watch_dog_feature_.DogIsOk(3)) {
      mapping_code.state_code.push_back(no_receive_feature_);
    }
    if (!watch_dog_integrated_navigation_.DogIsOk(3)) {
      mapping_code.state_code.push_back(no_receive_fusion_navigation_);
    }
    if (!make_file_ok_) mapping_code.state_code.push_back(make_file_error_);
    if (!save_pcd_line_ok_) mapping_code.state_code.push_back(save_line_error_);
    if (mapping_code.state_code.empty())
      mapping_code.state_code.push_back(mapping_mode_normal_);
    if (record_line_mode_)
      mapping_code.state_code.push_back(cin_record_line_mode_);
    pub_state_information_.publish(mapping_code);
  }

  void LoopClosureThread() {
    ros::Rate rate(50);
    while (ros::ok()) {
      cloud_loop_frequency++;
      if (cloud_loop_frequency % 5 == 0) GetMappingStateCode();
      if (using_loop_) {
        PerformGpsLoopClosure();
        if (cloud_loop_frequency % 100 == 0)  // 1秒一次
        {
          PerformCloudLoopClosure();
          cloud_loop_frequency = 0;
        }
      }
      dis_edge_ = fabs(last_loop_position_.x() - m_t_w_curr.x()) +
                  fabs(last_loop_position_.y() - m_t_w_curr.y()) +
                  fabs(last_loop_position_.z() - m_t_w_curr.z());
      if (odom_edge_num_ > 20 || gps_edge_num_ > 50 || dis_edge_ > 10.0) {
        StartOptimizeLoop();
        CorrectPoses();
        odom_edge_num_ = 0;
        gps_edge_num_ = 0;
        dis_edge_ = 0;
        last_loop_position_ = m_t_w_curr;
      }

      rate.sleep();
    }
  }

  //计算gps分数
  double gps_data_is_right(Eigen::Isometry3d last_gps_tran_) {
    if (laser_cloud_corner_map_ds->points.size() < 10) return 0;
    Eigen::Matrix4d matrix_tran;
    for (size_t i = 0; i < 4; i++)
      for (size_t j = 0; j < 4; j++) {
        matrix_tran(i, j) = last_gps_tran_(i, j);
      }
    pcl::PointCloud<PointType> tran_cloud;
    pcl::transformPointCloud(*last_gps_position_cloud_, tran_cloud,
                             matrix_tran);

    //计算欧式距离，来估算出评分
    pcl::KdTreeFLANN<pcl::PointXYZI> cloud_kd_tree;
    cloud_kd_tree.setInputCloud(laser_cloud_corner_map_ds);
    double two_cloud_distance = 0.0;
    // std::cout<<"cloud size : "<<input_cloud_size<<std::endl;
    int input_cloud_size = tran_cloud.points.size();
    std::cout << "cloud size : " << input_cloud_size << std::endl;
    for (int i = 0; i < input_cloud_size; i++) {
      int k = 1;
      std::vector<int> point_search(k);
      std::vector<float> point_sqrt_distance(k);
      pcl::PointXYZI search_point = tran_cloud.points[i];

      if (cloud_kd_tree.nearestKSearch(search_point, k, point_search,
                                       point_sqrt_distance) > 0) {
        two_cloud_distance += sqrt(point_sqrt_distance[0]);
      }
    }
    double cloud_points_size = input_cloud_size;
    double two_cloud_distance_average = two_cloud_distance / cloud_points_size;
    std::cout << "average distance : " << two_cloud_distance_average
              << std::endl;

    if (pub_gps_cloud_.getNumSubscribers() != 0) {
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(tran_cloud, cloudMsgTemp);
      cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pub_gps_cloud_.publish(cloudMsgTemp);
    }
    return two_cloud_distance_average;
  }

  // m_q_w_curr表示下一点到上一点的变换
  //后续修改，只读取一次会改变的全局并用锁
  void PerformGpsLoopClosure() {
    if (receive_gpss.size() < 2 || cloudKeyPoses3D->points.empty() ||
        frame_counts < 10)
      return;

    if (cloud_cin_size_ >= cloudKeyPoses6D.size()) return;
    cloud_cin_size_ = cloudKeyPoses6D.size();
    Gps_data gps_last, gps_now;
    double key_point_time = cloudKeyPoses6D[cloud_cin_size_ - 1].time;

    size_t gps_position_in_vector = 0;
    double min_delta_time = 1.0;
    for (size_t i = last_gps_position_in_vector_; i < receive_gpss.size();
         i++) {
      if (fabs(receive_gpss[i].time_gps - key_point_time) < min_delta_time) {
        gps_position_in_vector = i;
        min_delta_time = fabs(receive_gpss[i].time_gps - key_point_time);
      }
      if (receive_gpss[i].time_gps > key_point_time)
        break;  //建图的速度慢于10hz时
    }
    last_gps_position_in_vector_ =
        gps_position_in_vector;  //上一gps在vector中的位置，减少计算量。后续应用deque代替

    if (receive_gpss[gps_position_in_vector].time_gps > key_point_time ||
        gps_position_in_vector >= receive_gpss.size() - 1) {
      gps_last = receive_gpss[gps_position_in_vector - 1];
      gps_now = receive_gpss[gps_position_in_vector];
    } else if (gps_position_in_vector < receive_gpss.size() - 1) {
      gps_last = receive_gpss[gps_position_in_vector];
      gps_now = receive_gpss[gps_position_in_vector + 1];
    }

    if (key_point_time - gps_last.time_gps > 0.1 ||
        gps_now.time_gps - key_point_time > 0.1)
      return;

    if (gps_last.covariance_gps[0] > 0.1 || gps_now.covariance_gps[0] > 0.1 ||
        gps_last.covariance_gps[1] > 0.1 || gps_now.covariance_gps[1] > 0.1 ||
        gps_last.covariance_gps[5] > 20 || gps_now.covariance_gps[5] > 20)
      return;

    double gps_correspond_mapping_covariance[6];
    for (size_t i = 0; i < 6; i++) {
      gps_correspond_mapping_covariance[i] =
          (gps_last.covariance_gps[i] + gps_now.covariance_gps[i]) / 2.0;
    }

    //得到当前点云与姿态，以便判断gps是否符合使用要求
    pcl::PointCloud<PointType> now_corner_cloud =
        *cornerCloudKeyFrames[cloud_cin_size_ - 1];
    Eigen::Quaterniond lidar_quater_now(
        cloudKeyPoses6D[cloud_cin_size_ - 1].q_w,
        cloudKeyPoses6D[cloud_cin_size_ - 1].q_x,
        cloudKeyPoses6D[cloud_cin_size_ - 1].q_y,
        cloudKeyPoses6D[cloud_cin_size_ - 1].q_z);
    Eigen::Vector3d lidar_position_now(cloudKeyPoses6D[cloud_cin_size_ - 1].x,
                                       cloudKeyPoses6D[cloud_cin_size_ - 1].y,
                                       cloudKeyPoses6D[cloud_cin_size_ - 1].z);

    Eigen::Quaterniond gps_quaterniond_in_world = gps_last.quaternion_gps.slerp(
        (key_point_time - gps_last.time_gps) /
            (gps_now.time_gps - gps_last.time_gps),
        gps_now.quaternion_gps);
    Eigen::Vector3d gps_position_in_world =
        gps_last.position_gps +
        (key_point_time - gps_last.time_gps) /
            (gps_now.time_gps - gps_last.time_gps) *
            (gps_now.position_gps - gps_last.position_gps);

    if (is_frist_key_frame_point_) {
      is_frist_key_frame_point_ = false;
      frist_gps_3d_.rotate(gps_quaterniond_in_world.matrix());
      frist_gps_3d_.pretranslate(gps_position_in_world);
      last_gps_position_ = frist_gps_3d_;
      frist_gps_corresponding_lidar_vertex_ = cloud_cin_size_ - 1;
      Eigen::Isometry3d gps_correspond_gps_3d = Eigen::Isometry3d::Identity();
      gps_correspond_gps_3d.rotate(lidar_quater_now.matrix());
      gps_correspond_gps_3d.pretranslate(lidar_position_now);
      gps_to_map_tran_ = gps_correspond_gps_3d * frist_gps_3d_.inverse();
      return;
    }

    //进入g2o添加边
    int gps_correspond_mapping_num = cloud_cin_size_ - 1;
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T1_to_T2 = Eigen::Isometry3d::Identity();
    T1.rotate(gps_quaterniond_in_world.matrix());
    T1.pretranslate(gps_position_in_world);

    T1_to_T2 = gps_to_map_tran_ * T1;
    Eigen::Vector3d gps_in_map_pose(T1_to_T2(0, 3), T1_to_T2(1, 3),
                                    T1_to_T2(2, 3));

    // //计算gps的分数
    // //将点云转到过去的位置
    // pointcloudAssociateToMapEigen(now_corner_cloud,
    // *last_gps_position_cloud_,
    //                               last_odom_quater_in_gps_,
    //                               last_odom_position_in_gps_);
    // Eigen::Isometry3d last_gps_tran_ = last_gps_position_.inverse() * T1;

    // double gps_score = gps_data_is_right(last_gps_tran_);
    // if (gps_score > 2.0) return;
    // //过去的位置
    // last_odom_position_in_gps_ = lidar_position_now;
    // last_odom_quater_in_gps_ = lidar_quater_now;
    // last_gps_position_ = T1;

    std::cout << "gps_position_in_world " << gps_in_map_pose << std::endl;

    g2o::EdgeSE3PriorXYZ *edge(new g2o::EdgeSE3PriorXYZ());
    edge->vertices()[0] = optimizer.vertex(gps_correspond_mapping_num);
    Eigen::Matrix<double, 3, 3> information =
        Eigen::Matrix<double, 3, 3>::Identity();
    information(0, 0) = 1.0 / (gps_correspond_mapping_covariance[0] *
                               gps_correspond_mapping_covariance[0]);
    information(1, 1) = 1.0 / (gps_correspond_mapping_covariance[1] *
                               gps_correspond_mapping_covariance[1]);
    information(2, 2) = 0.01 / (gps_correspond_mapping_covariance[1] *
                                gps_correspond_mapping_covariance[1]);
    edge->setInformation(information);
    edge->setMeasurement(gps_in_map_pose);
    edge->setRobustKernel(new g2o::RobustKernelHuber());
    optimizer.addEdge(edge);
    std::cout << "add gps edge" << std::endl;

    gps_edge_num_++;
    loop_size_ = gps_correspond_mapping_num;
    Eigen::Matrix3d delta_rotation;
    for (size_t i = 0; i < 3; i++)
      for (size_t j = 0; j < 3; j++) {
        delta_rotation(i, j) = T1_to_T2(i, j);
      }
    Eigen::Quaterniond delta_quater(delta_rotation);

    if (pub_gps_position.getNumSubscribers() != 0) {
      nav_msgs::Odometry odomAftMapped;
      odomAftMapped.header.stamp = ros::Time().fromSec(cloudrealtime);
      odomAftMapped.header.frame_id = "camera_init";
      odomAftMapped.pose.pose.orientation.x = delta_quater.x();
      odomAftMapped.pose.pose.orientation.y = delta_quater.y();
      odomAftMapped.pose.pose.orientation.z = delta_quater.z();
      odomAftMapped.pose.pose.orientation.w = delta_quater.w();
      odomAftMapped.pose.pose.position.x = T1_to_T2(0, 3);
      odomAftMapped.pose.pose.position.y = T1_to_T2(1, 3);
      odomAftMapped.pose.pose.position.z = T1_to_T2(2, 3);
      pub_gps_position.publish(odomAftMapped);
    }
  }

  void StartOptimizeLoop() {
    // optimizer.save("../result_before.g2o");
    // double t1 = clock();
    optimizer.initializeOptimization();
    optimizer.optimize(30);  //可以指定优化步数
    // cout << "saving optimization results ..." << endl;
    // optimizer.save("../result_after.g2o");
    // std::cout << "time : " << (double)clock() - t1 << std::endl;
    // cout << "Optimization done." << endl;
    // PublishKeyPosesAndFrames();
  }

  bool DetectCloudLoopClosure() {
    // cloudKeyPoses3D关键帧的位置点
    latestSurfKeyFrameCloud->clear();
    latestSurfKeyFrameCloudDS->clear();
    nearHistorySurfKeyFrameCloud->clear();
    nearHistorySurfKeyFrameCloudDS->clear();

    // 资源分配时初始化
    // 在互斥量被析构前不解锁
    // std::lock_guard<std::mutex> lock(mtx);

    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
    latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
    PointType last_keypose = cloudKeyPoses3D->points[latestFrameIDLoopCloure];
    kdtreeHistoryKeyPoses->radiusSearch(last_keypose, icp_loop_radius,
                                        pointSearchIndLoop,
                                        pointSearchSqDisLoop, 0);
    std::cout << "loop size  :  " << pointSearchIndLoop.size() << std::endl;
    closestHistoryFrameID = -1;
    double max_time = 0.0;
    for (size_t i = 0; i < pointSearchIndLoop.size(); ++i) {
      int id = pointSearchIndLoop[i];
      double time = fabs(cloudKeyPoses6D[id].time -
                         cloudKeyPoses6D[latestFrameIDLoopCloure].time);
      if (max_time < time) max_time = time;
      if (time > 30.0) {
        std::cout << "time is ok!" << std::endl;
        closestHistoryFrameID = id;
        break;
      }
    }
    std::cout << " max time : " << max_time << std::endl;
    if (closestHistoryFrameID == -1) {
      return false;
    }

    pcl::PointCloud<PointType> surrounding_corner;
    pcl::PointCloud<PointType> surrounding_surf;
    pointcloudAssociateToMap(*cornerCloudKeyFrames[latestFrameIDLoopCloure],
                             surrounding_corner,
                             cloudKeyPoses6D[latestFrameIDLoopCloure]);
    pointcloudAssociateToMap(*surfCloudKeyFrames[latestFrameIDLoopCloure],
                             surrounding_surf,
                             cloudKeyPoses6D[latestFrameIDLoopCloure]);

    *latestSurfKeyFrameCloud += surrounding_corner;
    // *latestSurfKeyFrameCloud += surrounding_surf;

    for (int j = -icp_loop_history_search_num; j <= icp_loop_history_search_num;
         ++j) {
      if (closestHistoryFrameID + j < 0 ||
          closestHistoryFrameID + j > latestFrameIDLoopCloure)
        continue;
      pcl::PointCloud<PointType> history_corner;
      pcl::PointCloud<PointType> history_surf;
      pointcloudAssociateToMap(*cornerCloudKeyFrames[closestHistoryFrameID + j],
                               history_corner,
                               cloudKeyPoses6D[closestHistoryFrameID + j]);
      pointcloudAssociateToMap(*surfCloudKeyFrames[closestHistoryFrameID + j],
                               history_corner,
                               cloudKeyPoses6D[closestHistoryFrameID + j]);

      *nearHistorySurfKeyFrameCloud += history_corner;
      // *nearHistorySurfKeyFrameCloud += history_surf;
    }

    downSizeFilterHistoryKeyFrames.setInputCloud(nearHistorySurfKeyFrameCloud);
    downSizeFilterHistoryKeyFrames.filter(*nearHistorySurfKeyFrameCloudDS);

    // saveFilterCorner.setInputCloud(latestSurfKeyFrameCloud);
    // saveFilterCorner.filter(*latestSurfKeyFrameCloudDS);
    *latestSurfKeyFrameCloudDS = *latestSurfKeyFrameCloud;

    return true;
  }

  void PerformCloudLoopClosure() {
    if (cloudKeyPoses3D->points.empty() == true) return;
    if (potentialLoopFlag == false) {
      if (DetectCloudLoopClosure() == true) {
        potentialLoopFlag = true;
      }
      if (potentialLoopFlag == false) return;
    }
    potentialLoopFlag = false;
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(latestSurfKeyFrameCloudDS);
    icp.setInputTarget(nearHistorySurfKeyFrameCloudDS);
    pcl::PointCloud<PointType>::Ptr unused_result(
        new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    std::cout << "icp getFitnessScore" << icp.getFitnessScore() << std::endl;
    if (icp.hasConverged() == false || icp.getFitnessScore() > icp_loop_score)
      return;

    // 以下在点云icp收敛并且噪声量在一定范围内进行
    if (pubIcpKeyFrames.getNumSubscribers() != 0) {
      pcl::PointCloud<PointType>::Ptr closed_cloud(
          new pcl::PointCloud<PointType>());
      // icp.getFinalTransformation()的返回值是Eigen::Matrix<Scalar, 4, 4>
      pcl::transformPointCloud(*latestSurfKeyFrameCloud, *closed_cloud,
                               icp.getFinalTransformation());
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
      cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubIcpKeyFrames.publish(cloudMsgTemp);
    }
    Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
    std::cout << "now : " << latestFrameIDLoopCloure
              << " last : " << closestHistoryFrameID
              << " x, y, z :" << icp_trans(0, 3) << " " << icp_trans(1, 3)
              << " " << icp_trans(2, 3) << std::endl;
    Eigen::Isometry3d T;
    for (size_t i = 0; i < 4; i++)
      for (size_t j = 0; j < 4; j++) {
        T(i, j) = icp_trans(i, j);
      }
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T1toT2 = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q2(cloudKeyPoses6D[latestFrameIDLoopCloure].q_w,
                          cloudKeyPoses6D[latestFrameIDLoopCloure].q_x,
                          cloudKeyPoses6D[latestFrameIDLoopCloure].q_y,
                          cloudKeyPoses6D[latestFrameIDLoopCloure].q_z);
    Eigen::Quaterniond q1(cloudKeyPoses6D[closestHistoryFrameID].q_w,
                          cloudKeyPoses6D[closestHistoryFrameID].q_x,
                          cloudKeyPoses6D[closestHistoryFrameID].q_y,
                          cloudKeyPoses6D[closestHistoryFrameID].q_z);
    T1.rotate(q1.matrix());
    T1.pretranslate(Eigen::Vector3d(cloudKeyPoses6D[closestHistoryFrameID].x,
                                    cloudKeyPoses6D[closestHistoryFrameID].y,
                                    cloudKeyPoses6D[closestHistoryFrameID].z));

    T2.rotate(q2.matrix());//当前帧位置
    T2.pretranslate(
        Eigen::Vector3d(cloudKeyPoses6D[latestFrameIDLoopCloure].x,
                        cloudKeyPoses6D[latestFrameIDLoopCloure].y,
                        cloudKeyPoses6D[latestFrameIDLoopCloure].z));

    T2 = T * T2;  //当前帧的实际位置
    T1toT2 = T1.inverse() * T2;
    loop_size_ = latestFrameIDLoopCloure;
    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
    edge->vertices()[0] = optimizer.vertex(closestHistoryFrameID);
    edge->vertices()[1] = optimizer.vertex(latestFrameIDLoopCloure);
    Eigen::Matrix<double, 6, 6> information =
        Eigen::Matrix<double, 6, 6>::Identity();
    information(0, 0) = information(1, 1) = information(2, 2) =
        1.0 / (icp.getFitnessScore());
    information(3, 3) = information(4, 4) = information(5, 5) =
        1.0 / (icp.getFitnessScore());
    edge->setInformation(information);
    edge->setMeasurement(T1toT2);
    edge->setRobustKernel(new g2o::RobustKernelHuber());
    optimizer.addEdge(edge);
  }

  void PublishAndSaveMap() {
    globalMapKeyFramesDS->clear();
    if (cloudKeyPoses3D->points.empty() == true) return;
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;

    std::vector<PointTypePose> save_pose;
    if (using_loop_)
      save_pose = g2o_key_poses_6d_;
    else {
      save_pose = cloudKeyPoses6D;
    }

    // mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(currentRobotPosPoint, publish_range_map,
                                  pointSearchIndGlobalMap,
                                  pointSearchSqDisGlobalMap, 0);
    // mtx.unlock();
    // OptimizedMatch();
    int save_pose_size = save_pose.size();
    std::cout << "loop size : " << loop_size_ << std::endl;
    for (size_t i = 0; i < pointSearchIndGlobalMap.size(); ++i) {
      if (pointSearchIndGlobalMap[i] >= save_pose_size - 1) continue;
      globalMapKeyPoses->points.push_back(
          cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    }
    std::cout << "key size : " << globalMapKeyPoses->points.size() << std::endl;
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

    pcl::PointCloud<PointType>::Ptr global_corner_map(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr global_surf_map(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr global_intensity_map(
        new pcl::PointCloud<PointType>());

    pcl::PointCloud<PointType>::Ptr global_corner_map_ds(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr global_surf_map_ds(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr global_intensity_map_ds(
        new pcl::PointCloud<PointType>());

    for (size_t i = 0; i < globalMapKeyPosesDS->points.size(); i++) {
      int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
      pcl::PointCloud<PointType> surrounding_corner;
      pcl::PointCloud<PointType> surrounding_surf;
      pcl::PointCloud<PointType> surrounding_intensity;
      pointcloudAssociateToMap(*cornerCloudKeyFrames[thisKeyInd],
                               surrounding_corner, save_pose[thisKeyInd]);
      pointcloudAssociateToMap(*surfCloudKeyFrames[thisKeyInd],
                               surrounding_surf, save_pose[thisKeyInd]);
      pointcloudAssociateToMap(*intensity_cloud_key_frames_[thisKeyInd],
                               surrounding_intensity, save_pose[thisKeyInd]);
      for (size_t j = 0; j < surrounding_corner.points.size(); j++) {
        PointType intensity_point = surrounding_corner.points[j];
        intensity_point.intensity = 8.0;
        global_corner_map->points.push_back(intensity_point);
      }
      for (size_t j = 0; j < surrounding_surf.points.size(); j++) {
        PointType intensity_point = surrounding_surf.points[j];
        intensity_point.intensity = 0.2;
        global_surf_map->points.push_back(intensity_point);
      }
      for (size_t j = 0; j < surrounding_intensity.points.size(); j++) {
        PointType intensity_point = surrounding_intensity.points[j];
        intensity_point.intensity = 20;
        global_intensity_map->points.push_back(intensity_point);
      }
    }
    std::cout << "corner size : " << global_corner_map->points.size()
              << std::endl;
    std::cout << "surf size : " << global_surf_map->points.size() << std::endl;
    std::cout << "intensity size : " << global_intensity_map->points.size()
              << std::endl;
    downSizeFilterHistoryKeyFrames.setInputCloud(global_corner_map);
    downSizeFilterHistoryKeyFrames.filter(*global_corner_map_ds);
    downSizeFilterGlobalMapKeyFrames.setInputCloud(global_surf_map);
    downSizeFilterGlobalMapKeyFrames.filter(*global_surf_map_ds);
    downSizeFilterHistoryKeyFrames.setInputCloud(global_intensity_map);
    downSizeFilterHistoryKeyFrames.filter(*global_intensity_map_ds);

    for (size_t j = 0; j < global_corner_map_ds->points.size(); j++) {
      globalMapKeyFramesDS->points.push_back(global_corner_map_ds->points[j]);
    }
    for (size_t j = 0; j < global_surf_map_ds->points.size(); j++) {
      globalMapKeyFramesDS->points.push_back(global_surf_map_ds->points[j]);
    }
    for (size_t j = 0; j < global_intensity_map_ds->points.size(); j++) {
      globalMapKeyFramesDS->points.push_back(
          global_intensity_map_ds->points[j]);
    }

    globalMapKeyFramesDS->width = 1;
    globalMapKeyFramesDS->height = globalMapKeyFramesDS->points.size();
    std::cout << "map size : " << globalMapKeyFramesDS->points.size()
              << std::endl;

    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*globalMapKeyFramesDS, cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
    cloudMsgTemp.header.frame_id = "/camera_init";
    pubLaserCloudSurround.publish(cloudMsgTemp);

    if (!pcl::io::savePCDFileASCII(save_pcd_name_.c_str(),
                                   *globalMapKeyFramesDS))
      save_pcd_line_ok_ = true;
    else
      save_pcd_line_ok_ = false;
    globalMapKeyPoses->clear();
    globalMapKeyPosesDS->clear();
    globalMapKeyFrames->clear();
    // globalMapKeyFramesDS->clear();
  }

  void PublishKeyPosesAndFrames() {
    if (pubKeyPoses.getNumSubscribers() != 0) {
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
      cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubKeyPoses.publish(cloudMsgTemp);
    }

    if (pubRecentKeyFrames.getNumSubscribers() != 0) {
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*laserCloudCornerFromMapDS, cloudMsgTemp);
      cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubRecentKeyFrames.publish(cloudMsgTemp);
    }
    if (pubFramesnow.getNumSubscribers() != 0) {
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::PointCloud<PointType> now_cloud;
      now_cloud = *(plicp.laserCloudCornerStack);
      pointcloudAssociateToMap(now_cloud, now_cloud,
                               cloudKeyPoses6D[cloudKeyPoses6D.size() - 1]);
      pcl::toROSMsg(now_cloud, cloudMsgTemp);
      cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubFramesnow.publish(cloudMsgTemp);
    }
    // if (frame_counts % 10 == 0) {
    //   if (pub_full_cloud_.getNumSubscribers() != 0) {
    //     sensor_msgs::PointCloud2 cloudMsgTemp;
    //     pcl::PointCloud<PointType> now_cloud;
    //     // now_cloud = *(plicp.laserCloudCornerStack);
    //     // pointcloudAssociateToMap(*laser_cloud_full_last_, now_cloud,
    //     //                          cloudKeyPoses6D[cloudKeyPoses6D.size() -
    //     //                          1]);
    //     pointcloudAssociateToMapEigen(*laser_cloud_full_last_, now_cloud,
    //                                   m_q_w_curr, m_t_w_curr);
    //     pcl::toROSMsg(now_cloud, cloudMsgTemp);
    //     cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
    //     cloudMsgTemp.header.frame_id = "/camera_init";
    //     pub_full_cloud_.publish(cloudMsgTemp);
    //   }
    // }
  }
};

void LogSetting() {
  FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
  FLAGS_stderrthreshold = google::GLOG_INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::GLOG_WARNING;
  FLAGS_v = 1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "loam_new");
  ROS_INFO("\033[1;32m---->\033[0m lassing mapping is Started.");
  google::InitGoogleLogging(argv[0]);
  LogSetting();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  LaserMappingNew LMN(nh, nh_private);
  std::thread loopthread(&LaserMappingNew::LoopClosureThread, &LMN);
  // std::thread visualizeMapThread(&LaserMappingNew::VisualizeGlobalMapThread,
  //  &LMN);

  ros::Rate rate(200);
  while (ros::ok()) {
    ros::spinOnce();

    LMN.Run();

    rate.sleep();
  }
  LMN.optimizer.save("../result_before.g2o");
  // LMN.SaveLineInTxt();
  loopthread.join();
  // visualizeMapThread.join();

  LMN.optimizer.clear();
  // LMN.SaveLineInTxt();
  return 0;
}

// #include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include <vector>
#include "plane_line_icp.hpp"
// #include "tools/common.h"
// #include "tools/logger.hpp"
// #include "tools/pcl_tools.hpp"
#include <pcl/registration/icp.h>
#include <fstream>
#include <time.h>

//g2o模块
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

struct PointXYZIRPYT
{
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


struct Data_pair
{
    pcl::PointCloud<PointType> laser_cloud_corner;
    pcl::PointCloud<PointType> laser_cloud_surf;
    double laser_time_corner;
    double laser_time_surf;

    bool m_has_pc_corner = 0;
    bool m_has_pc_surf = 0;

    void add_pc_corner(sensor_msgs::PointCloud2ConstPtr ros_pc)
    {
        // m_pc_corner = ros_pc;
        laser_time_corner = ros_pc->header.stamp.toSec();
        pcl::fromROSMsg(*ros_pc, laser_cloud_corner);
        m_has_pc_corner = true;
    }

    void add_pc_surf(sensor_msgs::PointCloud2ConstPtr ros_pc)
    {
        laser_time_surf = ros_pc->header.stamp.toSec();
        pcl::fromROSMsg(*ros_pc, laser_cloud_surf);
        m_has_pc_surf = true;
    }

    bool is_completed()
    {
        return (m_has_pc_corner && m_has_pc_surf);
    }
    void reset_param()
    {
        m_has_pc_corner = false;
        m_has_pc_surf = false;
    }
};

void CorrectionGps(Eigen::Quaterniond &gps_quaterniond, Eigen::Vector3d &gps_position)
{
    Eigen::Vector3d correction_param_ea(-3.1415926 / 2.0, 0, 0);
    // Eigen::Quaterniond correction_param_quaterniond(0.998756535673134, 0, 0, 0.049853610202274);
    Eigen::Vector3d correction_param_ea_quater(0.100993827432253 + 0.0034, 0, 0);
    Eigen::Quaterniond correction_param_quaterniond =   Eigen::AngleAxisd(correction_param_ea_quater[0], Eigen::Vector3d::UnitZ()) *
                                                        Eigen::AngleAxisd(correction_param_ea_quater[1], Eigen::Vector3d::UnitY()) *
                                                        Eigen::AngleAxisd(correction_param_ea_quater[2], Eigen::Vector3d::UnitX());
    // Eigen::Quaterniond correction_param_quaterniond(0.998756535673134, 0, 0, 0.049853610202274);
    Eigen::Quaterniond correction_param_90 = Eigen::AngleAxisd(correction_param_ea[0], Eigen::Vector3d::UnitZ()) *
                                             Eigen::AngleAxisd(correction_param_ea[1], Eigen::Vector3d::UnitY()) *
                                             Eigen::AngleAxisd(correction_param_ea[2], Eigen::Vector3d::UnitX());
    // Eigen::Vector3d correction_param_position(0.604715009798085, 0.45070539187565, 0);
    Eigen::Vector3d correction_param_position(0.604715009798085 + 0.0283502293565214, 0.45070539187565 - 0.012033781989624, 0);

    gps_position = gps_quaterniond * correction_param_position + correction_param_90 * gps_position;
    gps_quaterniond = gps_quaterniond * correction_param_quaterniond;
}

struct Gps_data
{
    double time_gps;
    Eigen::Quaterniond quaternion_gps;
    Eigen::Vector3d position_gps;
    double covariance_gps[6];
    void get_datas(const nav_msgs::OdometryConstPtr input_odom)
    {
        time_gps = input_odom->header.stamp.toSec();
        quaternion_gps = Eigen::Quaterniond(input_odom->pose.pose.orientation.w, input_odom->pose.pose.orientation.x,
                                            input_odom->pose.pose.orientation.y, input_odom->pose.pose.orientation.z);
        position_gps = Eigen::Vector3d(input_odom->pose.pose.position.x, input_odom->pose.pose.position.y, input_odom->pose.pose.position.z);
        // CorrectionGps(quaternion_gps, position_gps);
        covariance_gps[0] = input_odom->pose.covariance[0];
        covariance_gps[1] = input_odom->pose.covariance[7];
        covariance_gps[2] = input_odom->pose.covariance[14];
        covariance_gps[3] = input_odom->pose.covariance[21];
        covariance_gps[4] = input_odom->pose.covariance[28];
        covariance_gps[5] = input_odom->pose.covariance[35];
    }
};

struct FusionOdom
{
    double time_fusion_odom;
    Eigen::Quaterniond quaternion_fusion;
    Eigen::Vector3d position_fusion;
    // double covariance_gps[6];
    void get_datas(const nav_msgs::Odometry input_odom)
    {
        time_fusion_odom = input_odom.header.stamp.toSec();
        quaternion_fusion = Eigen::Quaterniond(input_odom.pose.pose.orientation.w, input_odom.pose.pose.orientation.x,
                                            input_odom.pose.pose.orientation.y, input_odom.pose.pose.orientation.z);
        position_fusion = Eigen::Vector3d(input_odom.pose.pose.position.x, input_odom.pose.pose.position.y, input_odom.pose.pose.position.z);
    }
};

class LaserMappingNew
{
public:
    std::queue<Data_pair> receive_clouds;
    std::vector<Gps_data> receive_gpss;
    std::vector<FusionOdom> receive_odoms;
    Data_pair receive_cloud_now;
    double timeLaserCloudCornerLast;
    double timeLaserCloudSurfLast;
    double time_gps_now;
    double time_gps_last;
    double cloudrealtime;

    double timeLaserCloudOutlierLast;
    double timeLastGloalMapPublish;

    bool newLaserCloudCornerLast;
    bool newLaserCloudSurfLast;
    bool newLaserGps = false;
    bool newLaserOdometry;
    bool newLaserCloudOutlierLast;

    bool frist_localization_in_map = true;
    bool potentialLoopFlag;
    bool aLoopIsClosed;

    bool flag = true;
    int saved_frist_gps_and_odom = 0;

    int laserCloudCornerFromMapDSNum;
    int laserCloudSurfFromMapDSNum;
    int laserCloudCornerLastDSNum;
    int laserCloudSurfLastDSNum;
    int laserCloudOutlierLastDSNum;
    int laserCloudSurfTotalLastDSNum;
    int frist_localization_lidar_to_gps = 0;
    int closestHistoryFrameID;
    int latestFrameIDLoopCloure;
    size_t cloud_cin_size = 0;
    int loop_size = 0;
    int cloud_loop_frequency = 0;

    double plicp_search_radius = 50.0;
    // float icp_loop_radius = 8.0;
    // float icp_loop_score = 0.2;
    int using_loop_ = 0;
    int add_gps_edge = 0;
    int using_gps_data = 0;
    int save_odom_path = 0;
    int save_gps_path = 0;
    int save_gps_correspond_odom_data = 0;
    double gps_max_cov = 0.12;
    double publish_range_map = 500.0;
    double icp_loop_history_search_num = 10;
    double icp_loop_radius = 8.0;
    double icp_loop_score = 0.2;

    int calibration_times = 0;

    std::mutex mtx;

    g2o::SparseOptimizer optimizer;

    Plane_Line_ICP plicp;
    // plicp.m_para_icp_max_iterations = 10;
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D_100;

    std::vector<PointTypePose> cloudKeyPoses6D;
    std::vector<PointTypePose> cloudKeyPoses6D_100;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    //车开刚刚准备开时，保存一个局部地图
    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudFrame_100;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudFrame_100;

    pcl::PointCloud<PointType>::Ptr cornerCloudKeyFrame_now;
    pcl::PointCloud<PointType>::Ptr surfCloudKeyFrame_now;

    pcl::PointCloud<PointType>::Ptr ds_cornerCloudKeyFrame_now;
    pcl::PointCloud<PointType>::Ptr ds_surfCloudKeyFrame_now;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;

    pcl::VoxelGrid<PointType> saveFilterCorner;
    pcl::VoxelGrid<PointType> saveFilterSurf;

    // pcl::VoxelGrid<PointType> downSizeFilterOutlier;
    pcl::VoxelGrid<PointType> downSizeFilterHistoryKeyFrames;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    // 结尾有DS代表是downsize,进行过下采样
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses;
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS;
    //局部地图
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
    // pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;

    pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

    pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloudDS;
    std::vector<pcl::PointCloud<PointType>> correct_gps_cloud;

    int frame_counts = 0;
    int frame_counts_last = 0;

    PointType currentRobotPosPoint;
    Eigen::Quaterniond m_q_w_curr;
    Eigen::Vector3d m_t_w_curr;

    Eigen::Quaterniond m_q_w_last; //上一关键帧的位姿
    Eigen::Vector3d m_t_w_last;

    Eigen::Quaterniond gps_q_last; //上一关键帧的位姿
    Eigen::Vector3d gps_t_last;

    Eigen::Quaterniond gps_q_now;
    Eigen::Vector3d gps_t_now;

    Eigen::Quaterniond frist_gps_quaterniond;
    Eigen::Vector3d frist_gps_position;
    Eigen::Quaterniond frist_gps_correspond_odom_quaterniond;
    Eigen::Vector3d frist_gps_correspond_odom_position;
    int frist_gps_correspond_odom_num = 0;

    Eigen::Isometry3d T_last = Eigen::Isometry3d::Identity();

    Eigen::Isometry3d t_gps_frist = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d t_lidar_in_map_correspond_gps = Eigen::Isometry3d::Identity();

    Eigen::Vector3d euler_angle_last_mapping;
    Eigen::Vector3d last_save_position;
    Eigen::Vector3d last_g2o_position;

    Eigen::Vector3d covariance_t = Eigen::Vector3d(2.0, 1.0, 0.1);

    double last_lidar_position[3] = {0};
    Eigen::Quaterniond last_quate_ = Eigen::Quaterniond(1.0, 0, 0, 0);


    // double gps_covariance[6] = {0};
    ros::NodeHandle nh;

    ros::Subscriber sub_lasercloud_corner;
    ros::Subscriber sub_lasercloud_surf;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_odom_fusion_sweeper_imu;

    ros::Publisher pub_path;
    ros::Publisher pub_odom;
    ros::Publisher pub_fusion_odom;
    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubFramesnow;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pub_gps_position;

    LaserMappingNew()
    {
        ParamInit(nh);
        sub_lasercloud_corner = nh.subscribe<sensor_msgs::PointCloud2>("/pc2_corners", 10000, &LaserMappingNew::LaserCloudCornerLastHandler, this);
        sub_lasercloud_surf = nh.subscribe<sensor_msgs::PointCloud2>("/pc2_surface", 10000, &LaserMappingNew::LaserCloudSurfLastHandler, this);
        sub_gps = nh.subscribe<nav_msgs::Odometry>("/sweeper/sensor/gnss", 10000, &LaserMappingNew::LaserGpsHandler, this);
        sub_odom_fusion_sweeper_imu = nh.subscribe<nav_msgs::Odometry>("/sweeper/localization/sweeper_odom", 10000, &LaserMappingNew::GetFusionImuSweeperOdomData, this);

        pub_path = nh.advertise<nav_msgs::Path>("/loam/path", 1);

        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/recent_cloud", 2);
        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);

        pubFramesnow = nh.advertise<sensor_msgs::PointCloud2>("/now_cloud", 2);
        pub_odom = nh.advertise<nav_msgs::Odometry>("/pub_odom", 1);
        pub_fusion_odom = nh.advertise<nav_msgs::Odometry>("/pub_fusion_odom", 1);
        pub_gps_position = nh.advertise<nav_msgs::Odometry>("/gps_position", 1);

        pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/corrected_cloud", 2);
        downSizeFilterCorner.setLeafSize(0.4, 0.4, 0.4);
        downSizeFilterSurf.setLeafSize(1.0, 1.0, 1.0);

        saveFilterSurf.setLeafSize(0.4, 0.4, 0.4);
        saveFilterCorner.setLeafSize(0.4, 0.4, 0.4);

        downSizeFilterHistoryKeyFrames.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilterSurroundingKeyPoses.setLeafSize(0.01, 0.01, 0.01);

        downSizeFilterGlobalMapKeyPoses.setLeafSize(0.4, 0.4, 0.4);
        downSizeFilterGlobalMapKeyFrames.setLeafSize(0.2, 0.2, 0.2);

        kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
        globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
        globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
        globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
        // globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());

        timeLaserCloudCornerLast = 0;
        timeLaserCloudSurfLast = 0;
        timeLaserCloudOutlierLast = 0;
        timeLastGloalMapPublish = 0;

        newLaserCloudCornerLast = false;
        newLaserCloudSurfLast = false;

        newLaserOdometry = false;
        newLaserCloudOutlierLast = false;

        // ResetParame();
    }

    void ResetParame()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());

        cloudKeyPoses3D_100.reset(new pcl::PointCloud<PointType>());
        // cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        // cloudKeyPoses6D_100.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
        surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
        globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
        globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
        globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
        // globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());

        ds_cornerCloudKeyFrame_now.reset(new pcl::PointCloud<PointType>());
        ds_surfCloudKeyFrame_now.reset(new pcl::PointCloud<PointType>());

        latestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        nearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        nearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        potentialLoopFlag = false;

        m_q_w_curr.w() = 1.0;
        m_q_w_curr.x() = 0.0;
        m_q_w_curr.y() = 0.0;
        m_q_w_curr.z() = 0.0;
        m_t_w_curr = Eigen::Vector3d(0.0, 0.0, 0.0);

        m_t_w_last = Eigen::Vector3d(0.0, 0.0, 0.0);
        m_q_w_last = m_q_w_curr;
        frist_gps_correspond_odom_quaterniond = m_q_w_curr;
        frist_gps_correspond_odom_position = m_t_w_last;

        gps_q_now = m_q_w_curr;
        gps_q_last = m_q_w_curr;

        gps_t_last = Eigen::Vector3d(0.0, 0.0, 0.0);
        gps_t_now = Eigen::Vector3d(0.0, 0.0, 0.0);
        last_save_position = gps_t_now;
        last_g2o_position = gps_t_now;
        euler_angle_last_mapping = m_q_w_last.matrix().eulerAngles(2, 1, 0);

        //构建图优化
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
        std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverCholmod<Block::PoseMatrixType>());
        std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
        //选择梯度下降方法
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);
        plicp.m_para_icp_max_iterations = 20;
        
    }
    void ParamInit(ros::NodeHandle &nh)
    {
        nh.param<int>("using_loop",                         using_loop_,        1);
        nh.param<int>("add_gps_edge",                       add_gps_edge,       1);
        nh.param<int>("using_gps_data",                     using_gps_data,     1);
        nh.param<int>("save_odom_path",                     save_odom_path,     1);
        nh.param<int>("save_gps_path",                      save_gps_path,      1);
        nh.param<int>("save_gps_correspond_odom_data",      save_gps_correspond_odom_data, 1);
        nh.param<double>("gps_max_cov",                     gps_max_cov,        0.12);
        nh.param<double>("publish_range_map",               publish_range_map,   500.0);
        nh.param<double>("plicp_search_radius",             plicp_search_radius, 50.0);
        nh.param<double>("icp_loop_history_search_num",     icp_loop_history_search_num, 10.0);
        nh.param<double>("icp_loop_radius",                 icp_loop_radius,    8.0);
        nh.param<double>("icp_loop_score",                  icp_loop_score,     0.2);

        ResetParame();
        std::cout<<"receive param success !"<<std::endl;
        if(using_loop_)std::cout<<"receive ok !"<<std::endl;

    }

    void LaserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        if(!using_loop_) 
        {
            std::cout<<"don't use loop"<<std::endl;
            // return;
        }
        receive_cloud_now.add_pc_corner(msg);
        if (receive_cloud_now.is_completed())
        {
            receive_clouds.push(receive_cloud_now);
            receive_cloud_now.reset_param();
            std::cout << "laser mapping 411" << std::endl;
        }
    }

    void LaserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        receive_cloud_now.add_pc_surf(msg);
        if (receive_cloud_now.is_completed())
        {
            receive_clouds.push(receive_cloud_now);
            receive_cloud_now.reset_param();
            std::cout << "laser mapping 427" << std::endl;
        }
    }

    void LaserGpsHandler(const nav_msgs::OdometryConstPtr &input_odom)
    {
        // if(!using_gps_data) return;
        if (input_odom->pose.covariance[0] > gps_max_cov || input_odom->pose.covariance[7] > gps_max_cov)
            return;
        Gps_data gps_data;
        gps_data.get_datas(input_odom);
        receive_gpss.push_back(gps_data);
    }


    void GetFusionImuSweeperOdomData(const nav_msgs::Odometry odom)
    {
        FusionOdom fusion_odom;
        fusion_odom.get_datas(odom);
        receive_odoms.push_back(fusion_odom);
    }

    void SaveFusionImuAndLidarOdom()
    {
        std::cout<<"start save file"<<std::endl;
        if(receive_odoms.size() > 0)
        {
            double time = 10.0;
            int times = 0;
            for(size_t i = calibration_times; i < receive_odoms.size(); i++)
            {
                if(fabs(receive_odoms[i].time_fusion_odom - cloudrealtime) < time)
                {
                    time = fabs(receive_odoms[i].time_fusion_odom - cloudrealtime);
                    times = i;
                }
            }
            double car_move_distance = 0.0;
            //对角度求平均
            // Eigen::Quaterniond car_average_angle = receive_odoms[calibration_times].quaternion_fusion.slerp(0.5, receive_odoms[times].quaternion_fusion);
            Eigen::Quaterniond car_average_angle = receive_odoms[times].quaternion_fusion;
            Eigen::Vector3d car_odom = receive_odoms[times].position_fusion;
            for(int i = times; i > calibration_times; i--)
            {
                car_move_distance += receive_odoms[i].position_fusion.x();//这里x表示的就是位移
            }
            calibration_times = times;

            Eigen::Quaterniond lidar_average_quate = m_q_w_curr;//.slerp(0.5, last_quate_);

            std::cout<<"get times"<<std::endl;
            std::cout<<"time : "<<receive_odoms[calibration_times].time_fusion_odom - cloudrealtime<<std::endl;
            std::ofstream myfile("../path/calibration_fusion_odom.txt", std::ios::app);
            // std::cout<<"824"<<std::endl;
            myfile << to_string(m_t_w_curr.x()) << " " << to_string(m_t_w_curr.y()) << " " << to_string(m_t_w_curr.z())
                    << " " << to_string(lidar_average_quate.x()) << " " << to_string(lidar_average_quate.y()) << " " << to_string(lidar_average_quate.z())
                    << " " << to_string(lidar_average_quate.w()) << " " << to_string(cloudrealtime) << "\n"
                    << to_string(car_odom.x())  << " " << to_string(car_odom.y()) << " " << to_string(car_odom.z())
                    << " " << to_string(car_average_angle.x()) << " " << to_string(car_average_angle.y()) << " " << to_string(car_average_angle.z())
                    << " " << to_string(car_average_angle.w()) << " " << to_string(receive_odoms[calibration_times].time_fusion_odom) << "\n";
            myfile.close();

            std::cout << "save the file" << std::endl;

            nav_msgs::Odometry odomAftMapped;
            odomAftMapped.header.frame_id = "camera_init";
            odomAftMapped.pose.pose.orientation.x = receive_odoms[calibration_times].quaternion_fusion.x();
            odomAftMapped.pose.pose.orientation.y = receive_odoms[calibration_times].quaternion_fusion.y();
            odomAftMapped.pose.pose.orientation.z = receive_odoms[calibration_times].quaternion_fusion.z();
            odomAftMapped.pose.pose.orientation.w = receive_odoms[calibration_times].quaternion_fusion.w();
            odomAftMapped.pose.pose.position.x = receive_odoms[calibration_times].position_fusion.x();
            odomAftMapped.pose.pose.position.y = receive_odoms[calibration_times].position_fusion.y();
            odomAftMapped.pose.pose.position.z = receive_odoms[calibration_times].position_fusion.z();
            pub_fusion_odom.publish(odomAftMapped);

            // last_lidar_position[0] = 

        }
        last_lidar_position[0] = m_t_w_curr.x();
        last_lidar_position[1] = m_t_w_curr.y();
        last_lidar_position[2] = m_t_w_curr.z();

        last_quate_ = m_q_w_curr;

    }
    //对点云中的点进行修正
    void pointAssociateToMap(PointType const *const pi, PointType *const po, Eigen::Quaterniond m_q_quater, Eigen::Vector3d m_t_position)
    {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w;

        point_w = m_q_quater * point_curr + m_t_position;
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

    //将点云修正转换到地图中
    void pointcloudAssociateToMap(pcl::PointCloud<PointType> const &pc_in, pcl::PointCloud<PointType> &pt_out, PointTypePose pose_6d)
    {
        Eigen::Quaterniond m_q_quater(pose_6d.q_w, pose_6d.q_x, pose_6d.q_y, pose_6d.q_z);
        Eigen::Vector3d m_t_position(pose_6d.x, pose_6d.y, pose_6d.z);

        unsigned int points_size = pc_in.points.size();
        pt_out.points.resize(points_size);

        for (unsigned int i = 0; i < points_size; i++)
        {
            pointAssociateToMap(&pc_in.points[i], &pt_out.points[i], m_q_quater, m_t_position);
        }
    }

    void DownSampleCloud()
    {
        ds_cornerCloudKeyFrame_now->clear();
        saveFilterCorner.setInputCloud(laserCloudCornerLast);
        saveFilterCorner.filter(*ds_cornerCloudKeyFrame_now);
        // laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();

        ds_surfCloudKeyFrame_now->clear();
        saveFilterSurf.setInputCloud(laserCloudSurfLast);
        saveFilterSurf.filter(*ds_surfCloudKeyFrame_now);
        // laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();
    }

    //进行匹配
    void GetPLIcp()
    {
        // std::cout << "laserCloudCornerFromMapDS size " << laserCloudCornerFromMapDS->points.size() << std::endl;
        // std::cout << "laserCloudSurfFromMapDS size " << laserCloudSurfFromMapDS->points.size() << std::endl;
        *plicp.m_laser_cloud_corner_from_map = *laserCloudCornerFromMapDS; //载入局部地图
        *plicp.m_laser_cloud_surf_from_map = *laserCloudSurfFromMapDS;
        // std::cout << "corner size" << laserCloudCornerLast->points.size() << std::endl;
        // std::cout << "surf size  " << laserCloudSurfLast->points.size() << std::endl;
        plicp.GetPL_ICP(laserCloudCornerLast, laserCloudSurfLast); //载入当前特征点

        m_q_w_curr = plicp.m_q_w_curr; //得到位姿
        m_t_w_curr = plicp.m_t_w_curr;

        SaveFusionImuAndLidarOdom();
        
    }


    //拼接地图,得到地图包
    void GetMapBag()
    {
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        surroundingKeyPoses->clear();
        std::vector<float> pointSearchSqDis;
        std::vector<int> pointSearchInd;
        bool use_key_frame = false;
        //关键帧
        if (frame_counts > 50 && cloudKeyPoses3D->points.size() > 10)
        {
            kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
            kdtreeSurroundingKeyPoses->radiusSearch(currentRobotPosPoint, (double)plicp_search_radius, pointSearchInd, pointSearchSqDis, 0);
            //找当前位置附近的点云
            for (size_t i = 0; i < pointSearchInd.size(); ++i)
                surroundingKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchInd[i]]);
            //对位姿点进行下采样
            downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
            downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
            if (surroundingKeyPosesDS->points.size() < 10)
                *surroundingKeyPosesDS = *surroundingKeyPoses;
            use_key_frame = true;
        }
        else //如果没达到关键帧要求，则采取当前帧的前15帧
        {
            for (int i = 0; i < 15; i++)
            {
                int pose_size = cloudKeyPoses3D->points.size();

                if (i > pose_size)
                    break;

                surroundingKeyPosesDS->points.push_back(cloudKeyPoses3D_100->points[pose_size - i]);
            }
        }

        // std::cout << "size " << surroundingKeyPosesDS->points.size() << std::endl;
        // if(surroundingKeyPosesDS->points.size() < 15) *surroundingKeyPosesDS = *surroundingKeyPoses;
        //建立局部地图
        int numSurroundingPosesDS = surroundingKeyPosesDS->points.size();

        for (int i = 0; i < numSurroundingPosesDS; i++)
        {
            int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
            // std::cout<<"key "<<thisKeyInd<<std::endl;
            pcl::PointCloud<PointType> surrounding_corner;
            pcl::PointCloud<PointType> surrounding_surf;
            // std::cout<<"453"<<std::endl;
            if (use_key_frame)
            {
                pointcloudAssociateToMap(*cornerCloudKeyFrames[thisKeyInd], surrounding_corner, cloudKeyPoses6D[thisKeyInd]);
                pointcloudAssociateToMap(*surfCloudKeyFrames[thisKeyInd], surrounding_surf, cloudKeyPoses6D[thisKeyInd]);
            }
            else
            {
                pointcloudAssociateToMap(*cornerCloudFrame_100[thisKeyInd], surrounding_corner, cloudKeyPoses6D_100[thisKeyInd]);
                pointcloudAssociateToMap(*surfCloudFrame_100[thisKeyInd], surrounding_surf, cloudKeyPoses6D_100[thisKeyInd]);
            }
            // std::cout<<"cornerCloudKeyFrames.size "<<cornerCloudKeyFrames[thisKeyInd]->points.size()<<std::endl;
            // std::cout<<"surrounding_corner.size "<<surrounding_corner.points.size()<<std::endl;
            *laserCloudCornerFromMap += surrounding_corner;
            *laserCloudSurfFromMap += surrounding_surf;
            // std::cout<<"corner "<< laserCloudCornerFromMap->points.size()<<std::endl;
            // std::cout<<"corner "<< laserCloudSurfFromMap->points.size()<<std::endl;
        }

        //下采样
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();

        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();
    }

    void SavePoseAndCloud()
    {
        if (cloudKeyPoses3D->points.size() == 0 || sqrt((cloudKeyPoses3D->points[cloudKeyPoses3D->points.size() - 1].x - m_t_w_curr.x()) * (cloudKeyPoses3D->points[cloudKeyPoses3D->points.size() - 1].x - m_t_w_curr.x()) + (cloudKeyPoses3D->points[cloudKeyPoses3D->points.size() - 1].y - m_t_w_curr.y()) * (cloudKeyPoses3D->points[cloudKeyPoses3D->points.size() - 1].y - m_t_w_curr.y()) + 
        (cloudKeyPoses3D->points[cloudKeyPoses3D->points.size() - 1].z - m_t_w_curr.z()) * (cloudKeyPoses3D->points[cloudKeyPoses3D->points.size() - 1].z - m_t_w_curr.z())) > 0.2 || fabs(euler_angle_last_mapping(0) - m_q_w_curr.matrix().eulerAngles(2, 1, 0)(0)) > 0.05 || fabs(euler_angle_last_mapping(1) - m_q_w_curr.matrix().eulerAngles(2, 1, 0)(1)) > 0.05 || fabs(euler_angle_last_mapping(2) - m_q_w_curr.matrix().eulerAngles(2, 1, 0)(2)) > 0.05)
        {
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
            thisPose6D.time = (timeLaserCloudCornerLast + timeLaserCloudSurfLast) / 2.0;
            cloudKeyPoses6D.push_back(thisPose6D);
            euler_angle_last_mapping = m_q_w_curr.matrix().eulerAngles(2, 1, 0);

            // std::cout<<"thisPose3D  "<<thisPose3D.x<<" "<<thisPose3D.y<<" "<<thisPose3D.z
            // <<" "<<thisPose3D.intensity<<std::endl;
            // std::cout<<"thisPose6D  "<<thisPose6D.q_w<<" "<<thisPose6D.q_x<<" "<<thisPose6D.q_y<<" "<<
            // thisPose6D.q_z<<std::endl;

            pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*ds_cornerCloudKeyFrame_now, *thisCornerKeyFrame);
            pcl::copyPointCloud(*ds_surfCloudKeyFrame_now, *thisSurfKeyFrame);

            cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
            surfCloudKeyFrames.push_back(thisSurfKeyFrame);

            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            Eigen::Isometry3d T1toT2 = Eigen::Isometry3d::Identity();
            Eigen::Quaterniond quaternion(thisPose6D.q_w, thisPose6D.q_x, thisPose6D.q_y, thisPose6D.q_z);
            Eigen::Matrix3d rotation_matrix = quaternion.matrix();
            Eigen::Vector3d v_position(thisPose3D.x, thisPose3D.y, thisPose3D.z);
            T.rotate(rotation_matrix);
            T.pretranslate(v_position);

            g2o::VertexSE3 *v = new g2o::VertexSE3();
            v->setId(thisPose3D.intensity);
            std::cout << "add vertex" << std::endl;
            if (thisPose3D.intensity == 0)
            {
                v->setEstimate(Eigen::Isometry3d::Identity());
                v->setFixed(true);
                optimizer.addVertex(v);
            }
            else
            {
                v->setEstimate(T);
                optimizer.addVertex(v);
                g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
                edge->vertices()[0] = optimizer.vertex(thisPose3D.intensity - 1);
                edge->vertices()[1] = optimizer.vertex(thisPose3D.intensity);

                Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
                // information(0, 0) = 100.0 / (covariance_t[0] * covariance_t[0]);
                // information(1, 1) = 100.0 / (covariance_t[1] * covariance_t[1]);
                // information(2, 2) = 1.0 / (covariance_t[2] * covariance_t[2]);
                information(0, 0) = 100000.0;
                information(1, 1) = 100000.0;
                information(2, 2) = 1000000.0;
                information(3, 3) = 100000;
                information(4, 4) = 100000;
                information(5, 5) = 100000;
                edge->setInformation(information);

                T1toT2 = T_last.inverse() * T ;
                edge->setMeasurement(T1toT2);
                edge->setRobustKernel(new g2o::RobustKernelHuber());
                optimizer.addEdge(edge);
            }

            T_last = T;

            if (pub_odom.getNumSubscribers() != 0)
            {
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
            tf::Quaternion quat_tf(m_q_w_curr.x(), m_q_w_curr.y(), m_q_w_curr.z(), m_q_w_curr.w());
            double roll, pitch, yaw;
            tf::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
            std::cout<<"rpy : "<<roll<<" "<<pitch<<" "<<yaw<<std::endl;

        }
        if (frame_counts < 100)
        {
            PointType thisPose3D;
            PointTypePose thisPose6D;
            thisPose3D.x = m_t_w_curr.x();
            thisPose3D.y = m_t_w_curr.y();
            thisPose3D.z = m_t_w_curr.z();
            thisPose3D.intensity = cloudKeyPoses3D_100->points.size(); //代表当前帧
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

            pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*ds_cornerCloudKeyFrame_now, *thisCornerKeyFrame);
            pcl::copyPointCloud(*ds_surfCloudKeyFrame_now, *thisSurfKeyFrame);
            cornerCloudFrame_100.push_back(thisCornerKeyFrame);
            surfCloudFrame_100.push_back(thisSurfKeyFrame);
        }
        currentRobotPosPoint.x = m_t_w_curr.x();
        currentRobotPosPoint.y = m_t_w_curr.y();
        currentRobotPosPoint.z = m_t_w_curr.z();
    }

    void CorrectPoses()
    {
        if (aLoopIsClosed == true)
        {
            nav_msgs::Path map_path;
            map_path.header.stamp = ros::Time().fromSec(cloudrealtime);
            map_path.header.frame_id = "/camera_init";
            // for(size_t i = 0; i < cloudKeyPoses3D->points.size(); i++)
            // {
            //     geometry_msgs::PoseStamped CarPose;
            //     CarPose.pose.position.x = cloudKeyPoses3D->points[i].x;
            //     CarPose.pose.position.y = cloudKeyPoses3D->points[i].y;
            //     CarPose.pose.position.z = cloudKeyPoses3D->points[i].z;
            //     map_path.poses.push_back(CarPose);

            // }
            // pub_path.publish(map_path);

            // int numPoses = cloudKeyPoses6D.size();
            for (int i = 0; i < loop_size; ++i)
            {
                geometry_msgs::PoseStamped CarPose;

                g2o::VertexSE3 *vertex = dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
                Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
                Eigen::Matrix3d rotation_matrix;// = pose.block(0, 0, 3, 3);
                for (size_t k = 0; k < 3; k++)
                {
                    for (size_t m = 0; m < 3; m++)
                    {
                        rotation_matrix(k, m) = pose(k, m);
                    }
                }
                Eigen::Quaterniond quaternion = Eigen::Quaterniond(rotation_matrix);

                Eigen::Vector3d v_position(pose(0, 3), pose(1, 3), pose(2, 3));

                cloudKeyPoses3D->points[i].x = pose(0, 3);
                cloudKeyPoses3D->points[i].y = pose(1, 3);
                cloudKeyPoses3D->points[i].z = pose(2, 3);

                cloudKeyPoses6D[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D[i].q_w = quaternion.w();
                cloudKeyPoses6D[i].q_x = quaternion.x();
                cloudKeyPoses6D[i].q_y = quaternion.y();
                cloudKeyPoses6D[i].q_z = quaternion.z();

                CarPose.pose.position.x = pose(0, 3);
                CarPose.pose.position.y = pose(1, 3);
                CarPose.pose.position.z = pose(2, 3);
                map_path.poses.push_back(CarPose);
            }

            pub_path.publish(map_path);

            aLoopIsClosed = false;
        }
    }

    void ClearCloud()
    {
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        laserCloudCornerFromMapDS->clear();
        laserCloudSurfFromMapDS->clear();

        laserCloudSurfLast->clear();
        laserCloudCornerLast->clear();
        ds_surfCloudKeyFrame_now->clear();
        ds_cornerCloudKeyFrame_now->clear();
    }

    void Run()
    {
        // std::cout<<" receive size : "<<receive_clouds.size()<<std::endl;
        // while(receive_clouds.empty()) sleep(0.001);
        if (receive_clouds.empty())
            return;
        // std::cout<<" don't receive cloud"<<std::endl;
        mtx.lock();
        Data_pair receive_cloud_last = receive_clouds.front();
        receive_clouds.pop();
        mtx.unlock();

        *laserCloudSurfLast = receive_cloud_last.laser_cloud_surf;
        *laserCloudCornerLast = receive_cloud_last.laser_cloud_corner;
        timeLaserCloudSurfLast = receive_cloud_last.laser_time_surf;
        timeLaserCloudCornerLast = receive_cloud_last.laser_time_surf;

        if (std::abs(timeLaserCloudCornerLast - timeLaserCloudSurfLast) < 0.005 )
        {
            cloudrealtime = (timeLaserCloudCornerLast + timeLaserCloudSurfLast) / 2.0;

            if (frame_counts <= 20)
            {
                DownSampleCloud();
                SavePoseAndCloud();
            }
            else
            {
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

    void VisualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        while (ros::ok())
        {
            PublishAndSaveMap();
            rate.sleep();
        }
    }

    void LoopClosureThread()
    {
        // return;
        ros::Rate rate(100);
        while (ros::ok())
        {
            if(!using_loop_) 
            {
                continue;
            }
            cloud_loop_frequency++;
            PerformGpsLoopClosure();
            if (cloud_loop_frequency % 100 == 0) //1秒一次
            {
                //if(frame_counts == frame_counts_last) continue;
                PerformCloudLoopClosure();
                CorrectPoses();
                cloud_loop_frequency = 0;
                //frame_counts_last = frame_counts;
            }
            rate.sleep();
        }
    }

    //m_q_w_curr表示下一点到上一点的变换
    //后续修改，只读取一次会改变的全局并用锁
    void PerformGpsLoopClosure()
    {
        // if(receive_gpss.size() == 1 )
        // {
        //     std::ofstream myfile("../path.txt", std::ios::app);
        //     // std::cout<<"824"<<std::endl;
        //     myfile << to_string(0.0) << " " << to_string(0.0) << " " << to_string(0.0)
        //            << " " << to_string(0.0) << " " << to_string(0.0) << " " << to_string(0.0)
        //            << " " << to_string(1.0) << " " << to_string(0.0) << "\n"
        //            << to_string(receive_gpss[0].position_gps.x()) << " " << to_string(receive_gpss[0].position_gps.y()) << " " << to_string(receive_gpss[0].position_gps.z())
        //            << " " << to_string(receive_gpss[0].quaternion_gps.x()) << " " << to_string(receive_gpss[0].quaternion_gps.y()) << " " << to_string(receive_gpss[0].quaternion_gps.z())
        //            << " " << to_string(receive_gpss[0].quaternion_gps.w()) << " " << to_string(0.0) << "\n";
        //     myfile.close();
        // }
        if (receive_gpss.size() < 2 || cloudKeyPoses3D->points.empty())
            return;
        Gps_data gps_last, gps_now;

        if (cloud_cin_size >= cloudKeyPoses6D.size())
            return;
        cloud_cin_size = cloudKeyPoses6D.size();
        double key_point_time = cloudKeyPoses6D[cloudKeyPoses6D.size() - 1].time;
        for (size_t i = 0; i < receive_gpss.size(); i++)
        {
            gps_last = receive_gpss[i];

            if (i == receive_gpss.size() - 1 || gps_last.time_gps > key_point_time)
                return;

            gps_now = receive_gpss[i + 1];
            if (gps_last.time_gps < key_point_time && gps_now.time_gps > key_point_time)
            {
                // while(i--) receive_gpss.pop();
                break;
            }
        }
        if (key_point_time - gps_last.time_gps > 0.15 || gps_now.time_gps - key_point_time > 0.15)
            return;

        //加锁
        double gps_correspond_mapping_covariance[6];
        for (size_t i = 0; i < 6; i++)
        {
            gps_correspond_mapping_covariance[i] = (gps_last.covariance_gps[i] + gps_now.covariance_gps[i]) / 2.0;
            // std::cout << "cov " << gps_correspond_mapping_covariance[i] << std::endl;
        }
        Eigen::Quaterniond mapping_odom_quaterniond = m_q_w_curr;
        Eigen::Vector3d mapping_odom_position = m_t_w_curr;
        int gps_correspond_mapping_num = cloudKeyPoses6D.size() - 1;
        // double odom_correspond_mapping_covariance[3];
        // for(size_t i = 0; i < 6; i++)
        // {
        //     odom_correspond_mapping_covariance[i] = (gps_last.covariance_gps[i] + gps_now.covariance_gps[i])/2.0;
        //     std::cout<<"cov "<<odom_correspond_mapping_covariance[i]<<std::endl;
        // }


        if (1)
        {
            Eigen::Quaterniond gps_mapping_q = gps_last.quaternion_gps;//.slerp((key_point_time - gps_last.time_gps) / (gps_now.time_gps - gps_last.time_gps), gps_now.quaternion_gps);
            Eigen::Vector3d gps_mapping_t = gps_last.position_gps + (key_point_time - gps_last.time_gps) / (gps_now.time_gps - gps_last.time_gps) * (gps_now.position_gps - gps_last.position_gps);
            Eigen::Quaterniond q_gps_mapping = gps_mapping_q;
            Eigen::Vector3d t_gps_mapping = gps_mapping_t;
            // if(gps_now.time_gps - key_point_time > 0.05) return;
            // Eigen::Quaterniond q_gps_mapping = gps_now.quaternion_gps;
            // Eigen::Vector3d t_gps_mapping = gps_now.position_gps;

            std::ofstream myfile("../path.txt", std::ios::app);
            // std::cout<<"824"<<std::endl;
            myfile << to_string(mapping_odom_position.x()) << " " << to_string(mapping_odom_position.y()) << " " << to_string(mapping_odom_position.z())
                   << " " << to_string(mapping_odom_quaterniond.x()) << " " << to_string(mapping_odom_quaterniond.y()) << " " << to_string(mapping_odom_quaterniond.z())
                   << " " << to_string(mapping_odom_quaterniond.w()) << " " << to_string(key_point_time) << "\n"
                   << to_string(t_gps_mapping.x()) << " " << to_string(t_gps_mapping.y()) << " " << to_string(t_gps_mapping.z())
                   << " " << to_string(q_gps_mapping.x()) << " " << to_string(q_gps_mapping.y()) << " " << to_string(q_gps_mapping.z())
                   << " " << to_string(q_gps_mapping.w()) << " " << to_string(gps_last.time_gps) << "\n";
            myfile.close();

            std::cout << "save the file" << std::endl;
        }

        Eigen::Quaterniond gps_quaterniond_in_world = gps_last.quaternion_gps.slerp((key_point_time - gps_last.time_gps) / (gps_now.time_gps - gps_last.time_gps), gps_now.quaternion_gps);
        Eigen::Vector3d gps_position_in_world = gps_last.position_gps + (key_point_time - gps_last.time_gps) / (gps_now.time_gps - gps_last.time_gps) * (gps_now.position_gps - gps_last.position_gps);

        if(saved_frist_gps_and_odom == 0 && gps_correspond_mapping_covariance[0] < 0.1 && gps_correspond_mapping_covariance[1] < 0.1)
        {
            std::cout << " save the frist gps and odom point" << std::endl;
            frist_gps_quaterniond = gps_quaterniond_in_world;
            frist_gps_position = gps_position_in_world;

            frist_gps_correspond_odom_quaterniond = mapping_odom_quaterniond;
            frist_gps_correspond_odom_position = mapping_odom_position;
            frist_gps_correspond_odom_num = gps_correspond_mapping_num;
        }


        //记录符合要求的gps位置以及对应的odom位置,报存5个点消除误差
        if ( (!saved_frist_gps_and_odom  && gps_correspond_mapping_covariance[0] < 0.1 && gps_correspond_mapping_covariance[1] < 0.1) || 
            (saved_frist_gps_and_odom < 6 && gps_correspond_mapping_covariance[0] < 0.05 && gps_correspond_mapping_covariance[1] < 0.05 &&
             (last_save_position.x() - mapping_odom_position.x()) * (last_save_position.x() - mapping_odom_position.x()) + (last_save_position.y() - mapping_odom_position.y()) * (last_save_position.y() - mapping_odom_position.y()) > 4.0))
        {
            saved_frist_gps_and_odom++;
            if(save_gps_correspond_odom_data)
            {
                std::ofstream myfile("../path/frist_gps_odom.txt", std::ios::app);
                myfile.clear();
                myfile << to_string(mapping_odom_position.x()) << '|' << to_string(mapping_odom_position.y()) << '|' << to_string(mapping_odom_position.z()) << '|'
                    << to_string(mapping_odom_quaterniond.x()) << '|' << to_string(mapping_odom_quaterniond.y()) << '|' << to_string(mapping_odom_quaterniond.z()) << '|'
                    << to_string(mapping_odom_quaterniond.w()) << '|' << to_string(key_point_time) << "\n"
                    << to_string(gps_position_in_world.x()) << '|' << to_string(gps_position_in_world.y()) << '|' << to_string(gps_position_in_world.z()) << '|'
                    << to_string(gps_quaterniond_in_world.x()) << '|' << to_string(gps_quaterniond_in_world.y()) << '|' << to_string(gps_quaterniond_in_world.z()) << '|'
                    << to_string(gps_quaterniond_in_world.w()) << '|' << to_string(gps_last.time_gps) << "\n";
                myfile.close();
            }
            last_save_position = mapping_odom_position;
            
        }
        if(!saved_frist_gps_and_odom) return;
        //转到map坐标系中
        Eigen::Vector3d gps_position_in_map = frist_gps_correspond_odom_quaterniond * (frist_gps_quaterniond.inverse() * (gps_position_in_world - frist_gps_position)) + frist_gps_correspond_odom_position;
        Eigen::Quaterniond gps_quater_in_map = frist_gps_correspond_odom_quaterniond * (frist_gps_quaterniond.inverse() * gps_quaterniond_in_world);
       
            pcl::PointCloud<pcl::PointXYZI> cloud;
            cloud += *cornerCloudKeyFrames[gps_correspond_mapping_num] + *surfCloudKeyFrames[gps_correspond_mapping_num];
            PointTypePose posenow;
            posenow.q_x = gps_quaterniond_in_world.x();
            posenow.q_y = gps_quaterniond_in_world.y();
            posenow.q_z = gps_quaterniond_in_world.z();
            posenow.q_w = gps_quaterniond_in_world.w();
            posenow.x = gps_position_in_world.x() - frist_gps_position.x();
            posenow.y = gps_position_in_world.y() - frist_gps_position.y();
            posenow.z = 0;
            pointcloudAssociateToMap(cloud, cloud, posenow);
            correct_gps_cloud.push_back(cloud);


        if(add_gps_edge)
        if (saved_frist_gps_and_odom && gps_correspond_mapping_covariance[0] < 0.05 && gps_correspond_mapping_covariance[1] < 0.05 && (last_g2o_position.x() - gps_position_in_world.x()) * (last_g2o_position.x() - gps_position_in_world.x()) + (last_g2o_position.y() - gps_position_in_world.y()) * (last_g2o_position.y() - gps_position_in_world.y()) > 2.0)
        {
            last_g2o_position = gps_position_in_world;
            // return;
            //计算在第一个gps下的变换
            //转换到地图坐标系
            // Eigen::Quaterniond gps_in_lidar_quaterniond = (frist_gps_quaterniond.inverse() * gps_quaterniond_in_world);
            // Eigen::Vector3d gps_in_lidar_position = (frist_gps_quaterniond.inverse() * (gps_position_in_world - frist_gps_position));
            

            // Eigen::Matrix3d rotation_matrix = gps_in_lidar_quaterniond.matrix();
            Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
            Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
            Eigen::Isometry3d T1toT2 = Eigen::Isometry3d::Identity();
            T1.rotate(gps_quater_in_map.matrix());
            T1.pretranslate(gps_position_in_map);

            T2.rotate(frist_gps_correspond_odom_quaterniond.matrix());
            T2.pretranslate(frist_gps_correspond_odom_position);
            T1toT2 = T2.inverse() * T1;
            g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
            edge->vertices()[0] = optimizer.vertex(frist_gps_correspond_odom_num);
            edge->vertices()[1] = optimizer.vertex(gps_correspond_mapping_num);
            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
            // information(0, 0) = information(1, 1) = information(2, 2) = 100;
            // information(3, 3) = information(4, 4) = information(5, 5) = 100;
            information(0, 0) = 1.0 / (gps_correspond_mapping_covariance[0] * gps_correspond_mapping_covariance[0]) ;
            information(1, 1) = 1.0 / (gps_correspond_mapping_covariance[1] * gps_correspond_mapping_covariance[1]);
            information(2, 2) = 0.00001;
            information(3, 3) = 0.00001;
            information(4, 4) = 0.00001;
            information(5, 5) = 0.1;

            std::cout<<"gps edge : "<<T1toT2(0, 3)<<" "<<T1toT2(1, 3)<<" "<<T1toT2(2, 3)<<std::endl;
            edge->setInformation(information);
            edge->setMeasurement(T1toT2);
            edge->setRobustKernel(new g2o::RobustKernelHuber());
            optimizer.addEdge(edge);
            std::cout<<"add gps edge"<<std::endl;

            // StartOptimizeLoop();
            // aLoopIsClosed = true;
        }
        if (pub_gps_position.getNumSubscribers() != 0)
        {
            nav_msgs::Odometry odomAftMapped;
            odomAftMapped.header.frame_id = "camera_init";
            odomAftMapped.pose.pose.orientation.x = gps_quater_in_map.x();
            odomAftMapped.pose.pose.orientation.y = gps_quater_in_map.y();
            odomAftMapped.pose.pose.orientation.z = gps_quater_in_map.z();
            odomAftMapped.pose.pose.orientation.w = gps_quater_in_map.w();
            odomAftMapped.pose.pose.position.x = gps_position_in_map.x();
            odomAftMapped.pose.pose.position.y = gps_position_in_map.y();
            odomAftMapped.pose.pose.position.z = gps_position_in_map.z();
            pub_gps_position.publish(odomAftMapped);
        }

    }

    void StartOptimizeLoop()
    {
        return;
        optimizer.save("../result_before.g2o");
        double t1 = clock();
        optimizer.initializeOptimization();
        optimizer.optimize(30); //可以指定优化步数
        cout << "saving optimization results ..." << endl;
        optimizer.save("../result_after.g2o");
        std::cout << "time : " << (double)clock() - t1 << std::endl;
        cout << "Optimization done." << endl;
        // PublishKeyPosesAndFrames();
    }

    bool DetectCloudLoopClosure()
    {
        //cloudKeyPoses3D关键帧的位置点
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
        kdtreeHistoryKeyPoses->radiusSearch(last_keypose, icp_loop_radius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        std::cout << "loop size  :  " << pointSearchIndLoop.size() << std::endl;
        closestHistoryFrameID = -1;
        double max_time = 0.0;
        for (size_t i = 0; i < pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            double time = fabs(cloudKeyPoses6D[id].time - cloudKeyPoses6D[latestFrameIDLoopCloure].time);
            if (max_time < time)
                max_time = time;
            if (time > 30.0)
            {
                std::cout << "time is ok!" << std::endl;
                closestHistoryFrameID = id;
                break;
            }
        }
        std::cout << " max time : " << max_time << std::endl;
        if (closestHistoryFrameID == -1)
        {
            return false;
        }

        pcl::PointCloud<PointType> surrounding_corner;
        pcl::PointCloud<PointType> surrounding_surf;
        pointcloudAssociateToMap(*cornerCloudKeyFrames[latestFrameIDLoopCloure], surrounding_corner, cloudKeyPoses6D[latestFrameIDLoopCloure]);
        pointcloudAssociateToMap(*surfCloudKeyFrames[latestFrameIDLoopCloure], surrounding_surf, cloudKeyPoses6D[latestFrameIDLoopCloure]);

        *latestSurfKeyFrameCloud += surrounding_corner;
        // *latestSurfKeyFrameCloud += surrounding_surf;

        for (int j = -icp_loop_history_search_num; j <= icp_loop_history_search_num; ++j)
        {
            if (closestHistoryFrameID + j < 0 || closestHistoryFrameID + j > latestFrameIDLoopCloure)
                continue;
            pcl::PointCloud<PointType> history_corner;
            pcl::PointCloud<PointType> history_surf;
            pointcloudAssociateToMap(*cornerCloudKeyFrames[closestHistoryFrameID + j], history_corner, cloudKeyPoses6D[closestHistoryFrameID + j]);
            pointcloudAssociateToMap(*surfCloudKeyFrames[closestHistoryFrameID + j], history_corner, cloudKeyPoses6D[closestHistoryFrameID + j]);

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

    void PerformCloudLoopClosure()
    {
        return;
        // std::cout<<"861 861 861 861"<<std::endl;
        if (cloudKeyPoses3D->points.empty() == true)
            return;
        if (potentialLoopFlag == false)
        {

            if (DetectCloudLoopClosure() == true)
            {
                potentialLoopFlag = true;
            }
            if (potentialLoopFlag == false) // || flag == false)
                return;
        }
        potentialLoopFlag = false;
        // flag = false;
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(100);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
        icp.setInputSource(latestSurfKeyFrameCloudDS);
        icp.setInputTarget(nearHistorySurfKeyFrameCloudDS);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        std::cout << "icp getFitnessScore" << icp.getFitnessScore() << std::endl;
        if (icp.hasConverged() == false || icp.getFitnessScore() > icp_loop_score)
            return;

        // 以下在点云icp收敛并且噪声量在一定范围内进行
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            // icp.getFinalTransformation()的返回值是Eigen::Matrix<Scalar, 4, 4>
            pcl::transformPointCloud(*latestSurfKeyFrameCloud, *closed_cloud, icp.getFinalTransformation());
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
            cloudMsgTemp.header.frame_id = "/camera_init";
            pubIcpKeyFrames.publish(cloudMsgTemp);
        }
        Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
        std::cout<<"now : "<<latestFrameIDLoopCloure<<" last : "<<closestHistoryFrameID<<" x, y, z :"<<
        icp_trans(0, 3)<<" "<<icp_trans(1, 3)<<" "<<icp_trans(2, 3)<<std::endl;
        Eigen::Isometry3d T;
        for (size_t i = 0; i < 4; i++)
            for (size_t j = 0; j < 4; j++)
            {
                T(i, j) = icp_trans(i, j);
            }
        Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T1toT2 = Eigen::Isometry3d::Identity();
        Eigen::Quaterniond q2(cloudKeyPoses6D[latestFrameIDLoopCloure].q_w, cloudKeyPoses6D[latestFrameIDLoopCloure].q_x,
                                cloudKeyPoses6D[latestFrameIDLoopCloure].q_y, cloudKeyPoses6D[latestFrameIDLoopCloure].q_z);
        Eigen::Quaterniond q1(cloudKeyPoses6D[closestHistoryFrameID].q_w, cloudKeyPoses6D[closestHistoryFrameID].q_x,
                                cloudKeyPoses6D[closestHistoryFrameID].q_y, cloudKeyPoses6D[closestHistoryFrameID].q_z);
        T1.rotate(q1.matrix());
        T1.pretranslate(Eigen::Vector3d(cloudKeyPoses6D[closestHistoryFrameID].x, cloudKeyPoses6D[closestHistoryFrameID].y, 
                                cloudKeyPoses6D[closestHistoryFrameID].z));

        T2.rotate(q2.matrix());
        T2.pretranslate(Eigen::Vector3d(cloudKeyPoses6D[latestFrameIDLoopCloure].x, cloudKeyPoses6D[latestFrameIDLoopCloure].y, 
                                cloudKeyPoses6D[latestFrameIDLoopCloure].z));

        T2 = T * T2;//当前帧的实际位置
        T1toT2 = T1.inverse() * T2;                       
        loop_size = latestFrameIDLoopCloure;
        g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
        edge->vertices()[0] = optimizer.vertex(closestHistoryFrameID);
        edge->vertices()[1] = optimizer.vertex(latestFrameIDLoopCloure);
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
        information(0, 0) = information(1, 1) = information(2, 2) = 1.0 / (icp.getFitnessScore());
        information(3, 3) = information(4, 4) = information(5, 5) = 1.0 / (icp.getFitnessScore());
        edge->setInformation(information);
        edge->setMeasurement(T1toT2);
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);

        aLoopIsClosed = true;
        StartOptimizeLoop();
    }

    void PublishAndSaveMap()
    {
        // if (pubLaserCloudSurround.getNumSubscribers() == 0)
        //     return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;

        // mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(currentRobotPosPoint, publish_range_map, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        // mtx.unlock();

        for (size_t i = 0; i < pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);

        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

        pcl::PointCloud<PointType>::Ptr global_map_corner(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr global_map_surf(new pcl::PointCloud<PointType>());
        for (size_t i = 0; i < globalMapKeyPosesDS->points.size(); i++)
        {
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            pcl::PointCloud<PointType> surrounding_corner;
            pcl::PointCloud<PointType> surrounding_surf;
            pointcloudAssociateToMap(*cornerCloudKeyFrames[thisKeyInd], surrounding_corner, cloudKeyPoses6D[thisKeyInd]);
            pointcloudAssociateToMap(*surfCloudKeyFrames[thisKeyInd], surrounding_surf, cloudKeyPoses6D[thisKeyInd]);
            for (size_t i = 0; i < surrounding_corner.points.size(); i++)
            {
                surrounding_corner.points[i].intensity = 10.0;
            }
            for (size_t i = 0; i < surrounding_surf.points.size(); i++)
            {
                surrounding_surf.points[i].intensity = 0.2;
            }
            *global_map_corner += surrounding_corner;
            *global_map_surf += surrounding_surf;
        }

        pcl::PointCloud<PointType>::Ptr gps_cloud_out(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType> ds_global_map_coner;
        pcl::PointCloud<PointType> ds_global_map_surf;
        pcl::PointCloud<PointType> gps_cloud_out_ds;
        pcl::PointCloud<PointType> ds_global_map_full;
        for(size_t i = 0; i < correct_gps_cloud.size(); i++)
        {
            *gps_cloud_out += correct_gps_cloud[i];
        }
        // nav_msgs::Path map_path;
        // map_path.header.stamp = ros::Time().fromSec(cloudrealtime);
        // map_path.header.frame_id = "/camera_init";
        // for(size_t i = 0; i < cloudKeyPoses3D->points.size(); i++)
        // {
        //     geometry_msgs::PoseStamped CarPose;
        //     CarPose.pose.position.x = cloudKeyPoses3D->points[i].x;
        //     CarPose.pose.position.y = cloudKeyPoses3D->points[i].y;
        //     CarPose.pose.position.z = cloudKeyPoses3D->points[i].z;
        //     map_path.poses.push_back(CarPose);

        // }
        // pub_path.publish(map_path);
        downSizeFilterGlobalMapKeyFrames.setInputCloud(gps_cloud_out);
        downSizeFilterGlobalMapKeyFrames.filter(gps_cloud_out_ds);

        // 对globalMapKeyFrames进行下采样
        downSizeFilterGlobalMapKeyFrames.setInputCloud(global_map_corner);
        downSizeFilterGlobalMapKeyFrames.filter(ds_global_map_coner);

        downSizeFilterGlobalMapKeyPoses.setInputCloud(global_map_surf);
        downSizeFilterGlobalMapKeyPoses.filter(ds_global_map_surf);

        ds_global_map_full = ds_global_map_surf + ds_global_map_coner;

        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(ds_global_map_full, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
        cloudMsgTemp.header.frame_id = "/camera_init";
        pubLaserCloudSurround.publish(cloudMsgTemp);

        if(gps_cloud_out_ds.points.size() > 0)
        pcl::io::savePCDFileASCII("../Loam_livox_pcd/test_gps.pcd", gps_cloud_out_ds);
        pcl::io::savePCDFileASCII("../Loam_livox_pcd/surround.pcd", ds_global_map_full);
        std::ofstream path_file("../path/path.txt", std::ios::in | std::ios::trunc);
        path_file.clear();
        for (size_t i = 0; i < cloudKeyPoses6D.size(); i++)
        {
            path_file << to_string(cloudKeyPoses6D[i].x)   << '|' << to_string(cloudKeyPoses6D[i].y) << '|' << to_string(cloudKeyPoses6D[i].z) << '|'
                      << to_string(cloudKeyPoses6D[i].q_x) << '|' << to_string(cloudKeyPoses6D[i].q_y) << '|' << to_string(cloudKeyPoses6D[i].q_z) << '|'
                      << to_string(cloudKeyPoses6D[i].q_w) << std::endl;
        }
        path_file.close();

        globalMapKeyPoses->clear();
        globalMapKeyPosesDS->clear();
        globalMapKeyFrames->clear();
        // globalMapKeyFramesDS->clear();
    }

    void PublishKeyPosesAndFrames()
    {

        if (pubKeyPoses.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
            cloudMsgTemp.header.frame_id = "/camera_init";
            pubKeyPoses.publish(cloudMsgTemp);
        }

        if (pubRecentKeyFrames.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laserCloudCornerFromMapDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
            cloudMsgTemp.header.frame_id = "/camera_init";
            pubRecentKeyFrames.publish(cloudMsgTemp);
        }
        if (pubFramesnow.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::PointCloud<PointType> now_cloud;
            now_cloud = *(plicp.laserCloudCornerStack);
            pointcloudAssociateToMap(now_cloud, now_cloud, cloudKeyPoses6D[cloudKeyPoses6D.size() - 1]);
            pcl::toROSMsg(now_cloud, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(cloudrealtime);
            cloudMsgTemp.header.frame_id = "/camera_init";
            pubFramesnow.publish(cloudMsgTemp);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loam_new");
    ROS_INFO("\033[1;32m---->\033[0m lassing mapping is Started.");

    LaserMappingNew LMN;
    std::thread loopthread(&LaserMappingNew::LoopClosureThread, &LMN);
    std::thread visualizeMapThread(&LaserMappingNew::VisualizeGlobalMapThread, &LMN);

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        LMN.Run();

        rate.sleep();
    }

    loopthread.join();
    visualizeMapThread.join();

    LMN.optimizer.clear();
    return 0;
}

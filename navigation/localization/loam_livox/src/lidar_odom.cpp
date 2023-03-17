#include <ros/ros.h>
#include <pcl/point_cloud.h>
//msgs type and conversion
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//pcd io
#include <pcl/io/pcd_io.h>
//point types
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <time.h>
#include <pcl/common/transforms.h>
#include "plane_line_icp.hpp"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <string>
#include <iostream>
#include <time.h>
#include "tf/transform_datatypes.h"

void CorrectionGps(Eigen::Quaterniond &gps_quaterniond, Eigen::Vector3d &gps_position);

// static void toEulerAngle(const Eigen::Quaterniond &q, double &yaw, double &pitch, double &roll)
// {
//     // roll (x-axis rotation)
//     double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
//     double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
//     roll = atan2(sinr_cosp, cosr_cosp);

//     // pitch (y-axis rotation)
//     double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
//     if (fabs(sinp) >= 1)
//         pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//     else
//         pitch = asin(sinp);

//     // yaw (z-axis rotation)
//     double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
//     double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
//     yaw = atan2(siny_cosp, cosy_cosp);
// }

void ComputedMeanOfAngle(std::vector<double> angles, double &angle)
{
    // double angle_all;
    bool angle_in_zero = true;
    bool angle_less_zero = false;
    bool angle_greater_zero = false;
    for (size_t i = 0; i < angles.size(); i++)
    {
        if (angles[i] < 0)
            angle_less_zero = true;
        if (angles[i] >= 0)
            angle_greater_zero = true;
        if (angles[i] > 3.0 * 3.1415926 / 4.0)
            angle_in_zero = false;
    }
    if (angle_less_zero && angle_greater_zero && !angle_in_zero)
    {
        for (size_t i = 0; i < angles.size(); i++)
        {
            if (angles[i] < 0)
                angle += angles[i] + 2 * M_PI;
            else
            {
                angle += angles[i];
            }
        }
        // std::cout << "angle " << angle << " size : " << angles.size() << std::endl;
        angle = angle / angles.size();
        if (angle > M_PI)
            angle = angle - 2 * M_PI;
    }
    else
    {
        for (size_t i = 0; i < angles.size(); i++)
        {
            angle += angles[i];
        }
        // std::cout << "angle " << angle << " size : " << angles.size() << std::endl;
        angle = angle / angles.size();
    }
}

struct Gps_data
{
    double time_gps;
    Eigen::Quaterniond quaternion_gps;
    Eigen::Vector3d position_gps;
    double covariance_gps[6];
    bool receive_gps;
    void GetGpsDatas(const nav_msgs::OdometryConstPtr input_odom, const std::vector<Eigen::Quaterniond> quaternion_odom_frist, const std::vector<Eigen::Vector3d> position_odom_frist,
                     const std::vector<Eigen::Quaterniond> quaternion_gps_frist, const std::vector<Eigen::Vector3d> position_gps_frist)
    {
        time_gps = input_odom->header.stamp.toSec();
        //转换到odom的位置
        Eigen::Quaterniond quaternion_gps_src = Eigen::Quaterniond(input_odom->pose.pose.orientation.w, input_odom->pose.pose.orientation.x,
                                                                   input_odom->pose.pose.orientation.y, input_odom->pose.pose.orientation.z);
        Eigen::Vector3d position_gps_src = Eigen::Vector3d(input_odom->pose.pose.position.x, input_odom->pose.pose.position.y, input_odom->pose.pose.position.z);

        CorrectionGps(quaternion_gps_src, position_gps_src);

        Eigen::Vector3d position_gps_all(0.0, 0.0, 0.0);
        double yaw_angle = 0.0;
        std::vector<double> yaw_angles;
        yaw_angles.resize(quaternion_odom_frist.size());
        for (size_t i = 0; i < quaternion_odom_frist.size(); i++)
        {
            Eigen::Quaterniond quaternion_gps_now = quaternion_odom_frist[i] * (quaternion_gps_frist[i].inverse() * quaternion_gps_src);
            Eigen::Vector3d position_gps_now = quaternion_odom_frist[i] * (quaternion_gps_frist[i].inverse() * (position_gps_src - position_gps_frist[i])) + position_odom_frist[i];
            position_gps_all += position_gps_now;
            Eigen::Vector3d angle_rotation(0.0, 0.0, 0.0); //欧拉角度
            tf::Quaternion quat(quaternion_gps_now.x(), quaternion_gps_now.y(), quaternion_gps_now.z(), quaternion_gps_now.w());
            tf::Matrix3x3(quat).getRPY(angle_rotation[2], angle_rotation[1], angle_rotation[0]);
            // toEulerAngle(quaternion_gps_now, angle_rotation[0], angle_rotation[1], angle_rotation[2]);
            // if(angle_rotation[0] < 0)yaw_angle += angle_rotation[0] + 2 * M_PI;
            // else
            // yaw_angle += angle_rotation[0];
            yaw_angles[i] = angle_rotation[0];
            // std::cout << "angle now : " << angle_rotation[0] << " i" << i << std::endl;
        }
        position_gps = position_gps_all / quaternion_odom_frist.size();

        covariance_gps[0] = input_odom->pose.covariance[0];
        covariance_gps[1] = input_odom->pose.covariance[7];
        covariance_gps[2] = input_odom->pose.covariance[14];
        covariance_gps[3] = input_odom->pose.covariance[21];
        covariance_gps[4] = input_odom->pose.covariance[28];
        covariance_gps[5] = input_odom->pose.covariance[35];
        // double angle_gps_yaw = yaw_angle / quaternion_odom_frist.size();
        ComputedMeanOfAngle(yaw_angles, yaw_angle);
        // if(angle_gps_yaw > M_PI) angle_gps_yaw = angle_gps_yaw - 2* M_PI;
        quaternion_gps = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd((double)0.0, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd((double)0.0, Eigen::Vector3d::UnitX());
        // std::cout << "gps yaw angle: " << yaw_angle << std::endl;
        receive_gps = true;
    }
    bool GpsDataIsOk()
    {
        if (receive_gps && covariance_gps[0] < 0.1 && covariance_gps[1] < 0.1 && covariance_gps[2] < 0.1)
            return true;
        else
            return false;
    }
};

struct Frist_Odom_Pose
{
    bool received_odom_pose = false;
    bool receive_odom_pose = false;
    Eigen::Quaterniond frist_pose_quater;
    Eigen::Vector3d frist_pose_position;
    void GetFristPose(const geometry_msgs::PoseWithCovarianceStamped receivepose)
    {
        Eigen::Vector3d angle_rotation(0.0, 0.0, 0.0); //欧拉角度
        tf::Quaternion quat(receivepose.pose.pose.orientation.x, receivepose.pose.pose.orientation.y, receivepose.pose.pose.orientation.z, receivepose.pose.pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(angle_rotation[2], angle_rotation[1], angle_rotation[0]);
        frist_pose_quater = Eigen::AngleAxisd(angle_rotation[0] - M_PI_2, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd((double)0.0, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd((double)0.0, Eigen::Vector3d::UnitX());
        // frist_pose_quater = Eigen::Quaterniond(receivepose.pose.pose.orientation.w, receivepose.pose.pose.orientation.x, receivepose.pose.pose.orientation.y, receivepose.pose.pose.orientation.z);
        frist_pose_position = Eigen::Vector3d(receivepose.pose.pose.position.y, -receivepose.pose.pose.position.x, receivepose.pose.pose.position.z);
        receive_odom_pose = true;
    }
};

ros::Publisher corner_pub, surf_pub, full_pub, intensity_pub;
ros::Publisher lidar_odom_pub, gps_odom_pub, cloud_map_pub, pub_frist_odom;
std::vector<Eigen::Quaterniond> odom_quters;
std::vector<Eigen::Vector3d> odom_positions;
std::vector<Eigen::Quaterniond> gps_quters;
std::vector<Eigen::Vector3d> gps_positions;
std::vector<Eigen::Quaterniond> odom_quater_path;
std::vector<Eigen::Vector3d> odom_position_path;
Plane_Line_ICP plicp;
Gps_data gps_data;
Frist_Odom_Pose frist_odom_pose;

Eigen::Quaterniond gps_correspond_odom_quaterniond;
Eigen::Vector3d gps_correspond_odom_position;
Eigen::Quaterniond odom_correspond_gps_quaterniond;
Eigen::Vector3d odom_correspond_gps_position;

int lidar_odom_error_times_;
Eigen::Quaterniond fusion_quater_;
Eigen::Vector3d fusion_odom_;
bool receive_new_fusion_odom_ = false;
ros::Time fusion_time_;

bool showed_map_ = false;
void CorrectionGps(Eigen::Quaterniond &gps_quaterniond, Eigen::Vector3d &gps_position)
{
    Eigen::Vector3d correction_param_ea(-3.1415926 / 2.0, 0, 0);
    // Eigen::Quaterniond correction_param_quaterniond(0.998756535673134, 0, 0, 0.049853610202274);
    Eigen::Quaterniond correction_param_quaterniond(0.998756535673134, 0, 0, 0.049853610202274);
    Eigen::Quaterniond correction_param_90 = Eigen::AngleAxisd(correction_param_ea[0], Eigen::Vector3d::UnitZ()) *
                                             Eigen::AngleAxisd(correction_param_ea[1], Eigen::Vector3d::UnitY()) *
                                             Eigen::AngleAxisd(correction_param_ea[2], Eigen::Vector3d::UnitX());
    // Eigen::Vector3d correction_param_position(0.604715009798085, 0.45070539187565, 0);
    Eigen::Vector3d correction_param_position(0.604715009798085, 0.45070539187565, 0);

    gps_position = gps_quaterniond * correction_param_position + correction_param_90 * gps_position;
    gps_quaterniond = gps_quaterniond * correction_param_quaterniond;
}

void transformation_coordinate(pcl::PointCloud<pcl::PointXYZI> &raw_cloud, pcl::PointCloud<pcl::PointXYZI> &point_cloud)
{
    Eigen::Vector3d euler_angle(0.0, 0.0, 0.0);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion;
    quaternion = yawAngle * pitchAngle * rollAngle;
    Eigen::Translation3d translation(0, 0, 1.9921);
    Eigen::Affine3d affine = translation * quaternion;
    Eigen::Matrix4d matrix = affine.matrix();
    pcl::transformPointCloud(raw_cloud, point_cloud, matrix);
}

bool AdjustLidarOdom(double odom_time)
{
    if (fabs(odom_time - gps_data.time_gps) > 0.1)
        return false;
    if (fabs(gps_data.position_gps.x() - plicp.m_t_w_curr.x()) > 2.0 ||
        fabs(gps_data.position_gps.y() - plicp.m_t_w_curr.y()) > 2.0 ||
        fabs(gps_data.position_gps.z() - plicp.m_t_w_curr.z()) > 2.0)
        return true;
    else
        return false;
}
bool InitializeLidarPosition()
{
    if (frist_odom_pose.received_odom_pose)
        return true;
    if (gps_data.GpsDataIsOk())
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond quater;
        quater = gps_data.quaternion_gps;
        position.x() = gps_data.position_gps.x();
        position.y() = gps_data.position_gps.y();
        double min_distance = 200.0;
        for (size_t i = 0; i < odom_position_path.size(); i++)
        {
            double distance = fabs(sqrt(odom_position_path[i].x() * odom_position_path[i].x() + odom_position_path[i].y() * odom_position_path[i].y()) - sqrt(gps_data.position_gps.x() * gps_data.position_gps.x() + gps_data.position_gps.y() * gps_data.position_gps.y()));
            if (distance < min_distance)
            {
                position.z() = gps_data.position_gps.z();
                min_distance = distance;
            }
        }

        plicp.ReceivePosition(quater, position);
        frist_odom_pose.received_odom_pose = true;
        std::cout << "Initialize Succeed ! GPS data is right, using gps data to Initialize" << std::endl;
        return true;
    }
    else
        std::cout << "Initialize Faile ! GPS data is error, please receive odom data to Initialize in map" << std::endl;
    if (frist_odom_pose.receive_odom_pose)
    {

        Eigen::Vector3d position;
        Eigen::Quaterniond quater;
        quater = frist_odom_pose.frist_pose_quater;
        position.x() = gps_data.position_gps.x();
        position.y() = gps_data.position_gps.y();
        double min_distance = 200.0;
        for (size_t i = 0; i < odom_position_path.size(); i++)
        {
            double distance = fabs(sqrt(odom_position_path[i].x() * odom_position_path[i].x() + odom_position_path[i].y() * odom_position_path[i].y()) - sqrt(frist_odom_pose.frist_pose_position.x() * frist_odom_pose.frist_pose_position.x() + frist_odom_pose.frist_pose_position.y() * frist_odom_pose.frist_pose_position.y()));
            if (distance < min_distance)
            {
                position.z() = gps_data.position_gps.z();
                min_distance = distance;
            }
        }

        plicp.ReceivePosition(quater, position);
        frist_odom_pose.received_odom_pose = true;
        std::cout << "Initialize Succeed ! using gps odom data to Initialize in map" << std::endl;
        return true;
    }
    else
        std::cout << "Initialize Faile !  Please receive odom data to Initialize in map" << std::endl;
    return false;
}
void GetScan(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    // if(!InitializeLidarPosition()) return;
    float cloudCurvature[40000] = {0};
    float cloudintensity[40000] = {0};
    float cloudslope[40000] = {0};
    // int cloudSortInd[40000];
    int cloudNeighborPicked[40000] = {0};

    double cloud_distance[40000];
    // std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    //std::cout << "size  " << laserCloudIn.points.size() << std::endl;
    vector<vector<int>> scans;
    vector<int> scan;
    std::vector<int> indices;
    //移除空点
    //pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    transformation_coordinate(laserCloudIn, laserCloudIn);
    //std::cout << "size  " << laserCloudIn.points.size() << std::endl;
    int cloudSize = laserCloudIn.points.size();

    double t1 = clock();
    double angle_last = 0.0, dis_incre_last = 0.0, dis_incre_now = 0.0, angle = 0.0;
    for (int i = 0; i < cloudSize; i++)
    {
        angle = atan((laserCloudIn.points[i].z -1.9921)/ (sqrt(laserCloudIn.points[i].x * laserCloudIn.points[i].x + laserCloudIn.points[i].y * laserCloudIn.points[i].y)));
        // std::cout<<"angle "<<angle<<std::endl;
        if (i == 0)
        {
            scan.push_back(i);
            angle_last = angle;
            continue;
        }
        dis_incre_now = angle - angle_last;
        if ((dis_incre_now > 0 && dis_incre_last < 0) || (dis_incre_now < 0 && dis_incre_last > 0))
        {
            if (scan.size() > 30)
                scans.push_back(scan);
            scan.clear();
            scan.push_back(i);
        }
        else
            scan.push_back(i);

        // double angle=atan(laserCloudIn.points[i].z/sqrt(laserCloudIn.points[i].x*laserCloudIn.points[i].x+laserCloudIn.points[i].y*laserCloudIn.points[i].y));

        angle_last = angle;
        dis_incre_last = dis_incre_now;

        // cloud_distance[i] = sqrt(laserCloudIn.points[i].x * laserCloudIn.points[i].x + laserCloudIn.points[i].y * laserCloudIn.points[i].y + laserCloudIn.points[i].z * laserCloudIn.points[i].z );
    }
    //transformation_coordinate(laserCloudIn, laserCloudIn);
    // std::cout << "scans size" << scans.size() << std::endl;
    // cout << "\nicp time" << (double)(clock() - t1) / CLOCKS_PER_SEC << endl;
    //开始计算曲率
    for (size_t i = 0; i < scans.size(); i++)
        for (size_t j = 6; j < scans[i].size() - 6; j++)
        {
            float diffX = /* laserCloudIn.points[scans[i][j]-5].x + laserCloudIn.points[scans[i][j] - 4].x 
                    + laserCloudIn.points[scans[i][j] - 3].x + */
                laserCloudIn.points[scans[i][j] - 2].x +
                laserCloudIn.points[scans[i][j] - 1].x - 4 * laserCloudIn.points[scans[i][j]].x + laserCloudIn.points[scans[i][j] + 1].x + laserCloudIn.points[scans[i][j] + 2].x
                /*+ laserCloudIn.points[scans[i][j] + 3].x + laserCloudIn.points[scans[i][j] + 4].x
                    + laserCloudIn.points[scans[i][j] + 5].x*/
                ;
            float diffY = /*laserCloudIn.points[scans[i][j] - 5].y + laserCloudIn.points[scans[i][j] - 4].y 
                    + laserCloudIn.points[scans[i][j] - 3].y + */
                laserCloudIn.points[scans[i][j] - 2].y +
                laserCloudIn.points[scans[i][j] - 1].y - 4 * laserCloudIn.points[scans[i][j]].y + laserCloudIn.points[scans[i][j] + 1].y + laserCloudIn.points[scans[i][j] + 2].y
                /*+ laserCloudIn.points[scans[i][j] + 3].y + laserCloudIn.points[scans[i][j] + 4].y
                    + laserCloudIn.points[scans[i][j] + 5].y*/
                ;
            float diffZ = /*laserCloudIn.points[scans[i][j] - 5].z + laserCloudIn.points[scans[i][j] - 4].z 
                    + laserCloudIn.points[scans[i][j] - 3].z + */
                laserCloudIn.points[scans[i][j] - 2].z +
                laserCloudIn.points[scans[i][j] - 1].z - 4 * laserCloudIn.points[scans[i][j]].z + laserCloudIn.points[scans[i][j] + 1].z + laserCloudIn.points[scans[i][j] + 2].z
                /*+ laserCloudIn.points[scans[i][j] + 3].z + laserCloudIn.points[scans[i][j] + 4].z
                    + laserCloudIn.points[scans[i][j] + 5].z*/
                ;

            //曲率计算
            // cloudCurvature[scans[i][j]] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            // if(cloud_distance[scans[i][j]] > cloud_distance[scans[i][j]+1] && cloud_distance[scans[i][j]] > cloud_distance[scans[i][j]-1]
            //     && fabs(cloud_distance[scans[i][j]] - cloud_distance[scans[i][j]+1]) < 0.01 * cloud_distance[scans[i][j]] &&
            //     fabs(cloud_distance[scans[i][j]] - cloud_distance[scans[i][j]-1]) < 0.01 * cloud_distance[scans[i][j]])
            cloudCurvature[scans[i][j]] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            //  else cloudCurvature[scans[i][j]] = 0.0;
            // cloudSortInd[scans[i][j]] = scans[i][j];
            // std::cout<<cloudCurvature[scans[i][j]]<<"  " <<cloudSortInd[i]<<std::endl;

            float intensity_point = laserCloudIn.points[scans[i][j] - 4].intensity + laserCloudIn.points[scans[i][j] - 3].intensity + laserCloudIn.points[scans[i][j] - 2].intensity + laserCloudIn.points[scans[i][j] - 1].intensity -
                                    8 * laserCloudIn.points[scans[i][j]].intensity + laserCloudIn.points[scans[i][j] + 1].intensity +
                                    laserCloudIn.points[scans[i][j] + 2].intensity + laserCloudIn.points[scans[i][j] + 3].intensity + laserCloudIn.points[scans[i][j] + 4].intensity;

            cloudintensity[scans[i][j]] = sqrt(intensity_point * intensity_point);

            //与后一点的斜率 y与x方向
            cloudslope[scans[i][j]] = fabs(laserCloudIn.points[scans[i][j] + 2].y - laserCloudIn.points[scans[i][j] - 2].y) / (laserCloudIn.points[scans[i][j] + 2].x - laserCloudIn.points[scans[i][j] - 2].x);
        }

    //挑选点，排除容易被斜面挡住的点以及离群点
    for (size_t i = 5; i < laserCloudIn.points.size() - 6; i++)
    {
        float diffX = laserCloudIn.points[i + 1].x - laserCloudIn.points[i].x;
        float diffY = laserCloudIn.points[i + 1].y - laserCloudIn.points[i].y;
        float diffZ = laserCloudIn.points[i + 1].z - laserCloudIn.points[i].z;
        //计算有效曲率点与后一个点之间的距离平方和
        float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

        if (diff > 0.1)
        { //前提:两个点之间距离要大于0.1

            //点的深度
            float depth1 = sqrt(laserCloudIn.points[i].x * laserCloudIn.points[i].x +
                                laserCloudIn.points[i].y * laserCloudIn.points[i].y +
                                laserCloudIn.points[i].z * laserCloudIn.points[i].z);

            //后一个点的深度
            float depth2 = sqrt(laserCloudIn.points[i + 1].x * laserCloudIn.points[i + 1].x +
                                laserCloudIn.points[i + 1].y * laserCloudIn.points[i + 1].y +
                                laserCloudIn.points[i + 1].z * laserCloudIn.points[i + 1].z);

            //按照两点的深度的比例，将深度较大的点拉回后计算距离
            if (depth1 > depth2)
            {
                diffX = laserCloudIn.points[i + 1].x - laserCloudIn.points[i].x * depth2 / depth1;
                diffY = laserCloudIn.points[i + 1].y - laserCloudIn.points[i].y * depth2 / depth1;
                diffZ = laserCloudIn.points[i + 1].z - laserCloudIn.points[i].z * depth2 / depth1;

                //边长比也即是弧度值，若小于0.1，说明夹角比较小，斜面比较陡峭,点深度变化比较剧烈,点处在近似与激光束平行的斜面上
                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.15)
                { //排除容易被斜面挡住的点
                    //该点及前面五个点（大致都在斜面上）全部置为筛选过
                    // cloudNeighborPicked[i - 5] = 1;
                    // cloudNeighborPicked[i - 4] = 1;
                    // cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
            }
            else
            {
                diffX = laserCloudIn.points[i + 1].x * depth1 / depth2 - laserCloudIn.points[i].x;
                diffY = laserCloudIn.points[i + 1].y * depth1 / depth2 - laserCloudIn.points[i].y;
                diffZ = laserCloudIn.points[i + 1].z * depth1 / depth2 - laserCloudIn.points[i].z;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.15)
                {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    // cloudNeighborPicked[i + 4] = 1;
                    // cloudNeighborPicked[i + 5] = 1;
                    // cloudNeighborPicked[i + 6] = 1;
                }
            }
        }

        float diffX2 = laserCloudIn.points[i].x - laserCloudIn.points[i - 1].x;
        float diffY2 = laserCloudIn.points[i].y - laserCloudIn.points[i - 1].y;
        float diffZ2 = laserCloudIn.points[i].z - laserCloudIn.points[i - 1].z;
        //与前一个点的距离平方和
        float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

        //点深度的平方和
        float dis = laserCloudIn.points[i].x * laserCloudIn.points[i].x + laserCloudIn.points[i].y * laserCloudIn.points[i].y + laserCloudIn.points[i].z * laserCloudIn.points[i].z;

        cloud_distance[i] = sqrt(dis);
        //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
        if (diff > 0.05 * dis || diff2 > 0.05 * dis || dis > 2500 || dis < 0.04)
        {
            cloudNeighborPicked[i] = 1;
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr intensityPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());

    for (size_t i = 0; i < scans.size(); i++)
    {
        for (size_t j = 10; j < scans[i].size() - 10; j++)
        {
            if (cloudCurvature[scans[i][j]] > 0.05 && cloudNeighborPicked[scans[i][j]] != 1 && fabs(cloudslope[scans[i][j] - 1] - cloudslope[scans[i][j]]) > 0.5 &&
                (cloud_distance[scans[i][j] - 1] - cloud_distance[scans[i][j]] > 0) && (cloud_distance[scans[i][j] + 1] > cloud_distance[scans[i][j]]) &&
                laserCloudIn.points[scans[i][j]].z > 0.06)
            {
                cornerPointsSharp->points.push_back(laserCloudIn.points[scans[i][j]]);
            }
            else if (cloudCurvature[scans[i][j]] < 0.0005 && cloudNeighborPicked[scans[i][j]] != 1)
            {
                surfPointsSharp->points.push_back(laserCloudIn.points[scans[i][j]]);
            }
            if (cloudintensity[scans[i][j]] > 150 && laserCloudIn.points[scans[i][j]].z < 0.3 /*  && laserCloudIn.points[scans[i][j]].intensity > laserCloudIn.points[scans[i][j]-1].intensity*/)
            {
                intensityPointsSharp->points.push_back(laserCloudIn.points[scans[i][j]]);
            }
        }
    }

    std::cout << "cornerPointsLessSharp" << cornerPointsSharp->points.size() << std::endl;
    std::cout << "surfPointsSharp  " << surfPointsSharp->points.size() << std::endl;

    // if(gps_data.GpsDataIsOk() && AdjustLidarOdom(laserCloudMsg->header.stamp.toSec()))
    // plicp.ReceivePosition(gps_data.quaternion_gps, gps_data.position_gps);
    double time_feature = clock();
    std::cout<<"feature time : "<<(double)(time_feature - t1) / CLOCKS_PER_SEC << endl;
    plicp.GetPL_ICP(cornerPointsSharp, surfPointsSharp); //载入当前特征点

    std::cout<<"icp time : "<<(double)(clock() - time_feature) / CLOCKS_PER_SEC << endl;
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.header.stamp = laserCloudMsg->header.stamp;
    odomAftMapped.pose.pose.orientation.x = plicp.m_q_w_curr.x(); //得到位姿
    odomAftMapped.pose.pose.orientation.y = plicp.m_q_w_curr.y();
    odomAftMapped.pose.pose.orientation.z = plicp.m_q_w_curr.z();
    odomAftMapped.pose.pose.orientation.w = plicp.m_q_w_curr.w();

    odomAftMapped.pose.pose.position.x = plicp.m_t_w_curr.x();
    odomAftMapped.pose.pose.position.y = plicp.m_t_w_curr.y();
    odomAftMapped.pose.pose.position.z = plicp.m_t_w_curr.z();

    odomAftMapped.pose.covariance[0] = fabs(plicp.odom_cov_ -0.002) * 800;
    // ROS_ERROR("the lidar cov %f , %f： ", odomAftMapped.pose.covariance[0], plicp.odom_cov_);
    // odomAftMapped.pose.covariance[0] = plicp.odom_covariance[0];
    odomAftMapped.pose.covariance[7] = fabs(plicp.odom_cov_ -0.002) * 800;
    odomAftMapped.pose.covariance[14] = fabs(plicp.odom_cov_ -0.002) * 800;
    // double max_cov = 0.0;
    // for (int i = 6; i > 2; i--)
    // {
    //     if (max_cov < plicp.odom_covariance[i])
    //         max_cov = plicp.odom_covariance[i];
    // }
    // odomAftMapped.pose.covariance[21] = plicp.odom_covariance[4];
    // odomAftMapped.pose.covariance[28] = plicp.odom_covariance[5];
    // odomAftMapped.pose.covariance[35] = plicp.odom_covariance[6];
    lidar_odom_pub.publish(odomAftMapped);

    if(!showed_map_)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr surf_map(new pcl::PointCloud<pcl::PointXYZI>());
        surf_map = plicp.m_laser_cloud_corner_from_map;
        std::cout << "map cloud points sizes: " << surf_map->points.size() << std::endl;
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*surf_map, cloudMsgTemp);
        cloudMsgTemp.header.frame_id = "camera_init";
        cloudMsgTemp.header.stamp = laserCloudMsg->header.stamp;
        cloud_map_pub.publish(cloudMsgTemp);
        // ros::Time current_time = laserCloudMsg->header.stamp;
        // sensor_msgs::PointCloud2 temp_out_msg;
        showed_map_ = true;
    }
    if (odomAftMapped.pose.covariance[0] > 5)
    {
        if (/* lidar_odom_error_times_ > 0 &&*/ receive_new_fusion_odom_)
        {
            fusion_odom_.z() =  plicp.m_t_w_curr.z();
            plicp.ReceivePosition(fusion_quater_, fusion_odom_);
        }
        lidar_odom_error_times_++;
    }
    else
        lidar_odom_error_times_ = 0;

    cout << "lidar odom all time" << (double)(clock() - t1) / CLOCKS_PER_SEC << endl;
}

void GpsHandler(const nav_msgs::OdometryConstPtr &input_odom)
{
    double start = clock();
    gps_data.GetGpsDatas(input_odom, odom_quters, odom_positions,
                         gps_quters, gps_positions);

    nav_msgs::Odometry odomAftMapped_gps;
    odomAftMapped_gps.header.frame_id = "camera_init";
    odomAftMapped_gps.header.stamp = input_odom->header.stamp;
    odomAftMapped_gps.pose.pose.orientation.x = gps_data.quaternion_gps.x(); //得到位姿
    odomAftMapped_gps.pose.pose.orientation.y = gps_data.quaternion_gps.y();
    odomAftMapped_gps.pose.pose.orientation.z = gps_data.quaternion_gps.z();
    odomAftMapped_gps.pose.pose.orientation.w = gps_data.quaternion_gps.w();

    odomAftMapped_gps.pose.pose.position.x = gps_data.position_gps.x();
    odomAftMapped_gps.pose.pose.position.y = gps_data.position_gps.y();
    odomAftMapped_gps.pose.pose.position.z = gps_data.position_gps.z();

    odomAftMapped_gps.pose.covariance[0] = gps_data.covariance_gps[0];
    odomAftMapped_gps.pose.covariance[7] = gps_data.covariance_gps[1];
    odomAftMapped_gps.pose.covariance[14] = gps_data.covariance_gps[2];
    odomAftMapped_gps.pose.covariance[21] = gps_data.covariance_gps[3];
    odomAftMapped_gps.pose.covariance[28] = gps_data.covariance_gps[4];
    odomAftMapped_gps.pose.covariance[35] = gps_data.covariance_gps[5];
    gps_odom_pub.publish(odomAftMapped_gps);
    std::cout << "gps time" << (double)(clock() - start) / CLOCKS_PER_SEC << std::endl;
}

void FristOdomHander(const geometry_msgs::PoseWithCovarianceStamped receivepose)
{
    // Eigen::Quaterniond frist_pose_quater(receivepose.pose.orientation.w(), receivepose.pose.orientation.x(), receivepose.pose.orientation.y(), receivepose.pose.orientation.z());
    // Eigen::Vector3d frist_pose_position(receivepose.pose.position.x(), receivepose.pose.position.y(), receivepose.pose.position.z());
    frist_odom_pose.GetFristPose(receivepose);
    nav_msgs::Odometry odomAftMapped_frist;
    odomAftMapped_frist.header.frame_id = "camera_init";
    odomAftMapped_frist.header.stamp = receivepose.header.stamp;
    odomAftMapped_frist.pose.pose.orientation.x = frist_odom_pose.frist_pose_quater.x(); //得到位姿
    odomAftMapped_frist.pose.pose.orientation.y = frist_odom_pose.frist_pose_quater.y();
    odomAftMapped_frist.pose.pose.orientation.z = frist_odom_pose.frist_pose_quater.z();
    odomAftMapped_frist.pose.pose.orientation.w = frist_odom_pose.frist_pose_quater.w();

    odomAftMapped_frist.pose.pose.position.x = frist_odom_pose.frist_pose_position.x();
    odomAftMapped_frist.pose.pose.position.y = frist_odom_pose.frist_pose_position.y();
    odomAftMapped_frist.pose.pose.position.z = frist_odom_pose.frist_pose_position.x();
    pub_frist_odom.publish(odomAftMapped_frist);
}

void ReceiveFusionOdom(const nav_msgs::Odometry input_odom)
{
    fusion_odom_ = Eigen::Vector3d(input_odom.pose.pose.position.x, input_odom.pose.pose.position.y, 0.0);
    fusion_quater_ = Eigen::Quaterniond(input_odom.pose.pose.orientation.w, input_odom.pose.pose.orientation.x,
                                        input_odom.pose.pose.orientation.y, input_odom.pose.pose.orientation.z);
    fusion_time_ = input_odom.header.stamp;
    receive_new_fusion_odom_ = true;
}

std::vector<std::string> StringSplit(const std::string &str,
                                     const std::string &delim)
{
    std::vector<std::string> res;
    if ("" == str)
        return res;
    char *strs = new char[str.length() + 1];
    strcpy(strs, str.c_str());

    char *d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());

    char *p = strtok(strs, d);
    while (p)
    {
        std::string s = p;
        res.push_back(s);
        p = strtok(NULL, d);
    }

    return res;
}

void ReadPoseFile()
{
    std::ifstream read_stream;
    std::string gps_odom_pose =
        "../path/frist_gps_odom.txt";
    read_stream.open(gps_odom_pose, std::ios_base::in);
    std::string line_str;
    int i = 0;
    // Eigen::Vector3d ea = Eigen::Vector3d(-3.1415926 / 2.0, 0.0, 0.0);
    // Eigen::Quaterniond quaternion3 =
    //     Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
    //     Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
    //     Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    while (getline(read_stream, line_str, '\n'))
    {
        std::vector<std::string> pose_str = StringSplit(line_str, "|");
        if (i % 2 == 0)
        {
            Eigen::Quaterniond odom_correspond_gps_quaterniond;
            Eigen::Vector3d odom_correspond_gps_position;
            odom_correspond_gps_position.x() = atof(pose_str[0].c_str());
            odom_correspond_gps_position.y() = atof(pose_str[1].c_str());
            odom_correspond_gps_position.z() = atof(pose_str[2].c_str());
            odom_correspond_gps_quaterniond.x() = atof(pose_str[3].c_str());
            odom_correspond_gps_quaterniond.y() = atof(pose_str[4].c_str());
            odom_correspond_gps_quaterniond.z() = atof(pose_str[5].c_str());
            odom_correspond_gps_quaterniond.w() = atof(pose_str[6].c_str());
            // odom_angle.push_back(odom_correspond_gps_quaterniond.matrix().eulerAngles(2, 1, 0));
            odom_quters.push_back(odom_correspond_gps_quaterniond);
            odom_positions.push_back(odom_correspond_gps_position);
        }
        else
        {
            Eigen::Quaterniond gps_correspond_odom_quaterniond;
            Eigen::Vector3d gps_correspond_odom_position;
            gps_correspond_odom_position.x() = atof(pose_str[0].c_str());
            gps_correspond_odom_position.y() = atof(pose_str[1].c_str());
            gps_correspond_odom_position.z() = atof(pose_str[2].c_str());
            gps_correspond_odom_quaterniond.x() = atof(pose_str[3].c_str());
            gps_correspond_odom_quaterniond.y() = atof(pose_str[4].c_str());
            gps_correspond_odom_quaterniond.z() = atof(pose_str[5].c_str());
            gps_correspond_odom_quaterniond.w() = atof(pose_str[6].c_str());
            CorrectionGps(gps_correspond_odom_quaterniond, gps_correspond_odom_position);
            gps_quters.push_back(gps_correspond_odom_quaterniond);
            gps_positions.push_back(gps_correspond_odom_position);
        }
        i++;
    }
}
void ReadPathFile()
{
    std::ifstream read_stream;
    std::string gps_odom_pose =
        "../path/path.txt";
    read_stream.open(gps_odom_pose, std::ios_base::in);
    std::string line_str;

    while (getline(read_stream, line_str, '\n'))
    {
        std::vector<std::string> pose_str = StringSplit(line_str, "|");

        Eigen::Quaterniond odom_correspond_gps_quaterniond;
        Eigen::Vector3d odom_correspond_gps_position;
        odom_correspond_gps_position.x() = atof(pose_str[0].c_str());
        odom_correspond_gps_position.y() = atof(pose_str[1].c_str());
        odom_correspond_gps_position.z() = atof(pose_str[2].c_str());
        odom_correspond_gps_quaterniond.x() = atof(pose_str[3].c_str());
        odom_correspond_gps_quaterniond.y() = atof(pose_str[4].c_str());
        odom_correspond_gps_quaterniond.z() = atof(pose_str[5].c_str());
        odom_correspond_gps_quaterniond.w() = atof(pose_str[6].c_str());
        // odom_angle.push_back(odom_correspond_gps_quaterniond.matrix().eulerAngles(2, 1, 0));
        odom_quater_path.push_back(odom_correspond_gps_quaterniond);
        odom_position_path.push_back(odom_correspond_gps_position);
    }
}

void param_init(ros::NodeHandle nh)
{
    int max_iteration;
    nh.param<int>("max_iteration", max_iteration, 6);
    //得到地图
    pcl::PointCloud<pcl::PointXYZI> cloud_tgt;
    if (pcl::io::loadPCDFile("../sweeper_ws/src/sweeper_haide/data/map/surround.pcd", cloud_tgt))
        std::cout << "cloud load ok" << std::endl;
    plicp.GetCornerSurfMap(cloud_tgt);
    plicp.m_para_icp_max_iterations = max_iteration;
    //得到gps与odom的位置
    //ReadPoseFile(); //将gps转到地图
    //ReadPathFile(); //path点
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_feature");
    ros::NodeHandle nh;
    param_init(nh);
    ros::Subscriber sub, sub_gps, frist_odom_sub, sub_fusion_odom;
    sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, GetScan);
    // sub_gps = nh.subscribe<nav_msgs::Odometry>("/sweeper/sensor/gnss", 1, GpsHandler);
    frist_odom_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, FristOdomHander);
    sub_fusion_odom = nh.subscribe<nav_msgs::Odometry>("/sweeper/localization/fusion_position", 1, ReceiveFusionOdom);

    lidar_odom_pub = nh.advertise<nav_msgs::Odometry>("/sweeper/navigation/lidarodom", 1);
    gps_odom_pub = nh.advertise<nav_msgs::Odometry>("/sweeper/navigation/gpsodom", 1);
    pub_frist_odom = nh.advertise<nav_msgs::Odometry>("/sweeper/navigation/frist_odom", 1);
    cloud_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/sweeper/navigation/map", 1, true);
    ros::spin();
}

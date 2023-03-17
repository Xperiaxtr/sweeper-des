#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "sweeper_msgs/SweeperChassisDetail.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

const int imu_length_ = 200;

class ImuOdom
{
public:
    ImuOdom();
    ~ImuOdom(){};
    void GetSweeperData(const sweeper_msgs::SweeperChassisDetail sweeper_chassis_detail);
    void GetImuData(const sensor_msgs::Imu imu_data);
    void GetOdom();

private:
    ros::Subscriber sub_imu_data;
    ros::Subscriber sub_sweeper_data;
    ros::Publisher pub_odom_data;
    bool frist_imu_data_ = true;
    bool frist_sweeper_data_ = true;
    bool receive_new_imu_data_ = false;
    int imu_pointer_last_;
    int imu_point_front_;
    double imu_time_[imu_length_] = {0};
    double sweeper_time_;
    double sweeper_time_last_;
    double sweeper_speed_now_;
    double sweeper_speed_last_;
    double shift_x_;
    double shift_y_;
    double shift_z_;
    Eigen::Matrix3d imu_tran_angle_;
    Eigen::Quaterniond imu_quater_[imu_length_];
    Eigen::Quaterniond imu_quater_last_;
    Eigen::Quaterniond imu_quater_out_;
    Eigen::Quaterniond imu_quater_frist_;
    Eigen::Quaterniond lidar_imu_quater_;
    Eigen::Vector3d lidar_imu_position_;
};

ImuOdom::ImuOdom() : imu_pointer_last_(0), imu_point_front_(0), sweeper_time_(0), sweeper_time_last_(0.0),
                     sweeper_speed_now_(0.0), shift_x_(0.0), shift_y_(0.0), shift_z_(0.0), sweeper_speed_last_(0.0)
{
    ros::NodeHandle nh;
    sub_imu_data = nh.subscribe<sensor_msgs::Imu>("/imu", 10, &ImuOdom::GetImuData, this);
    sub_sweeper_data = nh.subscribe<sweeper_msgs::SweeperChassisDetail>("/sweeper/chassis/detail", 10, &ImuOdom::GetSweeperData, this);
    pub_odom_data = nh.advertise<nav_msgs::Odometry>("/sweeper/localization/sweeper_odom", 1);
    imu_quater_last_ = imu_quater_out_ = imu_quater_frist_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    lidar_imu_quater_ = Eigen::Quaterniond(0.999204363905233, 0.0394940572990208, -0.00553136979894671, 0.000512385544304862);
    lidar_imu_position_<<-0.231367950413783, 0.683594389933664, -2;
}

void ImuOdom::GetSweeperData(const sweeper_msgs::SweeperChassisDetail sweeper_chassis_detail)
{
    sweeper_time_ = sweeper_chassis_detail.header.stamp.toSec();
    sweeper_speed_now_ = sweeper_chassis_detail.vehicle_speed_output;
    if (frist_sweeper_data_)
    {
        sweeper_time_last_ = sweeper_time_;
        frist_sweeper_data_ = false;
    }
    if (!receive_new_imu_data_)
    {
        return;
    }
    receive_new_imu_data_ = false;
    GetOdom();

    Eigen::Quaterniond sweeper_quater_out;
    sweeper_quater_out =  /* imu_quater_frist_.inverse() */ imu_quater_out_;

    Eigen::Matrix<double, 4, 4> lidar_to_imu_matrix;
    lidar_to_imu_matrix.block(0, 0, 3, 3) = lidar_imu_quater_.matrix();
    lidar_to_imu_matrix.block(0, 3, 3, 1) = lidar_imu_position_;
    lidar_to_imu_matrix(3, 3) = 1;
    lidar_to_imu_matrix(3, 0) = 0;
    lidar_to_imu_matrix(3, 1) = 0;
    lidar_to_imu_matrix(3, 2) = 0;

    Eigen::Matrix<double, 4, 4> imu_to_world_matrix;
    imu_to_world_matrix.block(0, 0, 3, 3) = sweeper_quater_out.matrix();
    imu_to_world_matrix(0, 3) = shift_x_;
    imu_to_world_matrix(1, 3) = shift_y_;
    imu_to_world_matrix(2, 3) = shift_z_;

    imu_to_world_matrix(3, 3) = 1;
    imu_to_world_matrix(3, 0) = 0;
    imu_to_world_matrix(3, 1) = 0;
    imu_to_world_matrix(3, 2) = 0;

    Eigen::Matrix<double, 4, 4> lidar_to_world_matrix = /* lidar_to_imu_matrix.inverse() */ imu_to_world_matrix;// * lidar_to_imu_matrix;
    Eigen::Matrix3d lidar_to_world_rotion = lidar_to_world_matrix.block(0, 0, 3, 3);
    Eigen::Quaterniond lidar_to_world_quater(lidar_to_world_rotion);

    nav_msgs::Odometry odom_now;
    odom_now.header.frame_id = "/camera_init";
    // Eigen::Quaterniond quat(imu_tran_angle_);
    odom_now.header.stamp = sweeper_chassis_detail.header.stamp;

    odom_now.pose.pose.orientation.x = lidar_to_world_quater.x();
    odom_now.pose.pose.orientation.y = lidar_to_world_quater.y();
    odom_now.pose.pose.orientation.z = lidar_to_world_quater.z();
    odom_now.pose.pose.orientation.w = lidar_to_world_quater.w();
    odom_now.pose.pose.position.x = lidar_to_world_matrix(0, 3);
    odom_now.pose.pose.position.y = lidar_to_world_matrix(1, 3);
    odom_now.pose.pose.position.z = lidar_to_world_matrix(2, 3);

    odom_now.twist.twist.linear.x = 0.5 * (sweeper_speed_now_ + sweeper_speed_last_);
    pub_odom_data.publish(odom_now);
}

void ImuOdom::GetImuData(const sensor_msgs::Imu imu_data)
{
    double roll, pitch, yaw;
    Eigen::Quaterniond imu_orientation(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);
    Eigen::Vector3d yaw_tran_90(-3.1415926 / 2.0, 0.0, 0.0);
    Eigen::Quaterniond yaw_tran_90_quater;
    yaw_tran_90_quater = Eigen::AngleAxisd(yaw_tran_90[0], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(yaw_tran_90[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(yaw_tran_90[2], Eigen::Vector3d::UnitX());
    Eigen::Quaterniond tran_90_imu = yaw_tran_90_quater * imu_orientation;
    tf::Quaternion quat(tran_90_imu.x(), tran_90_imu.y(), tran_90_imu.z(), tran_90_imu.w());
    tf::Matrix3x3(quat).getRPY(yaw, roll, pitch);
    double time = imu_data.header.stamp.toSec();

    imu_pointer_last_ = (imu_pointer_last_ + 1) % imu_length_;

    // Eigen::Quaterniond calib_imu_data = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    //                                     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    //                                     Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    imu_time_[imu_pointer_last_] = time;
    imu_quater_[imu_pointer_last_] = tran_90_imu;

    if (frist_imu_data_)
    {

        frist_imu_data_ = false;
        imu_quater_frist_ = imu_quater_last_ = tran_90_imu;
    }
    receive_new_imu_data_ = true;
}

void ImuOdom::GetOdom()
{
    if (imu_pointer_last_ >= 0)
    {
        Eigen::Quaterniond imu_quater_now = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        //找到车速传感器时间戳小于imu时间戳的位置
        while (imu_point_front_ != imu_pointer_last_)
        {
            if (sweeper_time_ > imu_time_[imu_point_front_] && sweeper_time_ < imu_time_[(imu_point_front_ + 1) % imu_length_])
                break;
            imu_point_front_ = (imu_point_front_ + 1) % imu_length_;
        }
        if (sweeper_time_ > imu_time_[(imu_point_front_ + 1) % imu_length_])
        {
            imu_quater_now = imu_quater_[imu_point_front_];
        }
        else
        {
            int imu_pointer_back = (imu_point_front_ + 1) % imu_length_;
            double ratio_front = (sweeper_time_ - imu_time_[imu_pointer_back]) / (imu_time_[imu_pointer_back] - imu_time_[imu_point_front_]);
            double ratio_back = 1.0 - ratio_front;
            imu_quater_now = imu_quater_[imu_point_front_].slerp(ratio_front, imu_quater_[imu_pointer_back]);
        }
        imu_quater_out_ = imu_quater_now;

        //转换为旋转矩阵
        Eigen::Vector3d speed_sweeper(0.5 * (sweeper_speed_now_ + sweeper_speed_last_), 0.0, 0.0);

        Eigen::Quaterniond car_to_imu =  /* imu_quater_frist_.inverse() */ imu_quater_out_;

        Eigen::Vector3d speed_xyz = (/*imu_to_car_tran */ car_to_imu) * (speed_sweeper);
        sweeper_speed_last_ = sweeper_speed_now_;
        double diff_time = sweeper_time_ - sweeper_time_last_;

        shift_x_ = shift_x_ + speed_xyz.x() * diff_time;
        shift_y_ = shift_y_ + speed_xyz.y() * diff_time;
        shift_z_ = shift_z_ + speed_xyz.z() * diff_time;
        std::cout << "x,y,z : " << shift_x_ << " " << shift_y_ << " " << shift_z_ << std::endl;
    }
    sweeper_time_last_ = sweeper_time_;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_imu_and_sweeper");
    ImuOdom imu_odom;
    ros::spin();
    return 0;
}
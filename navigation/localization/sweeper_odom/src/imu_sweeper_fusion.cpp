#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "sweeper_msgs/SweeperChassisDetail.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

const int imu_length_ = 200;

std::vector<Eigen::Vector3d> body_to_imu_;

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
    Eigen::Vector3d imu_linacc_[imu_length_];
    double imu_angular_velocity_[imu_length_]; //z轴方向
    double imu_angular_velocity_now_;
    Eigen::Vector3d imu_linacc_now_;
    Eigen::Vector3d speed_xyz_;
    // std::vector<Eigen::Vector3d> body_to_imu_;
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
    
            double roll, pitch, yaw;
        // Eigen::Quaterniond car_to_imu_inverse = car_to_imu.inverse();
        tf::Quaternion quat(lidar_imu_quater_.x(), lidar_imu_quater_.y(), lidar_imu_quater_.z(), lidar_imu_quater_.w());
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        std::cout << "imu angluar : " << yaw << " " << roll << " " << pitch << std::endl;

    lidar_imu_position_ << -0.231367950413783, 0.683594389933664, -2;
    body_to_imu_.clear();
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
    sweeper_quater_out = imu_quater_out_ * lidar_imu_quater_;

    // double roll, pitch, yaw;
    // roll = -0.00886667;
    // pitch = -0.00633416;
    // yaw = 0.00425338;
    // Eigen::Quaterniond imu_to_body_quater;
    // imu_to_body_quater = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
    //             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
    //             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    // sweeper_quater_out = imu_to_body_quater * sweeper_quater_out;

    nav_msgs::Odometry odom_now;
    odom_now.header.frame_id = "/camera_init";
    odom_now.header.stamp = sweeper_chassis_detail.header.stamp;
    odom_now.pose.pose.orientation.x = sweeper_quater_out.x();
    odom_now.pose.pose.orientation.y = sweeper_quater_out.y();
    odom_now.pose.pose.orientation.z = sweeper_quater_out.z();
    odom_now.pose.pose.orientation.w = sweeper_quater_out.w();
    odom_now.pose.pose.position.x = shift_x_;
    odom_now.pose.pose.position.y = shift_y_;
    odom_now.pose.pose.position.z = shift_z_;
    pub_odom_data.publish(odom_now);
}

void ImuOdom::GetImuData(const sensor_msgs::Imu imu_data)
{
    // double roll, pitch, yaw;
    Eigen::Quaterniond imu_orientation(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);
    double time = imu_data.header.stamp.toSec();
    Eigen::Vector3d imu_acc(imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);
    Eigen::Vector3d g_acc(0.0, 0.0, 9.81);
    Eigen::Vector3d g_in_imu = imu_orientation.inverse() * g_acc;
    Eigen::Vector3d imu_linacc = imu_acc + g_in_imu;
    std::cout << "imu acc : " << imu_linacc << std::endl;
    double imu_angular_velocity = imu_data.angular_velocity.z;

    // double imu_linacc_xy = sqrt(acc_in_imu.x() * acc_in_imu.x() + acc_in_imu.y() * acc_in_imu.y());
    imu_pointer_last_ = (imu_pointer_last_ + 1) % imu_length_;
    imu_time_[imu_pointer_last_] = time;
    imu_quater_[imu_pointer_last_] = imu_orientation;
    imu_angular_velocity_[imu_pointer_last_] = imu_angular_velocity;
    imu_linacc_[imu_pointer_last_] = imu_linacc;

    if (frist_imu_data_)
    {

        frist_imu_data_ = false;
        imu_quater_frist_ = imu_quater_last_ = imu_orientation;
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
        //没找到
        if (sweeper_time_ > imu_time_[(imu_point_front_ + 1) % imu_length_])
        {
            imu_quater_now = imu_quater_[imu_point_front_];
            imu_angular_velocity_now_ = imu_angular_velocity_[imu_point_front_];
            imu_linacc_now_ = imu_linacc_[imu_point_front_];
        }
        else
        {
            int imu_pointer_back = (imu_point_front_ + 1) % imu_length_;
            double ratio_front = (sweeper_time_ - imu_time_[imu_pointer_back]) / (imu_time_[imu_pointer_back] - imu_time_[imu_point_front_]);
            double ratio_back = 1.0 - ratio_front;
            imu_quater_now = imu_quater_[imu_point_front_].slerp(ratio_front, imu_quater_[imu_pointer_back]);

            imu_angular_velocity_now_ = ratio_front * imu_angular_velocity_[imu_point_front_] + ratio_back * imu_angular_velocity_[imu_pointer_back];
            imu_linacc_now_ = ratio_front * imu_linacc_[imu_point_front_] + ratio_back * imu_linacc_[imu_pointer_back];
        }
        imu_quater_out_ = imu_quater_now;

        //转换为旋转矩阵
        Eigen::Vector3d speed_sweeper(0.5 * (sweeper_speed_now_ + sweeper_speed_last_), 0.0, 0.0);

        Eigen::Quaterniond car_to_imu = imu_quater_out_ * lidar_imu_quater_;

        // double roll, pitch, yaw;
        // Eigen::Quaterniond car_to_imu_inverse = car_to_imu.inverse();
        // tf::Quaternion quat(car_to_imu_inverse.x(), car_to_imu_inverse.y(), car_to_imu_inverse.z(), car_to_imu_inverse.w());
        // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        // std::cout << "imu angluar : " << yaw << " " << roll << " " << pitch << std::endl;
        // Eigen::Vector3d body_to_imu(roll, pitch, yaw);
        // body_to_imu_.push_back(body_to_imu);



    // double roll1, pitch1, yaw1;
    // roll1 = 0.00425331;
    // pitch1 = -0.00886667;
    // yaw1 = -0.00633422;
    // Eigen::Quaterniond imu_to_body_quater;
    // imu_to_body_quater = Eigen::AngleAxisd(yaw1, Eigen::Vector3d::UnitZ()) * 
    //             Eigen::AngleAxisd(pitch1, Eigen::Vector3d::UnitY()) * 
    //             Eigen::AngleAxisd(roll1, Eigen::Vector3d::UnitX());

    // car_to_imu = imu_to_body_quater * car_to_imu;

        speed_xyz_ = (/*imu_to_car_tran */ car_to_imu) * (speed_sweeper);
        sweeper_speed_last_ = sweeper_speed_now_;
        double diff_time = sweeper_time_ - sweeper_time_last_;

        shift_x_ = shift_x_ + speed_xyz_.x() * diff_time;
        shift_y_ = shift_y_ + speed_xyz_.y() * diff_time;
        shift_z_ = shift_z_ + speed_xyz_.z() * diff_time;
        std::cout << "x,y,z : " << shift_x_ << " " << shift_y_ << " " << shift_z_ << std::endl;

    }
    sweeper_time_last_ = sweeper_time_;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_imu_and_sweeper");
    ImuOdom imu_odom;
    ros::spin();
    double roll_avearge = 0.0, pitch_avearge = 0.0, yaw_avearge = 0.0;
    for(size_t i = 0; i < body_to_imu_.size();i++)
    {
        roll_avearge += body_to_imu_[i].x();
        pitch_avearge += body_to_imu_[i].y();
        yaw_avearge += body_to_imu_[i].z();
    }
    double roll = roll_avearge/body_to_imu_.size();
    double pitch = pitch_avearge/body_to_imu_.size();
    double yaw = yaw_avearge/body_to_imu_.size();
    std::cout<<"rpy : "<<roll<<" "<<pitch<<" "<<yaw<<std::endl;
    Eigen::Quaterniond imu_to_body_quater;
    imu_to_body_quater = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    std::cout<<"quater : "<<imu_to_body_quater.w()<<" "<<imu_to_body_quater.x()<<" "<<imu_to_body_quater.y()<<" "<<imu_to_body_quater.z()<<std::endl;
    return 0;
}
//得到imu到gps的pitch角度差，
//对于gps，横滚角没有意义，但是俯仰角对gps位置有影响。
//方法：对于gps俯仰角可以由速度计算得到俯仰角和偏航角

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "sensor_msgs/MagneticField.h"
#include <tf/transform_broadcaster.h>

Eigen::Quaterniond gps_quaters_[20];
Eigen::Vector3d gps_positions_[20];
double imu_times_[100];
Eigen::Quaterniond imu_quaters_[100];
int imu_pointer_last_ = 0;
Eigen::Quaterniond imu_min_quater_;
int init_gps_times_ = 0;
Eigen::Vector3d gps_position_last_;
std::vector<double> delta_roll_;
std::vector<double> delta_pitch_;

ros::Publisher pub_fusion_odom;

void GpsReceive(const nav_msgs::Odometry gps_data)
{
    if (gps_data.pose.covariance[0] > 0.3 || gps_data.pose.covariance[35] > 20)
        return;
    Eigen::Vector3d yaw_tran_90(3.1415 / 2.0, 0.0, 0.0);
    Eigen::Quaterniond yaw_tran_90_quater;
    yaw_tran_90_quater = Eigen::AngleAxisd(yaw_tran_90[0], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(yaw_tran_90[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(yaw_tran_90[2], Eigen::Vector3d::UnitX());

    Eigen::Quaterniond gps_quater(gps_data.pose.pose.orientation.w, gps_data.pose.pose.orientation.x, gps_data.pose.pose.orientation.y, gps_data.pose.pose.orientation.z);

    Eigen::Vector3d gps_position(gps_data.pose.pose.position.x, gps_data.pose.pose.position.y, gps_data.pose.pose.position.z);
    double time = gps_data.header.stamp.toSec();

    gps_quater = yaw_tran_90_quater * gps_quater;

    if (init_gps_times_ < 70)
    {
        init_gps_times_++;
        gps_position_last_ = gps_position;
        return;
    }

    double min_time = 1.0;
    int imu_point = 0;
    for (size_t i = 0; i < 100; i++)
    {
        if (fabs(imu_times_[i] - time) < min_time)
        {
            min_time = fabs(imu_times_[i] - time);
            imu_point = i;
        }
    }

    tf::Quaternion quat1(gps_quater.x(), gps_quater.y(), gps_quater.z(), gps_quater.w());
    double roll1, pitch1, yaw1;
    tf::Matrix3x3(quat1).getRPY(roll1, pitch1, yaw1);
    std::cout << "raw gps rpy : " << roll1 << " " << pitch1 << " " << yaw1 << std::endl;

    if (fabs(imu_times_[imu_point] - time) < 0.05)
    {
        Eigen::Quaterniond imu_quater_now = imu_quaters_[imu_point];
        //计算gps的位姿
        double delta_x = gps_position.x() - gps_position_last_.x();
        double delta_y = gps_position.y() - gps_position_last_.y();
        double delta_z = gps_position.z() - gps_position_last_.z();
        // if (delta_x * delta_x + delta_y * delta_y + delta_z * delta_z < 1)
        // {
        //     return;
        // }

        double pitch = atan2(delta_z, sqrt(delta_y * delta_y + delta_x * delta_x));
        double yaw = atan2(delta_x, sqrt(delta_y * delta_y + delta_z * delta_z));
        std::cout << "gps yaw : " << yaw << std::endl;
        std::cout << "gps pitch : " << pitch << std::endl;

        Eigen::Quaterniond delta_gps_angular = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                               Eigen::AngleAxisd(roll1, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond delta_gps_imu = imu_quater_now.inverse() * delta_gps_angular;//gps与imu的角度差

        tf::Quaternion quat(delta_gps_imu.x(), delta_gps_imu.y(), delta_gps_imu.z(), delta_gps_imu.w());
        double roll2, pitch2, yaw2;
        tf::Matrix3x3(quat).getRPY(roll2, pitch2, yaw2);
        std::cout << "rpy : " << roll2 << " " << pitch2 << " " << yaw2 << std::endl;
        gps_position_last_ = gps_position;

        tf::Quaternion quat3(imu_quater_now.x(), imu_quater_now.y(), imu_quater_now.z(), imu_quater_now.w());
        double roll3, pitch3, yaw3;
        tf::Matrix3x3(quat3).getRPY(roll3, pitch3, yaw3);
        std::cout << "rpy : " << roll3 << " " << pitch3 << " " << yaw3 << std::endl;

        delta_roll_.push_back(roll2);
        delta_pitch_.push_back(pitch2);

        nav_msgs::Odometry odom_now;
        odom_now.header.frame_id = "/camera_init";
        odom_now.header.stamp = gps_data.header.stamp;
        odom_now.pose.pose.position.x = roll2;
        odom_now.pose.pose.position.y = pitch2;
        odom_now.pose.pose.position.z = yaw2;

        odom_now.pose.pose.orientation.w = yaw;
        odom_now.pose.pose.orientation.x = pitch;
        odom_now.pose.pose.orientation.y = roll1;
        odom_now.pose.pose.orientation.z = pitch3;

        pub_fusion_odom.publish(odom_now);
    }
}

void ImuHandler(const sensor_msgs::Imu imu_data)
{
    double time = imu_data.header.stamp.toSec();
    imu_pointer_last_ = (imu_pointer_last_ + 1) % 100;
    Eigen::Quaterniond imu_quater_now(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);
    imu_quaters_[imu_pointer_last_] = imu_quater_now;
    imu_times_[imu_pointer_last_] = time;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_imu");
    ros::NodeHandle nh;
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu", 1, ImuHandler);
    ros::Subscriber sub_gnss = nh.subscribe<nav_msgs::Odometry>("/sweeper/sensor/gnss", 1, GpsReceive);
    pub_fusion_odom = nh.advertise<nav_msgs::Odometry>("/sweeper/localization/gps_odom", 1);
    ros::spin();

    double delta_roll, delta_pitch;
    for (size_t i = 0; i < delta_roll_.size(); i++)
    {
        delta_roll += delta_roll_[i];
        delta_pitch += delta_pitch_[i];
    }
    std::cout<<"pitch ,roll : "<<delta_pitch/delta_roll_.size()<<" "<<delta_roll/delta_roll_.size()<<std::endl;
    return 0;
}

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "sweeper_msgs/SweeperChassisDetail.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <string>

using namespace std;
const int imu_length_ = 200;
    std::vector<double > yaw_gps_to_imu_;
    Eigen::Quaterniond now_imu_quater_;
class GpsImuFusion
{
public:
    GpsImuFusion();
    ~GpsImuFusion(){};
    void GetImuData(const sensor_msgs::Imu imu_data);
    void GetGpsData(const nav_msgs::Odometry gps_data);

private:
    ros::Subscriber sub_gps;
    ros::Subscriber sub_imu;
    ros::Publisher pub_fusion_odom;
    // double gps_time_;
    int imu_pointer_last_;
    int imu_point_front_;
    Eigen::Quaterniond imu_quater_[imu_length_];
    double imu_time_[imu_length_];
    Eigen::Vector3d imu_position_[imu_length_];
    Eigen::Vector3d frist_gps_position_;
    Eigen::Vector3d last_gps_position_;
    Eigen::Quaterniond last_imu_quater_;
    bool frist_gps_ = false;
    Eigen::Vector3d gps_in_imu_;
    Eigen::Quaterniond imu_to_body_;
    // std::vector<double > yaw_gps_to_imu_;
    // Eigen::Quaterniond now_imu_quater_;
    Eigen::Quaterniond now_gps_quater_;
    Eigen::Quaterniond gps_to_body_;
};

GpsImuFusion::GpsImuFusion()
{
    ros::NodeHandle nh;
    sub_gps = nh.subscribe<nav_msgs::Odometry>("/sweeper/sensor/gnss", 1, &GpsImuFusion::GetGpsData, this);
    sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu", 1, &GpsImuFusion::GetImuData, this);
    pub_fusion_odom = nh.advertise<nav_msgs::Odometry>("/sweeper/localization/gps_odom", 1);
    gps_in_imu_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    yaw_gps_to_imu_.clear();
    gps_to_body_ = Eigen::Quaterniond( 0.938907, 0.00686325, 0.0248803, 0.343203);
    gps_to_body_ = gps_to_body_.normalized();
    imu_to_body_ = Eigen::Quaterniond(0.999204363905233, 0.0394940572990208, -0.00553136979894671, 0.000512385544304862);
    imu_to_body_ = imu_to_body_.normalized();
}

void GpsImuFusion::GetImuData(const sensor_msgs::Imu imu_data)
{
    double time = imu_data.header.stamp.toSec();
    imu_pointer_last_ = (imu_pointer_last_ + 1) % imu_length_;

    Eigen::Quaterniond imu_quater_now(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);
    Eigen::Quaterniond imu_in_body = imu_quater_now * imu_to_body_;
    imu_quater_[imu_pointer_last_] = imu_in_body;

    imu_time_[imu_pointer_last_] = time;
}

void GpsImuFusion::GetGpsData(const nav_msgs::Odometry gps_data)
{
    if (gps_data.pose.covariance[35] > 20)
        return;
    Eigen::Vector3d yaw_tran_90(3.1415 / 2.0, 0.0, 0.0);
    Eigen::Quaterniond yaw_tran_90_quater;
    yaw_tran_90_quater = Eigen::AngleAxisd(yaw_tran_90[0], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(yaw_tran_90[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(yaw_tran_90[2], Eigen::Vector3d::UnitX());
    double time = gps_data.header.stamp.toSec();
    Eigen::Quaterniond gps_quater_now(gps_data.pose.pose.orientation.w, gps_data.pose.pose.orientation.x, gps_data.pose.pose.orientation.y, gps_data.pose.pose.orientation.z);
    Eigen::Quaterniond gps_quater_tran = yaw_tran_90_quater * gps_quater_now;
    Eigen::Vector3d gps_position_now(gps_data.pose.pose.position.x, gps_data.pose.pose.position.y, gps_data.pose.pose.position.z);
    
    //找到最近的一帧
    // gps_position_now = gps_to_body_ * gps_position_now;
    // gps_quater_tran = gps_to_body_ * gps_quater_tran;
    if (!frist_gps_)
    {
        last_gps_position_ = frist_gps_position_ = gps_position_now;
        frist_gps_ = true;
    }

    // Eigen::Vector3d gps_position = gps_position_now - frist_gps_position_;

    double min_time = 1.0;
    int imu_point = 0;
    for (size_t i = 0; i < imu_length_; i++)
    {
        if (fabs(imu_time_[i] - time) < min_time)
        {
            min_time = fabs(imu_time_[i] - time);
            imu_point = i;
        }
    }
    if (fabs(imu_time_[imu_point] - time) < 0.05)
    {
        // Eigen::Vector3d imu_position_now = imu_position_[imu_point];
        Eigen::Quaterniond imu_quater_now = imu_quater_[imu_point];
        Eigen::Quaterniond gps_to_body = imu_quater_now.inverse() * gps_quater_tran;
        
        double roll, pitch, yaw;
        tf::Quaternion quat(gps_to_body.x(), gps_to_body.y(), gps_to_body.z(), gps_to_body.w());
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        yaw_gps_to_imu_.push_back(yaw);
        now_imu_quater_ = imu_quater_now;
        now_gps_quater_ = gps_quater_tran;
        std::cout<<"yaw : "<<yaw<<std::endl;


        double roll1, pitch1, yaw1;
        tf::Quaternion quat1(imu_quater_now.x(), imu_quater_now.y(), imu_quater_now.z(), imu_quater_now.w());
        tf::Matrix3x3(quat1).getRPY(roll1, pitch1, yaw1);
        std::cout<<"imu yaw : "<<yaw1<<std::endl;

        double roll2, pitch2, yaw2;
        tf::Quaternion quat2(gps_quater_tran.x(), gps_quater_tran.y(), gps_quater_tran.z(), gps_quater_tran.w());
        tf::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);
        std::cout<<"gps yaw : "<<yaw2<<std::endl;

        double delta_yaw;
        //double yaw1;
        //double yaw2;

        if(yaw2 - yaw1 > 3.1415926)
        {
        std::cout<<"delta yaw : "<<yaw2 - yaw1 - 2*3.1415926<<std::endl;
        delta_yaw = yaw2 - yaw1 - 2*3.1415926;
        }
        else if(yaw2 - yaw1 < -3.1415926)
        {
        std::cout<<"delta yaw : "<<yaw2 - yaw1 + 2*3.1415926<<std::endl;
        delta_yaw = yaw2 - yaw1 - 2*3.1415926;
        }
        else 
        {
            std::cout<<"delta yaw : "<<yaw2 - yaw1 <<std::endl;
            delta_yaw = yaw2 - yaw1;
        }


        // Eigen::Quaterniond imu_quater_now_save = imu_quater_now.slerp(0.5, last_imu_quater_);
        // // Eigen::Vector3d imu_move_distance << 0.0, 0.0, 0.0;
        // if (1)
        // {
        //     Eigen::Vector3d delta_imu = gps_position_now - last_gps_position_;
        //     double distance = sqrt(delta_imu.x() * delta_imu.x() + delta_imu.y() * delta_imu.y() + delta_imu.z() * delta_imu.z());
        //     Eigen::Vector3d imu_move_distance(distance, 0.0, 0.0);
        //     Eigen::Vector3d imu_move = imu_quater_now_save * imu_move_distance;
        //     gps_in_imu_ += imu_move;
        // }
        // std::ofstream myfile("../path/gps_imu_angular.txt", std::ios::app);
        // myfile << to_string(gps_position.x()) << " " << to_string(gps_position.y()) << " " << to_string(gps_position.z())
        //        << " " << to_string(gps_quater_tran.x()) << " " << to_string(gps_quater_tran.y()) << " " << to_string(gps_quater_tran.z())
        //        << " " << to_string(gps_quater_tran.w()) << "\n"
        //        << to_string(gps_in_imu_.x()) << " " << to_string(gps_in_imu_.y()) << " " << to_string(gps_in_imu_.z())
        //        << " " << to_string(imu_quater_now.x()) << " " << to_string(imu_quater_now.y()) << " " << to_string(imu_quater_now.z())
        //        << " " << to_string(imu_quater_now.w()) << "\n";
        // myfile.close();
        // std::cout << "save file" << std::endl;

        nav_msgs::Odometry odom_now;
        odom_now.header.frame_id = "/camera_init";
        odom_now.header.stamp = gps_data.header.stamp;
        odom_now.pose.pose.orientation.x = gps_quater_tran.x();
        odom_now.pose.pose.orientation.y = gps_quater_tran.y();
        odom_now.pose.pose.orientation.z = gps_quater_tran.z();
        odom_now.pose.pose.orientation.w = gps_quater_tran.w();
        odom_now.pose.pose.position.x = gps_position_now.x() - frist_gps_position_.x();
        odom_now.pose.pose.position.y = gps_position_now.y() - frist_gps_position_.y();
        odom_now.pose.pose.position.z = gps_position_now.z() - frist_gps_position_.z();
        odom_now.twist.twist.linear.x = yaw;
        odom_now.twist.twist.linear.y = delta_yaw;
        odom_now.twist.twist.linear.z = yaw2;
        odom_now.twist.twist.angular.y = yaw1;
        pub_fusion_odom.publish(odom_now);

        // last_gps_position_ = gps_position_now;
        // last_imu_quater_ = imu_quater_now;

        // Eigen::Quaterniond quater_imu_to_gps = gps_quater_now.inverse() * imu_quater_now;
        // tf::Quaternion quat1(quater_imu_to_gps.x(), quater_imu_to_gps.y(), quater_imu_to_gps.z(), quater_imu_to_gps.w());
        // double roll1, pitch1, yaw1;
        // tf::Matrix3x3(quat1).getRPY(roll1, pitch1, yaw1);

        // std::cout << "yaw(z) pitch(y) roll(x) = " << yaw1 << " " << pitch1 << " " << roll1 << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_imu_gps_data");
    ROS_INFO("\033[1;32m---->\033[get_imu_gps_data.");
    GpsImuFusion gps_imu_receive;
    ros::spin();

    double average_yaw = 0.0;
    for(size_t i = 0; i < yaw_gps_to_imu_.size(); i++)
    {
        average_yaw += yaw_gps_to_imu_[i];
    }
    average_yaw = average_yaw/yaw_gps_to_imu_.size();

    Eigen::Quaterniond gps_to_body_before;
    gps_to_body_before = Eigen::AngleAxisd(average_yaw, Eigen::Vector3d::UnitZ()) * 
                  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond gps_in_world = gps_to_body_before.inverse() * now_imu_quater_;
    Eigen::Quaterniond gps_to_body = now_imu_quater_.inverse() * gps_in_world;


        double roll, pitch, yaw;
        tf::Quaternion quat(gps_to_body.x(), gps_to_body.y(), gps_to_body.z(), gps_to_body.w());
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        std::cout << "imu angluar : " << yaw << " " << roll << " " << pitch << std::endl;

        std::cout<<"gps to body quater : "<<gps_to_body.w()<<" "<<gps_to_body.x()<<" "<<gps_to_body.y()<<" "<<gps_to_body.z()<<std::endl;


    return 0;
}

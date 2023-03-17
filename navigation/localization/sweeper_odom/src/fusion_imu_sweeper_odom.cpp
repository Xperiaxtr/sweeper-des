#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "../../../../common/watch_dog.h"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweeperChassisDetail.h"

#define UN_RECEIVED_IMU_DATA 3105
#define UN_RECEIVED_CAR_SPEED 3106

const int imu_length_ = 200;

class ImuOdom {
 public:
  ImuOdom();
  ~ImuOdom(){};
  void GetSweeperData(
      const sweeper_msgs::SweeperChassisDetail sweeper_chassis_detail);
  void GetImuData(const sensor_msgs::Imu imu_data);
  void GetOdom();
  void GetSweeperInformation(const ros::TimerEvent &e);

 private:
  ros::Subscriber sub_imu_data;
  ros::Subscriber sub_sweeper_data;
  ros::Publisher pub_odom_data;
  ros::Publisher pub_fusion_imu_state_information_;
  ros::Timer timer_fusion_imu_;
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
  double imu_angular_velocity_[imu_length_];  // z轴方向
  double imu_angular_velocity_now_;
  Eigen::Vector3d imu_linacc_now_;
  Eigen::Vector3d speed_xyz_;
  sweeper::common::WatchDog watch_dog_imu_;
  sweeper::common::WatchDog watch_dog_speed_;
};

ImuOdom::ImuOdom()
    : imu_pointer_last_(0),
      imu_point_front_(0),
      sweeper_time_(0),
      sweeper_time_last_(0.0),
      sweeper_speed_now_(0.0),
      shift_x_(0.0),
      shift_y_(0.0),
      shift_z_(0.0),
      sweeper_speed_last_(0.0) {
  ros::NodeHandle nh, nh_private("~");

  double lidar_to_imu_q_x, lidar_to_imu_q_y, lidar_to_imu_q_z, lidar_to_imu_q_w,
      lidar_to_imu_v_x, lidar_to_imu_v_y, lidar_to_imu_v_z;
  nh_private.param<double>("lidar_to_imu_q_x", lidar_to_imu_q_x, 0.0);
  nh_private.param<double>("lidar_to_imu_q_y", lidar_to_imu_q_y, 0.0);
  nh_private.param<double>("lidar_to_imu_q_z", lidar_to_imu_q_z, 0.0);
  nh_private.param<double>("lidar_to_imu_q_w", lidar_to_imu_q_w, 1.0);

  nh_private.param<double>("lidar_to_imu_v_x", lidar_to_imu_v_x, 0.0);
  nh_private.param<double>("lidar_to_imu_v_y", lidar_to_imu_v_y, 0.0);
  nh_private.param<double>("lidar_to_imu_v_z", lidar_to_imu_v_z, 0.0);

  std::cout << "init param: " << lidar_to_imu_q_x << " " << lidar_to_imu_q_y
            << " " << lidar_to_imu_q_z << " " << lidar_to_imu_q_w << " "
            << lidar_to_imu_v_x << " " << lidar_to_imu_v_y << " "
            << lidar_to_imu_v_z;
  lidar_imu_quater_ = Eigen::Quaterniond(lidar_to_imu_q_w, lidar_to_imu_q_x,
                                         lidar_to_imu_q_y, lidar_to_imu_q_z);
  lidar_imu_position_ =
      Eigen::Vector3d(lidar_to_imu_v_x, lidar_to_imu_v_y, lidar_to_imu_v_z);

  imu_quater_last_ = imu_quater_out_ = imu_quater_frist_ =
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  sub_imu_data =
      nh.subscribe<sensor_msgs::Imu>("/sweeper/sensor/imu", 10, &ImuOdom::GetImuData, this);
  sub_sweeper_data = nh.subscribe<sweeper_msgs::SweeperChassisDetail>(
      "/sweeper/chassis/detail", 10, &ImuOdom::GetSweeperData, this);
  pub_odom_data =
      nh.advertise<nav_msgs::Odometry>("/sweeper/localization/sweeper_odom", 1);
  pub_fusion_imu_state_information_ =
      nh.advertise<sweeper_msgs::SensorFaultInformation>(
          "/sweeper/localization/diagnose", 1);
  timer_fusion_imu_ =
      nh.createTimer(ros::Duration(0.1), &ImuOdom::GetSweeperInformation, this);
}

void ImuOdom::GetSweeperInformation(const ros::TimerEvent &e) {
  sweeper_msgs::SensorFaultInformation fusion_navigation_code;
  fusion_navigation_code.header.frame_id = "localization";
  fusion_navigation_code.header.stamp = ros::Time::now();
  if (!watch_dog_imu_.DogIsOk(3)) {
    fusion_navigation_code.state_code.push_back(UN_RECEIVED_IMU_DATA);
  }
  if (!watch_dog_speed_.DogIsOk(3)) {
    fusion_navigation_code.state_code.push_back(UN_RECEIVED_CAR_SPEED);
  }
  if (fusion_navigation_code.state_code.empty())
    return;
  else
    pub_fusion_imu_state_information_.publish(fusion_navigation_code);
}

void ImuOdom::GetSweeperData(
    const sweeper_msgs::SweeperChassisDetail sweeper_chassis_detail) {
  watch_dog_speed_.UpdataNow();
  double kmph_to_mps = 1000.0/3600.0;
  sweeper_time_ = sweeper_chassis_detail.header.stamp.toSec();
  sweeper_speed_now_ = sweeper_chassis_detail.vehicle_speed_output * kmph_to_mps;
  if (frist_sweeper_data_) {
    sweeper_time_last_ = sweeper_time_;
    frist_sweeper_data_ = false;
  }
  if (!receive_new_imu_data_) {
    return;
  }
  receive_new_imu_data_ = false;
  GetOdom();

  Eigen::Quaterniond sweeper_quater_out;
  sweeper_quater_out = /* imu_quater_frist_.inverse() */ imu_quater_out_;

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

  Eigen::Matrix<double, 4, 4> imu_to_world_frist_matrix;
  imu_to_world_frist_matrix.block(0, 0, 3, 3) = imu_quater_frist_.matrix();
  imu_to_world_frist_matrix(0, 3) = 0;
  imu_to_world_frist_matrix(1, 3) = 0;
  imu_to_world_frist_matrix(2, 3) = 0;

  imu_to_world_frist_matrix(3, 3) = 1;
  imu_to_world_frist_matrix(3, 0) = 0;
  imu_to_world_frist_matrix(3, 1) = 0;
  imu_to_world_frist_matrix(3, 2) = 0;

  Eigen::Matrix<double, 4, 4> lidar_to_world_matrix =
      lidar_to_imu_matrix.inverse() * imu_to_world_frist_matrix.inverse() *
      imu_to_world_matrix * lidar_to_imu_matrix;
  Eigen::Matrix3d lidar_to_world_rotion =
      lidar_to_world_matrix.block(0, 0, 3, 3);
  Eigen::Quaterniond lidar_to_world_quater(lidar_to_world_rotion);

  // //计算转弯半径
  // double twist_r = odom_now.twist.twist.linear.x / imu_angular_velocity_now_;

  nav_msgs::Odometry odom_now;
  odom_now.header.frame_id = "/camera_init";
  odom_now.header.stamp = sweeper_chassis_detail.header.stamp;
  odom_now.pose.pose.orientation.x = lidar_to_world_quater.x();
  odom_now.pose.pose.orientation.y = lidar_to_world_quater.y();
  odom_now.pose.pose.orientation.z = lidar_to_world_quater.z();
  odom_now.pose.pose.orientation.w = lidar_to_world_quater.w();
  odom_now.pose.pose.position.x = lidar_to_world_matrix(0, 3);
  odom_now.pose.pose.position.y = lidar_to_world_matrix(1, 3);
  odom_now.pose.pose.position.z = lidar_to_world_matrix(2, 3);

  odom_now.twist.twist.linear.x =
      sqrt(speed_xyz_.x() * speed_xyz_.x() + speed_xyz_.y() * speed_xyz_.y());

  //计算转弯半径
  double twist_r = odom_now.twist.twist.linear.x / imu_angular_velocity_now_;
  // std::cout << "r : " << twist_r << std::endl;
  odom_now.twist.twist.angular.z = imu_angular_velocity_now_;  //角速度
  // std::cout << "angular velocity : " << imu_angular_velocity_now_ <<
  // std::endl;
  odom_now.twist.twist.angular.x = imu_linacc_now_.x();  //车辆加速度
  if (twist_r < 0.001)
    odom_now.twist.twist.angular.y = 0.0;
  else
    odom_now.twist.twist.angular.y = imu_linacc_now_.y() / twist_r;  //角加速度
  pub_odom_data.publish(odom_now);
}

void ImuOdom::GetImuData(const sensor_msgs::Imu imu_data) {
  watch_dog_imu_.UpdataNow();
  // double roll, pitch, yaw;
  Eigen::Quaterniond imu_orientation(
      imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y,
      imu_data.orientation.z);
  Eigen::Vector3d yaw_tran_90(-3.1415926 / 2.0, 0.0, 0.0);
  Eigen::Quaterniond yaw_tran_90_quater;
  yaw_tran_90_quater =
      Eigen::AngleAxisd(yaw_tran_90[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(yaw_tran_90[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw_tran_90[2], Eigen::Vector3d::UnitX());
  Eigen::Quaterniond tran_90_imu = yaw_tran_90_quater * imu_orientation;
  // tf::Quaternion quat(tran_90_imu.x(), tran_90_imu.y(), tran_90_imu.z(),
  // tran_90_imu.w()); tf::Matrix3x3(quat).getRPY(yaw, roll, pitch);
  double time = imu_data.header.stamp.toSec();
  Eigen::Vector3d imu_acc(imu_data.linear_acceleration.x,
                          imu_data.linear_acceleration.y,
                          imu_data.linear_acceleration.z);
  //得到imu的加速度与角速度
  Eigen::Vector3d g_acc(0.0, 0.0, 9.8);
  Eigen::Vector3d g_in_imu = imu_orientation * g_acc;
  Eigen::Vector3d imu_linacc = imu_acc + g_in_imu;
  // std::cout << "imu acc : " << imu_linacc << std::endl;
  double imu_angular_velocity = imu_data.angular_velocity.z;

  // double imu_linacc_xy = sqrt(acc_in_imu.x() * acc_in_imu.x() +
  // acc_in_imu.y() * acc_in_imu.y());
  imu_pointer_last_ = (imu_pointer_last_ + 1) % imu_length_;
  imu_time_[imu_pointer_last_] = time;
  imu_quater_[imu_pointer_last_] = tran_90_imu;
  imu_angular_velocity_[imu_pointer_last_] = imu_angular_velocity;
  imu_linacc_[imu_pointer_last_] = imu_linacc;

  if (frist_imu_data_) {
    frist_imu_data_ = false;
    imu_quater_frist_ = imu_quater_last_ = tran_90_imu;
  }
  receive_new_imu_data_ = true;
}

void ImuOdom::GetOdom() {
  if (imu_pointer_last_ >= 0) {
    Eigen::Quaterniond imu_quater_now = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    //找到车速传感器时间戳小于imu时间戳的位置
    while (imu_point_front_ != imu_pointer_last_) {
      if (sweeper_time_ > imu_time_[imu_point_front_] &&
          sweeper_time_ < imu_time_[(imu_point_front_ + 1) % imu_length_])
        break;
      imu_point_front_ = (imu_point_front_ + 1) % imu_length_;
    }
    //没找到
    if (sweeper_time_ > imu_time_[(imu_point_front_ + 1) % imu_length_]) {
      imu_quater_now = imu_quater_[imu_point_front_];
      imu_angular_velocity_now_ = imu_angular_velocity_[imu_point_front_];
      imu_linacc_now_ = imu_linacc_[imu_point_front_];
    } else {
      int imu_pointer_back = (imu_point_front_ + 1) % imu_length_;
      double ratio_front =
          (sweeper_time_ - imu_time_[imu_pointer_back]) /
          (imu_time_[imu_pointer_back] - imu_time_[imu_point_front_]);
      double ratio_back = 1.0 - ratio_front;
      imu_quater_now = imu_quater_[imu_point_front_].slerp(
          ratio_front, imu_quater_[imu_pointer_back]);

      imu_angular_velocity_now_ =
          ratio_front * imu_angular_velocity_[imu_point_front_] +
          ratio_back * imu_angular_velocity_[imu_pointer_back];
      imu_linacc_now_ = ratio_front * imu_linacc_[imu_point_front_] +
                        ratio_back * imu_linacc_[imu_pointer_back];
    }
    imu_quater_out_ = imu_quater_now;

    //转换为旋转矩阵
    Eigen::Vector3d speed_sweeper(
        0.5 * (sweeper_speed_now_ + sweeper_speed_last_), 0.0, 0.0);

    Eigen::Quaterniond car_to_imu =
        /* imu_quater_frist_.inverse() */ imu_quater_out_;

    speed_xyz_ = (/*imu_to_car_tran */ car_to_imu) * (speed_sweeper);
    sweeper_speed_last_ = sweeper_speed_now_;
    double diff_time = sweeper_time_ - sweeper_time_last_;

    shift_x_ = shift_x_ + speed_xyz_.x() * diff_time;
    shift_y_ = shift_y_ + speed_xyz_.y() * diff_time;
    shift_z_ = shift_z_ + speed_xyz_.z() * diff_time;
    // std::cout << "x,y,z : " << shift_x_ << " " << shift_y_ << " " << shift_z_
    //           << std::endl;
  }
  sweeper_time_last_ = sweeper_time_;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fusion_imu_and_sweeper");
  ImuOdom imu_odom;
  ros::spin();
  return 0;
}
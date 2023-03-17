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

#define un_received_imu_data_ 3105
#define un_received_gps_data_ 3107

const int imu_length_ = 200;
class GpsImuFusion {
 public:
  GpsImuFusion();
  ~GpsImuFusion(){};
  void GetImuData(const sensor_msgs::Imu imu_data);
  void GetGpsData(const nav_msgs::Odometry gps_data);
  void GetSweeperInformation(const ros::TimerEvent &e);

 private:
  ros::Subscriber sub_gps_;
  ros::Subscriber sub_imu_;
  ros::Publisher pub_fusion_odom_;
  ros::Publisher pub_state_information_;
  ros::Publisher pub_state_information_localization_;
  ros::Timer timer_fusion_navigation_;
  double gps_time_;
  int imu_pointer_last_;
  int imu_point_front_;
  double imu_time_[imu_length_];
  double imu_roll_[imu_length_];
  double imu_pitch_[imu_length_];
  sweeper::common::WatchDog watch_dog_imu_;
  sweeper::common::WatchDog watch_dog_gps_;
  Eigen::Quaterniond imu_to_gps_;
  bool frist_gps_ = false;
  Eigen::Vector3d gps_frist_;
  Eigen::Quaterniond tran_90_yaw_;
  Eigen::Vector3d lidar_to_gps_position_;
  //  =
  //     // Eigen::Vector3d(0.540927338082619,
  //     // 0.517892452768321, 2.10387247263517);
  //     Eigen::Vector3d(0.978790272111024, 0.534726712322805,
  //     0.847803130466346);
  Eigen::Quaterniond lidar_to_gps_quater_;
  //  =
  //     // Eigen::Quaterniond(0.998331907815185, 0.0163162424449328,
  //     //                   -0.00721288088429475, 0.0549104399902179);
  //     Eigen::Quaterniond(0.999914945457086, -0.00265999210362202,
  //                        -0.00897669777718909, 0.00907993340168112);
};

GpsImuFusion::GpsImuFusion() {
  ros::NodeHandle nh, nh_private("~");

  double lidar_to_gps_q_x, lidar_to_gps_q_y, lidar_to_gps_q_z, lidar_to_gps_q_w,
      lidar_to_gps_v_x, lidar_to_gps_v_y, lidar_to_gps_v_z, imu_to_gps_q_x,
      imu_to_gps_q_y, imu_to_gps_q_z, imu_to_gps_q_w;
  nh_private.param<double>("lidar_to_gps_q_x", lidar_to_gps_q_x, 0);
  nh_private.param<double>("lidar_to_gps_q_y", lidar_to_gps_q_y, 0);
  nh_private.param<double>("lidar_to_gps_q_z", lidar_to_gps_q_z, 0);
  nh_private.param<double>("lidar_to_gps_q_w", lidar_to_gps_q_w, 1.0);

  nh_private.param<double>("lidar_to_gps_v_x", lidar_to_gps_v_x, 0);
  nh_private.param<double>("lidar_to_gps_v_y", lidar_to_gps_v_y, 0);
  nh_private.param<double>("lidar_to_gps_v_z", lidar_to_gps_v_z, 0);

  nh_private.param<double>("imu_to_gps_q_x", imu_to_gps_q_x, 0);
  nh_private.param<double>("imu_to_gps_q_y", imu_to_gps_q_y, 0);
  nh_private.param<double>("imu_to_gps_q_z", imu_to_gps_q_z, 0);
  nh_private.param<double>("imu_to_gps_q_w", imu_to_gps_q_w, 1.0);

  std::cout << "init param: " << lidar_to_gps_q_x << " " << lidar_to_gps_q_y
            << " " << lidar_to_gps_q_z << " " << lidar_to_gps_q_w << " "
            << lidar_to_gps_v_x << " " << lidar_to_gps_v_y << " "
            << lidar_to_gps_v_z << " " << imu_to_gps_q_x << " "
            << imu_to_gps_q_y << " " << imu_to_gps_q_z << " " << imu_to_gps_q_w
            << std::endl;

  lidar_to_gps_quater_ = Eigen::Quaterniond(lidar_to_gps_q_w, lidar_to_gps_q_x,
                                            lidar_to_gps_q_y, lidar_to_gps_q_z);
  lidar_to_gps_position_ =
      Eigen::Vector3d(lidar_to_gps_v_x, lidar_to_gps_v_y, lidar_to_gps_v_z);

  // imu_to_gps_ = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
  //               Eigen::AngleAxisd(-0.0355256, Eigen::Vector3d::UnitY()) *
  //               Eigen::AngleAxisd(0.0261033, Eigen::Vector3d::UnitX());
  imu_to_gps_ = Eigen::Quaterniond(imu_to_gps_q_w, imu_to_gps_q_x,
                                   imu_to_gps_q_y, imu_to_gps_q_z);

  std::cout << "imu_to_gps : " << imu_to_gps_.x() << " " << imu_to_gps_.y()
            << " " << imu_to_gps_.z() << " " << imu_to_gps_.w() << std::endl;

  tran_90_yaw_ = Eigen::AngleAxisd(3.1415926 / 2.0, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  lidar_to_gps_quater_ = lidar_to_gps_quater_.normalized();

  sub_gps_ = nh.subscribe<nav_msgs::Odometry>("/sweeper/sensor/gnss", 1,
                                              &GpsImuFusion::GetGpsData, this);
  sub_imu_ = nh.subscribe<sensor_msgs::Imu>("/imu", 10,
                                            &GpsImuFusion::GetImuData, this);
  pub_fusion_odom_ =
      nh.advertise<nav_msgs::Odometry>("/sweeper/localization/gnss", 1);
  timer_fusion_navigation_ = nh.createTimer(
      ros::Duration(0.1), &GpsImuFusion::GetSweeperInformation, this);
  pub_state_information_ = nh.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/mapping/diagnose", 1);
  pub_state_information_localization_ =
      nh.advertise<sweeper_msgs::SensorFaultInformation>(
          "/sweeper/localization/diagnose", 1);
}

void GpsImuFusion::GetSweeperInformation(const ros::TimerEvent &e) {
  sweeper_msgs::SensorFaultInformation fusion_navigation_code;
  fusion_navigation_code.header.frame_id = "localization";
  fusion_navigation_code.header.stamp = ros::Time::now();
  // if (!watch_dog_imu_.DogIsOk(3)) {
  //   fusion_navigation_code.state_code.push_back(un_received_imu_data_);
  // }
  if (!watch_dog_gps_.DogIsOk(3)) {
    fusion_navigation_code.state_code.push_back(un_received_gps_data_);
  }
  if (fusion_navigation_code.state_code.empty())
    return;
  else {
    pub_state_information_.publish(fusion_navigation_code);
    pub_state_information_localization_.publish(fusion_navigation_code);
  }
}

void GpsImuFusion::GetImuData(const sensor_msgs::Imu imu_data) {
  watch_dog_imu_.UpdataNow();
  double time = imu_data.header.stamp.toSec();
  imu_pointer_last_ = (imu_pointer_last_ + 1) % imu_length_;
  imu_time_[imu_pointer_last_] = time;
  Eigen::Quaterniond imu_quater(imu_data.orientation.w, imu_data.orientation.x,
                                imu_data.orientation.y, imu_data.orientation.z);
  Eigen::Quaterniond gps_quater = imu_quater * imu_to_gps_.inverse();

  double roll, pitch, yaw;
  tf::Quaternion quat(gps_quater.x(), gps_quater.y(), gps_quater.z(),
                      gps_quater.w());
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  Eigen::Quaterniond gps_quater_rp =
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  imu_roll_[imu_pointer_last_] = roll;
  imu_pitch_[imu_pointer_last_] = pitch;
}

void GpsImuFusion::GetGpsData(const nav_msgs::Odometry gps_data) {
  watch_dog_gps_.UpdataNow();
  // std::cout << "receive gps" << std::endl;
  if (!frist_gps_) {
    frist_gps_ = true;
    gps_frist_ = Eigen::Vector3d(gps_data.pose.pose.position.x,
                                 gps_data.pose.pose.position.y,
                                 gps_data.pose.pose.position.z);
  }

  double time = gps_data.header.stamp.toSec();

  Eigen::Quaterniond gps_new_quater(
      gps_data.pose.pose.orientation.w, gps_data.pose.pose.orientation.x,
      gps_data.pose.pose.orientation.y, gps_data.pose.pose.orientation.z);
  gps_new_quater = tran_90_yaw_ * gps_new_quater;

  double roll, pitch, yaw;
  tf::Quaternion quat(gps_new_quater.x(), gps_new_quater.y(),
                      gps_new_quater.z(), gps_new_quater.w());
  // tf::quaternionMsgToTF(gps_data.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  double min_time = 1.0;
  int imu_point;
  for (size_t i = 0; i < imu_length_; i++) {
    if (fabs(time - imu_time_[i]) < min_time) {
      min_time = fabs(time - imu_time_[i]);
      imu_point = i;
    }
  }

  if (min_time > 0.1) return;

  Eigen::Quaterniond gps_quater;
  gps_quater =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(imu_pitch_[imu_point], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(imu_roll_[imu_point], Eigen::Vector3d::UnitX());

  Eigen::Vector3d gps_position(gps_data.pose.pose.position.x,
                               gps_data.pose.pose.position.y,
                               gps_data.pose.pose.position.z);
  Eigen::Quaterniond out_quater = gps_quater * lidar_to_gps_quater_;
  Eigen::Vector3d out_position =
      gps_quater * lidar_to_gps_position_ + gps_position;

  // //转换到第一个点
  // Eigen::Quaterniond to_frist_position =

  nav_msgs::Odometry odom_now;
  odom_now.header.frame_id = "/camera_init";
  odom_now.header.stamp = gps_data.header.stamp;
  odom_now.pose.pose.position.x =
      out_position.x();  // - 684769;// - gps_frist_.x();
  odom_now.pose.pose.position.y =
      out_position.y();  // - 3112589;// - gps_frist_.y();
  odom_now.pose.pose.position.z =
      out_position.z();  // - 57;// - gps_frist_.z();
  odom_now.pose.pose.orientation.w = out_quater.w();
  odom_now.pose.pose.orientation.x = out_quater.x();
  odom_now.pose.pose.orientation.y = out_quater.y();
  odom_now.pose.pose.orientation.z = out_quater.z();
  odom_now.pose.covariance = gps_data.pose.covariance;
  pub_fusion_odom_.publish(odom_now);
  std::cout << "pub gnss" << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fusion_imu_and_sweeper");
  GpsImuFusion imu_odom;
  ros::spin();
  return 0;
}

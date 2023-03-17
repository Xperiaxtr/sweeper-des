
#include <tf/transform_broadcaster.h>

#include <string>

#include "../../../../common/watch_dog.h"
#include "lpsensor/LpmsIG1I.h"
#include "lpsensor/LpmsIG1Registers.h"
#include "lpsensor/SensorDataI.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sweeper_msgs/SensorFaultInformation.h"

//! Manages connection with the sensor, publishes data
/*!
  \TODO: Make noncopyable!
 */
class LpIG1Proxy {
 public:
  LpIG1Proxy() : private_nh("~") {
    // Get node parameters
    private_nh.param<std::string>("port", comportNo, "/dev/ig1");
    private_nh.param("baudrate", baudrate, 115200);
    private_nh.param<std::string>("frame_id", frame_id, "imu");
    private_nh.param("rate", rate, 100);

    // Create LpmsIG1 object
    sensor1 = IG1Factory();

    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1);
    mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag", 1);

    pub_state_information_ = nh.advertise<sweeper_msgs::SensorFaultInformation>(
        "/sweeper/common/diagnose", 1);

    // Connects to sensor
    if (!sensor1->connect(comportNo, baudrate)) {
      // logd(TAG, "Error connecting to sensor\n");
      ROS_ERROR("Error connecting to sensor\n");
      sensor1->release();
      ros::Duration(3).sleep();  // sleep 3 s
    }
  }

  ~LpIG1Proxy(void) { sensor1->release(); }

  void update(const ros::TimerEvent &te) {
    rate_times_++;
    if (rate_times_ % 100 == 0) {  // 10hz
      sweeper_msgs::SensorFaultInformation fusion_navigation_code;
      fusion_navigation_code.header.frame_id = "imu";
      fusion_navigation_code.header.stamp = ros::Time::now();
      if (!watch_dog_imu_.DogIsOk(3)) {
        fusion_navigation_code.state_code.push_back(2601);
        pub_state_information_.publish(fusion_navigation_code);
      } else {
        fusion_navigation_code.state_code.push_back(2600);
        pub_state_information_.publish(fusion_navigation_code);
      }
      rate_times_ = 0;
    }
    if (sensor1->getStatus() == STATUS_CONNECTED && sensor1->hasImuData()) {
      IG1ImuDataI sd;
      sensor1->getImuData(sd);

      /* Fill the IMU message */

      // Fill the header
      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = frame_id;

      // Fill orientation quaternion
      imu_msg.orientation.w = sd.quaternion.data[0];
      imu_msg.orientation.x = -sd.quaternion.data[1];
      imu_msg.orientation.y = -sd.quaternion.data[2];
      imu_msg.orientation.z = -sd.quaternion.data[3];

      // Fill angular velocity data
      // - scale from deg/s to rad/s
      imu_msg.angular_velocity.x =
          sd.gyroIAlignmentCalibrated.data[0] * 3.1415926 / 180;
      imu_msg.angular_velocity.y =
          sd.gyroIAlignmentCalibrated.data[1] * 3.1415926 / 180;
      imu_msg.angular_velocity.z =
          sd.gyroIAlignmentCalibrated.data[2] * 3.1415926 / 180;

      // Fill linear acceleration data
      imu_msg.linear_acceleration.x = sd.accCalibrated.data[0] * 9.81;
      imu_msg.linear_acceleration.y = sd.accCalibrated.data[1] * 9.81;
      imu_msg.linear_acceleration.z = sd.accCalibrated.data[2] * 9.81;

      /* Fill the magnetometer message */
      mag_msg.header.stamp = imu_msg.header.stamp;
      mag_msg.header.frame_id = frame_id;

      // Units are microTesla in the LPMS library, Tesla in ROS.
      mag_msg.magnetic_field.x = sd.magCalibrated.data[0] * 1e-6;
      mag_msg.magnetic_field.y = sd.magCalibrated.data[1] * 1e-6;
      mag_msg.magnetic_field.z = sd.magCalibrated.data[2] * 1e-6;

      watch_dog_imu_.UpdataNow();
      // Publish the messages
      imu_pub.publish(imu_msg);
      mag_pub.publish(mag_msg);
    }
  }

  void run(void) {
    // The timer ensures periodic data publishing
    updateTimer = ros::Timer(
        nh.createTimer(ros::Duration(0.1 / rate), &LpIG1Proxy::update, this));
  }

 private:
  // Access to LPMS data
  IG1I *sensor1;

  // Access to ROS node
  ros::NodeHandle nh, private_nh;
  ros::Timer updateTimer;
  ros::Publisher imu_pub, mag_pub;
  sensor_msgs::Imu imu_msg;
  sensor_msgs::MagneticField mag_msg;

  // Parameters
  std::string comportNo;
  int baudrate;
  std::string frame_id;
  int rate;
  int rate_times_;

  sweeper::common::WatchDog watch_dog_imu_;
  ros::Publisher pub_state_information_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "lpms_ig1");
  ros::NodeHandle nh, private_nh;

  LpIG1Proxy lpIG1;

  lpIG1.run();

  ros::spin();

  return 0;
}

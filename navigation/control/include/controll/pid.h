#pragma once

#include <iostream>
#include <cmath>
#include <string>
#include <ros/ros.h>

namespace sweeper
{
namespace navigation
{
namespace controll
{

class VelocityPID
{
public:
    VelocityPID(ros::NodeHandle &nh_private);

    ~VelocityPID(){};

    double VelocityControl(const double target_speed, const double current_speed);

private:
    double ComputeTorque(const double target_speed, const double current_speed);
    double ComputeBrake(const double target_speed, const double current_speed);

    double pid_dt_;

    bool brake_state_flag_;

    // torque pid param
    double torque_pid_kp_;
    double torque_pid_ki_;
    double torque_pid_kd_;
    double torque_min_;
    double torque_max_;
    double last_torque_error_;
    double last_last_torque_error_;
    double last_torque_;
    double max_torque_;
    double min_torque_;

    // brake pid param
    double brake_pid_kp_;
    double brake_pid_ki_;
    double brake_pid_kd_;
    double brake_min_;
    double brake_max_;
    double last_brake_error_;
    double last_last_brake_error_;
    double last_brake_;
    double min_brake_;
    double max_brake_;
};

} // namespace controll
} // namespace navigation
} // namespace sweeper
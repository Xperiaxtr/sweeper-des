#include "../include/controll/pid.h"

namespace sweeper
{
namespace navigation
{
namespace controll
{

VelocityPID::VelocityPID(ros::NodeHandle &nh_private) : brake_state_flag_(true)
{
    nh_private.param("pid_dt", pid_dt_, 0.05);
    nh_private.param("torque_pid_kp", torque_pid_kp_, 4.0);
    nh_private.param("torque_pid_ki", torque_pid_ki_, 0.0);
    nh_private.param("torque_pid_kd", torque_pid_kd_, 0.0);
    nh_private.param("torque_min", torque_min_, 0.0);
    nh_private.param("torque_max", torque_max_, 80.0);
    nh_private.param("last_torque", last_torque_, 0.0);

    nh_private.param("brake_pid_kp", brake_pid_kp_, 4.0);
    nh_private.param("brake_pid_ki", brake_pid_ki_, 0.0);
    nh_private.param("brake_pid_kd", brake_pid_kd_, 0.0);
    nh_private.param("brake_min", brake_min_, 0.0);
    nh_private.param("brake_max", brake_max_, 80.0);
    nh_private.param("last_brake", last_brake_, 0.0);
}

double VelocityPID::VelocityControl(const double target_speed, const double current_speed)
{
    double output;

    if (target_speed > 0.001)
    {
        if (brake_state_flag_)
        {

            if (current_speed < target_speed)
            {
                brake_state_flag_ = false;
            }

            last_torque_ = 0.0;
            last_torque_error_ = 0.0;
            last_last_torque_error_ = 0.0;

            output = ComputeBrake(target_speed, current_speed);
            output = -output;
            return output;
        }
        else
        {

            if (current_speed - target_speed > 0.1)
            {
                brake_state_flag_ = true;
            }

            last_brake_ = 0.0;
            last_brake_error_ = 0.0;
            last_last_brake_error_ = 0.0;

            output = ComputeTorque(target_speed, current_speed);

            return output;
        }
    }
    else
    {
        last_torque_ = 0.0;
        last_torque_error_ = 0.0;
        last_last_torque_error_ = 0.0;

        last_brake_ = 0.0;
        last_brake_error_ = 0.0;
        last_last_brake_error_ = 0.0;

        output = -50.0;
        return output;
    }
}

double VelocityPID::ComputeTorque(const double target_speed, const double current_speed)
{
    double error = target_speed - current_speed;

    // Proportional portion
    double Pout = torque_pid_kp_ * (error - last_torque_error_);

    // Integral portion

    double Iout = torque_pid_ki_ * (error * pid_dt_);

    // Derivative portion
    double derivative = 0.0;

    double Dout = 0.0;

    if ((last_torque_error_ > 0.000001 || last_torque_error_ < -0.000001) && (last_last_torque_error_ > 0.000001 || last_last_torque_error_ < -0.000001))
    {
        derivative = (error - 2 * last_torque_error_ + last_last_torque_error_) / pid_dt_;
        Dout = torque_pid_kd_ * derivative;
    }
    else
    {
        Dout = 0.0;
    }

    // Total output
    double output = Pout + Iout + Dout + last_torque_;

    // Limit to max/min
    if (output > torque_max_)
        output = torque_max_;
    else if (output < torque_min_)
        output = torque_min_;

    // Save error to previous error
    last_last_torque_error_ = last_torque_error_;
    last_torque_error_ = error;
    last_torque_ = output;

    return output;
}

double VelocityPID::ComputeBrake(const double target_speed, const double current_speed)
{
    // error
    double error = current_speed - target_speed;

    // Proportional portion
    double Pout = brake_pid_kp_ * (error - last_brake_error_);

    // Integral portion

    double Iout = brake_pid_ki_ * (error * pid_dt_);

    // Derivative portion
    double derivative = 0.0;

    double Dout = 0.0;

    if ((last_brake_error_ > 0.000001 || last_brake_error_ < -0.000001) && (last_last_brake_error_ > 0.000001 || last_last_brake_error_ < -0.000001))
    {
        derivative = (error - 2 * last_brake_error_ + last_last_brake_error_) / pid_dt_;
        Dout = brake_pid_kd_ * derivative;
    }
    else
    {
        Dout = 0.0;
    }

    // Total output
    double output = Pout + Iout + Dout + last_brake_;

    // Limit to max/min
    if (output > brake_max_)
        output = brake_max_;
    else if (output < brake_min_)
        output = brake_min_;

    // Save error to previous error
    last_last_brake_error_ = last_brake_error_;
    last_brake_error_ = error;
    last_brake_ = output;

    return output;
}

} // namespace controll
} //namespace navigation
} //namespace sweeper

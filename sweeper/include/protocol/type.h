#pragma once
#include <stdint.h>

#include <cstdint>
#include <string>
#include <vector>

namespace sweeper {
namespace bus {

struct Brake {
  double brake_output;
  double brake_input;
  double brake_cmd;
  double brake_watchdog;
  bool brake_enabled;
  bool brake_driver_override_flag;
  bool brake_driver_active_flag;
  bool brake_watchdog_fault;
  bool brake_channel_1_fault;
  bool brake_channel_2_fault;
  bool brake_power_fault;
  bool brake_timeout;  // 0: fresh; 1: timeout > 100ms
};

struct Throttle {
  double throttle_output;
  double throttle_input;
  double throttle_cmd;
  double throttle_watchdog;
  bool throttle_enabled;
  bool throttle_driver_override_flag;
  bool throttle_driver_active_flag;
  bool throttle_watchdog_fault;
  bool throttle_channel_1_fault;
  bool throttle_channel_2_fault;
  bool throttle_power_fault;
  bool throttle_timeout;  // 0: fresh; 1: timeout > 100ms
};

struct Eps {
  double steering_angle_input;
  double steering_angle_output;
  double steering_angle_speed;
  double steering_torque;
  double steering_watchdog;
  bool steering_enabled;
  bool steering_driver_override_flag;
  bool steering_power_fault;
  bool eps_timeout;  // 0: fresh; 1: timeout > 100ms
};

struct VehicleStatus {
  double dcu_status;
  double dcu_workmode_request;
  double dcu_workmode_response;
  bool dcu_csu_reset_response;
  bool dcu_csu_reset_request;
  bool dcu_djlidar_reset_response;
  bool dcu_slidar_reset_response;
  bool dcu_ebs_reset_status;
  bool dcu_eps_reset_status;
  bool dcu_uradar_reset_status;
  bool dcu_timeout;
  bool dcu_uradar_relay_status;
  bool dcu_djlidar_relay_status;
  bool dcu_wjlidar_relay_status;
  double dcu_watchdog_counter;
  double vehicle_accelertion;
  double vehicle_emergency_stop_status;
  bool auto_mode_ready_status;
  bool remote_mode_ready_status;
  double dcu_sweep_side_request;
  double dcu_vehicle_speed_mode_request;
  double dcu_vehicle_soc;
};

struct VehicleMode {
  double motor_speed;
  double motor_torque;
  double vehicle_speed;
  double vehicle_watchdog;
  double vehicle_drive_mode;
  double vehicle_current_gear;
  double vehicle_power_mode_req;
  bool vehicle_drive_enable;
};

struct VehicleFault {
  double dcu_fault_num;
  double vehicle_fault_num;
  double dcu_fault_code;
  double dcu_fault_level;
};

typedef struct CarFeedBackInfo {
  double timestamp;
  struct Eps eps;
  struct Brake brake;
  struct Throttle throttle;
  struct VehicleMode vehiclemode;
  struct VehicleStatus vehiclestatus;
  struct VehicleFault vehiclefault;
} CarFeedBackInfo;

}  // namespace bus
}  // namespace sweeper
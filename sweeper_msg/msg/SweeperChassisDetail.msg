#sweeper chassis detail data

Header      header         # standard ROS message header
float64     brake_input
float64     brake_output
float64     throttle_input
float64     throttle_output 
float64     steering_angle_input
float64     steering_angle_output
float64     vehicle_speed_input
float64     vehicle_speed_output
float64     motor_speed_input
float64     motor_speed_output
float64     motor_torque_input
float64     motor_torque_output
float64     chassis_control_mode
float64     chassis_work_status
float64     chassis_speed_mode
float64     chassis_vehicle_soc
float64     dcu_fault_level
float64     dcu_fault_code
float64     csu_fault_level
int32[]     csu_fault_code      

#ifndef SWEEPER_H_
#define SWEEPER_H_

#include <ros/ros.h>
#include <iostream>
#include <math.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "../../common/log.h"
#include "../../common/can/can_drive.h"
#include "common/data_center.h"

#include "sweeper_msgs/SweeperCmd.h"
#include "sweeper_msgs/StateReport.h"
#include "sweeper_msgs/SweepMission.h"
#include "sweeper_msgs/SweeperChassisDetail.h"
#include "sweeper_msgs/SensorFaultInformation.h"

#include "protocol/brake_60.h"
#include "protocol/brake_61.h"
#include "protocol/throttle_62.h"
#include "protocol/throttle_63.h"
#include "protocol/steering_64.h"
#include "protocol/steering_65.h"
#include "protocol/vehicle_control_66.h"
#include "protocol/vehicle_control_67.h"
#include "protocol/vehicle_mode_68.h"
#include "protocol/vehicle_mode_69.h"
#include "protocol/vehicle_status_70.h"
#include "protocol/vehicle_fault_71.h"
#include "protocol/vehicle_status_72.h"
#include "protocol/sensor_fault_76.h"
#include "protocol/voice_prompts_2AA.h"

#include "states/idle_state.h"
#include "states/init_state.h"
#include "states/sweeper_states.h"
#include "states/charging_state.h"
#include "states/global_sweep_state.h"
#include "states/gnss_trace_state.h"
#include "states/lidar_trace_state.h"
#include "states/curb_sweep_state.h"
#include "states/record_lidar_line_state.h"
#include "states/record_gnss_line_state.h"
#include "states/update_state.h"

#include "nodemanager/nodemanager.h"

#include "../../src/matlab/modecontrol/SweeperDriveModeControl.h"
#include "../../src/matlab/chassiscontrol/SweeperDriveChassisControl.h"
#include "../../src/matlab/nodemanager/SweeperNodeManageMode.h"
#include "../../src/matlab/faultmanager/SweeperFaultManageMode.h"
#include "../../src/matlab/sensormanager/SweeperSensorManageMode.h"
#include "../../src/matlab/voicemanager/SweeperVoiceManager.h"

using sweeper::bus::CarFeedBackInfo;

namespace sweeper
{

class Sweeper
{
public:
    Sweeper();
    ~Sweeper();
	
    bool Init(ros::NodeHandle &node, ros::NodeHandle &private_nh);
    void Run();

private:
    void ChassisDetailPublish();
    void ChassisDetailReceive();
    void ChassisCommandSend(); 
    void ChassisCommandUpdate();   
    void ChassisCommmandCallback(const sweeper_msgs::SweeperCmd &sweeper_cmd);
    void SweeperNodeReset();
    void SweeperSensorCheck();
    void SweeperLogManager();      
    void SweeperAppManager();
    void SweeperNodeManager();
    void SweeperStateReport();      
    void SweeperStateManager();
    void SweeperSystemManager();
    bool SweeperNetCheck();      
    void SweeperStatusCheck();
    void SweeperSystemCheck();      
    void SweeperNodeStateCallback(const sweeper_msgs::SensorFaultInformation &node_state);
    void SweeperMissionCallback(const sweeper_msgs::SweepMission &app_mission);  
    void SweeperStatePublish();                     
    void SweeperChassisControlInput();    
    void SweeperDriveModeControlInput();
    void SweeperNodeManagerModelInput();
    void SweeperFaultManagerModelInput();
    void SweeperVoiceManagerModelInput();
    void SweeperSensorManagerModelInput();
    void receive_func(); //接收线程的处理函数        
    std::vector<std::string> StringSplit(const std::string &str, const std::string &delim);

    sweeper::bus::Brake60 brake_60;
    sweeper::bus::Brake61 brake_61;
    sweeper::bus::Throttle62 throttle_62;
    sweeper::bus::Throttle63 throttle_63;
    sweeper::bus::Steering64 steering_64;
    sweeper::bus::Steering65 steering_65;
    sweeper::bus::VehicleControl66 vehiclecontrol_66;
    sweeper::bus::VehicleControl67 vehiclecontrol_67;
    sweeper::bus::VehicleMode68 vehiclemode_68;
    sweeper::bus::VehicleMode69 vehiclemode_69;
    sweeper::bus::VehicleFault71 vehiclefault_71;
    sweeper::bus::VehicleStatus70 vehiclestatus_70;    
    sweeper::bus::VehicleStatus72 vehiclestatus_72;
    sweeper::bus::SensorFault76 sensorfault_76;
    sweeper::bus::VoicePrompts2AA voiceprompts_200;

    SweeperNodeManageModeModelClass nodemanager;
    SweeperDriveModeControlModelClass  drivemode;
    SweeperFaultManageModeModelClass faultmanager;
    SweeperSensorManageModeModelClass sensormanager;
    SweeperDriveChassisControlModelClass chassiscontrol;
    SweeperVoiceManagerModelClass voicemanager;

    sweeper_msgs::SensorFaultInformation::_state_code_type display_state_code_;
    sweeper_msgs::SensorFaultInformation::_state_code_type csu_state_code_;   
    sweeper_msgs::SensorFaultInformation::_state_code_type planning_state_code_;
    sweeper_msgs::SensorFaultInformation::_state_code_type control_state_code_;
    sweeper_msgs::SensorFaultInformation::_state_code_type djlidar_state_code_;
    sweeper_msgs::SensorFaultInformation::_state_code_type gnss_state_code_;
    sweeper_msgs::SensorFaultInformation::_state_code_type wjlidar_state_code_;
    sweeper_msgs::SensorFaultInformation::_state_code_type radar_fusion_state_code_;
    sweeper_msgs::SensorFaultInformation::_state_code_type imu_state_code_;
    sweeper_msgs::SensorFaultInformation::_state_code_type mapping_state_code_;
    sweeper_msgs::SensorFaultInformation::_state_code_type location_state_code_;
    sweeper_msgs::SensorFaultInformation::_state_code_type app_state_code_;
    sweeper_msgs::SensorFaultInformation::_state_code_type v2x_state_code_;

    CarFeedBackInfo car_feed_back_info;  

    ros::Publisher  sweeper_state_pub_;
    ros::Publisher  chassis_detail_pub_;
    ros::Subscriber node_state_sub_;
    ros::Subscriber chassis_cmd_sub_;
    ros::Subscriber sweeper_mission_sub_;

    //定义状态容器及各状态对象
    std::vector<SweeperStates *> state_class_;  
    InitState *init_state_;      
    IdleState *idle_state_;    
    RecordGnssLineState *record_gnss_line_state_;
    RecordLidarLineState *record_lidar_line_state_;
    GnssTraceState *gnss_trace_state_;
    LidarTraceState *lidar_trace_state_;                         
    GlobalSweepState *global_sweep_state_;
    CurbSweepState *curb_sweep_state_;
    UpdateState *update_state_;
    SweepDataCenter *sweep_data_center_;
    
    NodeManager *app_node;
    NodeManager *gnss_node;    
    NodeManager *djlidar_node;
    NodeManager *wjlidar_node;
    NodeManager *fusion_lidar_node;
    NodeManager *planning_node;
    NodeManager *control_node;
    NodeManager *imu_node;
    NodeManager *mapping_node;
    NodeManager *location_node;  
    NodeManager *matcher_node; 
    NodeManager *fusion_imu_node; 
    NodeManager *fusion_gps_imu_node; 
    NodeManager *v2x_node;

    int currentState;   //机器人当前状态
    int lastState;      //机器人上一状态


    sweeper::common::can::CanDrive    canDrive_;
    int csu_can_head_;
    int rardar_can_head_;

    int csu_can_;

    double chassis_speed_request_;
    double chassis_angle_request_;
    double chassis_angle_speed_request_;       

    ros::Time recieve_planning_time_, recieve_control_time_, recieve_djlidar_time_;
    ros::Time recieve_gnss_time_, recieve_wjlidar_time_, recieve_fusion_lidar_time_, recieve_v2x_time_;
    ros::Time recieve_imu_time_, recieve_mapping_time_, recieve_location_time_, recieve_app_time_;

    ros::Time reset_djlidar_relay_time_, reset_wjlidar_relay_time_;    

    ros::Time current_run_time_, current_node_time_;
    ros::Time recieve_can_time_, recieve_cmd_time_;          

    double recieve_planning_delta_time_, recieve_control_delta_time_, recieve_djlidar_delta_time_;
    double recieve_gnss_delta_time_, recieve_wjlidar_delta_time_, recieve_fusion_lidar_delta_time_, recieve_v2x_delta_time_;
    double recieve_imu_delta_time_, recieve_mapping_delta_time_, recieve_location_delta_time_, recieve_app_delta_time_;

    double recieve_can_delta_time_, recieve_cmd_delta_time_;

    double reset_planning_delta_time_, reset_control_delta_time_, reset_djlidar_delta_time_, reset_app_delta_time_;
    double reset_gnss_delta_time_, reset_wjlidar_delta_time_, reset_fusion_lidar_delta_time_, reset_v2x_delta_time_;
    double reset_imu_delta_time_, reset_mapping_delta_time_, reset_location_delta_time_;

    double reset_djlidar_relay_delta_time_, reset_wjlidar_relay_delta_time_;   

    double run_task_time; 

    int app_fault_code_;
    int imu_fault_code_;
    int gnss_fault_code_;
    int sweeper_fault_code_;   
    int wjlidar_fault_code_;    
    int djlidar_fault_code_;    
    int control_fault_code_;
    int planning_fault_code_;
    int location_fault_code_;
    int fusion_lidar_fault_code_; 

    int imu_signal_status_;
    int gnss_signal_status_;
    int sweeper_signal_status_;
    int djlidar_signal_status_;
    int control_signal_status_;
    int planning_signal_status_;
    int sweeper_speed_mode_;
    int sweeper_turn_status_;
    int sweeper_stop_status_;
    int sweeper_far_status_;
    int sweeper_width_status_;
    int sweeper_egde_status_;
    int sweeper_left_;
    int sweeper_left_front_;
    int sweeper_right_;
    int sweeper_right_front_;
    int sweeper_lidar_s_status_;

    bool v2x_restart_req_;
    bool app_restart_req_;   
    bool gnss_restart_req_;   
    bool wjlidar_restart_req_;    
    bool djlidar_restart_req_;    
    bool control_restart_req_;
    bool planning_restart_req_;
    bool fusion_lidar_restart_req_;   
    bool imu_restart_req_;
    bool mapping_restart_req_;    
    bool location_restart_req_;    
    bool matcher_restart_req_;   
    bool fusion_imu_restart_req_;   
    bool fusion_gps_imu_restart_req_;   
    bool sweeper_net_status_flag_;

    int app_requeset_mode_, app_request_cmd_, app_request_side_, app_request_flag_;
    int app_request_run_time_, app_request_trim_accuracy_, app_response_flag_, app_request_point_attribute_;
    int dcu_request_mode_, dcu_request_side_, dcu_request_right_side_accuracy_, dcu_request_left_side_accuracy_;
    std::string app_request_line_, app_request_map_, dcu_request_line_;
    std::string default_config_file_, trim_accuracy_config_file_, cpu_temp_file_, gpu_temp_file_;
};

}; // namespace sweeper
#endif

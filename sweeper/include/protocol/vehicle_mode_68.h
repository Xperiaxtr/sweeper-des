#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class VehicleMode68 : public ProtocolData<CarFeedBackInfo>
{

public:

    static const int32_t ID;

    void Reset() override;
    void UpdateData(uint8_t *data) override;    

    VehicleMode68 *request_motor_speed(double speed);
    VehicleMode68 *request_motor_torque(double torque);
    VehicleMode68 *set_drive_control_enable(bool enable);
    VehicleMode68 *request_vehicle_velocity(double speed);    
    VehicleMode68 *request_drive_control_mode(uint8_t mode);
    VehicleMode68 *request_drive_control_gear(uint8_t gear);                    
    VehicleMode68 *request_sweep_side_mode(uint8_t side);
    VehicleMode68 *request_vehicle_velocity_mode(uint8_t mode);
private:

    double  motor_speed_ = 0.0;
    double  motor_torque_ = 0.0;
    uint8_t sweep_side_mode_ = 0;
    uint8_t watchdog_counter_ = 0;
    double  vehicle_velocity_ = 0.0; 
    uint8_t drive_control_mode_ = 0;      
    uint8_t drive_control_gear_ = 0;
    uint8_t vehicle_velocity_mode_ = 0; 
    bool    drive_control_enable_ = false;    

    void set_watchdog_counter_p(uint8_t *data, uint8_t count);
    void set_drive_control_enable_p(uint8_t *data, bool enable);    
    void set_request_motor_speed_p(uint8_t *data, double speed);
    void set_request_motor_torque_p(uint8_t *data, double torque);
    void set_request_vehicle_velocity_p(uint8_t *data, double speed);
    void set_request_drive_control_mode_p(uint8_t *data, uint8_t mode);
    void set_request_drive_control_gear_p(uint8_t *data, uint8_t gear);
    void set_request_sweep_side_mode_p(uint8_t *data, uint8_t side);
    void set_request_vehicle_velocity_mode_p(uint8_t *data, uint8_t mode);    
};

} // namespace bus
} // namespace sweeper

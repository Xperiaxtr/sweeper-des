#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class VehicleStatus72 : public ProtocolData<CarFeedBackInfo>
{

public:

    static const int32_t ID;

    void Reset() override;
    void UpdateData(uint8_t *data) override;    

    VehicleStatus72 *send_autodriving_totaltime(double time);   
    VehicleStatus72 *send_gnss_signal_status(uint8_t signal);                
    VehicleStatus72 *send_autodriving_ready_status(bool ready); 
    VehicleStatus72 *send_chassis_sweep_status(bool enable);
    VehicleStatus72 *send_chassis_sucker_ctrl_status(uint8_t status);
    VehicleStatus72 *send_chassis_flush_ctrl_status(uint8_t status);
    VehicleStatus72 *send_chassis_broom_ctrl_status(uint8_t status);
    VehicleStatus72 *send_chassis_diaphragm_pump_ctrl_status(bool status);
    VehicleStatus72 *send_chassis_water_pump_ctrl_status(bool status);
    VehicleStatus72 *send_chassis_foot_sucker_ctrl_status(bool status);
    VehicleStatus72 *send_internet_signal_status(uint8_t signal);

private:

    double  autodriving_totaltime_ = 0.0;
    uint8_t gnss_signal_status_ = 0;
    uint8_t chassis_sucker_ctrl_status_ = 0;
    uint8_t chassis_flush_ctrl_status_ = 0;
    uint8_t chassis_broom_ctrl_status_ = 0;
    uint8_t initernet_signal_status_ = 0;
 
    bool    autodriving_ready_ = false;
    bool    chassis_sweep_enable_ = false;
    bool    chassis_water_pump_ctrl_status_ = false;   
    bool    chassis_foot_sucker_ctrl_status_ = false;   
    bool    chassis_diaphragm_pump_ctrl_status_ = false;   

    void set_autodriving_totaltime_p(uint8_t *data, double time);
    void set_gnss_signal_status_p(uint8_t *data, uint8_t signal);    
    void set_autodriving_ready_status_p(uint8_t *data, bool ready);
    void set_chassis_sweep_status_p(uint8_t *data, bool enable);
    void send_chassis_sucker_ctrl_status_p(uint8_t *data, uint8_t status);
    void send_chassis_flush_ctrl_status_p(uint8_t *data, uint8_t status);
    void send_chassis_broom_ctrl_status_p(uint8_t *data, uint8_t status);
    void send_chassis_diaphragm_pump_ctrl_status_p(uint8_t *data, bool status);
    void send_chassis_water_pump_ctrl_status_p(uint8_t *data, bool status);
    void send_chassis_foot_sucker_ctrl_status_p(uint8_t *data, bool status);
    void send_internet_signal_status_p(uint8_t *data, uint8_t signal);    
};

} // namespace bus
} // namespace sweeper

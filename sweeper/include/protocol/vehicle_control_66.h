#pragma once
#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class VehicleControl66 : public ProtocolData<CarFeedBackInfo>
{
public:

    static const int32_t ID;
    void Reset() override;
    void UpdateData(uint8_t *data) override;   
    VehicleControl66 *request_reset(bool req);
    VehicleControl66 *response_reset(bool res);
    VehicleControl66 *set_workstatus(uint8_t state);    
    VehicleControl66 *request_slidar_reset(bool req);    
    VehicleControl66 *request_uradar_reset(bool req);
    VehicleControl66 *response_csu_timeout(bool res);
    VehicleControl66 *request_djlidar_reset(bool req);    
    VehicleControl66 *response_csu_faultnum(uint8_t num);
    VehicleControl66 *request_controlstatus(uint8_t state);
    VehicleControl66 *response_controlstatus(uint8_t state);     
    VehicleControl66 *response_csu_faultcode(uint16_t code);
    VehicleControl66 *response_csu_faultlevel(uint8_t level);   
    VehicleControl66 *response_vehicle_acceleration(double acc);
    VehicleControl66 *csu_current_powerconsume_mode(uint8_t mode);              

private:

    bool     csu_reset_request_ = false;
    bool     csu_reset_response_ = false;
    bool     csu_timeout_response_ = false; 
    bool     csu_slidar_reset_request_ = false;        
    bool     csu_uradar_reset_request_ = false; 
    bool     csu_djlidar_reset_request_ = false;

    uint8_t  csu_faultnum_ = 0;
    uint8_t  csu_faultlevel_ = 0;    
    uint16_t csu_faultcode_ = 0;
    uint8_t  watchdog_counter_ = 0;
    double   vehicle_acceleration_ = 0.0;     

    uint8_t csu_workmode_request_ = 0;
    uint8_t csu_workmode_response_ = 0; 
    uint8_t csu_current_workstatus_ = 0;  
    uint8_t csu_current_powerconsume_mode_ = 0;   

    void set_request_reset_p(uint8_t *data, bool req);
    void set_response_reset_p(uint8_t *data, bool res);
    void set_request_slidar_reset_p(uint8_t *data, bool req);    
    void set_request_uradar_reset_p(uint8_t *data, bool req); 
    void set_response_csu_timeout_p(uint8_t *data, bool res);
    void set_watchdog_counter_p(uint8_t *data, uint8_t count);
    void set_request_djlidar_reset_p(uint8_t *data, bool req);
    void set_response_csu_faultnum_p(uint8_t *data, uint8_t num);
    void set_response_csu_faultcode_p(uint8_t *data, uint16_t code);
    void set_response_csu_faultlevel_p(uint8_t *data, uint8_t level);        
    void set_response_vehicle_acceleration_p(uint8_t *data, double acc);    

    void set_workstatus_p(uint8_t *data, uint8_t state);
    void set_request_controlstatus_p(uint8_t *data, uint8_t state);     
    void set_response_controlstatus_p(uint8_t *data, uint8_t state);
    void set_csu_current_powerconsume_mode_p(uint8_t *data, uint8_t mode);     
};

} // namespace bus
} // namespace sweeper

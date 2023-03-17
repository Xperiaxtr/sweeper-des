#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class VehicleStatus70 : public ProtocolData<CarFeedBackInfo>
{

public:

    static const int32_t ID;

    void Reset() override;
    void UpdateData(uint8_t *data) override;    

    VehicleStatus70 *send_csu_temperature(double temp);   
    VehicleStatus70 *send_csu_voltage(double voltage);    
    VehicleStatus70 *send_vehicle_temperature(double temp);    
    VehicleStatus70 *send_total_autodrivingdistance(double voltage);         

private:

    double  csu_temperature_ = 0.0;
    double  csu_voltage_ = 0.0;
    double  vehicle_temperature_ = 0.0;
    double  total_autodrivingdistance_ = 0.0;

    void set_csu_temperature_p(uint8_t *data, double temp);
    void set_csu_voltage_p(uint8_t *data, double voltage);    
    void set_vehicle_temperature_p(uint8_t *data, double temp);
    void set_total_autodrivingdistance_p(uint8_t *data, double distance);        
};

} // namespace bus
} // namespace sweeper

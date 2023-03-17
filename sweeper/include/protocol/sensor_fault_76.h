#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class SensorFault76 : public ProtocolData<CarFeedBackInfo>
{

public:

    static const int32_t ID;

    void Reset() override;
    void UpdateData(uint8_t *data) override;    

    SensorFault76 *send_djlidar_fault_code(double code);   
    SensorFault76 *send_gnss_fault_code(double code);
    SensorFault76 *send_wjlidar_left_fault_code(double code);
    SensorFault76 *send_wjlidar_right_fault_code(double code);
    SensorFault76 *send_imu_fault_code(double code);

private:

    double  djlidar_fault_code_ = 0.0;
    double  gnss_fault_code_ = 0.0;
    double  wjlidar_left_fault_code_ = 0.0;
    double  wjlidar_right_fault_code_ = 0.0;    
    double  imu_fault_code_ = 0.0;

    void set_djlidar_fault_code_p(uint8_t *data, double code);
    void set_gnss_fault_code_p(uint8_t *data, double code);
    void set_wjlidar_left_fault_code_p(uint8_t *data, double code);
    void set_wjlidar_right_fault_code_p(uint8_t *data, double code);
    void set_imu_fault_code_p(uint8_t *data, double code);

};

} // namespace bus
} // namespace sweeper

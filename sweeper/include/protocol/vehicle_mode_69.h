#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class VehicleMode69 : public ProtocolData<CarFeedBackInfo>
{
    
public:
    static const int32_t ID;
    virtual void Parse(const std::uint8_t *bytes, int32_t length, CarFeedBackInfo *chassis_detail) const;

private:
    double is_vehicle_speed(const std::uint8_t *bytes, int32_t length) const;
    double is_motor_speed(const std::uint8_t *bytes, int32_t length) const;
    double is_motor_torque(const std::uint8_t *bytes, int32_t length) const;
    double is_vehicle_drive_mode(const std::uint8_t *bytes, int32_t length) const;
    double is_watchdog_counter(const std::uint8_t *bytes, int32_t length) const;
    double is_vehicle_power_mode_req(const std::uint8_t *bytes, int32_t length) const;
    bool    is_vehicle_drive_enable(const std::uint8_t *bytes, int32_t length) const;
    double is_vehicle_current_gear(const std::uint8_t *bytes, int32_t length) const;
};

} // namespace bus
} // namespace sweeper

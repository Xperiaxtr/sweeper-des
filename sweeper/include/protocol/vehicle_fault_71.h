#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class VehicleFault71 : public ProtocolData<CarFeedBackInfo>
{
    
public:
    static const int32_t ID;
    virtual void Parse(const std::uint8_t *bytes, int32_t length, CarFeedBackInfo *chassis_detail) const;

private:
    double is_dcu_fault_num(const std::uint8_t *bytes, int32_t length) const;
    double is_vehicle_fault_num(const std::uint8_t *bytes, int32_t length) const;
    double is_dcu_fault_code(const std::uint8_t *bytes, int32_t length) const;
    double is_dcu_fault_level(const std::uint8_t *bytes, int32_t length) const;
};

} // namespace bus
} // namespace sweeper

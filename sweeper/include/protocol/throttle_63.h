#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class Throttle63 : public ProtocolData<CarFeedBackInfo>
{

public:

    static const int32_t ID;

    virtual void Parse(const std::uint8_t *bytes, int32_t length, CarFeedBackInfo *chassis_detail) const;

private:

    double pedal_cmd(const std::uint8_t *bytes, int32_t length) const;
    double pedal_input(const std::uint8_t *bytes, int32_t length) const;
    double pedal_output(const std::uint8_t *bytes, int32_t length) const;
    double is_watchdog_counter(const std::uint8_t *bytes, int32_t length) const;
    double parse_two_frames(const std::uint8_t low_byte, const std::uint8_t high_byte) const;    

    bool is_timeout(const std::uint8_t *bytes, int32_t length) const;
    bool is_enabled(const std::uint8_t *bytes, int32_t length) const;
    bool is_power_fault(const std::uint8_t *bytes, int32_t length) const;    
    bool is_watchdog_fault(const std::uint8_t *bytes, int32_t length) const;
    bool is_channel_1_fault(const std::uint8_t *bytes, int32_t length) const;
    bool is_channel_2_fault(const std::uint8_t *bytes, int32_t length) const;
    bool is_driver_active_flag(const std::uint8_t *bytes, int32_t length) const;
    bool is_driver_override_flag(const std::uint8_t *bytes, int32_t length) const;
};

} // namespace bus
} // namespace sweeper

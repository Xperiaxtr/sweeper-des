#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class Throttle62 : public ProtocolData<CarFeedBackInfo>
{
public:
    static const int32_t ID;

    virtual void UpdateData(uint8_t *data);
    virtual void Reset();
    Throttle62 *set_pedal(double pedal);
    Throttle62 *set_pedal_enable(bool enable);
    Throttle62 *set_clear_driver_override_enable(bool enable); 
    Throttle62 *set_ignore_driver_override_enable(bool enable);

private:
    double  pedal_cmd_ = 0.0;
    uint8_t watchdog_counter_ = 0;

    bool    pedal_enable_ = false;
    bool    clear_driver_override_enable_ = false;
    bool    ignore_driver_override_enable_ = false;

    void set_pedal_p(uint8_t *data, double pcmd);
    void set_pedal_enable_p(uint8_t *data, bool en);
    void set_clear_driver_override_flag_p(uint8_t *data, bool clear);
    void set_ignore_driver_override_p(uint8_t *data, bool ignore);
    void set_watchdog_counter_p(uint8_t *data, uint8_t count);
};

} // namespace bus
} // namespace sweeper

#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class Brake60 : public ProtocolData<CarFeedBackInfo>
{
public:
    static const int32_t ID;

    void Reset() override;
    void UpdateData(uint8_t *data) override;

    Brake60 *set_pedal_enable(bool enable);
    Brake60 *set_pedal(double pcmd);    
    Brake60 *set_clear_driver_override_enable(bool enable); 
    Brake60 *set_ignore_driver_override_enable(bool enable);

private:

    double  pedal_cmd_ = 0.0;
    uint8_t watchdog_counter_ = 0;

    bool    pedal_enable_ = false;
    bool    clear_driver_override_enable_ = false;
    bool    ignore_driver_override_enable_ = false;

    void set_pedal_p(uint8_t *data, double pedal);
    void set_pedal_enable_p(uint8_t *data, bool enable);
    void set_clear_driver_override_flag_p(uint8_t *data, bool clear);
    void set_ignore_driver_override_p(uint8_t *data, bool ignore);
    void set_watchdog_counter_p(uint8_t *data, uint8_t count);
};

} // namespace bus
} // namespace sweeper

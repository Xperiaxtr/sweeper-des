#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class Steering64 : public ProtocolData<CarFeedBackInfo> {

  public:

    static const int32_t ID;
    virtual void Reset();    
    virtual void UpdateData(uint8_t *data);
    Steering64 *set_enable(bool enable);
    Steering64 *set_steering_angle(double angle);
    Steering64 *set_steering_angle_speed(double angle_speed);
    Steering64 *set_driver_override_enable(bool enable); 
    Steering64 *set_ignore_driver_override_enable(bool enable);

  private:

    double  steering_angle_ = 0.0;
    uint8_t watchdog_counter_ = 0; 
    double  steering_angle_speed_ = 0.0; 

    bool steering_enable_ = false;
    bool disable_audible_warning_ = false;    
    bool clear_driver_override_enable_ = false;
    bool ignore_driver_override_enable_ = false;

    void set_enable_p(uint8_t *bytes, bool enable);
    void set_steering_angle_p(uint8_t *data, double angle);
    void set_watchdog_counter_p(uint8_t *data, uint8_t count);
    void set_ignore_driver_override_p(uint8_t *data, bool ignore);
    void set_clear_driver_override_flag_p(uint8_t *data, bool clear);    
    void set_steering_angle_speed_p(uint8_t *data, double angle_speed);

};

}  // namespace bus
}  // namespace sweeper


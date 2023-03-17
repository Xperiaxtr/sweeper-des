#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class Steering65 : public ProtocolData<CarFeedBackInfo>
{

public:

  static const int32_t ID;

  virtual void Parse(const std::uint8_t *bytes, int32_t length, CarFeedBackInfo *chassis_detail) const;

private:

  bool is_timeout(const std::uint8_t *bytes, int32_t length) const;
  bool is_enabled(const std::uint8_t *bytes, int32_t length) const; 
  double eps_torque(const std::uint8_t *bytes, int32_t length) const;   
  bool is_power_fault(const std::uint8_t *bytes, int32_t length) const; 
  double steering_angle(const std::uint8_t *bytes, int32_t length) const; 
  double is_watchdog_counter(const std::uint8_t *bytes, int32_t length) const; 
  bool is_driver_override_flag(const std::uint8_t *bytes, int32_t length) const;     
  double reported_steering_angle_cmd(const std::uint8_t *bytes, int32_t length) const;
  double reported_steering_angle_speed(const std::uint8_t *bytes, int32_t length) const;
  
};

} // namespace bus
} // namespace sweeper

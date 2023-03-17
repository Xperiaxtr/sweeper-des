#pragma once
#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class VehicleControl67 : public ProtocolData<CarFeedBackInfo>
{

public:

  static const int32_t ID;
  virtual void Parse(const std::uint8_t *bytes, int32_t length, CarFeedBackInfo *chassis_detail) const;

private:

  double   is_dcu_current_status(const std::uint8_t *bytes, int32_t length) const;
  double   is_dcu_work_mode_request(const std::uint8_t *bytes, int32_t length) const; 
  double   is_dcu_work_mode_response(const std::uint8_t *bytes, int32_t length) const;   
  bool     is_dcu_csu_reset_request(const std::uint8_t *bytes, int32_t length) const; 
  bool     is_dcu_csu_reset_response(const std::uint8_t *bytes, int32_t length) const; 
  bool     is_dcu_djlidar_reset_response(const std::uint8_t *bytes, int32_t length) const;
  bool     is_dcu_slidar_reset_response(const std::uint8_t *bytes, int32_t length) const;
  bool     is_dcu_ebs_reset_status(const std::uint8_t *bytes, int32_t length) const;
  bool     is_dcu_eps_reset_status(const std::uint8_t *bytes, int32_t length) const;
  bool     is_dcu_uradar_reset_status(const std::uint8_t *bytes, int32_t length) const;
  bool     is_dcu_timeout(const std::uint8_t *bytes, int32_t length) const;
  double   is_dcu_watchdog_counter(const std::uint8_t *bytes, int32_t length) const;
  bool     is_dcu_uradar_relay_status(const std::uint8_t *bytes, int32_t length) const;
  bool     is_dcu_djlidar_relay_status(const std::uint8_t *bytes, int32_t length) const;
  bool     is_dcu_wjlidar_relay_status(const std::uint8_t *bytes, int32_t length) const;  
  double   is_dcu_vehicle_acceleration(const std::uint8_t *bytes, int32_t length) const;  
  double   is_dcu_emergency_stop_status(const std::uint8_t *bytes, int32_t length) const;
  bool     is_dcu_auto_mode_ready_status(const std::uint8_t *bytes, int32_t length) const;
  bool     is_dcu_remote_mode_ready_status(const std::uint8_t *bytes, int32_t length) const;  
  double   is_dcu_sweep_side_request(const std::uint8_t *bytes, int32_t length) const;  
  double   is_dcu_vehicle_speed_mode_request(const std::uint8_t *bytes, int32_t length) const;
  double   is_dcu_vehicle_soc(const std::uint8_t *bytes, int32_t length) const;
};

} // namespace bus
} // namespace sweeper

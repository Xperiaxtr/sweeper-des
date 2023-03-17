#pragma once

#include <cstdint>

namespace sweeper
{
namespace bus
{

const int32_t CAN_FRAME_SIZE = 8;
const int32_t MAX_CAN_SEND_FRAME_LEN = 1;
const int32_t MAX_CAN_RECV_FRAME_LEN = 10;
const int32_t MAX_KVASER_CAN_RECV_FRAME_LEN = 1;

const int32_t CANBUS_MESSAGE_LENGTH = 8; // according to ISO-11891-1
const int32_t MAX_CAN_PORT = 3;

enum GearPosition
{
    GEAR_NEUTRAL = 0,
    GEAR_DRIVE = 1,
    GEAR_REVERSE = 2,
    GEAR_PARKING = 3,
    GEAR_LOW = 4,
    GEAR_INVALID = 5,
    GEAR_NONE = 6,
};

enum GearTransType
{
    GearAuto = 0,
    GearMaunual = 1,
};

struct Brake
{
    double brake_input;
    double brake_cmd;
    double brake_output;
    bool brake_enabled;
    bool channel_1_fault;
    bool channel_2_fault;
    bool brake_timeout; // 0: fresh; 1: timeout > 100ms
};

struct Throttle
{
    double throttle_input;
    double throttle_cmd;
    double throttle_output;
    bool throttle_enabled;
    bool channel_1_fault;
    bool channel_2_fault;
    bool throttle_timeout; // 0: fresh; 1: timeout > 100ms
};

struct Eps
{
    double steering_angle;
    double steering_angle_cmd;
    double vehicle_speed;
    double epas_torque;
    bool steering_enabled;
    bool channel_1_fault;
    bool channel_2_fault;
    bool eps_timeout; // 0: fresh; 1: timeout > 100ms
};

struct Gear
{
    bool is_shift_position_valid;
    GearPosition gear_state;
    GearPosition gear_mode;
    GearTransType gear_trans_type;
};

typedef struct CarControlCommand
{

} CarControlCommand;

typedef struct CarFeedBackInfo
{
    double timestamp;
    struct Brake brake;
    struct Throttle throttle;
    struct Eps eps;
    struct Gear gear;
} CarFeedBackInfo;

} // namespace bus
} // namespace sweeper
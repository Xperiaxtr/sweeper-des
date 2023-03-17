#pragma once

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "byte.h"

namespace sweeper {
namespace common {

const int32_t CAN_FRAME_SIZE = 8;
const int32_t MAX_CAN_SEND_FRAME_LEN = 1;
const int32_t MAX_CAN_RECV_FRAME_LEN = 10;
const int32_t MAX_KVASER_CAN_RECV_FRAME_LEN = 1;

const int32_t CANBUS_MESSAGE_LENGTH = 8;  // according to ISO-11891-1
const int32_t MAX_CAN_PORT = 3;

enum SystemStatus {
  SYSTEM_OK = 0,
  SYSTEM_FAULT = 1,
  EMERGECY_MODE = 2,
};

enum Mode {
  REMOTE_CONTROL = 0,
  CAN_COMMANDED_CONTROL = 1,
  SERAIL_COMMANDED_CONTROL = 2,
};

struct CanFrame {
  /// Message id
  uint32_t id;
  /// Message length
  uint8_t len;
  /// Message content
  uint8_t data[8];
  /// Time stamp
  struct timeval timestamp;

  /**
   * @brief Constructor
   */
  CanFrame() : id(0), len(0), timestamp{0} {
    std::memset(data, 0, sizeof(data));
  }

  /**
   * @brief CanFrame string including essential information about the message.
   * @return The info string.
   */
  std::string CanFrameString() const {
    std::stringstream output_stream("");
    output_stream << "id:0x" << Byte::byte_to_hex(id)
                  << ",len:" << static_cast<int>(len) << ",data:";
    for (uint8_t i = 0; i < len; ++i) {
      output_stream << Byte::byte_to_hex(data[i]);
    }
    output_stream << ",";
    return output_stream.str();
  }
};

typedef struct {
  int64_t id;        /* can-id                                     */
  uint8_t data[8];   /* 8 data-bytes                               */
  uint32_t len;      /* length of message: 0-8                     */
  uint32_t flag;     /*Pointer to a buffer which receives the message flags, */
  uint64_t time;     /* count of lost rx-messages                  */
  uint64_t time_out; /* count of lost rx-messages                  */

} Kvaser_MSG;


}  // namespace common
}  // namespace sweeper
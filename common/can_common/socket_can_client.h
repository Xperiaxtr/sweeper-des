/**
 * @file
 * @brief Defines the SocketCanClientRaw class which inherits CanClient.
 */

#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "../log.h"
#include "type.h"

namespace sweeper {
namespace common {

class SocketCanClientRaw {
 public:
  ~SocketCanClientRaw();
  bool Init(int can_channel_id_);
  bool Start(int can_filter_nums,
             std::vector<unsigned int> &receeive_can_frame_id);
  void Stop();

  bool Send(const std::vector<CanFrame> &frames, int32_t *const frame_num);
  bool Receive(std::vector<CanFrame> *const frames, int32_t *const frame_num,
                   std::vector<unsigned int> &receive_can_frame_id);
  std::string GetErrorString(const int32_t status);

 private:
  bool is_started_;
  int port_;
  int dev_handler_ = 0;
  can_frame send_frames_[MAX_CAN_SEND_FRAME_LEN];
  can_frame recv_frames_[MAX_CAN_RECV_FRAME_LEN];
};

}  // namespace common
}  // namespace sweeper

#include "socket_can_client.h"

namespace sweeper {
namespace common {

bool SocketCanClientRaw::Init(int can_channel_id) {
  port_ = can_channel_id;
  is_started_ = false;
  return is_started_;
}

SocketCanClientRaw::~SocketCanClientRaw() {
  if (dev_handler_) {
    Stop();
  }
}

bool SocketCanClientRaw::Start(
    int can_filter_nums, std::vector<unsigned int> &receeive_can_frame_id) {
  if (is_started_) {
    return true;
  }
  struct sockaddr_can addr;
  struct ifreq ifr;

  // open device
  // guss net is the device minor number, if one card is 0,1
  // if more than one card, when install driver u can specify the minior id
  // int32_t ret = canOpen(net, pCtx->mode, txbufsize, rxbufsize, 0, 0,
  // &dev_handler_);
  if (port_ > MAX_CAN_PORT || port_ < 0) {
    AERROR << "can port number [" << port_ << "] is out of the range [0,"
           << MAX_CAN_PORT << "]";
    return false;
  }

  dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (dev_handler_ < 0) {
    AERROR << "open device error code [" << dev_handler_ << "]: ";
    return false;
  }

  // init config and state
  // 1. set receive message_id filter, ie white list
  struct can_filter filter[can_filter_nums];
  for (int i = 0; i < can_filter_nums; ++i) {
    filter[i].can_id = receeive_can_frame_id[i];
    filter[i].can_mask = CAN_SFF_MASK;
  }

  int ret = setsockopt(dev_handler_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter,
                       sizeof(filter));
  if (ret < 0) {
    AERROR << "add receive msg id filter error code: " << ret;
    return false;
  }

  // 2. enable reception of can frames.
  int enable = 1;
  ret = ::setsockopt(dev_handler_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable,
                     sizeof(enable));
  if (ret < 0) {
    AERROR << "enable reception of can frame error code: " << ret;
    return false;
  }

  std::string can_name("can" + std::to_string(port_));
  std::strncpy(ifr.ifr_name, can_name.c_str(), IFNAMSIZ);
  if (ioctl(dev_handler_, SIOCGIFINDEX, &ifr) < 0) {
    AERROR << "ioctl error";
    return false;
  }

  // bind socket to network interface

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),
               sizeof(addr));

  if (ret < 0) {
    AERROR << "bind socket to network interface error code: " << ret;
    return false;
  }

  is_started_ = true;
  return true;
}

void SocketCanClientRaw::Stop() {
  if (is_started_) {
    is_started_ = false;

    int ret = close(dev_handler_);
    usleep(500000);

    if (ret < 0) {
      AERROR << "close error code:" << ret << ", " << GetErrorString(ret);
    } else {
      AINFO << "close socket can ok. port:" << port_;
    }
  }
}

// Synchronous transmission of CAN messages
bool SocketCanClientRaw::Send(const std::vector<CanFrame> &frames,
                              int32_t *const frame_num) {
  CHECK_NOTNULL(frame_num);
  CHECK_EQ(frames.size(), static_cast<size_t>(*frame_num));

  if (!is_started_) {
    AERROR << "Nvidia can client has not been initiated! Please init first!";
    return false;
  }
  for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i) {
    if (frames[i].len != CANBUS_MESSAGE_LENGTH) {
      AERROR << "frames[" << i << "].len = " << frames[i].len
             << ", which is not equal to can message data length ("
             << CANBUS_MESSAGE_LENGTH << ").";
      return false;
    }
    send_frames_[i].can_id = frames[i].id;
    send_frames_[i].can_dlc = frames[i].len;
    std::memcpy(send_frames_[i].data, frames[i].data, frames[i].len);

    // Synchronous transmission of CAN messages
    int ret = write(dev_handler_, &send_frames_[i], sizeof(send_frames_[i]));
    if (ret <= 0) {
      AERROR << "send message failed, error code: " << ret;
      return false;
    }
  }

  return true;
}

// buf size must be 8 bytes, every time, we receive only one frame
bool SocketCanClientRaw::Receive(
    std::vector<CanFrame> *const frames, int32_t *const frame_num,
    std::vector<unsigned int> &receive_can_frame_id) {
  if (!is_started_) {
    AERROR << "Can client is not init! Please init first!";
    return false;
  }

  if (*frame_num > MAX_CAN_RECV_FRAME_LEN || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
           << "], frame_num:" << *frame_num;
    // TODO(Authors): check the difference of returning frame_num/error_code
    return false;
  }

  for (int32_t i = 0; i < *frame_num && i < MAX_CAN_RECV_FRAME_LEN; ++i) {
    CanFrame cf;
    int ret = read(dev_handler_, &recv_frames_[i], sizeof(recv_frames_[i]));

    if (ret < 0) {
      AERROR << "receive message failed, error code: " << ret;
      return false;
    }
    // if ((recv_frames_[i].can_dlc != CANBUS_MESSAGE_LENGTH &&
    //      recv_frames_[i].can_id != receive_can_frame_id[1]) ||
    //     (recv_frames_[i].can_dlc != CANBUS_MESSAGE_LENGTH / 2 &&
    //      recv_frames_[i].can_id == receive_can_frame_id[1]) ||
    //     (recv_frames_[i].can_dlc != (CANBUS_MESSAGE_LENGTH -1) &&
    //      recv_frames_[i].can_id == receive_can_frame_id[3])) {
    //   AERROR << "recv_frames_[" << i
    //          << "].can_dlc = " << (int)recv_frames_[i].can_dlc
    //          << ", which is not equal to can message data length ("
    //          << CANBUS_MESSAGE_LENGTH << ").";
    //   return false;
    // }
    cf.id = recv_frames_[i].can_id;
    cf.len = recv_frames_[i].can_dlc;
    std::memcpy(cf.data, recv_frames_[i].data, recv_frames_[i].can_dlc);
    frames->push_back(cf);
  }
  return true;
}

std::string SocketCanClientRaw::GetErrorString(const int32_t /*status*/) {
  return "";
}

}  // namespace common
}  // namespace sweeper

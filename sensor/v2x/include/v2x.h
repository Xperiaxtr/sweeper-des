#pragma once

#include <arpa/inet.h>
#include <iostream>
#include <netinet/in.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "../../../common/frame_transform.h"
#include "../../../common/log.h"
#include "../../../common/watch_dog.h"
#include "sweeper_msgs/Light.h"
#include "sweeper_msgs/Obu.h"
#include "sweeper_msgs/SensorFaultInformation.h"

#define BUFF_SIZE 2000
namespace sweeper {
namespace v2x {

using namespace std;
using namespace ros;

class V2x {
public:
  V2x(ros::NodeHandle &node, ros::NodeHandle &private_nh);
  ~V2x();
  bool Init();
  void CommunicateObu();

private:
  int UdpSocketServer(struct sockaddr_in &addr_serv);
  void ParseObuMsg(const char *buffer);
  void SelfDiagnose();

  bool initialized_;
  int port_;
  int sock_fd_;
  ros::Publisher v2x_msg_pub_, v2x_diagnose_pub_;
  struct sockaddr_in addr_serv_;
  sweeper::common::WatchDog watch_dog_reci_v2x_;
  sweeper_msgs::Light left_turn_light_, straight_light_, right_turn_light_;
  sweeper_msgs::Obu obu_msg_;
};
} // namespace v2x
} // namespace sweeper

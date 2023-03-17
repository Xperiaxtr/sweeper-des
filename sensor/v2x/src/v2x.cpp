#include "v2x.h"
#include <stdio.h>
namespace sweeper {
namespace v2x {

V2x::V2x(ros::NodeHandle &node, ros::NodeHandle &private_nh) : sock_fd_(-1) {
  private_nh.param("port", port_, 5000);

  v2x_msg_pub_ = node.advertise<sweeper_msgs::Obu>("/sweeper/sensor/obu", 10);
  v2x_diagnose_pub_ = node.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/common/diagnose", 1);

  sock_fd_ = UdpSocketServer(addr_serv_);
  if (sock_fd_ < 0) {
    initialized_ = false;
  } else {
    initialized_ = true;
  }
  std::thread v2x_diagnose(&V2x::SelfDiagnose, this);
  v2x_diagnose.detach();
}

V2x::~V2x() {}

bool V2x::Init() { return initialized_; }

void V2x::ParseObuMsg(const char *buffer) {
  sweeper_msgs::Light traffic_light_msg;
  traffic_light_msg.data_valid_flag = buffer[0];
  traffic_light_msg.current_light_flag = buffer[1];
  traffic_light_msg.light_direction = buffer[2];
  traffic_light_msg.light_color = buffer[3];
  traffic_light_msg.countdown = buffer[4] + buffer[5] * 256;
  printf("buffer :%x %x %x %c %x %x %d\n", buffer[0], buffer[1], buffer[2],
         buffer[3], buffer[4], buffer[5], traffic_light_msg.countdown);
  switch (buffer[2]) {
  case 1:
    left_turn_light_ = traffic_light_msg;
    break;
  case 2:
    straight_light_ = traffic_light_msg;
    break;
  case 3:
    right_turn_light_ = traffic_light_msg;
    break;
  default:
    break;
  }
}

int V2x::UdpSocketServer(struct sockaddr_in &addr_serv) {
  int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd < 0) {
    return -1;
  }

  /* 将套接字和IP、端口绑定 */
  int len;
  memset(&addr_serv, 0, sizeof(struct sockaddr_in)); //每个字节都用0填充
  addr_serv.sin_family = AF_INET;                    //使用IPV4地址
  addr_serv.sin_port = htons(port_);                 //端口
  /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到
   */
  addr_serv.sin_addr.s_addr = htonl(INADDR_ANY); //自动获取IP地址
  len = sizeof(addr_serv_);

  /* 绑定socket */
  if (bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0) {
    return -1;
  }
  return sock_fd;
}

void V2x::CommunicateObu() {
  struct sockaddr_in addr_client;
  int len = sizeof(addr_serv_);
  int recv_num = -1;
  char recv_buf[BUFF_SIZE];

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    memset(recv_buf, 0, sizeof(recv_buf));
    recv_num = recvfrom(sock_fd_, recv_buf, sizeof(recv_buf), 0,
                        (struct sockaddr *)&addr_client, (socklen_t *)&len);
    if (recv_num < 0) {
      AINFO << "recvfrom error";
    } else {
      // AINFO << "recv_num: " << recv_num;
      watch_dog_reci_v2x_.UpdataNow();
      int msg_nums = recv_num / sizeof(sweeper_msgs::Light);
      for (int i = 0; i < msg_nums; i++) {
        V2x::ParseObuMsg(&recv_buf[i * sizeof(sweeper_msgs::Light)]);
      }
      // printf("left_turn_light :%x %x %x %c
      // %d\n",left_turn_light_.data_valid_flag,left_turn_light_.current_light_flag,left_turn_light_.light_direction,left_turn_light_.light_color,left_turn_light_.countdown);
      // printf("straight_light :%x %x %x %c
      // %d\n",straight_light_.data_valid_flag,straight_light_.current_light_flag,straight_light_.light_direction,straight_light_.light_color,straight_light_.countdown);
      // printf("right_turn_light :%x %x %x %c
      // %d\n",right_turn_light_.data_valid_flag,right_turn_light_.current_light_flag,right_turn_light_.light_direction,right_turn_light_.light_color,right_turn_light_.countdown);

      obu_msg_.header.frame_id = "obu";
      obu_msg_.header.stamp = ros::Time::now();
      obu_msg_.traffic_light.push_back(left_turn_light_);
      obu_msg_.traffic_light.push_back(straight_light_);
      obu_msg_.traffic_light.push_back(right_turn_light_);
      v2x_msg_pub_.publish(obu_msg_);
       //printf("obu_msg_.traffic_light0 :%x %x %x %c %d\n",obu_msg_.traffic_light[0].data_valid_flag,obu_msg_.traffic_light[0].current_light_flag,obu_msg_.traffic_light[0].light_direction,obu_msg_.traffic_light[0].light_color,obu_msg_.traffic_light[0].countdown);
       //printf("obu_msg_.traffic_light1 :%x %x %x %c %d\n",obu_msg_.traffic_light[1].data_valid_flag,obu_msg_.traffic_light[1].current_light_flag,obu_msg_.traffic_light[1].light_direction,obu_msg_.traffic_light[1].light_color,obu_msg_.traffic_light[1].countdown);
       //printf("obu_msg_.traffic_light2 :%x %x %x %c %d\n",obu_msg_.traffic_light[2].data_valid_flag,obu_msg_.traffic_light[2].current_light_flag,obu_msg_.traffic_light[2].light_direction,obu_msg_.traffic_light[2].light_color,obu_msg_.traffic_light[2].countdown);
      obu_msg_.traffic_light.clear();
    }
    loop_rate.sleep();
  }
}

void V2x::SelfDiagnose() {
  ros::Rate loop(5);
  while (ros::ok()) {
    sweeper_msgs::SensorFaultInformation state_v2x;
    if (!initialized_) {
      state_v2x.state_code.push_back(2701);
    }

    if (!watch_dog_reci_v2x_.DogIsOk(5)) {
      state_v2x.state_code.push_back(2702);
    }

    if (state_v2x.state_code.empty()) {
      state_v2x.state_code.push_back(2700);
      state_v2x.header.frame_id = "v2x";
      state_v2x.header.stamp = ros::Time::now();
    } else {
      state_v2x.header.frame_id = "v2x";
      state_v2x.header.stamp = ros::Time::now();
    }
    v2x_diagnose_pub_.publish(state_v2x);
    loop.sleep();
  }
}
} // namespace v2x
} // namespace sweeper

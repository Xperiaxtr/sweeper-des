#include "gnss.h"

namespace sweeper {
namespace gnss {
const char TABLE[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

const char *triplet_base64(int triplet) {
  static char result[4];
  result[0] = TABLE[(triplet >> 18) & 0x3f];
  result[1] = TABLE[(triplet >> 12) & 0x3f];
  result[2] = TABLE[(triplet >> 6) & 0x3f];
  result[3] = TABLE[triplet & 0x3f];
  return result;
}

Gnss::Gnss(ros::NodeHandle &node, ros::NodeHandle &private_nh)
    : pos_satellite_num_(-1),
      sockfd_bynav_receive_(-1),
      sockfd_bynav_send_(-1),
      socketfd_base_(-1),
      pos_diff_edge_(0.0),
      counts_(0),
      connect_base_flag_(false),
      connect_bynav_receive_flag_(false),
      connect_bynav_send_flag_(false),
      initialized_(false),
      recieve_gpgga_flag_(false),
      recieve_inspvaxa_flag_(false),
      recieve_base_data_flag_(true) {
  private_nh.param("gnss_mode", gnss_mode_, std::string("cidi"));
  private_nh.param("protocol", protocol_, std::string("RTCMV3"));
  private_nh.param("base_station_ip", base_station_ip_,
                   std::string("222.240.49.4"));
  private_nh.param("bynav_ip", bynav_ip_, std::string("192.168.1.98"));
  private_nh.param("base_station_port", base_station_port_, 2101);
  private_nh.param("bynav_send_port", bynav_send_port_, 2222);
  private_nh.param("bynav_receive_port", bynav_receive_port_, 8000);
  private_nh.param("user", user_, std::string("sweep"));
  private_nh.param("password", password_, std::string("sweep"));
  private_nh.param("error_position", error_position_, 0.03);
  private_nh.param("flag_diff_write", flag_diff_write_, false);
  private_nh.param("flag_position_accuracy", flag_position_accuracy_, -1);
  private_nh.param("flag_longtitude_error", flag_longtitude_error_, false);
  private_nh.param("install_angle", install_angle_, 0.03);
  private_nh.param("gpgga_rate", pub_gpgga_rate_, 20.0);

  gps_odom_pub_ =
      node.advertise<nav_msgs::Odometry>("/sweeper/sensor/gnss", 10);
  gps_localization_pub_ =
      node.advertise<nav_msgs::Odometry>("/sweeper/localization/gnss", 10);
  gps_fix_pub_ = node.advertise<sensor_msgs::NavSatFix>("/sweeper/gps/fix", 10);
  imu_raw_pub_ = node.advertise<sensor_msgs::Imu>("/sweeper/sensor/imu", 10);

  gnss_diagnose_pub_ = node.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/common/diagnose", 1);
  login_data_ = "GET " + protocol_ +
                " HTTP/1.0\r\n"
                "User-Agent: NTRIP gnss_driver/0.0\r\n"
                "accept: */* \r\n"
                "Authorization: Basic " +
                encode_base64(user_ + ":" + password_) + "\r\n\r\n";

  std::thread t_base(&Gnss::SendBaseToBynav, this);
  t_base.detach();

  std::thread t_move(&Gnss::RecieveGnssData, this);
  t_move.detach();

  std::thread gnss_diagnose(&Gnss::SelfDiagnose, this);
  gnss_diagnose.detach();
  initialized_ = true;
}
Gnss::~Gnss() {}

bool Gnss::Init() { return initialized_; }

int Gnss::TcpClient(const std::string &addr, const int port,
                    struct sockaddr_in *servaddr) {
  int sockfd = -1;
  int connect_num = 0;
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    AERROR << "Create socket fail!";
    return -1;
  }
  memset(servaddr, 0, sizeof(struct sockaddr_in));
  servaddr->sin_family = AF_INET;
  servaddr->sin_addr.s_addr = inet_addr(addr.c_str());
  servaddr->sin_port = htons(port);
  return sockfd;
}

void Gnss::DiffDataForward() {
  unsigned char buff[1024];
  memset(&buff[0], 0, sizeof(buff));
  int receive_nums = recv(socketfd_base_, buff, 1024, 0);
  int send_nums = send(sockfd_bynav_send_, buff, receive_nums, 0);

  if (receive_nums == send_nums && receive_nums != 0) {
    AWARN << "Recieve sucess";
    flag_diff_write_ = true;
    flag_cus_write_ = true;
    flag_cus_read_ = true;
    watch_dog_reci_diff_.UpdataNow();
    watch_dog_write_diff_.UpdataNow();
  } else if (receive_nums <= 0) {
    connect_base_flag_ = false;
    flag_diff_write_ = false;
    flag_cus_read_ = false;
    AWARN << "Not recieve tcp data from base.";
  } else if (send_nums <= 0) {
    connect_bynav_send_flag_ = false;
    flag_diff_write_ = false;
    flag_cus_write_ = false;
    AWARN << "Not tcp data send to bynav.";
  } else if (send_nums != receive_nums) {
    flag_diff_write_ = false;
    AWARN << "Send base serial data to bynav failed.";
  }
}

int Gnss::SendBaseToBynav() {
  struct sockaddr_in base_serv_addr;
  struct sockaddr_in bynav_send_addr;
  socketfd_base_ =
      TcpClient(base_station_ip_, base_station_port_, &base_serv_addr);
  sockfd_bynav_send_ = TcpClient(bynav_ip_, bynav_send_port_, &bynav_send_addr);

  ros::Rate rate(10);

  while (ros::ok()) {
    if (socketfd_base_ >= 0 && !connect_base_flag_) {
      int res = connect(socketfd_base_, (struct sockaddr *)&base_serv_addr,
                        sizeof(base_serv_addr));
      if (res == -1) {
        ADEBUG << "Tcp connect base faided!";
      } else {
        AINFO << "Tcp connect base success!";
        connect_base_flag_ = true;
        ssize_t write_size =
            write(socketfd_base_,
                  reinterpret_cast<const uint8_t *>(login_data_.data()),
                  login_data_.size());
      }
    }

    if (sockfd_bynav_send_ >= 0 && !connect_bynav_send_flag_) {
      int con = connect(sockfd_bynav_send_, (struct sockaddr *)&bynav_send_addr,
                        sizeof(bynav_send_addr));
      if (con == -1) {
        ADEBUG << "Tcp connect bynav send port faided!";
      } else {
        AINFO << "Tcp connect bynav send port success!";
        connect_bynav_send_flag_ = true;
      }
    }

    // AWARN << connect_base_flag_ << " " << connect_bynav_send_flag_ << " "
    //       << gnss_gpgga_ << " " << gnss_mode_;
    if (connect_base_flag_ && connect_bynav_send_flag_) {
      if (gnss_gpgga_.size() > 10 && gnss_mode_ == "qianxun") {
        int write_count =
            write(socketfd_base_,
                  reinterpret_cast<const uint8_t *>(gnss_gpgga_.data()),
                  gnss_gpgga_.size());
        DiffDataForward();
      } else if (gnss_mode_ == "cidi") {
        DiffDataForward();
      }
    } else {
      rate.sleep();
    }
  }
  close(socketfd_base_);
  close(sockfd_bynav_send_);
}

int Gnss::RecieveGnssData() {
  struct sockaddr_in bynav_receive_addr;
  sockfd_bynav_receive_ =
      TcpClient(bynav_ip_, bynav_receive_port_, &bynav_receive_addr);
  ros::Rate rate(100);
  while (ros::ok()) {
    if (sockfd_bynav_receive_ >= 0 && !connect_bynav_receive_flag_) {
      int con =
          connect(sockfd_bynav_receive_, (struct sockaddr *)&bynav_receive_addr,
                  sizeof(bynav_receive_addr));
      if (con == -1) {
        ADEBUG << "Tcp connect bynav receive port faided!";
      } else {
        AINFO << "Tcp connect bynav receive port success!";
        connect_bynav_receive_flag_ = true;
      }
    }

    if (connect_bynav_receive_flag_) {
      char buffer[5000];
      std::string gnss_ntrip;
      int recv_len = -1;
      memset(buffer, 0, sizeof(buffer));
      recv_len = recv(sockfd_bynav_receive_, buffer, sizeof(buffer), 0);
      // AINFO << "Recv len : " << recv_len;
      if (recv_len > 0) {
        watch_dog_reci_ctrl_.UpdataNow();
        gnss_ntrip = buffer;
        ParseNtripData(gnss_ntrip);
      } else if (recv_len <= 0) {
        connect_bynav_receive_flag_ = false;
      }
    } else {
      rate.sleep();
    }
  }
}

std::string Gnss::encode_base64(const std::string &in) {
  std::string out;
  if (in.empty()) {
    return out;
  }

  int in_size = in.size();

  out.reserve(((in_size - 1) / 3 + 1) * 4);

  int i = 2;
  for (; i < in_size; i += 3) {
    out.append(triplet_base64((in[i - 2] << 16) | (in[i - 1] << 8) | in[i]), 4);
  }
  if (i == in_size) {
    out.append(triplet_base64((in[i - 2] << 16) | (in[i - 1] << 8)), 3);
    out.push_back('=');
  } else if (i == in_size + 1) {
    out.append(triplet_base64(in[i - 2] << 16), 2);
    out.append("==");
  }
  return out;
}

void Gnss::GpsToOdom(const sensor_msgs::NavSatFix &navSat,
                     nav_msgs::Odometry &odom) {
  double northing, easting;
  sweeper::UTMCoor utm_xy;

  gps_transform_.LatlonToUtmXY(navSat.longitude * TORAD,
                               navSat.latitude * TORAD, &utm_xy);

  // AWARN<<"latlontoUtmxy: "<<utm_xy.x<<" "<<utm_xy.y<<" "<<navSat.altitude;
  localization_gps_.header.stamp = odom.header.stamp = ros::Time::now();

  double yaw_angle = yaw_ + install_angle_;
  if (yaw_angle >= 360.0) {
    yaw_angle = yaw_angle - 360.0;
  }
  if (yaw_angle > 180) {
    yaw_angle = -(360 - yaw_angle);
  }
  double yaw = yaw_angle * TORAD;
  double roll = roll_ * TORAD;
  double pitch = pitch_ * TORAD;
  geometry_msgs::Quaternion gps_quaternion =
      tf::createQuaternionMsgFromRollPitchYaw(-0.0, 0.0, -yaw);
  odom.pose.pose.orientation.x = gps_quaternion.x;
  odom.pose.pose.orientation.y = gps_quaternion.y;
  odom.pose.pose.orientation.z = gps_quaternion.z;
  odom.pose.pose.orientation.w = gps_quaternion.w;

  localization_gps_quaternion_ =
      tf::createQuaternionMsgFromRollPitchYaw(roll, -pitch, -yaw);
  localization_gps_.pose.pose.orientation.x = localization_gps_quaternion_.x;
  localization_gps_.pose.pose.orientation.y = localization_gps_quaternion_.y;
  localization_gps_.pose.pose.orientation.z = localization_gps_quaternion_.z;
  localization_gps_.pose.pose.orientation.w = localization_gps_quaternion_.w;

  Eigen::Vector3d world_positon(utm_xy.x, utm_xy.y, navSat.altitude);

  localization_gps_.pose.pose.position.x = odom.pose.pose.position.x =
      world_positon(0);  // - 684602;
  localization_gps_.pose.pose.position.y = odom.pose.pose.position.y =
      world_positon(1);  // - 3112642;
  localization_gps_.pose.pose.position.z = odom.pose.pose.position.z =
      world_positon(2);  // - 45;

  double x_dev = navSat.position_covariance[0];
  double y_dev = navSat.position_covariance[4];
  double z_dev = navSat.position_covariance[8];

  // Use ENU covariance to build XYZRPY covariance
  boost::array<double, 36> covariance = {
      x_dev, 0, 0,     0, 0,          0, 0, y_dev, 0, 0,         0, 0,
      0,     0, z_dev, 0, 0,          0, 0, 0,     0, roll_dev_, 0, 0,
      0,     0, 0,     0, pitch_dev_, 0, 0, 0,     0, 0,         0, yaw_dev_};

  localization_gps_.pose.covariance = odom.pose.covariance = covariance;
}

std::vector<std::string> Gnss::SplitNtripMsg(const std::string &str,
                                             const std::string &delim) {
  std::vector<std::string> res;
  if ("" == str) return res;
  char *strs = new char[str.length() + 1];
  strcpy(strs, str.c_str());

  char *d = new char[delim.length() + 1];
  strcpy(d, delim.c_str());

  char *p = strtok(strs, d);
  while (p) {
    std::string s = p;
    res.push_back(s);
    p = strtok(NULL, d);
  }
  return res;
}
void Gnss::ParseGpgga(std::string &data, std::vector<std::string> &list) {
  recieve_gpgga_flag_ = true;
  gnss_gpgga_ = data + "\r\n";
  // AWARN<<"gpgga : "<<gnss_gpgga_;
  pos_state_ = atoi(list[6].c_str());
  // AINFO << "gps pos state: " << pos_state_;
  /*
if (pos_state_ == 4)
 nav_sat_fix_.status.status = nav_sat_fix_.status.STATUS_FIX;
else
 nav_sat_fix_.status.status = nav_sat_fix_.status.STATUS_NO_FIX;*/
  // if(pos_state_ == 4 ) gnss_state_ = true;
  pos_satellite_num_ = atoi(list[7].c_str());
  pos_diff_edge_ = atof(list[13].c_str());
}

void Gnss::ParseInspvaxa(std::string &data) {
  std::vector<std::string> list_semicolon = SplitNtripMsg(data, ";");
  if (list_semicolon.size() == 2) {
    std::vector<std::string> list_field = SplitNtripMsg(list_semicolon[1], ",");

    solve_state_ = list_field[0];
    // AINFO << "solve_state_ : " << solve_state_;
    if (list_field.size() == 23) {
      recieve_inspvaxa_flag_ = true;

      nav_sat_fix_.latitude = atof(list_field[2].c_str());
      nav_sat_fix_.longitude = atof(list_field[3].c_str());
      nav_sat_fix_.altitude = atof(list_field[4].c_str());
      localization_gps_.twist.twist.linear.x = odom_gps_.twist.twist.linear.x =
          atof(list_field[7].c_str());
      localization_gps_.twist.twist.linear.y = odom_gps_.twist.twist.linear.y =
          atof(list_field[6].c_str());
      localization_gps_.twist.twist.linear.z = odom_gps_.twist.twist.linear.z =
          atof(list_field[8].c_str());

      yaw_ = atof(list_field[11].c_str());
      pitch_ = atof(list_field[10].c_str());
      roll_ = atof(list_field[9].c_str());

      yaw_dev_ = atof(list_field[20].c_str());    // / 10.0;
      roll_dev_ = atof(list_field[18].c_str());   // / 10.0;
      pitch_dev_ = atof(list_field[19].c_str());  // / 10.0;

      nav_sat_fix_.position_covariance[0] = atof(list_field[12].c_str());
      nav_sat_fix_.position_covariance[4] = atof(list_field[13].c_str());
      nav_sat_fix_.position_covariance[8] = atof(list_field[14].c_str());

      if (nav_sat_fix_.longitude > -0.1 && nav_sat_fix_.longitude < 0.1 &&
          nav_sat_fix_.latitude > -0.1 && nav_sat_fix_.latitude < 0.1) {
        flag_longtitude_error_ = false;
      } else {
        flag_longtitude_error_ = true;
      }
    }
  }
}

void Gnss::ParseRawimusa(std::string &data) {
  sensor_msgs::Imu imu_raw;
  std::vector<std::string> list_semicolon = SplitNtripMsg(data, ";");
  if (list_semicolon.size() == 2) {
    std::vector<std::string> list_field = SplitNtripMsg(list_semicolon[1], ",");
    if (list_field.size() == 9) {
      watch_dog_reci_imu_.UpdataNow();
      imu_raw.linear_acceleration.x =
          atof(list_field[5].c_str()) * SPEED_SCALE_FACTOR;
      imu_raw.linear_acceleration.y =
          -atof(list_field[4].c_str()) * SPEED_SCALE_FACTOR;
      imu_raw.linear_acceleration.z =
          atof(list_field[3].c_str()) * SPEED_SCALE_FACTOR;

      imu_raw.angular_velocity.x =
          atof(list_field[8].c_str()) * ANGLE_SCALE_FACTOR / 57.3;
      imu_raw.angular_velocity.y =
          -atof(list_field[7].c_str()) * ANGLE_SCALE_FACTOR / 57.3;
      imu_raw.angular_velocity.z =
          atof(list_field[6].c_str()) * ANGLE_SCALE_FACTOR / 57.3;

      imu_raw.orientation = localization_gps_quaternion_;
      imu_raw.header.stamp = ros::Time::now();
      imu_raw.header.frame_id = "imu";
      imu_raw_pub_.publish(imu_raw);
    }
  }
}

void Gnss::JudgeAccuracyLevel() {
  // 判断gps位置精度等级，0:bad; 1:general; 2:good; 3:best;
  if (solve_state_ == "INS_SOLUTION_GOOD") {
    if (nav_sat_fix_.position_covariance[0] < 0.15 &&
        nav_sat_fix_.position_covariance[4] < 0.15) {
      flag_position_accuracy_ = 3;
    } else if (nav_sat_fix_.position_covariance[0] < error_position_ &&
               nav_sat_fix_.position_covariance[4] < error_position_) {
      flag_position_accuracy_ = 2;
    } else {
      flag_position_accuracy_ = 0;
    }
  } else if (solve_state_ == "INS_ALIGNMENT_COMPLETE") {
    if (nav_sat_fix_.position_covariance[0] < 0.15 &&
        nav_sat_fix_.position_covariance[4] < 0.15) {
      flag_position_accuracy_ = 2;
    } else if (nav_sat_fix_.position_covariance[0] < error_position_ &&
               nav_sat_fix_.position_covariance[4] < error_position_) {
      flag_position_accuracy_ = 1;
    }
  } else {
    flag_position_accuracy_ = 0;
  }

  if (recieve_gpgga_flag_) {
    if (pos_diff_edge_ > 6 || pos_diff_edge_ < 0.01) {
      flag_position_accuracy_ = 0;
    }
    recieve_gpgga_flag_ = false;
    watch_dog_reci_all_.UpdataNow();
  }
}

void Gnss::PublishOdom() {
  // odom_gps_.header.frame_id = "gnss";
  if (flag_position_accuracy_ == 3) {
    odom_gps_.header.frame_id = "best";
  } else if (flag_position_accuracy_ == 2) {
    odom_gps_.header.frame_id = "good";
  } else if (flag_position_accuracy_ == 1) {
    odom_gps_.header.frame_id = "general";
  } else {
    odom_gps_.header.frame_id = "bad";
  }
  odom_gps_.header.stamp = ros::Time::now();
  gps_odom_pub_.publish(odom_gps_);
}

void Gnss::ParseNtripData(std::string info) {
  std::vector<std::string> list_logs = SplitNtripMsg(info, "\r\n");
  for (int i = 0; i < list_logs.size(); i++) {
    std::vector<std::string> list = SplitNtripMsg(list_logs[i], ",");
    if ((strstr(list[0].c_str(), "GPGGA") != NULL) && (list.size() == 15)) {
      ParseGpgga(list_logs[i], list);
    }

    else if (strstr(list[0].c_str(), "INSPVAXA") != NULL) {
      ParseInspvaxa(list_logs[i]);
    }

    else if (strstr(list[0].c_str(), "RAWIMUSA") != NULL) {
      ParseRawimusa(list_logs[i]);
    }
  }

  GpsToOdom(nav_sat_fix_, odom_gps_);

  if (recieve_inspvaxa_flag_ && nav_sat_fix_.latitude > 0.00001 &&
      nav_sat_fix_.longitude > 0.00001) {
    recieve_inspvaxa_flag_ = false;
    JudgeAccuracyLevel();
    double latitude = nav_sat_fix_.latitude;
    nav_sat_fix_.latitude = nav_sat_fix_.longitude;
    nav_sat_fix_.longitude = latitude;
    nav_sat_fix_.header.frame_id = "gnss";
    if (counts_ % (int)(125 / pub_gpgga_rate_) == 0) {
      gps_fix_pub_.publish(nav_sat_fix_);
      PublishOdom();
    }
    counts_++;
    if (counts_ > 10000) counts_ = 0;
    localization_gps_.header.frame_id = "lidar";
    localization_gps_.header.stamp = ros::Time::now();
    gps_localization_pub_.publish(localization_gps_);

    // 判断是否接收到基站数据
    // if (pos_state_ == 1 && pos_diff_edge_ > 6.0 && pos_satellite_num_ > 12 &&
    // orientation_state_ == "NARROW_INT" && flag_diff_write_)
    if (pos_diff_edge_ > 6.0 || pos_diff_edge_ < 0.01)  // && flag_diff_write_)
      recieve_base_data_flag_ = false;
    else
      recieve_base_data_flag_ = true;

    // AWARN << "\n"
    //       << "Position satellite num: " << pos_satellite_num_ << "\n"
    //       << "Position diff edge: " << pos_diff_edge_ << "\n";
  }
}  // namespace gnss

void Gnss::SelfDiagnose() {
  ros::Rate r(5);

  while (ros::ok()) {
    // AWARN << "\n"
    //   << "Position satellite num: " << pos_satellite_num_ << "\n"
    //   << "Position diff edge: " << pos_diff_edge_ << "\n";
    sweeper_msgs::SensorFaultInformation state_gnss;

    // gnss未差分
    if (!(pos_state_ == 4 || pos_state_ == 5)) {
      state_gnss.state_code.push_back(2001);
    }

    //组合导航数据误差大
    if (flag_position_accuracy_ != 3 && flag_position_accuracy_ != 2 &&
        watch_dog_reci_ctrl_.DogIsOk(5)) {
      state_gnss.state_code.push_back(2002);
    }

    // csu长时间读取差分数据失败
    if (/*!flag_cus_read_ ||watch_dog_reci_diff_.DogIsOk(5)|| */
        !recieve_base_data_flag_) {
      state_gnss.state_code.push_back(2003);
    }
    //基站与csu端口打开失败
    if (!connect_base_flag_) {
      state_gnss.state_code.push_back(2004);
    }
    // csu长时间写入差分数据失败
    if (/*!flag_cus_write_ || !watch_dog_write_diff_.DogIsOk(5) || */
        !recieve_base_data_flag_) {
      state_gnss.state_code.push_back(2005);
    }
    // csu与北云差分端口打开失败
    if (!connect_bynav_send_flag_) {
      state_gnss.state_code.push_back(2006);
    }
    //解析导航数据失败
    // if (!watch_dog_reci_all_.DogIsOk(5)) {
    //   state_gnss.state_code.push_back(2006);
    // }

    //北云输出端口打开失败
    if (!connect_bynav_receive_flag_) {
      state_gnss.state_code.push_back(2007);
    }

    // imu未授时
    if (!watch_dog_reci_imu_.DogIsOk(5)) {
      state_gnss.state_code.push_back(2009);
    }

    if (state_gnss.state_code.empty()) {
      state_gnss.state_code.push_back(2000);
      state_gnss.header.frame_id = "gnss";
      state_gnss.header.stamp = ros::Time::now();
    } else {
      state_gnss.header.frame_id = "gnss";
      state_gnss.header.stamp = ros::Time::now();
    }
    gnss_diagnose_pub_.publish(state_gnss);
    r.sleep();
  }
}

}  // namespace gnss
}  // namespace sweeper

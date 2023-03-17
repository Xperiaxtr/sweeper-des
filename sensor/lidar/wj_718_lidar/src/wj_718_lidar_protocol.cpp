#include "wj_718_lidar_protocol.h"
#include <iostream>
#include <laser_geometry/laser_geometry.h>


namespace wj_lidar
{
bool wj_718_lidar_protocol::setConfig(wj_718_lidar::wj_718_lidarConfig &new_config, uint32_t level)
{
    config_ = new_config;
    scan.header.frame_id = config_.frame_id;
    scan.angle_min = config_.min_ang;
    scan.angle_max = config_.max_ang;
    scan.angle_increment = config_.angle_increment;
    scan.time_increment = config_.time_increment;
    scan.range_min = config_.range_min;
    scan.range_max = config_.range_max;
    freq_scan = config_.frequency_scan;
    resizeNum = config_.resize;
    if (freq_scan == 0) //15hz/0.33
    {
        scan.angle_increment = config_.angle_increment / 3;
        scan.time_increment = 1 / 15.00000000 / 811;
        resizeNum = 811;
    }

    scan.ranges.resize(resizeNum);
    scan.intensities.resize(resizeNum);

    cout << "frame_id:" << scan.header.frame_id << endl;
    cout << "min_ang:" << scan.angle_min << endl;
    cout << "max_ang:" << scan.angle_max << endl;
    cout << "angle_increment:" << scan.angle_increment << endl;
    cout << "time_increment:" << scan.time_increment << endl;
    cout << "range_min:" << scan.range_min << endl;
    cout << "range_max:" << scan.range_max << endl;
    cout << "resizeNum:" << resizeNum << endl;
    return true;
}

wj_718_lidar_protocol::wj_718_lidar_protocol(int radar_index) : flag_load_extrinisic_(true)
{
    memset(&m_sdata, 0, sizeof(m_sdata));

    if (radar_index == 0)
        marker_pub = nh.advertise<sensor_msgs::PointCloud>("/sweeper/sensor/radar_left", 1);
    else if (radar_index == 1)
        marker_pub = nh.advertise<sensor_msgs::PointCloud>("/sweeper/sensor/radar_right", 1);

    pub_wj_diagnose_=nh.advertise<sweeper_msgs::SensorFaultInformation>("/sweeper/common/diagnose",1);

    radar_index_ = radar_index;

    ros::Time scan_time = ros::Time::now(); //make a virtual data per sec
    scan.header.stamp = scan_time;

    g_u32PreFrameNo = 0;

    scan.header.frame_id = "wj_718_lidar_frame";
    scan.angle_min = -2.35619449;
    scan.angle_max = 2.35619449;
    scan.angle_increment = 0.00582;
    scan.time_increment = 1 / 15.00000000 / 811;
    scan.range_min = 0;
    scan.range_max = 10;
    scan.ranges.resize(811);
    scan.intensities.resize(811);

    std::string radar_left_to_straight_extrinsic_path = "../sweeper_ws/src/sweeper_haide/calibration/data/left_wj_to_straight.yaml";
    std::string radar_right_to_straight_extrinsic_path = "../sweeper_ws/src/sweeper_haide/calibration/data/right_wj_to_straight.yaml";
    std::string radar_right_to_left_path = "../sweeper_ws/src/sweeper_haide/calibration/data/radar_right_to_left.yaml";
    std::string radar_to_lidar_extrinsic_path = "../sweeper_ws/src/sweeper_haide/calibration/data/radar_to_lidar.yaml";

    if (!sweeper::common::LoadExtrinsic(radar_left_to_straight_extrinsic_path,
                                        &radar_left_to_straight_extrinsic_))
    {
        std::cout << "Failed to load left straight extrinsic." << std::endl;
        flag_load_extrinisic_ = false;
    }
    else
    {
        flag_load_extrinisic_ = true;
    }

    if (!sweeper::common::LoadExtrinsic(radar_right_to_straight_extrinsic_path,
                                        &radar_right_to_straight_extrinsic_))
    {
        std::cout << "Failed to load right straight extrinsic." << std::endl;
    }
    else
    {
        flag_load_extrinisic_ = true;
    }

    if (!sweeper::common::LoadExtrinsic(radar_right_to_left_path,
                                        &radar_right_to_left_extrinsic_))
    {
        std::cout << "Failed to load right to left extrinsic." << std::endl;
    }
    else
    {
        flag_load_extrinisic_ = true;
    }

    if (!sweeper::common::LoadExtrinsic(radar_to_lidar_extrinsic_path,
                                        &radar_to_lidar_extrinsic_))
    {
        std::cout << "Failed to load radar to lidar extrinsic." << std::endl;
    }
    else
    {
        flag_load_extrinisic_ = true;
    }

    std::cout << "wj_718_lidar_protocl start success" << endl;
}

bool wj_718_lidar_protocol::dataProcess(unsigned char *data, const int reclen)
{
    if (reclen > MAX_LENGTH_DATA_PROCESS)
    {
        return false;
    }

    if (m_sdata.m_u32in + reclen > MAX_LENGTH_DATA_PROCESS)
    {
        memset(&m_sdata, 0, sizeof(m_sdata));
        return false;
    }
    memcpy(&m_sdata.m_acdata[m_sdata.m_u32in], data, reclen * sizeof(char));
    m_sdata.m_u32in += reclen;
    while (m_sdata.m_u32out < m_sdata.m_u32in)
    {
        if (m_sdata.m_acdata[m_sdata.m_u32out] == 0xFF && m_sdata.m_acdata[m_sdata.m_u32out + 1] == 0xAA)
        {
            unsigned l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 2] << 8) |
                                    (m_sdata.m_acdata[m_sdata.m_u32out + 3] << 0);
            l_u32reallen = l_u32reallen + 4;

            if (l_u32reallen <= (m_sdata.m_u32in - m_sdata.m_u32out + 1))
            {
                if (OnRecvProcess(&m_sdata.m_acdata[m_sdata.m_u32out], l_u32reallen))
                {
                    m_sdata.m_u32out += l_u32reallen;
                }
                else
                {
                    cout << "continuous frame" << endl;
                    int i;
                    for (i = 1; i <= l_u32reallen; i++)
                    {
                        if ((m_sdata.m_acdata[m_sdata.m_u32out + i] == 0xFF) &&
                            (m_sdata.m_acdata[m_sdata.m_u32out + i + 1] == 0xAA))
                        {
                            m_sdata.m_u32out += i;
                            break;
                        }
                        if (i == l_u32reallen)
                        {
                            m_sdata.m_u32out += l_u32reallen;
                        }
                    }
                }
            }
            else if (l_u32reallen >= MAX_LENGTH_DATA_PROCESS)
            {
                cout << "l_u32reallen >= MAX_LENGTH_DATA_PROCESS" << endl;
                cout << "reallen: " << l_u32reallen << endl;
                memset(&m_sdata, 0, sizeof(m_sdata));
            }
            else
            {
                //cout<<"reallen: "<<l_u32reallen<<" indata: "<<m_sdata.m_u32in<<" outdata: "<<m_sdata.m_u32out<<endl;
                break;
            }
        }
        else
        {
            m_sdata.m_u32out++;
        }
    } //end while(m_sdata.m_u32out < m_sdata.m_u32in)

    if (m_sdata.m_u32out >= m_sdata.m_u32in)
    {
        memset(&m_sdata, 0, sizeof(m_sdata));
    }
    return true;
}

bool wj_718_lidar_protocol::OnRecvProcess(unsigned char *data, int len)
{
    if (len > 0)
    {
        if (checkXor(data, len))
        {
            protocl(data, len);
        }
    }
    else
    {
        return false;
    }
    return true;
}

bool wj_718_lidar_protocol::protocl(unsigned char *data, const int len)
{
    if ((data[22] == 0x02 && data[23] == 0x02) || (data[22] == 0x02 && data[23] == 0x01)) //command type:0x02 0x01/0X02
    {
        static int s_n32ScanIndex;
        int l_n32TotalPackage = data[35];                                                             //total package
        int l_n32PackageNo = data[36];                                                                //package number
        unsigned int l_u32FrameNo = (data[31] << 24) + (data[32] << 16) + (data[33] << 8) + data[34]; //frame number

        if ((freq_scan == 0 && l_n32TotalPackage != 2) || (freq_scan == 1 && l_n32TotalPackage != 1))
        {
            heartstate = true;
            cout << "The scan frequency does not match the resolution!" << endl;
            return false;
        }

        if (freq_scan == 0) //15hz/0.33
        {
            if (l_n32PackageNo == 1)
            {
                s_n32ScanIndex = 0;
                g_u32PreFrameNo = l_u32FrameNo;
                heartstate = true;
                int l_n32PointCount = (data[37] << 8) + data[38];
                for (int j = 0; j < l_n32PointCount; j++)
                {
                    scandata[s_n32ScanIndex] = ((data[39 + j * 2]) << 8) + (data[40 + j * 2]);
                    scandata[s_n32ScanIndex] /= 1000.0;
                    if (scandata[s_n32ScanIndex] >= 10 || scandata[s_n32ScanIndex] == 0)
                    {
                        scandata[s_n32ScanIndex] = NAN;
                    }
                    s_n32ScanIndex++;
                }
            }
            else if (l_n32PackageNo == 2)
            {
                if (g_u32PreFrameNo == l_u32FrameNo)
                {
                    int l_n32PointCount = (data[37] << 8) + data[38];
                    for (int j = 0; j < l_n32PointCount; j++)
                    {
                        scandata[s_n32ScanIndex] = ((data[39 + j * 2]) << 8) + (data[40 + j * 2]);
                        scandata[s_n32ScanIndex] /= 1000.0;
                        if (scandata[s_n32ScanIndex] >= 10 || scandata[s_n32ScanIndex] == 0)
                        {
                            scandata[s_n32ScanIndex] = NAN;
                        }
                        s_n32ScanIndex++;
                    }

                    for (int j = 0; j < resizeNum; j++)
                    {
                        scan.ranges[resizeNum - 1 - j] = scandata[j];
                    }

                    laser_geometry::LaserProjection projector;
                    sensor_msgs::PointCloud cloud;
                    projector.projectLaser(scan, cloud);

                    sensor_msgs::PointCloud cloud_correct;

                    if (radar_index_ == 0)
                    {
                        Eigen::Matrix4d transform_radar_left_to_straight = radar_left_to_straight_extrinsic_.matrix();
                        Eigen::Matrix4d transform_radar_to_lidar = radar_to_lidar_extrinsic_.matrix();

                        int num_cloud = cloud.points.size();
                        for (unsigned int j = 0; j < num_cloud; ++j)
                        {
                            geometry_msgs::Point32 cloud_correct_points;
                            Eigen::Matrix<double, 4, 1> orign_cloud_point;
                            Eigen::Matrix<double, 4, 1> calib_cloud_point;
                            double x = cloud.points[j].x;
                            double y = cloud.points[j].y;
                            if (sqrt(pow(x, 2) + pow(y, 2) > 0.02))
                            {
                                orign_cloud_point << x, y, 0, 1;
                                calib_cloud_point = transform_radar_left_to_straight * orign_cloud_point;
                                
                                calib_cloud_point = transform_radar_to_lidar * calib_cloud_point;

                                cloud_correct_points.x = calib_cloud_point(0);
                                cloud_correct_points.y = calib_cloud_point(1);
                                cloud_correct_points.z = -1.6;
                                cloud_correct.points.push_back(cloud_correct_points);
                            }
                        }
                    }
                    else if (radar_index_ == 1)
                    {
                        Eigen::Matrix4d transform_radar_right_to_straight = radar_right_to_straight_extrinsic_.matrix();
                        Eigen::Matrix4d transform_radar_right_to_left = radar_right_to_left_extrinsic_.matrix();
                        Eigen::Matrix4d transform_radar_to_lidar = radar_to_lidar_extrinsic_.matrix();
 
                        int num_cloud = cloud.points.size();
                        for (unsigned int j = 0; j < num_cloud; ++j)
                        {
                            geometry_msgs::Point32 cloud_correct_points;
                            Eigen::Matrix<double, 4, 1> orign_cloud_point;
                            Eigen::Matrix<double, 4, 1> calib_cloud_point;
                            double x = cloud.points[j].x;
                            double y = cloud.points[j].y;
                            if (sqrt(pow(x, 2) + pow(y, 2)) > 0.02)
                            {
                                orign_cloud_point << x, y, 0, 1;
                                calib_cloud_point = transform_radar_right_to_straight * orign_cloud_point;
                               
                                calib_cloud_point = transform_radar_right_to_left * calib_cloud_point;
                                calib_cloud_point = transform_radar_to_lidar * calib_cloud_point;

                                cloud_correct_points.x = calib_cloud_point(0);
                                cloud_correct_points.y = calib_cloud_point(1);

                                cloud_correct_points.z = -1.6;
                                cloud_correct.points.push_back(cloud_correct_points);
                            }
                        }
                    }

                    ros::Time scan_time = ros::Time::now();
                    cloud_correct.header.stamp = scan_time;
                    cloud_correct.header.frame_id = "livox_frame";
                    marker_pub.publish(cloud_correct);
                }
                else
                {
                    s_n32ScanIndex = 0;
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        else //25hz/1
        {
            if (l_n32PackageNo == 1)
            {
                s_n32ScanIndex = 0;
                g_u32PreFrameNo = l_u32FrameNo;
                heartstate = true;
                int l_n32PointCount = (data[37] << 8) + data[38];

                for (int j = 0; j < l_n32PointCount; j++)
                {
                    scandata[s_n32ScanIndex] = ((data[39 + j * 2]) << 8) + (data[40 + j * 2]);
                    scandata[s_n32ScanIndex] /= 1000.0;
                    if (scandata[s_n32ScanIndex] >= 10 || scandata[s_n32ScanIndex] == 0)
                    {
                        scandata[s_n32ScanIndex] = NAN;
                    }
                    s_n32ScanIndex++;
                }

                for (int j = 0; j < resizeNum; j++)
                {
                    scan.ranges[resizeNum - 1 - j] = scandata[j];
                }

                ros::Time scan_time = ros::Time::now();
                scan.header.stamp = scan_time;
                marker_pub.publish(scan);
            }
            else
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool wj_718_lidar_protocol::checkXor(unsigned char *recvbuf, int recvlen)
{
    int i = 0;
    unsigned char check = 0;
    unsigned char *p = recvbuf;
    int len;
    if (*p == 0xFF)
    {
        p = p + 2;
        len = recvlen - 6;
        for (i = 0; i < len; i++)
        {
            check ^= *p++;
        }
        p++;
        if (check == *p)
        {
            return true;
        }
        else
            return false;
    }
    else
    {
        return false;
    }
}

std::vector<int> wj_718_lidar_protocol::SelfDiagnose(int index)
{
  sweeper_msgs::SensorFaultInformation state_radar;

  if (!watch_dog_radar_.DogIsOk(5))
  {
    if (index == 0)
    {
      state_radar.state_code.push_back(2104);
    }
    else if (index == 1)
    {
      state_radar.state_code.push_back(2105);
    }
  }

  if (!flag_load_extrinisic_)
  {
    if (index == 0)
    {
      state_radar.state_code.push_back(2106);
    }
    else if (index == 1)
    {
      state_radar.state_code.push_back(2107);
    }
  }
  
  
  if (index == 0)
  {
    if (state_radar.state_code.empty())
    {
      state_radar.state_code.push_back(2100);
    //   std::cout<<"line:"<<__LINE__<<std::endl;
    }
  }
  else if (index == 1)
  {
    if (state_radar.state_code.empty())
    {
      state_radar.state_code.push_back(2101);
    //   std::cout<<"line:"<<__LINE__<<std::endl;
    }
  }
  return state_radar.state_code;
}

} // namespace wj_lidar

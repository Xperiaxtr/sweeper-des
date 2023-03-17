#ifndef WJ_718_LIDAR_PROTOCOL_H
#define WJ_718_LIDAR_PROTOCOL_H
#include <iostream>
#include "string.h"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <boost/bind/bind.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <dynamic_reconfigure/server.h>
#include <wj_718_lidar/wj_718_lidarConfig.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sweeper_msgs/SensorFaultInformation.h>
#include "../../../../common/watch_dog.h"
#include "../../../../common/pose_util.h"
#include "../../../../common/log.h"

using namespace std;
namespace wj_lidar
{
#define MAX_LENGTH_DATA_PROCESS 200000
typedef struct TagDataCache
{
    unsigned char m_acdata[MAX_LENGTH_DATA_PROCESS];
    unsigned int m_u32in;
    unsigned int m_u32out;
} DataCache;

class wj_718_lidar_protocol
{
public:
    wj_718_lidar_protocol(int radar_index);
    bool dataProcess(unsigned char *data, const int reclen);
    bool protocl(unsigned char *data, const int len);
    bool OnRecvProcess(unsigned char *data, int len);
    bool checkXor(unsigned char *recvbuf, int recvlen);
    void send_scan(const char *data, const int len);
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Publisher pub_wj_diagnose_;
    sensor_msgs::LaserScan scan;
    bool setConfig(wj_718_lidar::wj_718_lidarConfig &new_config, uint32_t level);
    std::vector<int> SelfDiagnose(int index);
    bool heartstate;
    bool flag_load_extrinisic_;
    sweeper::common::WatchDog watch_dog_radar_;

private:
    unsigned char data_[MAX_LENGTH_DATA_PROCESS];
    DataCache m_sdata;
    wj_718_lidar::wj_718_lidarConfig config_;
    unsigned int g_u32PreFrameNo;
    float scandata[811];
    float scandata_te[811];
    float scaninden[811];
    int freq_scan;
    int resizeNum;
    int radar_index_;
    Eigen::Affine3d radar_left_to_straight_extrinsic_, radar_right_to_straight_extrinsic_,
        radar_right_to_left_extrinsic_, radar_to_lidar_extrinsic_;
};

} // namespace wj_lidar
#endif // WJ_718_LIDAR_PROTOCOL_H

#include <ros/ros.h>
#include <thread>
#include "async_client.h"
#include "wj_718_lidar_protocol.h"
using namespace wj_lidar;

/* ------------------------------------------------------------------------------------------
 *  show demo --
 * ------------------------------------------------------------------------------------------ */
wj_718_lidar_protocol *protocol;
Async_Client *client;

void WjDiagnose(int radar_index)
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        sweeper_msgs::SensorFaultInformation state_radar;

        std::vector<int> state_radar_left, state_radar_right;
      
        if (radar_index == 0)
        {
            state_radar_left = protocol->SelfDiagnose(0);
            state_radar.state_code = state_radar_left;
            state_radar.header.frame_id = "wjlidar";
            state_radar.header.stamp = ros::Time::now();
            protocol->pub_wj_diagnose_.publish(state_radar);
        }
        else if (radar_index == 1)
        {
            state_radar_right = protocol->SelfDiagnose(1);
            state_radar.state_code = state_radar_right;
            state_radar.header.frame_id = "wjlidar";
            state_radar.header.stamp = ros::Time::now();
            protocol->pub_wj_diagnose_.publish(state_radar);
        }
        loop_rate.sleep();
    }
}

void CallBackRead(const char *addr, int port, unsigned char *data, const int len)
{
    protocol->dataProcess(data, len);
    protocol->watch_dog_radar_.UpdataNow();
}

void callback(wj_718_lidar::wj_718_lidarConfig &config, uint32_t level)
{
    protocol->setConfig(config, level);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wj_718_lidar_01");
    ros::NodeHandle nh("~");
    std::string hostname;
    nh.getParam("hostname", hostname);
    std::string port;
    nh.getParam("port", port);
    cout << "laser ip: " << hostname << ", port:" << port << endl;

    int radar_index = -1;
    if (hostname == "192.168.2.100")
    {
        //左单线激光雷达
        radar_index = 0;
    }
    else if (hostname == "192.168.3.100")
    {
        //右单线激光雷达
        radar_index = 1;
    }

    protocol = new wj_718_lidar_protocol(radar_index);
    dynamic_reconfigure::Server<wj_718_lidar::wj_718_lidarConfig> server;
    dynamic_reconfigure::Server<wj_718_lidar::wj_718_lidarConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    client = new Async_Client(&CallBackRead);
    protocol->heartstate = false;

    std::thread wj_diagnose_thread(WjDiagnose, radar_index);
    wj_diagnose_thread.detach();

    while (!client->m_bConnected)
    {
        ROS_INFO("Start connecting laser!");
        if (client->connect(hostname.c_str(), atoi(port.c_str())))
        {
            ROS_INFO("Succesfully connected. Hello wj_718_lidar!");
        }
        else
        {
            ROS_INFO("Failed to connect to laser. Waiting 5s to reconnect!");
        }
        ros::Duration(5).sleep();
    }
    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(2).sleep();
        if (client->m_bConnected)
        {
            if (protocol->heartstate)
            {
                protocol->heartstate = false;
            }
            else
            {
                client->m_bConnected = false;
            }
        }
        else
        {
            //reconnect
            if (!client->m_bReconnecting)
            {
                boost::thread t(boost::bind(&Async_Client::reconnect, client));
            }
        }
    }
}

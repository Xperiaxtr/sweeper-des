#include <ros/ros.h>
#include <stdlib.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/string.h>
#include <thread>
#include "sweeper_msgs/PerceptionObstacle.h"
#include "sweeper_msgs/Object.h"
#include "sweeper_msgs/SensorFaultInformation.h"
#include <can_msgs/Frame.h>
#include <string>
#include <std_msgs/Header.h>

ros::Publisher pub_mmw, pub_diagnose;

sweeper_msgs::PerceptionObstacle mmw_datas;
sweeper_msgs::SensorFaultInformation fault_mmw_;

int num_mmw_datas_;
unsigned int mmw_datas_counter_;
unsigned int error_counter_ = 0;
unsigned int heart_beat_counter_ = 0;
unsigned int slef_diagnose_counter_ = 0;

void SelfDiagnose()
{
    ros::Rate loop_rate(0.8);
    while (ros::ok())
    {
        if (slef_diagnose_counter_ != heart_beat_counter_)
        {
            error_counter_ = error_counter_ + 1;
        }
        if (error_counter_ > 2)
        {
            fault_mmw_.fault.push_back(false);
            fault_mmw_.type = 1;
            fault_mmw_.header.frame_id = "diagnose";
            fault_mmw_.header.stamp = ros::Time::now();
            pub_diagnose.publish(fault_mmw_);
            fault_mmw_.fault.clear();
        }
        else
        {
            fault_mmw_.fault.push_back(true);
            fault_mmw_.type = 0;
            fault_mmw_.header.frame_id = "diagnose";
            fault_mmw_.header.stamp = ros::Time::now();
            pub_diagnose.publish(fault_mmw_);
            fault_mmw_.fault.clear();
        }
        slef_diagnose_counter_ = slef_diagnose_counter_ + 1;
        loop_rate.sleep();
    }
}

void InterpretMmwMessages(uint8_t mmw_messages[])
{
    sweeper_msgs::Object mmw_object;
    mmw_object.id = mmw_messages[0];

    unsigned int object_radial_distance_high = (mmw_messages[1]) * 32;
    unsigned int object_radial_distance_low = mmw_messages[2] >> 3;

    unsigned int object_lateral_distance_high = (mmw_messages[2] & 0x07) * 256;
    unsigned int object_lateral_distance_low = mmw_messages[3];

    mmw_object.position.x = (object_radial_distance_high + object_radial_distance_low) * 0.2 - 500.0;
    mmw_object.position.y = (object_lateral_distance_high + object_lateral_distance_low) * 0.2 - 204.6;

    std::cout << "radial distance:" << mmw_object.position.x << std::endl;
    std::cout << "lateral distance:" << mmw_object.position.y << std::endl;

    mmw_datas.object.push_back(mmw_object);

    mmw_datas_counter_ = mmw_datas_counter_ + 1;

    std::cout << "ID " << mmw_object.id << ": " << mmw_object.position.x << std::endl;
}

int main(int argv, char **argc)
{
    ros::init(argv, argc, "mmw");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    mmw_datas_counter_ = 0;
    num_mmw_datas_ = -1;

    int s, num_datas;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct can_filter rfilter[3];

    //construct the can of socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strcpy(ifr.ifr_name, "can0");

    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));

    //Set ID of Accepted Messages
    rfilter[0].can_id = 0x60A;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = 0x60B;
    rfilter[1].can_mask = CAN_SFF_MASK;
    rfilter[2].can_id = 0x700;
    rfilter[2].can_mask = CAN_SFF_MASK;

    //Set up the rules of filtering
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    pub_mmw = nh.advertise<sweeper_msgs::PerceptionObstacle>("/sweeper/sensor/mmw", 1);
    pub_diagnose = nh.advertise<sweeper_msgs::SensorFaultInformation>("/sweeper/sensor/mmw/diagnose", 1);

    std::thread mmw_seldiagnose_thread(SelfDiagnose);
    mmw_seldiagnose_thread.detach();

    while (ros::ok())
    {
        num_datas = read(s, &frame, sizeof(frame));
        if (num_datas > 0)
        {
            std_msgs::Header mmw_header;
            struct can_frame mmw_msg = frame;
            uint8_t mmw_received_buffer[8] = {0};
            for (uint8_t i = 0; i < mmw_msg.can_dlc; i++)
            {
                mmw_received_buffer[i] = mmw_msg.data[i];
            }

            if (mmw_msg.can_id == 0x700)
            {
                heart_beat_counter_ = slef_diagnose_counter_;
                error_counter_ = 0;
            }
            else if (mmw_msg.can_id == 0x60A)
            {
                num_mmw_datas_ = mmw_received_buffer[0];
            }
            else if (mmw_msg.can_id == 0x60B)
            {
                if (num_mmw_datas_ > 0)
                {
                    if (mmw_datas_counter_ >= (num_mmw_datas_ - 1))
                    {
                        InterpretMmwMessages(mmw_received_buffer);
                        mmw_header.frame_id = "millimeter_wave";
                        mmw_header.stamp = ros::Time::now();
                        mmw_datas.header = mmw_header;
                        pub_mmw.publish(mmw_datas);
                        mmw_datas.object.clear();
                        mmw_datas_counter_ = 0;
                    }
                    else
                    {
                        InterpretMmwMessages(mmw_received_buffer);
                    }
                }
            }
        }
    }

    ros::spin();
    close(s);
    usleep(50000);
    return 0;
}

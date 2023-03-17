#include <stdio.h>
#include <ctime>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <string>
#include <thread>
#include "byte.h"
#include "type.h"
#include <ros/ros.h>
#include "sweeper_msgs/PerceptionObstacle.h"
#include "sweeper_msgs/Object.h"
#include "sweeper_msgs/SensorFaultInformation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <can_msgs/Frame.h>
#include "../../../common/watch_dog.h"

#define ULTRA_NUMS_ 10

using sweeper::bus::Byte;

ros::Publisher pub_ultra, pub_fault_information_;
std::list<struct can_frame> ultra_messages_;
sweeper::common::WatchDog watch_dog_ultra_;

int error_counter_[ULTRA_NUMS_] = {0};
bool flag_ultra_sync_ = true;

struct can_frame frame_;
int s_;

void UltraDiagnose(sweeper_msgs::PerceptionObstacle datas)
{
    sweeper_msgs::SensorFaultInformation ultra_self_diagnose;

    if (datas.object.empty())
    {
        for (unsigned int i = 0; i < ULTRA_NUMS_; ++i)
        {
            error_counter_[i] = error_counter_[i] + 1;
        }
    }
    else
    {
        int pub_ultra_num = datas.object.size();
        std::vector<int> normal_ultra_index(ULTRA_NUMS_);
        for (unsigned int i = 0; i < pub_ultra_num; ++i)
        {
            int ultra_index = datas.object[i].id - 448;
            error_counter_[ultra_index] = 0;
            normal_ultra_index.push_back(ultra_index);
        }

        for (unsigned int i = 0; i < ULTRA_NUMS_; ++i)
        {
            bool flag_ultra_error = false;
            for (unsigned int j = 0; j < normal_ultra_index.size(); ++j)
            {
                if (i == normal_ultra_index[j])
                {
                    flag_ultra_error = true;
                }
                else
                {
                    error_counter_[i] = 0;
                }
            }
            if (!flag_ultra_error)
            {
                if (error_counter_[i] > 8)
                {
                    error_counter_[i] = 9;
                }
                else
                {
                    error_counter_[i] = error_counter_[i] + 1;
                }
            }
        }
    }
}

void SelfDiagnose()
{
    sweeper_msgs::SensorFaultInformation state_ultra;
    if (!flag_ultra_sync_)
    {
        state_ultra.state_code.push_back(2501);
    }

    for (unsigned int idx = 0; idx < ULTRA_NUMS_; ++idx)
    {
        if (error_counter_[idx] > 8)
        {
            state_ultra.state_code.push_back(2502);
            break;
        }
    }

    if (state_ultra.state_code.empty())
    {
        state_ultra.state_code.push_back(2500);
        state_ultra.header.frame_id = "ultrasonic";
        state_ultra.header.stamp = ros::Time::now();
    }
    else
    {
        state_ultra.header.frame_id = "ultrasonic";
        state_ultra.header.stamp = ros::Time::now();
    }
    pub_fault_information_.publish(state_ultra);
}

void ReadUltraCanMsg()
{
    int num_datas;
    while (ros::ok())
    {
        num_datas = read(s_, &frame_, sizeof(frame_));
        if (num_datas > 0)
        {
            ultra_messages_.push_back(frame_);
        }
    }
}

void InterpretUltraData(uint8_t *ultra_buffer, int ultra_id, sweeper_msgs::PerceptionObstacle &ultra_datas)
{
    bool flag_repeated = false;
    sweeper_msgs::Object ultrasonic_object;
    std_msgs::Header ultrasonic_header;

    ultrasonic_object.id = ultra_id;
    std::cout << "id:" << ultra_id << std::endl;

    Byte frame_high(ultra_buffer + 1);
    int32_t high = frame_high.get_byte(0, 8);
    Byte frame_low(ultra_buffer + 0);
    int32_t low = frame_low.get_byte(0, 8);
    int32_t value = (high << 8) | low;

    if (ultra_id == 448 || ultra_id == 449 || ultra_id == 450 || ultra_id == 451 || ultra_id == 452 ||
        ultra_id == 453 || ultra_id == 454 || ultra_id == 455 || ultra_id == 456 || ultra_id == 457)
    {
        if (!ultra_datas.object.empty())
        {
            for (unsigned int j = 0; j < ultra_datas.object.size(); ++j)
            {
                if (ultra_id == ultra_datas.object[j].id)
                {
                    ROS_INFO("Have find repeated ultrasonic data !");
                    flag_repeated = true;
                    ultra_datas.object[j].position.x = value / 1000.0;
                    break;
                }
            }
            if (flag_repeated == false)
            {
                ultrasonic_object.position.x = value / 1000.0;
                ultrasonic_object.id = ultra_id;
                ultra_datas.object.push_back(ultrasonic_object);
            }
        }
        else
        {
            ultrasonic_object.position.x = value / 1000.0;
            ultrasonic_object.id = ultra_id;
            ultra_datas.object.push_back(ultrasonic_object);
        }
    }
    ultrasonic_header.frame_id = "ultrasonic";
    ultrasonic_header.stamp = ros::Time::now();
    ultra_datas.header = ultrasonic_header;
}

void DealUltraData()
{
    ROS_INFO_STREAM("The thread of interpreting ultrasonic messages had createad!");
    ros::Rate loop_rate(8);
    while (ros::ok())
    {
        sweeper_msgs::PerceptionObstacle ultrasonic_datas;
        while (!ultra_messages_.empty())
        {
            struct can_frame ultra_msg = ultra_messages_.front();
            uint8_t ultra_received_buffer[8] = {0};
            int ultra_id = ultra_msg.can_id;
            if (ultra_id != 320)
            {
                flag_ultra_sync_ = true;

                for (uint8_t i = 0; i < ultra_msg.can_dlc; ++i)
                {
                    ultra_received_buffer[i] = ultra_msg.data[i];
                }

                ultra_messages_.pop_front();

                std::cout << "ultra id: " << ultra_msg.can_id << std::endl;

                InterpretUltraData(ultra_received_buffer, ultra_id, ultrasonic_datas);
            }
            else
            {
                flag_ultra_sync_ = false;
            }
        }

        UltraDiagnose(ultrasonic_datas);

        if (!ultrasonic_datas.object.empty())
        {
            pub_ultra.publish(ultrasonic_datas);
            SelfDiagnose();
        }
        else
        {
            SelfDiagnose();
        }

        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultrasonic");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    pub_ultra = nh.advertise<sweeper_msgs::PerceptionObstacle>("/sweeper/sensor/ultrasonic", 1);
    pub_fault_information_ = nh.advertise<sweeper_msgs::SensorFaultInformation>("/sweeper/common/diagnose", 1);
    
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_filter rfilter[11];

    //construct the can of socket
    s_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strcpy(ifr.ifr_name, "can3");

    ioctl(s_, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s_, (struct sockaddr *)&addr, sizeof(addr));

    //Set ID of Accepted Messages
    rfilter[0].can_id = 0x1c0;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = 0x1c1;
    rfilter[1].can_mask = CAN_SFF_MASK;
    rfilter[2].can_id = 0x1c2;
    rfilter[2].can_mask = CAN_SFF_MASK;
    rfilter[3].can_id = 0x1c3;
    rfilter[3].can_mask = CAN_SFF_MASK;
    rfilter[4].can_id = 0x1c4;
    rfilter[4].can_mask = CAN_SFF_MASK;
    rfilter[5].can_id = 0x1c5;
    rfilter[5].can_mask = CAN_SFF_MASK;
    rfilter[6].can_id = 0x1c6;
    rfilter[6].can_mask = CAN_SFF_MASK;
    rfilter[7].can_id = 0x1c7;
    rfilter[7].can_mask = CAN_SFF_MASK;
    rfilter[8].can_id = 0x1c8;
    rfilter[8].can_mask = CAN_SFF_MASK;
    rfilter[9].can_id = 0x1c9;
    rfilter[9].can_mask = CAN_SFF_MASK;
    rfilter[10].can_id = 0x240;
    rfilter[10].can_mask = CAN_SFF_MASK;

    //Set up the rules of filtering
    setsockopt(s_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    std::thread ultra_data_thread(DealUltraData);
    ultra_data_thread.detach();

    std::thread can_read_thread(ReadUltraCanMsg);
    can_read_thread.detach();

    ros::spin();

    close(s_);
    usleep(50000);

    return 0;
}

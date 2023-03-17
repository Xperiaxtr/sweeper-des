#include <iostream>
#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include "sweeper_msgs/PerceptionObstacle.h"
#include "sweeper_msgs/Object.h"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweeperUltraCmd.h"
#include "sweeper_msgs/Object.h"
#include "../../../common/log.h"
#include "../../../common/watch_dog.h"

ros::Publisher pub_ultra_, pub_diagnose_;
int ultra_nums_, start_nums_;
int interval_time_;
std::vector<int> ultra_cmds_;
std::vector<int> ultra_totall_cmds_{6,7,8,9};
std::string device_port_;
int baudrate_;
bool flag_test_;
bool flag_missing_ = false;
sweeper::common::WatchDog ultra_watchdog_;

void SelfDiagnose(){
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        sweeper_msgs::SensorFaultInformation state_ultra;
        if (!ultra_watchdog_.DogIsOk())
        {
            state_ultra.state_code.push_back(2501);
        }

        if (flag_missing_)
        {
            state_ultra.state_code.push_back(2502);
        }

        if (state_ultra.state_code.empty())
        {
            state_ultra.state_code.push_back(2500);
        }

        state_ultra.header.frame_id = "ultrasonic";
        state_ultra.header.stamp = ros::Time::now();
        pub_diagnose_.publish(state_ultra);

        loop_rate.sleep();
    }
}

void ReceiveUltraCmd(const sweeper_msgs::SweeperUltraCmd &ultra_cmds){
    ultra_cmds_ = ultra_cmds.cmd;
}

void InterpretCmd(const int origin_cmd, uint8_t *cmd){
    switch (origin_cmd)
    {
    case 1:
        *cmd = 0x14;
        break;
    case 2:
        *cmd = 0x1c;
        break;
    case 3:
        *cmd = 0x24;
        break;
    case 4:
        *cmd = 0x2c;
        break;
    case 5:
        *cmd = 0x34;
        break;
    case 6:
        *cmd = 0x3c;
        break;
    case 7:
        *cmd = 0x44;
        break;
    case 8:
        *cmd = 0x4c;
        break;
    case 9:
        *cmd = 0x54;
        break;
    case 10:
        *cmd = 0x5c;
        break;
    case 11:
        cmd[0] = 0x64;
        break;
    case 12:
        cmd[0] = 0x6c;
        break;
    }
}

void ReadUltraMsgs()
{
    int counter = 0;
    serial::Serial sp;
    while (counter < start_nums_ && !sp.isOpen())
    {
        ++counter;
        try
        {
            sp.setPort(device_port_);
            sp.setBaudrate(baudrate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(200);
            sp.setTimeout(to);
            sp.open();
        }
        catch (serial::IOException &e)
        {
            AERROR<<"Unable to open port";
            sp.close();
            sleep(2);
        }
    }

    if (sp.isOpen())
    {
        ros::Rate loop_rate(10);
        uint8_t port_address[1] = {0xe8};
        uint8_t register_address[1] = {0x02};
        uint8_t send_buffer[1];

        while (ros::ok())
        {
            std::vector<int> diff_cmds_;
            std::vector<int> cmds = ultra_cmds_;

            if (flag_test_)
            {
                cmds.clear();
                cmds.assign(ultra_totall_cmds_.begin(), ultra_totall_cmds_.end());
            }

            sweeper_msgs::PerceptionObstacle ultra_msgs;

            if (cmds.size() > 0)
            {
                for (unsigned int i = 0; i < cmds.size(); ++i)
                {

                    uint8_t receive_buffer[2];
                    uint8_t cmd[1];
                    InterpretCmd(cmds[i], cmd);
                    sp.write(port_address, 1);
                    usleep(100);
                    sp.write(register_address, 1);
                    usleep(100);
                    sp.write(cmd, 1);

                    int len = sp.available();
                    if (len > 0)
                    {
                        sweeper_msgs::Object ultra_object;
                        int real_len = sp.read(receive_buffer, len);
                        int high_8_bit_low = (receive_buffer[0] & 0x0f);
                        int high_8_bit_high = ((receive_buffer[0] & 0xf0) >> 4);
                        int low_8_bit_low = (receive_buffer[1] & 0x0f);
                        int low_8_bit_high = ((receive_buffer[1] & 0xf0) >> 4);

                        double ultra_distance = (low_8_bit_low + low_8_bit_high * 16 +
                                                 high_8_bit_low * pow(16, 2) + high_8_bit_high * pow(16, 3)) /
                                                1000.0;
                        // if(cmds[i]==7 || cmds[i]==9)
                         std::cout<<"id:"<<cmds[i] <<" "<<"ultra_distance:"<<ultra_distance<<std::endl;
                        ultra_object.id = cmds[i];
                        ultra_object.position.x = ultra_distance;
                        ultra_msgs.object.push_back(ultra_object);
                    }
                    usleep(interval_time_);
                }

                int real_ = ultra_msgs.object.size();
                if (real_ != cmds.size())
                {
                    flag_missing_ = true;
                }
                else
                {
                    flag_missing_ = false;
                }
            }
            
            if(cmds.size()<=0){
                for(unsigned int i=0;i<ultra_totall_cmds_.size();++i){
                    sweeper_msgs::Object ultra_object;
                    ultra_object.id=ultra_totall_cmds_[i];
                    ultra_object.position.x = 3.04;
                    ultra_msgs.object.push_back(ultra_object);
                }
            }else if(cmds.size()>0){
                for(unsigned int j=0;j<ultra_totall_cmds_.size();++j)
                {
                    std::vector<int>::iterator iter = std::find(cmds.begin(), cmds.end(), ultra_totall_cmds_[j]);
                    if(iter==cmds.end())
                        diff_cmds_.push_back(ultra_totall_cmds_[j]);
                }

                for(unsigned int i=0;i<diff_cmds_.size();++i){
                    sweeper_msgs::Object ultra_object;
                    ultra_object.id=diff_cmds_[i];
                    ultra_object.position.x = 3.04;
                    ultra_msgs.object.push_back(ultra_object);
                }
            }
            

            ultra_msgs.header.frame_id = "ultrasonic";
            ultra_msgs.header.stamp = ros::Time::now();
             
            pub_ultra_.publish(ultra_msgs);
            ultra_watchdog_.UpdataNow();
            ultra_cmds_.clear();

            ultra_msgs.object.clear();

            loop_rate.sleep();
        }
        sp.close();
        usleep(50000);
    }

    if (counter >= start_nums_)
    {
        AERROR<<"Please check the serial port and restart !";
    }
}

void LogSetting()
{
    FLAGS_log_dir = "../sweeper_ws/src/sweeper_haide/data/log/";
    FLAGS_stderrthreshold = google::GLOG_INFO;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_minloglevel = google::GLOG_INFO;
    FLAGS_v = 1;
}

int main(int argv, char **argc)
{
    ros::init(argv, argc, "ultrasonic_pub");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    google::InitGoogleLogging(argc[0]);
    LogSetting();

    nh_private.param("device_port", device_port_, std::string("/dev/ttyUSB0"));
    nh_private.param("baudrate", baudrate_, 9600);
    nh_private.param("interval_time", interval_time_, 10000);
    nh_private.param("ultra_nums", ultra_nums_, 10);
    nh_private.param("start_nums", start_nums_, 5);
    nh_private.param("flag_test", flag_test_, false);

    pub_ultra_ = nh_private.advertise<sweeper_msgs::PerceptionObstacle>("/sweeper/sensor/ultrasonic", 1);

    pub_diagnose_ = nh_private.advertise<sweeper_msgs::SensorFaultInformation>("/sweeper/common/diagnose", 1);
    ros::Subscriber sub_ultra_cmd = nh_private.subscribe("/sweeper/control/ultra_cmd", 1, &ReceiveUltraCmd);

    std::thread ultra_msgs_read(ReadUltraMsgs);
    ultra_msgs_read.detach();
    ROS_INFO("the thread of read ultra msgs has ctreated !");

    std::thread ultra_watchdog(SelfDiagnose);
    ultra_watchdog.detach();
    ROS_INFO("the thread of diagnose has ctreated !");

    ros::spin();
    return 0;
}
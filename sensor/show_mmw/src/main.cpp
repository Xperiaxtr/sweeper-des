#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <sweeper_msgs/PerceptionObstacle.h>
#include <opencv2/opencv.hpp>

//
double car_position_y = 500.0;
double car_position_x = 0.0;

void MmwDataShow(const sweeper_msgs::PerceptionObstacle &mmw_data)
{
    cv::Mat mmw_show_ = cv::Mat::zeros(1000, 1000, CV_8UC1);
    int num = mmw_data.object.size();
    for (unsigned int i = 0; i < num; ++i)
    {
        if (mmw_data.object[i].position.x <= 5.0 && mmw_data.object[i].position.y <= 5.0 && mmw_data.object[i].position.y >= -5.0)
        {
            double temp_position_x = mmw_data.object[i].position.x / 0.01;
            double temp_position_y = mmw_data.object[i].position.y / 0.01;
            temp_position_x = temp_position_x + car_position_x;
            temp_position_y = temp_position_y + car_position_y;
            mmw_show_.at<uint8_t>(temp_position_x, temp_position_y) = 255;
            cv::circle(mmw_show_,cv::Point(temp_position_x,temp_position_y),5,cv::Scalar(255, 0, 0),-1,8);
        }
    }
    imshow("show_mmw", mmw_show_);
    cv::waitKey(1);
}

int main(int argv, char **argc)
{
    ros::init(argv, argc, "show_mmw");
    ros::NodeHandle nh;
    ros::Subscriber sub_mmw_ = nh.subscribe("/sweeper/sensor/mmw/datas", 1, &MmwDataShow);
    ros::spin();
    return 0;
}

#pragma once
#include <iostream>
#include <cmath>
#include "sweeper_msgs/PerceptionObstacle.h"

namespace sweeper
{
namespace navigation
{
namespace controll
{
class DealUltrasonic
{
public:
    DealUltrasonic();
    ~DealUltrasonic();
    void InterpretUltrasonicData(const sweeper_msgs::PerceptionObstacle &ultra_origin, int indx);

    int filter_save_nums_;

    double ultrasonic_datas_[200];
    int ultra_1_saved_datas_num_;
    int ultra_2_saved_datas_num_;
    int ultra_3_saved_datas_num_;
    int ultra_4_saved_datas_num_;
    int ultra_5_saved_datas_num_;
    int ultra_6_saved_datas_num_;
    int ultra_7_saved_datas_num_;
    int ultra_8_saved_datas_num_;
    int ultra_9_saved_datas_num_;
    int ultra_10_saved_datas_num_;

    double ultra_1_filter_data_;
    double ultra_2_filter_data_;
    double ultra_3_filter_data_;
    double ultra_4_filter_data_;
    double ultra_5_filter_data_;
    double ultra_6_filter_data_;
    double ultra_7_filter_data_;
    double ultra_8_filter_data_;
    double ultra_9_filter_data_;
    double ultra_10_filter_data_;

private:
    double UltrasonicFilter(const sweeper_msgs::PerceptionObstacle &ultra_origin, int ultra_index, int id, int &saved_nums);
};

} // namespace controll
} // namespace navigation
} // namespace sweeper
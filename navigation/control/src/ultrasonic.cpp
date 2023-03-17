#include "../include/controll/ultrasonic.h"

namespace sweeper
{
namespace navigation
{
namespace controll
{
DealUltrasonic::DealUltrasonic() : ultra_1_filter_data_(-1.0), ultra_2_filter_data_(-1.0), ultra_3_filter_data_(-1.0),
                                   ultra_4_filter_data_(-1.0), ultra_5_filter_data_(-1.0), ultra_6_filter_data_(-1.0),
                                   ultra_7_filter_data_(-1.0), ultra_8_filter_data_(-1.0), ultra_9_filter_data_(-1.0),
                                   ultra_10_filter_data_(-1.0), ultra_1_saved_datas_num_(0), ultra_2_saved_datas_num_(0),
                                   ultra_3_saved_datas_num_(0), ultra_4_saved_datas_num_(0), ultra_5_saved_datas_num_(0),
                                   ultra_6_saved_datas_num_(0), ultra_7_saved_datas_num_(0), ultra_8_saved_datas_num_(0),
                                   ultra_9_saved_datas_num_(0), ultra_10_saved_datas_num_(0),filter_save_nums_(10)
{
}
DealUltrasonic::~DealUltrasonic() {}

void DealUltrasonic::InterpretUltrasonicData(const sweeper_msgs::PerceptionObstacle &ultra_origin, int indx)
{
    switch (ultra_origin.object[indx].id)
    {
    case 1:
        ultra_1_filter_data_ = UltrasonicFilter(ultra_origin, indx, ultra_origin.object[indx].id, ultra_1_saved_datas_num_);
        break;
    case 2:
        ultra_2_filter_data_ = UltrasonicFilter(ultra_origin, indx, ultra_origin.object[indx].id, ultra_2_saved_datas_num_);
        break;
    case 3:
        ultra_3_filter_data_ = UltrasonicFilter(ultra_origin, indx, ultra_origin.object[indx].id, ultra_3_saved_datas_num_);
        break;
    case 4:
        ultra_4_filter_data_ = UltrasonicFilter(ultra_origin, indx, ultra_origin.object[indx].id, ultra_4_saved_datas_num_);
        break;
    case 5:
        ultra_5_filter_data_ = UltrasonicFilter(ultra_origin, indx, ultra_origin.object[indx].id, ultra_5_saved_datas_num_);
        break;
    case 6:
        ultra_6_filter_data_ = UltrasonicFilter(ultra_origin, indx, ultra_origin.object[indx].id, ultra_6_saved_datas_num_);
        break;
    case 7:
        ultra_7_filter_data_ = UltrasonicFilter(ultra_origin, indx, ultra_origin.object[indx].id, ultra_7_saved_datas_num_);
        break;
    case 8:
        ultra_8_filter_data_ = UltrasonicFilter(ultra_origin, indx, ultra_origin.object[indx].id, ultra_8_saved_datas_num_);
        break;
    case 9:
        ultra_9_filter_data_ = UltrasonicFilter(ultra_origin, indx, ultra_origin.object[indx].id, ultra_9_saved_datas_num_);
        break;
    case 10:
        ultra_10_filter_data_ = UltrasonicFilter(ultra_origin, indx, ultra_origin.object[indx].id, ultra_10_saved_datas_num_);
        break;
    }
}

double DealUltrasonic::UltrasonicFilter(const sweeper_msgs::PerceptionObstacle &ultra_origin, int ultra_index, int id, int& saved_nums)
{
    double output;
    int ultr_saved_index_start = (id - 1) * filter_save_nums_;
    int saved_current_index= (id - 1) * filter_save_nums_ + saved_nums;
    if (saved_nums < filter_save_nums_)
    {
        ultrasonic_datas_[saved_current_index] = ultra_origin.object[ultra_index].position.x;
        saved_nums = saved_nums + 1;

        if (saved_nums == 1)
        {
            output = ultra_origin.object[ultra_index].position.x;
            return output;
        }
        else
        {
            output = ultrasonic_datas_[ultr_saved_index_start];
            for (int j = ultr_saved_index_start; j < (ultr_saved_index_start + saved_nums - 1); ++j)
            {
                if (output > ultrasonic_datas_[j + 1])
                {
                    output = ultrasonic_datas_[j + 1];
                }
            }
            return output;
        }
    }
    else if (saved_nums >= filter_save_nums_)
    {
        for (int j = ultr_saved_index_start; j < (ultr_saved_index_start + saved_nums - 1); ++j)
        {
            ultrasonic_datas_[j] = ultrasonic_datas_[j + 1];
        }
        ultrasonic_datas_[ ultr_saved_index_start+ (filter_save_nums_-1)] = ultra_origin.object[ultra_index].position.x;

        output = ultrasonic_datas_[ultr_saved_index_start];
        for (int j = ultr_saved_index_start; j < (ultr_saved_index_start + saved_nums - 1); ++j)
        {
            if (output > ultrasonic_datas_[j + 1])
            {
                output = ultrasonic_datas_[j + 1];
            }
        }
        saved_nums = filter_save_nums_;
        return output;
    }
}

} //namespace controll
} //namespace navigation
} //namespace sweeper
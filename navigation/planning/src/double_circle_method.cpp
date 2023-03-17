#include "double_circle_method.h"

namespace sweeper {
namespace navigation {
DoubleCircleMethod ::DoubleCircleMethod(ros::NodeHandle &private_nh,
                                        double car_width, double e_stop_x,
                                        double e_stop_y,
                                        RoadEdgeDetect *road_edge_detect)
    : last_line_times_(0),
      avoid_obstacle_direction_(0),
      obstacle_count_(0),
      no_obstacle_count_(0),
      current_type_(-1),
      last_time_(-1.0),
      last_stop_time_(-1.0),
      straight_distance_(0.0),
      straight_distance_thre_(3.0),
      accumulation_time_(0.0),
      last_obj_speed_x_(INT_MAX),
      current_stop_time_(FLT_MAX),
      last_obstacle_dis_x_(FLT_MAX),
      last_obstacle_dis_y_(FLT_MAX),
      pause_flag_(false),
      last_obstacle_flag_(false),
      last_have_obstacle_flag_(false),
      dis_mutation_flag_(false) {
  car_width_ = car_width;
  e_stop_x_ = e_stop_x;
  e_stop_y_ = e_stop_y;
  road_edge_detect_ = road_edge_detect;

  private_nh.param<int>("height", height_, 1000);
  private_nh.param<int>("width", width_, 1000);

  private_nh.param<double>("sweep_width", sweep_width_, 1.8);
  private_nh.param<double>("broadcast_thre", broadcast_thre_, 0.5);
  private_nh.param<double>("erode_x", erode_x_, 0.2);
  private_nh.param<double>("erode_y", erode_y_, 0.8);
  private_nh.param<double>("obstacle_x", obstacle_x_, 0.7);
  private_nh.param<double>("obstacle_y", obstacle_y_, 8.0);
  private_nh.param<double>("obj_expand_x", obj_expand_x_, 10);
  private_nh.param<double>("obj_expand_y", obj_expand_y_, 10);
  private_nh.param<double>("intersect_x", intersect_x_, 10);
  private_nh.param<double>("intersect_y", intersect_y_, 10);
  private_nh.param<double>("curb_obstacle_min", curb_obstacle_min_, 0.1);
  private_nh.param<double>("curb_obstacle_max", curb_obstacle_max_, 0.7);
  private_nh.param<double>("trace_obstacle_min", trace_obstacle_min_, 0.1);
  private_nh.param<double>("trace_obstacle_max", trace_obstacle_max_, 0.7);
  private_nh.param<double>("obstacle_thickness", obstacle_thickness_, 10);
  private_nh.param<double>("min_turn_radius", min_turn_radius_, 150);
  private_nh.param<double>("turn_obstacle_radius", turn_obstacle_radius_, 200);
  private_nh.param<double>("speed_thre", speed_thre_, 0.1);
  private_nh.param<double>("resolution", resolution_, 0.02);
  private_nh.param<double>("barrier_min_height", barrier_min_height_, 0.3);
  private_nh.param<double>("delta_dis_min_trace", delta_dis_min_trace_, 100);
  private_nh.param<double>("trace_velocity_x_thre", trace_velocity_x_thre_,
                           1.0);
  private_nh.param<double>("trace_velocity_y_thre", trace_velocity_y_thre_,
                           1.0);
  private_nh.param<double>("curb_velocity_x_thre", curb_velocity_x_thre_, 1.0);
  private_nh.param<double>("curb_velocity_y_thre", curb_velocity_y_thre_, 1.0);
  private_nh.param<double>("front_dis", front_dis_, 10);

  private_nh.param<bool>("double_circle_flag", double_circle_flag_, false);

  center_.x = round(width_ / 2.0);
  center_.y = round(front_dis_ / resolution_);
  default_x_ = center_.x;
}

DoubleCircleMethod::~DoubleCircleMethod() {}

void DoubleCircleMethod::Reset() {
  last_time_ = -1.0;
  last_stop_time_ = -1.0;
  avoid_obstacle_direction_ = 0;
  obstacle_count_ = 0;
  no_obstacle_count_ = 0;
  current_type_ = -1;
  straight_distance_ = 0.0;
  straight_distance_thre_ = 3.0;
  accumulation_time_ = 0.0;
  last_obj_speed_x_ = INT_MAX;
  current_stop_time_ = FLT_MAX;
  last_obstacle_dis_x_ = FLT_MAX;
  last_obstacle_dis_y_ = FLT_MAX;
  pause_flag_ = false;
  last_obstacle_flag_ = false;
  dis_mutation_flag_ = false;
  last_have_obstacle_flag_ = false;
}

void DoubleCircleMethod::GetPlanningTrajectory(
    const PlanParam &plan_param, bool *estop, cv::Mat *result_img,
    std::vector<cv::Point> *output_line,
    sweeper_msgs::SensorFaultInformation *state_code) {
  state_code_ = state_code;
  mode_ = plan_param.mode;
  curb_sweep_mode_ = plan_param.curb_sweep_mode;
  avoid_obstacle_flag_ = plan_param.avoid_obstacle_flag;
  only_trace_flag_ = plan_param.only_trace_flag;
  bool trace_good_flag = plan_param.trace_good_flag;
  bool road_flag = plan_param.road_edge_flag;
  road_edge_boundary_ = plan_param.road_edge_boundary;
  current_type_ = plan_param.current_type;
  double car_speed = plan_param.vehicle_speed;
  double road_edge_thickness = plan_param.road_edge_thickness;
  Objects track_objects = plan_param.obj;
  result_img_ = result_img;
  ContoursVector contours_boundary;
  ContoursVector contours_boundary_barrier;
  ContoursVector contours_avoid_barrier;
  ContoursVector contours_dilate_object;
  std::unordered_map<int, int> obstacle_v_x;
  std::unordered_map<int, int> continuous_v_x;
  std::unordered_map<int, int> continuous_v_x_1;
  std::unordered_map<int, int> continuous_v_y;
  std::unordered_map<int, int> continuous_v_y_1;

  double obstacle_dis_x = FLT_MAX;
  double obstacle_dis_y = FLT_MAX;
  int obj_speed_x = INT_MAX;
  double distance_min = FLT_MAX;
  double obj_distance_min = FLT_MAX;
  cv::Mat reference_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
  cv::Mat obstacle_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
  cv::Mat boundary_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));

  std::vector<cv::Point> reference_line = plan_param.reference_line;
  std::vector<cv::Point> patch_line;

  if (mode_ == 1 && !only_trace_flag_) {
    if (!OptimizePathKinematic(reference_img, boundary_img, &patch_line,
                               &reference_line)) {
      reference_line = plan_param.reference_line;
    }
  } else {
    reference_line = plan_param.reference_line;
  }
  // reference_line = plan_param.reference_line;

  reference_line_.clear();
  for (size_t i = 0; i < reference_line.size(); ++i) {
    if (i % 10 != 0) continue;
    reference_line_.push_back(reference_line[i]);
  }
  cv::polylines(reference_img, reference_line_, false, cv::Scalar(255), 1, 8,
                0);

  int obj_index = -1;
  int obstacle_index = -1;
  int obj_size = static_cast<int>(track_objects.size());
  std::vector<cv::Point> obj_velocity(obj_size);
  std::vector<double> line_bottom_left_min(obj_size, FLT_MAX);
  std::vector<double> line_bottom_right_min(obj_size, FLT_MAX);
  for (size_t i = 0; i < track_objects.size(); ++i) {
    std::vector<cv::Point> contour = track_objects[i]->contour;
    int obstacle_expand_width = track_objects[i]->obstacle_expand_width;
    int id = track_objects[i]->track_id;
    float v_x = track_objects[i]->velocity(0);
    float v_y = track_objects[i]->velocity(1);
    float v_object = fabs(v_x + plan_param.vehicle_speed);
    auto continuous_v_x_it = continuous_v_x_.find(id);
    if (continuous_v_x_it != continuous_v_x_.end()) {
      if ((mode_ == 1 && fabs(v_object) > 0.3 /*trace_velocity_x_thre_*/) ||
          (mode_ == 2 && v_x >= curb_velocity_x_thre_))
        continuous_v_x[id] = continuous_v_x_it->second + 1;
      else
        continuous_v_x[id] = 0;
    } else {
      continuous_v_x[id] = 0;
    }
    auto continuous_v_x_1_it = continuous_v_x_1_.find(id);
    if (continuous_v_x_1_it != continuous_v_x_1_.end()) {
      if ((mode_ == 1 && fabs(v_object) <= 0.3 /*trace_velocity_x_thre_*/) ||
          (mode_ == 2 && v_x < curb_velocity_x_thre_))
        continuous_v_x_1[id] = continuous_v_x_1_it->second + 1;
      else
        continuous_v_x_1[id] = 0;
    } else {
      continuous_v_x_1[id] = 0;
    }

    auto continuous_v_y_it = continuous_v_y_.find(id);
    if (continuous_v_y_it != continuous_v_y_.end()) {
      if ((mode_ == 1 && fabs(v_y) >= trace_velocity_y_thre_) ||
          (mode_ == 2 && fabs(v_y) >= curb_velocity_y_thre_))
        continuous_v_y[id] = continuous_v_y_it->second + 1;
      else
        continuous_v_y[id] = 0;
    } else {
      continuous_v_y[id] = 0;
    }

    auto continuous_v_y_1_it = continuous_v_y_1_.find(id);
    if (continuous_v_y_1_it != continuous_v_y_1_.end()) {
      if ((mode_ == 1 && fabs(v_y) < trace_velocity_y_thre_) ||
          (mode_ == 2 && fabs(v_y) < curb_velocity_y_thre_))
        continuous_v_y_1[id] = continuous_v_y_1_it->second + 1;
      else
        continuous_v_y_1[id] = 0;
    } else {
      continuous_v_y_1[id] = 0;
    }
    int speed_x = INT_MAX, speed_y = INT_MAX;
    auto obstacle_v_x_it = obstacle_v_x_.find(id);

    if (continuous_v_x[id] >= 3)
      speed_x = 1;
    else if (continuous_v_x_1[id] >= 3)
      speed_x = 0;
    else {
      if (obstacle_v_x_it != obstacle_v_x_.end()) speed_x = obstacle_v_x_[id];
    }
    if (continuous_v_y[id] >= 5) speed_y = 1;
    if (continuous_v_y_1[id] >= 3) speed_y = 0;
    obstacle_v_x[id] = speed_x;
    obj_velocity[i].x = speed_x;
    obj_velocity[i].y = speed_y;

    // 对边界进行分类，与轨迹线相交为障碍物层，反之为边界层
    cv::Mat current_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
    ContoursVector current_contour;
    current_contour.push_back(contour);
    if (!current_contour.empty())
      cv::drawContours(current_img, current_contour, -1, cv::Scalar(255), -1,
                       8);
    int object_type = -1;
    cv::Point bottom_left_point = track_objects[i]->bottom_left_point;
    cv::Point bottom_right_point = track_objects[i]->bottom_right_point;
    double bottom_left_dis_min = FLT_MAX, bottom_right_dis_min = FLT_MAX;
    for (size_t j = 0; j < reference_line.size(); ++j) {
      int x = reference_line[j].x;
      int y = reference_line[j].y;
      if (x <= 2 || x >= width_ - 2 || y <= 2 || y >= height_ - 2) continue;
      double bottom_left_dis = sqrt(pow(bottom_left_point.x - x, 2) +
                                    pow(bottom_left_point.y - y, 2));
      double bottom_right_dis = sqrt(pow(bottom_right_point.x - x, 2) +
                                     pow(bottom_right_point.y - y, 2));
      if (bottom_left_dis < bottom_left_dis_min) {
        bottom_left_dis_min = bottom_left_dis;
        if (bottom_left_point.x - x < 0.001)
          line_bottom_left_min[i] = -bottom_left_dis_min;
        else
          line_bottom_left_min[i] = bottom_left_dis_min;
      }
      if (bottom_right_dis < bottom_right_dis_min) {
        bottom_right_dis_min = bottom_right_dis;
        if (bottom_right_point.x - x < 0.001)
          line_bottom_right_min[i] = -bottom_right_dis_min;
        else
          line_bottom_right_min[i] = bottom_right_dis_min;
      }
      if (current_img.at<uchar>(y, x) == 255 && object_type != 1 &&
          (mode_ == 1 || (road_flag && mode_ == 2))) {
        object_type = 1;
        // break;
      } else if (current_img.at<uchar>(y, x) == 0 && object_type != 1 &&
                 (mode_ == 1 || (road_flag && mode_ == 2))) {
        object_type = 2;
      }
    }

    switch (object_type) {
      case 1: {
        for (size_t k = 0; k < contour.size(); ++k) {
          int x = contour[k].x;
          int y = contour[k].y;
          double distance;
          distance = sqrt(pow(x - center_.x, 2) + pow(y - center_.y, 2));
          if (distance < distance_min) {
            distance_min = distance;
            obj_index = i;
          }

          if (distance < obj_distance_min &&
              !track_objects[i]->inside_road_edge_flag) {
            obj_distance_min = distance;
            obstacle_index = i;
          }
        }

        ContoursVector contours_temp;
        ContoursVector contours_temp_1;

        // obstacle_avoid_img:
        // 对障碍物进行膨胀，用来获取绕障碍物的轮廓轨迹；
        // obstacle_boundary_img
        // 用来判断生成的轨迹是否与障碍物本身相交，若相交寻找下一个轨迹点进行规划；

        cv::Mat obstacle_avoid_img =
            cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
        cv::Mat obstacle_boundary_img =
            cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
        cv::Mat current_contour_img =
            cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));

        int obj_expand_y = obj_expand_y_ / resolution_ - obstacle_expand_width;
        int obj_expand_x = obj_expand_x_ / resolution_ - obstacle_expand_width;

        if (mode_ == 2 && track_objects[i]->inside_road_edge_flag) {
          obj_expand_x = (int)(obstacle_thickness_ / resolution_);
        }

        if (obj_expand_x <= 2) obj_expand_x = 2;
        if (obj_expand_y <= 2) obj_expand_y = 2;
        road_edge_detect_->DilateImg(current_img, current_contour_img,
                                     obj_expand_x, obj_expand_y);
        cv::findContours(current_contour_img, contours_temp, CV_RETR_EXTERNAL,
                         CV_CHAIN_APPROX_NONE);
        cv::findContours(obstacle_boundary_img, contours_temp_1,
                         CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        if (!contours_temp.empty())
          contours_avoid_barrier.push_back(contours_temp[0]);
        if (!contours_temp_1.empty())
          contours_boundary_barrier.push_back(contours_temp_1[0]);
      } break;
      case 2:
        if (!track_objects[i]->inside_road_edge_flag)
          contours_boundary.push_back(contour);
        break;
    }
  }
  obstacle_v_x_ = obstacle_v_x;
  continuous_v_x_ = continuous_v_x;
  continuous_v_y_ = continuous_v_y;
  continuous_v_x_1_ = continuous_v_x_1;
  continuous_v_y_1_ = continuous_v_y_1;

  bool inside_flag = false;
  if (obj_index != -1) {
    obstacle_dis_x = track_objects[obj_index]->dis_x_y.x;
    obstacle_dis_y = track_objects[obj_index]->dis_x_y.y;
    obj_speed_x = obj_velocity[obj_index].x;
    inside_flag = track_objects[obj_index]->inside_road_edge_flag;
  }

  if (obstacle_dis_y < center_.y) {
    obstacle_count_++;
    if (obstacle_count_ > 2) no_obstacle_count_ = 0;
  } else {
    no_obstacle_count_++;
    if (no_obstacle_count_ > 2)
      obstacle_count_ = 0;
    else {
      obj_speed_x = last_obj_speed_x_;
      obstacle_dis_x = last_obstacle_dis_x_;
      obstacle_dis_y = last_obstacle_dis_y_;
    }
  }
  bool have_obstacle_flag = false;
  if (obstacle_count_ > 2) have_obstacle_flag = true;

  double current_pause_time = ros::Time::now().toSec();
  if (have_obstacle_flag && !last_have_obstacle_flag_ && !pause_flag_ &&
      obstacle_dis_y > 5 / resolution_ && obstacle_dis_y < center_.y &&
      abs(obstacle_dis_x - center_.x) < 0.9 / resolution_ &&
      (mode_ == 1 && !only_trace_flag_)) {
    AWARN << "Have obstacle, pause two secend";
    AWARN << "obstacle_dis_x: " << obstacle_dis_x;
    pause_flag_ = true;
  }
  if (pause_flag_) accumulation_time_ += current_pause_time - last_stop_time_;
  if (accumulation_time_ > 3.0) {
    AWARN << "End obstacle pause.";
    pause_flag_ = false;
    accumulation_time_ = 0;
  }
  last_stop_time_ = current_pause_time;

  if (!contours_boundary.empty())
    cv::drawContours(boundary_img, contours_boundary, -1, cv::Scalar(255), 1,
                     8);

  bool right_flag;
  start_in_obj_flag_ = false;
  end_in_obj_flag_ = false;
  if (!dis_mutation_flag_) {
    if (!pause_flag_) {
      /*if ((mode_ == 1 && !only_trace_flag_) ||
          (mode_ == 2 && (avoid_obstacle_flag_ ||
                          (!avoid_obstacle_flag_ && obj_index != -1 &&
                           (track_objects[obj_index]->inside_road_edge_flag ||
                            (curb_sweep_mode_ == 0 &&
                             line_bottom_right_min[obj_index] < -0.1) ||
                            (curb_sweep_mode_ == 1 &&
                             line_bottom_left_min[obj_index] > 0.1)))))) {
        cv::drawContours(obstacle_img, contours_avoid_barrier, -1,
                         cv::Scalar(255), -1, 8);
      }*/

      if (have_obstacle_flag &&
          ((mode_ == 1 && !only_trace_flag_) || mode_ == 2)) {
        cv::drawContours(obstacle_img, contours_avoid_barrier, -1,
                         cv::Scalar(255), -1, 8);
        cv::Point start = reference_line_.front();
        cv::Point end = reference_line_.back();
        if (obstacle_img.at<uchar>(start.y, start.x) == 255)
          start_in_obj_flag_ = true;
        if (obstacle_img.at<uchar>(end.y, end.x) == 255)
          end_in_obj_flag_ = true;
      }

      cv::polylines(obstacle_img, reference_line_, false, cv::Scalar(255), 1, 8,
                    0);
      GetOptimalPath(obstacle_img, boundary_img, have_obstacle_flag,
                     output_line, &right_flag);
    }

    int stop_y = 6.5 / resolution_;
    if (only_trace_flag_ && obstacle_dis_y > stop_y && have_obstacle_flag &&
        abs(obstacle_dis_x - 5 / resolution_) < 0.8 / resolution_ &&
        !(*estop)) {
      output_line->clear();
      if (fabs(car_speed) < 0.1) state_code_->state_code.push_back(3207);
      AWARN << "Have obstacle in only trace flag.";
    }

    if (obj_index != -1 && mode_ == 1 && !only_trace_flag_) {
      double line_bottom_left = line_bottom_left_min[obj_index];
      double line_bottom_right = line_bottom_right_min[obj_index];
      double bottom_right_x = track_objects[obj_index]->bottom_right_point.x;
      double bottom_left_x = track_objects[obj_index]->bottom_left_point.x;
      if (obj_speed_x == 1) {
        if (/*obstacle_dis_y <= 700 &&*/ trace_good_flag) {
          if (obstacle_dis_y < 5.0 / resolution_) {
            *output_line = reference_line;
            AWARN << "Trace good have moving obstacle,not avoid obstacle.";
          } else if (obstacle_dis_y < center_.y) {
            // output_line->clear();
            // AWARN << "Trace good moving obstacle too closed,stop.";
          }
        } else {
          // if (obstacle_dis_y < 6 / resolution_) output_line->clear();
          // AWARN << "avoiding obstacles,obstacle moving,stop.";
        }

      } else if (last_obj_speed_x_ == 1 && obj_speed_x == 0) {
        AWARN << "Obj velocity set 1 to 0,pause.";
        // pause_flag_ = true;
      }

      if (!output_line->empty() && obstacle_dis_y < center_.y) {
        int trace_obstacle_min = trace_obstacle_min_ / resolution_;
        int trace_obstacle_max = trace_obstacle_max_ / resolution_;
        if (!right_flag) {
          // if (trace_good_flag) {
          if (line_bottom_left < -trace_obstacle_max ||
              line_bottom_right < -trace_obstacle_min) {
            *output_line = reference_line;
            if (obstacle_dis_y > 6.0 / resolution_) {
              output_line->clear();
              if (fabs(car_speed) < 0.1) {
                state_code_->state_code.push_back(3214);
                AWARN << "3214";
              }
            } else {
              AWARN << "Obstacle is too width,not avoid.";
            }
          } else {
            if (!output_line->empty() && have_obstacle_flag &&
                obstacle_dis_y > 4.0 / resolution_ && !(*estop)) {
              state_code_->state_code.push_back(3209);
              // AWARN << "3209";
            }
          }
        } else {
          if (line_bottom_left > trace_obstacle_min ||
              line_bottom_right > trace_obstacle_max) {
            *output_line = reference_line;
            if (obstacle_dis_y > 6.0 / resolution_) {
              output_line->clear();
              if (fabs(car_speed) < 0.1) {
                state_code_->state_code.push_back(3214);
                AWARN << "3214";
              }
            } else {
              AWARN << "Obstacle is too width,not avoid.";
            }
          } else {
            if (!output_line->empty() && have_obstacle_flag &&
                obstacle_dis_y > 4.0 / resolution_ && !(*estop)) {
              state_code_->state_code.push_back(3210);
              // AWARN << "3210";
            }
          }
        }
      }
    }

    if (obstacle_index != -1 && mode_ == 2) {
      double line_bottom_left = line_bottom_left_min[obstacle_index];
      double line_bottom_right = line_bottom_right_min[obstacle_index];
      double bottom_right_y =
          track_objects[obstacle_index]->bottom_right_point.y;
      double bottom_left_y = track_objects[obstacle_index]->bottom_left_point.y;
      int curb_obstacle_min = curb_obstacle_min_ / resolution_;
      int curb_obstacle_max = curb_obstacle_max_ / resolution_;
      if (curb_sweep_mode_ == 0) {
        if (line_bottom_left > curb_obstacle_min ||
            line_bottom_right > curb_obstacle_max) {
          *output_line = reference_line;
          if (bottom_right_y > 5 / resolution_ && bottom_right_y < center_.y) {
            output_line->clear();
            if (fabs(car_speed) < 0.1) {
              state_code_->state_code.push_back(3214);
              AWARN << "3214.";
            }
          }
        } else if (!output_line->empty() && have_obstacle_flag &&
                   (!inside_flag ||
                    (line_bottom_left > -0.4 / resolution_ && inside_flag)) &&
                   bottom_left_y > 4.0 / resolution_) {
          state_code_->state_code.push_back(3209);
          // AWARN << "3209.";
        }
      }
      if (curb_sweep_mode_ == 1) {
        if (line_bottom_left < -curb_obstacle_max ||
            line_bottom_right < -curb_obstacle_min) {
          *output_line = reference_line;
          if (bottom_left_y > 5 / resolution_ && bottom_right_y < center_.y) {
            output_line->clear();
            if (fabs(car_speed) < 0.1) {
              AWARN << "3214";
              state_code_->state_code.push_back(3214);
            }
          }
        } else if (!output_line->empty() && have_obstacle_flag &&
                   (!inside_flag ||
                    (line_bottom_right < 0.4 / resolution_ && inside_flag)) &&
                   bottom_left_y > 4.0 / resolution_) {
          state_code_->state_code.push_back(3210);
          // AWARN << "3210";
        }
      }
    }

    if (!avoid_obstacle_flag_ && have_obstacle_flag && !inside_flag &&
        !output_line->empty() && mode_ == 2) {
      output_line->clear();
      if (curb_sweep_mode_ == 1) {
        if (fabs(car_speed) < 0.1) {
          AWARN << "3212";
          state_code_->state_code.push_back(3212);
        }
      } else {
        if (fabs(car_speed) < 0.1) {
          AWARN << "3213";
          state_code_->state_code.push_back(3213);
        }
      }
    }

    if (have_obstacle_flag && no_obstacle_count_ > 0 && no_obstacle_count_ <= 2)
      *output_line = last_trace_line_;
    if (pause_flag_ && !(*estop)) {
      // *estop = true;
      if (fabs(car_speed) < 0.1) state_code_->state_code.push_back(3207);
      output_line->clear();
    }
    if (output_line->empty()) {
      last_time_ = -1.0;
      straight_distance_ = 0.0;
      last_trace_line_.clear();
      last_obstacle_dis_x_ = FLT_MAX;
      last_obstacle_dis_y_ = FLT_MAX;
      last_have_obstacle_flag_ = have_obstacle_flag;
      last_obj_speed_x_ = obj_speed_x;
      return;
    }
  }

  if (mode_ == 1) {
    // 在寻迹避障过程中，判断障碍物是否已经在感知范围外
    if (!dis_mutation_flag_ && last_have_obstacle_flag_ &&
        !have_obstacle_flag && last_obstacle_dis_y_ != FLT_MAX &&
        last_obstacle_dis_y_ >= 6.5 / resolution_ &&
        last_obstacle_dis_y_ < center_.y &&
        ((mode_ == 1 && !only_trace_flag_ && !trace_good_flag)) &&
        (plan_param.vehicle_speed > 0.1)) {
      if (right_flag) {
        default_x_ = center_.x +
                     (obj_expand_x_ / (2 * resolution_) -
                      abs(last_obstacle_dis_x_ - center_.x)) +
                     0.2 / resolution_;
      } else {
        default_x_ = center_.x -
                     (obj_expand_x_ / (2 * resolution_) -
                      abs(last_obstacle_dis_x_ - center_.x)) -
                     0.2 / resolution_;
      }

      if (default_x_ > center_.x + 1 / resolution_)
        default_x_ = center_.x + 1 / resolution_;
      else if (default_x_ < center_.x - 1 / resolution_)
        default_x_ = center_.x - 1 / resolution_;

      dis_mutation_flag_ = true;
      straight_distance_thre_ =
          (center_.y - last_obstacle_dis_y_) * resolution_;
      if (straight_distance_thre_ < 0.0001) straight_distance_thre_ = 0.0001;
      if (straight_distance_thre_ > 3.0) straight_distance_thre_ = 3.0;
      AWARN << "default_x_: " << default_x_;
      AWARN << "last obstacle dis x: " << last_obstacle_dis_x_;
      AWARN << "last obstacle dis y: " << last_obstacle_dis_y_;
      AWARN << "straight distance thre: " << straight_distance_thre_;
    }

    // 在感知范围外，则车保持该方向继续往前走一段，保证车不碰到障碍物
    if (dis_mutation_flag_) {
      output_line->clear();
      double current_time = ros::Time::now().toSec();
      if (last_time_ > 0.0001) {
        straight_distance_ +=
            (current_time - last_time_) * plan_param.vehicle_speed;
      }
      if (straight_distance_ <= straight_distance_thre_) {
        for (size_t i = 0; i < 5 / resolution_; i++) {
          output_line->push_back(cv::Point(default_x_, center_.y - i));
        }
        // *output_line = last_trace_line_;
        last_time_ = current_time;
      } else {
        last_time_ = -1.0;
        straight_distance_ = 0.0;
        dis_mutation_flag_ = false;
      }
    }
  } else if (mode_ == 2) {
    // 在马路沿模式下，将马路沿边界轮廓加入到障碍物边界轮廓中
    if (!road_edge_boundary_.empty())
      contours_boundary_barrier.insert(contours_boundary_barrier.end(),
                                       road_edge_boundary_.begin(),
                                       road_edge_boundary_.end());
  }
  last_obj_speed_x_ = obj_speed_x;
  last_obstacle_dis_x_ = obstacle_dis_x;
  last_obstacle_dis_y_ = obstacle_dis_y;
  last_have_obstacle_flag_ = have_obstacle_flag;
  last_trace_line_ = *output_line;

  // 判断最终生成的轨迹是否与边界相交，若相交发送estop指令
  // if (IsIntersectBoundary(*output_line, boundary_img)) *estop = 7;
}

void DoubleCircleMethod::GetOptimalPath(const cv::Mat &obstacle_img,
                                        const cv::Mat &boundary_img,
                                        const bool have_obstacle_flag,
                                        std::vector<cv::Point> *output_line,
                                        bool *right_flag) {
  std::vector<cv::Point> left_line, right_line;
  ContoursVector contours;
  cv::findContours(obstacle_img, contours, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_NONE);
  cv::Mat test_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
  if (!contours.empty())
    cv::drawContours(test_img, contours, -1, cv::Scalar(255), 1, 8);
  if (contours.size() == 1) GetPathPoint(contours[0], &left_line, &right_line);
  if (!output_line->empty()) output_line->clear();
  bool is_left_line_intersect = false;
  bool is_right_line_intersect = false;
  int left_size = static_cast<int>(left_line.size());
  int right_size = static_cast<int>(right_line.size());
  if ((mode_ == 1) || (curb_sweep_mode_ == 1 && mode_ == 2)) {
    for (int i = 0; i < left_line.size(); ++i) {
      int x = left_line[i].x;
      int y = left_line[i].y;
      if (fabs(x - center_.x) >= intersect_x_ / resolution_ ||
          center_.y - y > intersect_y_ / resolution_)
        continue;
      if (boundary_img.at<uchar>(y, x) == 255) {
        circle(*result_img_, left_line[i], 2, cv::Scalar(0, 0, 255), -1, 10);
        is_left_line_intersect = true;
        break;
      }
    }
  }
  if ((mode_ == 1) || (curb_sweep_mode_ == 0 && mode_ == 2)) {
    for (int i = 0; i < right_line.size(); ++i) {
      int x = right_line[i].x;
      int y = right_line[i].y;
      if (fabs(x - center_.x) >= intersect_x_ / resolution_ ||
          center_.y - y > intersect_y_ / resolution_)
        continue;
      if (boundary_img.at<uchar>(y, x) == 255) {
        circle(*result_img_, right_line[i], 2, cv::Scalar(0, 0, 255), -1, 10);
        is_right_line_intersect = true;
        break;
      }
    }
  }

  // AWARN << "current_type_: " << current_type_;

  if (mode_ == 1 && (current_type_ == 0 || current_type_ == 5)) {
    if (!is_left_line_intersect && is_right_line_intersect) {
      if (!last_have_obstacle_flag_ && have_obstacle_flag) {
        avoid_obstacle_direction_ = 0;
        AWARN << "Right line intersect boundary,start left avoid the obstacle.";
      }
    } else if (is_left_line_intersect && !is_right_line_intersect) {
      if (!last_have_obstacle_flag_ && have_obstacle_flag) {
        avoid_obstacle_direction_ = 1;
        AWARN << "Left line intersect boundary,start right avoid the obstacle.";
      }
    } else if (!is_left_line_intersect && !is_right_line_intersect) {
      if (!last_have_obstacle_flag_ && have_obstacle_flag) {
        AWARN << "left_size: " << left_size;
        AWARN << "right_size: " << right_size;
        if (left_size == right_size) {
          if (curb_sweep_mode_ == 0)
            avoid_obstacle_direction_ = 1;
          else if (curb_sweep_mode_ == 1)
            avoid_obstacle_direction_ = 0;
        } else if (left_size - right_size <= 30) {
          avoid_obstacle_direction_ = 0;
          AWARN << "Start left avoid the obstacle.";
        } else {
          avoid_obstacle_direction_ = 1;
          AWARN << "Start right avoid the obstacle.";
        }
      }
    } else {
      AWARN << "Intersection of trajectories and boundaries,stop!!!";
    }
    if (last_have_obstacle_flag_ && !have_obstacle_flag) {
      // last_have_obstacle_flag_ = 0;
      AWARN << "End avoid the obstacle";
    }
    if (avoid_obstacle_direction_ == 0) {
      *output_line = left_line;
      *right_flag = false;
    } else if (avoid_obstacle_direction_ == 1) {
      *output_line = right_line;
      *right_flag = true;
    }
  } else if (mode_ == 1) {
    if (current_type_ == 9 && !is_right_line_intersect) {
      *output_line = right_line;
      *right_flag = true;
    }

    if (current_type_ == 10 && !is_left_line_intersect) {
      *output_line = left_line;
      *right_flag = false;
    }

    if ((current_type_ == 4 || current_type_ == 6 || current_type_ == 7 ||
         current_type_ == 8) &&
        !is_right_line_intersect) {
      if (only_trace_flag_) {
        *output_line = left_line;
        *right_flag = false;
      } else {
        if (curb_sweep_mode_ == 0 && !is_right_line_intersect) {
          *output_line = right_line;
          *right_flag = true;
        } else if (curb_sweep_mode_ == 1 && !is_left_line_intersect) {
          *output_line = left_line;
          *right_flag = false;
        }
      }
    }

    if (*right_flag && is_right_line_intersect) {
      AWARN << "Right intersection of trajectories and boundaries,stop!!!";
      output_line->clear();
    }

    if (!(*right_flag) && is_left_line_intersect) {
      AWARN << "Left intersection of trajectories and boundaries,stop!!!";
      output_line->clear();
    }
  } else if (mode_ == 2) {
    if (curb_sweep_mode_ == 0 && !is_right_line_intersect) {
      *output_line = right_line;
      *right_flag = true;
    } else if (curb_sweep_mode_ == 1 && !is_left_line_intersect) {
      *output_line = left_line;
      *right_flag = false;
    } else {
      AWARN << "Intersection of trajectories and boundaries,stop!!!";
    }
  }
}

void DoubleCircleMethod::GetPathPoint(const std::vector<cv::Point> &contours,
                                      std::vector<cv::Point> *left_line,
                                      std::vector<cv::Point> *right_line) {
  if (contours.empty()) return;
  cv::Point start_point = reference_line_.front();
  cv::Point end_point = reference_line_.back();

  if (end_in_obj_flag_) {
    if (curb_sweep_mode_ == 0)
      end_point = cv::Point(width_, 0);
    else if (curb_sweep_mode_ == 1) {
      end_point = cv::Point(0, 0);
    }
  }

  if (start_in_obj_flag_) start_point = cv::Point(center_.x, center_.y);

  double near_dis_min = FLT_MAX;
  double far_dis_min = FLT_MAX;
  int near_index = 0;
  int far_index = 0;

  for (size_t i = 0; i < contours.size(); ++i) {
    if (i % 10 != 0) continue;
    int x = contours[i].x;
    int y = contours[i].y;

    double near_distance =
        sqrt(pow(x - start_point.x, 2) + pow(y - start_point.y, 2));
    if (near_distance < near_dis_min) {
      near_dis_min = near_distance;
      near_index = i;
    }

    double far_distance =
        sqrt(pow(x - end_point.x, 2) + pow(y - end_point.y, 2));
    if (far_distance < far_dis_min) {
      far_dis_min = far_distance;
      far_index = i;
    }
  }

  int size = static_cast<int>(contours.size());

  std::vector<cv::Point> temp_vec;
  bool left_flag = false, right_flag = false;
  for (int i = 0; i < size; ++i) {
    cv::Point point = contours[i];
    if (i == near_index) {
      left_flag = false;
      right_flag = true;
    }
    if (i == far_index) {
      left_flag = true;
      right_flag = false;
    }
    if (left_flag) left_line->push_back(point);
    if (right_flag) right_line->push_back(point);
    if (!left_flag && !right_flag) temp_vec.push_back(point);
  }

  if (near_index >= far_index) {
    std::reverse(left_line->begin(), left_line->end());
    right_line->insert(right_line->end(), temp_vec.begin(), temp_vec.end());
  } else {
    left_line->insert(left_line->end(), temp_vec.begin(), temp_vec.end());
    std::reverse(left_line->begin(), left_line->end());
  }
  // cv::polylines(*result_img_, *left_line, false, cv::Scalar(0, 255, 0), 2, 8,
  //              0);
  // cv::polylines(*result_img_, *right_line, false, cv::Scalar(255, 0, 0), 2,
  // 8,
  //              0);
}

void DoubleCircleMethod::SubsectionExpand(const int obj_expand_x,
                                          const int obj_expand_y,
                                          const int patch_expand_width,
                                          const std::vector<cv::Point> &contour,
                                          cv::Mat *expand_img) {
  std::vector<cv::Point> up_line, down_line, left_line, right_line;
  cv::polylines(*expand_img, contour, true, cv::Scalar(255), 1, 8, 0);
  SplitContour(contour, &left_line, &right_line, &up_line, &down_line);
  std::vector<cv::Point> expand_up_line;
  std::vector<cv::Point> expand_down_line;
  if (!up_line.empty()) {
    cv::Point up_front = up_line.front();
    cv::Point up_back = up_line.back();
    cv::Point patch_up_front, patch_up_back;
    if (up_front.x + obj_expand_x / 2 > 9.98 / resolution_)
      patch_up_front.x = 9.98 / resolution_;
    else
      patch_up_front.x = up_front.x + patch_expand_width / 2;
    patch_up_front.y = up_front.y;
    if (up_back.x - obj_expand_x / 2 < 2)
      patch_up_back.x = 2;
    else
      patch_up_back.x = up_back.x - patch_expand_width / 2;
    patch_up_back.y = up_back.y;
    up_line.insert(up_line.begin(), patch_up_front);
    up_line.insert(up_line.end(), patch_up_back);

    int up_line_size = static_cast<int>(up_line.size());
    expand_up_line.resize(up_line_size);
    for (size_t k = 0; k < up_line_size; ++k) {
      cv::Point point;
      point.x = up_line[k].x;
      point.y = up_line[k].y - obj_expand_y / 2;
      if (point.y < 2) point.y = 2;
      expand_up_line[up_line_size - 1 - k] = point;
    }
    expand_up_line.insert(expand_up_line.end(), up_line.begin(), up_line.end());
  }

  if (!down_line.empty()) {
    cv::Point down_front = down_line.front();
    cv::Point down_back = down_line.back();
    cv::Point patch_down_front, patch_down_back;

    if (down_front.x - obj_expand_x / 2 < 2)
      patch_down_front.x = 2;
    else
      patch_down_front.x = down_front.x - patch_expand_width / 2;
    patch_down_front.y = down_front.y;
    if (down_back.x + obj_expand_x / 2 > 9.98 / resolution_)
      patch_down_back.x = 9.98 / resolution_;
    else
      patch_down_back.x = down_back.x + patch_expand_width / 2;
    patch_down_back.y = down_back.y;
    down_line.insert(down_line.begin(), patch_down_front);
    down_line.insert(down_line.end(), patch_down_back);

    int down_line_size = static_cast<int>(down_line.size());
    expand_down_line.resize(down_line_size);
    for (size_t k = 0; k < down_line_size; ++k) {
      cv::Point point;
      point.x = down_line[k].x;
      point.y = down_line[k].y + obj_expand_y / 2;
      if (point.y > 9.98 / resolution_) point.y = 9.98 / resolution_;
      expand_down_line[down_line_size - 1 - k] = point;
    }
    expand_down_line.insert(expand_down_line.end(), down_line.begin(),
                            down_line.end());
  }
  cv::polylines(*expand_img, expand_up_line, true, cv::Scalar(255), 1, 8, 0);
  cv::polylines(*expand_img, expand_down_line, true, cv::Scalar(255), 1, 8, 0);
  std::vector<cv::Point> sparse_left_line, sparse_right_line;
  SparseLine(10, left_line, &sparse_left_line);
  SparseLine(10, right_line, &sparse_right_line);
  if (curb_sweep_mode_ == 0 && mode_ == 2)
    cv::polylines(*expand_img, sparse_right_line, false, cv::Scalar(255),
                  obj_expand_x, 8, 0);
  else if (curb_sweep_mode_ == 1 && mode_ == 2)
    cv::polylines(*expand_img, sparse_left_line, false, cv::Scalar(255),
                  obj_expand_x, 8, 0);
  if (mode_ == 1) {
    cv::polylines(*expand_img, sparse_right_line, false, cv::Scalar(255),
                  obj_expand_x, 8, 0);
    cv::polylines(*expand_img, sparse_left_line, false, cv::Scalar(255),
                  obj_expand_x, 8, 0);
  }
}

void DoubleCircleMethod::SparseLine(const int sparse_factor,
                                    const std::vector<cv::Point> &line,
                                    std::vector<cv::Point> *sparse_line) {
  int line_size = static_cast<int>(line.size());
  for (size_t i = 0; i < line_size; ++i) {
    if (i == 0 || i == line_size - 1 || i % sparse_factor != 0) continue;
    sparse_line->push_back(line[i]);
  }
}

void DoubleCircleMethod::SplitContour(const std::vector<cv::Point> &contours,
                                      std::vector<cv::Point> *left_line,
                                      std::vector<cv::Point> *right_line,
                                      std::vector<cv::Point> *up_line,
                                      std::vector<cv::Point> *down_line,
                                      bool hull_flag) {
  if (contours.empty()) return;
  std::vector<cv::Point> contour;
  std::vector<cv::Point> hull;
  if (!hull_flag) {
    contour = contours;
    cv::convexHull(cv::Mat(contour), hull, false);
  } else {
    ContoursVector tmp_contours;
    cv::Mat img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
    cv::polylines(img, contours, true, cv::Scalar(255), 1, 8, 0);
    cv::findContours(img, tmp_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    contour = tmp_contours[0];
    hull = contours;
  }

  double left_down_dis_min = FLT_MAX;
  double right_down_dis_min = FLT_MAX;
  double left_top_dis_min = FLT_MAX;
  double right_top_dis_min = FLT_MAX;
  double left_top_dis, right_top_dis;
  double left_down_dis, right_down_dis;
  cv::Point left_top, right_top;
  cv::Point left_down, right_down;

  for (size_t i = 0; i < hull.size(); ++i) {
    int x = hull[i].x;
    int y = hull[i].y;
    left_top_dis = sqrt(pow(x, 2) + pow(y, 2));
    left_down_dis = sqrt(pow(x, 2) + pow(height_ - y, 2));
    right_top_dis = sqrt(pow(width_ - x, 2) + pow(y, 2));
    right_down_dis = sqrt(pow(width_ - x, 2) + pow(height_ - y, 2));
    if (left_down_dis < left_down_dis_min) {
      left_down_dis_min = left_down_dis;
      left_down = hull[i];
    }
    if (left_top_dis < left_top_dis_min) {
      left_top_dis_min = left_top_dis;
      left_top = hull[i];
    }
    if (right_down_dis < right_down_dis_min) {
      right_down_dis_min = right_down_dis;
      right_down = hull[i];
    }
    if (right_top_dis < right_top_dis_min) {
      right_top_dis_min = right_top_dis;
      right_top = hull[i];
    }
  }

  std::vector<cv::Point> temp_vec;
  int left_down_index = 0, right_down_index = 0;
  int left_top_index = 0, right_top_index = 0;
  bool left_flag = false, right_flag = false;
  bool up_flag = false, down_flag = false;
  for (int i = 0; i < contour.size(); ++i) {
    cv::Point point = contour[i];
    if (point == left_top) {
      left_top_index = i;
      left_flag = true;
      right_flag = false;
      up_flag = false;
      down_flag = false;
    }
    if (point == left_down) {
      left_down_index = i;
      left_flag = false;
      right_flag = false;
      up_flag = false;
      down_flag = true;
    }
    if (point == right_down) {
      right_down_index = i;
      left_flag = false;
      right_flag = true;
      up_flag = false;
      down_flag = false;
    }
    if (point == right_top) {
      right_top_index = i;
      left_flag = false;
      right_flag = false;
      up_flag = true;
      down_flag = false;
    }
    if (left_flag) left_line->push_back(point);
    if (right_flag) right_line->push_back(point);
    if (up_flag) up_line->push_back(point);
    if (down_flag) down_line->push_back(point);
    if (!left_flag && !right_flag && !up_flag && !down_flag)
      temp_vec.push_back(point);
  }
  if (left_down_index >= left_top_index) {
    std::reverse(left_line->begin(), left_line->end());
    up_line->insert(up_line->end(), temp_vec.begin(), temp_vec.end());
  } else {
    left_line->insert(left_line->end(), temp_vec.begin(), temp_vec.end());
    std::reverse(left_line->begin(), left_line->end());
  }
}

bool DoubleCircleMethod::OptimizePathKinematic(
    const cv::Mat &reference_img, const cv::Mat &obstacle_img,
    std::vector<cv::Point> *patch_line, std::vector<cv::Point> *final_line,
    int interval_num) {
  std::vector<PathPoint> path_points;
  std::vector<PathPoint> final_turning_points;
  std::vector<int> inflection_indexs;

  if (final_line->empty()) return false;
  std::vector<cv::Point> origin_line = *final_line;
  final_line->clear();
  int line_size = static_cast<int>(origin_line.size());
  int line_turn_index = -1;
  bool first_flag = false;
  bool model_flag = false;
  bool first_turn_flag = false;
  double r_max = -1.0;
  int r_max_index = -1;

  // 求轨迹点曲率，以及切线斜率
  for (int i = 0; i < line_size; ++i) {
    PathPoint path_point;
    path_point.point = origin_line[i];
    path_point.index = i;
    path_point.flag = -1;
    path_point.location = -1;
    path_point.slop_angle = FLT_MAX;
    if (i % interval_num == 0 &&
        (i - interval_num >= 0 && i + interval_num < line_size)) {
      // 转到车身坐标
      cv::Point last, current, next;
      current.x = center_.y - origin_line[i].y;
      current.y = center_.x - origin_line[i].x;
      last.x = center_.y - origin_line[i - interval_num].y;
      last.y = center_.x - origin_line[i - interval_num].x;
      next.x = center_.y - origin_line[i + interval_num].y;
      next.y = center_.x - origin_line[i + interval_num].x;

      // 求斜率,相对车身坐标的斜率，-180 < slop_angle < 180,
      // 沿x轴顺时针为负，逆时针为正；
      path_point.slop_angle =
          atan2((double)(next.y - current.y), (double)(next.x - current.x));

      CalculateRadiusAndLocation(&path_point);
      if (path_point.r > r_max) {
        r_max = path_point.r;
        r_max_index = i;
      }

      path_point.flag = 0;
      if (path_point.r > min_turn_radius_ / resolution_ && !first_turn_flag) {
        first_turn_flag = true;
        line_turn_index = i;
      }

      // 求曲率
      path_point.curvity = CalculateCurvity(last, current, next);

      if ((1.0 / path_point.curvity) <= min_turn_radius_) {
        path_point.flag = 1;
        inflection_indexs.push_back(i);
      }
    }
    path_points.push_back(path_point);
  }

  PathPoint tmp_path_point;
  if (first_turn_flag && line_turn_index != -1) {
    tmp_path_point = path_points[line_turn_index];
  } else if (r_max_index != -1) {
    tmp_path_point = path_points[r_max_index];
  }
  if (fabs(tmp_path_point.r + 1) < 0.001) {
    AWARN << "Not exist raduis.";
    *final_line = origin_line;
  } else if (tmp_path_point.r > 0.1) {
    std::vector<cv::Point> tmp_patch_points;
    if (!DoubleCircleMethodPlan(tmp_path_point, obstacle_img,
                                &tmp_patch_points)) {
      AWARN << "Do not plan use double circle method.";
    }

    // cv::Point tmp_point = tmp_path_point.point;
    // // if(tmp_point.x == 0)
    // for (size_t i = 0; i < tmp_point.x; ++i) {
    //   cv::Point point;
    //   point.y = 1000 - i;
    //   point.x = 500 - i * ((tmp_point.y * 1.0) / tmp_point.x);
    //   tmp_patch_points.push_back(point);
    // }

    std::vector<cv::Point>::const_iterator first =
        origin_line.begin() + line_turn_index;
    std::vector<cv::Point>::const_iterator last = origin_line.end();
    *patch_line = tmp_patch_points;
    tmp_patch_points.insert(tmp_patch_points.end(), first, last);
    *final_line = tmp_patch_points;
    // cv::circle(*result_img_, origin_line[i], 10, cv::Scalar(255, 0, 0), -1,
    // 5);
  }

  /*
   bool obstacle_turn_flag = false;
   int line_index_max = INT_MIN;
   int path_index = 0;

   if (!inflection_indexs.empty()) {
     std::vector<PathPoint> line_points;
     std::vector<std::vector<PathPoint>> obstacle_points;
     PathPoint last_point;
     bool have_last_point_flag = false;
     for (size_t i = 0; i < inflection_indexs.size(); ++i) {
       int index = inflection_indexs[i];
       PathPoint tmp = path_points[index];
       cv::Point current;
       current.x = 1000 - tmp.point.y;
       current.y = 500 - tmp.point.x;
       double distance = sqrt(pow(current.x, 2) + pow(current.y, 2));
       if (distance > turn_obstacle_radius_ + 2 * turn_obstacle_radius_)
         continue;

       if (reference_img.at<uchar>(tmp.point.y, tmp.point.x) == 255) {
         line_index_max = std::max(line_index_max, tmp.index);
       } else {
         if (!have_last_point_flag) {
           have_last_point_flag = true;
           last_point = tmp;
           line_points.push_back(tmp);
           continue;
         }
         int delta_x = last_point.point.x - tmp.point.x;
         int delta_y = last_point.point.y - tmp.point.y;
         double distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
         if (distance < 60) {
           line_points.push_back(tmp);
         } else {
           if (!line_points.empty()) obstacle_points.push_back(line_points);
           line_points.clear();
         }
         last_point = tmp;
       }
     }

     if (!line_points.empty()) obstacle_points.push_back(line_points);

     int obstacle_index_min = INT_MAX;
     for (size_t i = 0; i < obstacle_points.size(); ++i) {
       PathPoint tmp_point = obstacle_points[i].back();
       int index = tmp_point.index;
       obstacle_index_min = std::min(obstacle_index_min, index);
     }
     if (obstacle_index_min != INT_MAX) {
       obstacle_turn_flag = true;
       path_index = obstacle_index_min;
     }
     // if (line_index_max != INT_MIN && obstacle_index_min != INT_MAX) {
     //   obstacle_turn_flag = true;
     //   path_index = std::max(line_index_max, obstacle_index_min);
     // } else if (line_index_max != INT_MIN && obstacle_index_min ==
     // INT_MAX) {
     //   obstacle_turn_flag = true;
     //   path_index = line_index_max;
     // } else if (line_index_max == INT_MIN && obstacle_index_min !=
     // INT_MAX) {
     //   obstacle_turn_flag = true;
     //   path_index = obstacle_index_min;
     // }
     cv::circle(*result_img_, origin_line[path_index], 10, cv::Scalar(0, 0,
   255), -1, 5);
   }

  // if (first_turn_flag && !obstacle_turn_flag) {
  //   path_index = line_turn_index;
  // }

  // 暂时不考虑用双圆法避障
  // path_index = std::max(path_index, line_turn_index);

  path_index = line_turn_index;

  int intersect_flag = -1;
  int goal_index = 0;
  bool flag_goal_turn_point = false;
  std::vector<cv::Point> patch_points;
  double slop_angle;
  for (; path_index < line_size; ++path_index) {
    switch (path_points[path_index].flag) {
      case 0: {
        slop_angle = path_points[path_index].slop_angle;
        if (!obstacle_turn_flag) {
          PathPoint goal_turn_point;
          CorrectPointSlop(interval_num, path_index, path_points,
                           &goal_turn_point);
          slop_angle = goal_turn_point.slop_angle;
          if (DoubleCircleMethodPlan(goal_turn_point, obstacle_img,
                                     &patch_points)) {
            intersect_flag = 0;
            goal_index = path_points[path_index].index;
            flag_goal_turn_point = true;
            break;
          } else {
            patch_points.clear();
            intersect_flag = 1;
          }
        }
      } break;
      case 1: {
        slop_angle = path_points[path_index].slop_angle;
        PathPoint goal_turn_point;
        CorrectPointSlop(interval_num, path_index, path_points,
                         &goal_turn_point);
        slop_angle = goal_turn_point.slop_angle;
        if (DoubleCircleMethodPlan(goal_turn_point, obstacle_img,
                                   &patch_points)) {
          intersect_flag = 0;
          goal_index = path_points[path_index].index;
          flag_goal_turn_point = true;
          break;
        } else {
          patch_points.clear();
          intersect_flag = 1;
        }
      } break;
      default:
        break;
    }
    if (flag_goal_turn_point) break;
    if (intersect_flag == 1) {
      PathPoint goal_turn_point;
      goal_turn_point.point = path_points[path_index].point;
      goal_turn_point.slop_angle = slop_angle;
      CalculateRadiusAndLocation(&goal_turn_point);
      if (DoubleCircleMethodPlan(goal_turn_point, obstacle_img,
                                 &patch_points)) {
        flag_goal_turn_point = true;
        goal_index = path_index;
        break;
      } else {
        patch_points.clear();
        flag_goal_turn_point = false;
        continue;
      }
    }
  }

  if (flag_goal_turn_point && origin_line.size() > goal_index &&
      goal_index != 0) {
    last_line_times_ = 0;
    std::vector<cv::Point>::const_iterator first =
        origin_line.begin() + goal_index;
    std::vector<cv::Point>::const_iterator last = origin_line.end();
    *patch_line = patch_points;
    patch_points.insert(patch_points.end(), first, last);
    *final_line = patch_points;
    ADEBUG << "Caculate the path point success.";
  } else {
    char name_buf[50];
    last_line_times_++;
    if (last_line_times_ <= 5) {
      if (!last_line_.empty()) *final_line = last_line_;
    } else {
      snprintf(name_buf, sizeof(name_buf), "NULL:%d", last_line_times_);
      cv::putText(*result_img_, name_buf, cv::Point(500, 500),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(100, 255, 255), 1,
  8); ADEBUG << "Use double circle method planning,not plan out trajectory";
      return false;
    }
  }
  last_line_ = *final_line;*/
  return true;
}

double DoubleCircleMethod::CalculateCurvity(const cv::Point &last,
                                            const cv::Point &current,
                                            const cv::Point &next) {
  double curvity;
  if (last.x == current.x == next.x || last.y == current.y == next.y) {
    curvity = 0;
  } else {
    double dis1, dis2, dis3;
    double cosA, sinA, dis;
    // 求三角形三条边长
    dis1 = sqrt(((last.x - current.x) * 0.01) * ((last.x - current.x) * 0.01) +
                ((last.y - current.y) * 0.01) * ((last.y - current.y) * 0.01));

    dis2 = sqrt(((last.x - next.x) * 0.01) * ((last.x - next.x) * 0.01) +
                ((last.y - next.y) * 0.01) * ((last.y - next.y) * 0.01));
    dis3 = sqrt(((current.x - next.x) * 0.01) * ((current.x - next.x) * 0.01) +
                ((current.y - next.y) * 0.01) * ((current.y - next.y) * 0.01));
    // 余弦定理求角度
    dis = dis1 * dis1 + dis3 * dis3 - dis2 * dis2;
    cosA = dis / (2 * dis1 * dis3);
    // 求正弦
    sinA = sqrt(1 - cosA * cosA);
    // 正弦定理求外接圆半径
    curvity = 0.5 * dis2 / sinA;
    // 半径的倒数是曲率，半径越小曲率越大
    curvity = 1 / curvity;
  }
  return curvity;
}

void DoubleCircleMethod::CalculateRadiusAndLocation(PathPoint *path_point) {
  cv::Point current_point;
  current_point.x = center_.y - path_point->point.y;
  current_point.y = center_.x - path_point->point.x;
  double theta = path_point->slop_angle;
  double a, b, c, r0, r1;

  a = 2 * (cos(theta) - 1);
  if (current_point.y <= -0.0001) {
    path_point->location = 1;
    b = 2 * (1 + cos(theta)) * current_point.y -
        2 * sin(theta) * current_point.x;
  } else if (current_point.y >= 0.0001) {
    path_point->location = 0;
    b = 2 * sin(theta) * current_point.x -
        2 * (1 + cos(theta)) * current_point.y;
  } else {
    if (theta >= 0.0001 && theta <= PI) {
      path_point->location = 1;
      b = -2 * sin(theta) * current_point.x;
    } else if (theta <= -0.0001 && theta >= -PI) {
      path_point->location = 0;
      b = 2 * sin(theta) * current_point.x;
    } else {
      path_point->location = 2;
      path_point->r = -1;
      return;
    }
  }
  c = pow(current_point.x, 2) + pow(current_point.y, 2);
  if (theta > -0.0001 && theta < 0.0001) {
    if (b > -0.000001 && b < 0.000001) {
      AWARN << "Not exist raduis!";
      path_point->r = -1;
      return;
    } else {
      path_point->r = -c / b;
    }
  } else if ((b * b - 4 * a * c) > -0.000001) {
    r0 = (-b + std::sqrt(b * b - 4 * a * c)) / (2 * a);
    r1 = (-b - std::sqrt(b * b - 4 * a * c)) / (2 * a);
    if (r0 > 0.001) {
      path_point->r = r0;
    } else if (r1 > 0.001) {
      path_point->r = r1;
    } else if (r0 < -0.0001 && r1 < -0.00001) {
      AWARN << "Not exist raduis!";
      path_point->r = -1;
      return;
    }
  } else {
    AWARN << "Not exist raduis!";
    path_point->r = -1;
    return;
  }
}

void DoubleCircleMethod::CorrectPointSlop(
    const int interval_num, const int path_index,
    const std::vector<PathPoint> &path_points, PathPoint *point) {
  *point = path_points[path_index];
  int tmp_index = 0;
  int line_size = static_cast<int>(path_points.size());
  if (path_index + 1 * interval_num < line_size) {
    for (int j = 0; j < 2; ++j) {
      int current_index = path_index + j * interval_num;
      int next_index = path_index + (j + 1) * interval_num;
      double a1 = path_points[current_index].slop_angle;
      double a2 = path_points[next_index].slop_angle;
      if (fabs(a1 - a2) * RAD_TO_DEGREE > 2.0) {
        tmp_index = next_index;
        break;
      }
    }
    if (tmp_index != 0) {
      point->slop_angle = path_points[tmp_index].slop_angle;
      CalculateRadiusAndLocation(point);
    }
  }
}

bool DoubleCircleMethod::DoubleCircleMethodPlan(
    const PathPoint &current_point, const cv::Mat &obstacle_img,
    std::vector<cv::Point> *patch_points) {
  cv::Point p0, p1, p2;
  int location_flag = current_point.location;
  double r = current_point.r;
  double theta = current_point.slop_angle;
  if (location_flag == 1) {
    p0 = cv::Point(0, -r);
    p1.x = center_.y - current_point.point.y - r * sin(theta);
    p1.y = center_.x - current_point.point.x + r * cos(theta);
  } else if (location_flag == 0) {
    p0 = cv::Point(0, r);
    p1.x = center_.y - current_point.point.y + r * sin(theta);
    p1.y = center_.x - current_point.point.x - r * cos(theta);
  } else if (location_flag == 2) {
    int num = center_.y - current_point.point.y;
    for (size_t i = 0; i < num; ++i) {
      if (i % 20 != 0) continue;
      cv::Point tmp(center_.x, center_.y - i);
    }
    return true;
  }
  p2.x = (p0.x + p1.x) / 2;
  p2.y = (p0.y + p1.y) / 2;

  double c_slop = atan2(p1.y - p0.y, p1.x - p0.x);

  int start;
  double t_slop, theta_0, theta_1;
  if (location_flag == 0) {
    t_slop = c_slop + PI / 2;
    theta_1 = (PI - theta) * RAD_TO_DEGREE;
    start = (PI - t_slop) * RAD_TO_DEGREE;
    if (start >= theta_1) {
      // return false;
    }
  } else if (location_flag == 1) {
    t_slop = c_slop - PI / 2;
    theta_1 = (PI + theta) * RAD_TO_DEGREE;
    start = (PI + t_slop) * RAD_TO_DEGREE;
    if (start >= theta_1) {
      // return false;
    }
  }

  theta_0 = fabs(t_slop * RAD_TO_DEGREE);
  if (theta_0 < 0.1 && theta_0 > -0.1) {
    theta_0 = 180;
  }

  int delta_x_num_0 = abs(p2.x - p0.x);
  double delta_theta_0 = theta_0 / delta_x_num_0;
  for (int k = 0; k < delta_x_num_0; ++k) {
    double temp_angle = k * delta_theta_0 * DEGREE_TO_RAD;
    double x, y;
    if (location_flag == 0) {
      x = p0.x + r * sin(temp_angle);
      y = p0.y - r * cos(temp_angle);
    } else if (location_flag == 1) {
      x = p0.x + r * sin(temp_angle);
      y = p0.y + r * cos(temp_angle);
    }
    cv::Point tmp;
    tmp.x = center_.x - round(y);
    tmp.y = center_.y - round(x);
    if (tmp.x < 0 || tmp.x > width_ || tmp.y < 0 || tmp.y > height_) continue;
    // if (obstacle_img.at<uchar>(tmp.y, tmp.x) == 255) {
    //   patch_points->clear();
    //   return false;
    // }
    patch_points->push_back(tmp);
  }

  int delta_x_num_1 = abs(p2.x - p1.x);
  double delta_theta_1 = fabs(theta_1 - start) / delta_x_num_1;
  for (int k = 0; k < delta_x_num_1; ++k) {
    double temp_angle = (start + k * delta_theta_1) * DEGREE_TO_RAD;
    double x, y;
    if (location_flag == 0) {
      x = p1.x - r * sin(temp_angle);
      y = p1.y - r * cos(temp_angle);
    } else if (location_flag == 1) {
      x = p1.x - r * sin(temp_angle);
      y = p1.y + r * cos(temp_angle);
    } else {
      AWARN << "Location flag: " << location_flag;
    }

    cv::Point tmp;
    tmp.x = center_.x - round(y);
    tmp.y = center_.y - round(x);
    if (tmp.x < 0 || tmp.x > width_ || tmp.y < 0 || tmp.y > height_) continue;
    // if (obstacle_img.at<uchar>(tmp.y, tmp.x) == 255) {
    //   patch_points->clear();
    //   return false;
    // }
    patch_points->push_back(tmp);
  }

  cv::Point tmp_point(current_point.point.x, current_point.point.y);
  cv::Point p2_1(center_.x - p2.y, center_.y - p2.x);
  // cv::circle(*result_img_, p2_1, 2, cv::Scalar(0, 255, 0), -1, 8);
  char name_buf[50];
  snprintf(name_buf, sizeof(name_buf), "(r: %.2f)", r);
  cv::putText(*result_img_, name_buf, tmp_point, cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(0, 0, 255), 2, 8);
  return true;
}

bool DoubleCircleMethod::IsIntersectBoundary(const std::vector<cv::Point> &line,
                                             const cv::Mat boundary_img) {
  for (size_t i = 0; i < line.size(); ++i) {
    int x = line[i].x;
    int y = line[i].y;
    if (y < 300) continue;
    if (boundary_img.at<uchar>(y, x) == 255) {
      return true;
      break;
    }
  }
  return false;
}

void DoubleCircleMethod::ExpandRotatedRect(
    const int x, const int y, const std::vector<cv::Point> &contour,
    cv::Mat *img) {
  cv::RotatedRect box = cv::minAreaRect(cv::Mat(contour));
  cv::Point2f p[4], expand_rect_points[4];
  box.points(p);
  std::vector<LinePara> line_func;
  line_func.resize(4);
  if (box.angle == -90) {
    expand_rect_points[0].x = p[0].x + x;
    expand_rect_points[0].y = p[0].y + y;
    expand_rect_points[1].x = p[1].x - x;
    expand_rect_points[1].y = p[1].y + y;
    expand_rect_points[2].x = p[2].x - x;
    expand_rect_points[2].y = p[2].y - y;
    expand_rect_points[3].x = p[3].x + x;
    expand_rect_points[3].y = p[3].y - y;
  } else if (box.angle == 0) {
    expand_rect_points[0].x = p[0].x - x;
    expand_rect_points[0].y = p[0].y + y;
    expand_rect_points[1].x = p[1].x - x;
    expand_rect_points[1].y = p[1].y - y;
    expand_rect_points[2].x = p[2].x + x;
    expand_rect_points[2].y = p[2].y - y;
    expand_rect_points[3].x = p[3].x + x;
    expand_rect_points[3].y = p[3].y + y;
  } else {
    line_func[0].k = tan(box.angle * DEGREE_TO_RAD);
    line_func[1].k = -1.0 / line_func[0].k;
    line_func[2].k = line_func[0].k;
    line_func[3].k = line_func[1].k;
    double angle = -box.angle * DEGREE_TO_RAD;

    if (p[1].y > p[3].y ||
        (p[1].y == p[3].y && box.size.width >= box.size.height)) {
      line_func[0].b =
          p[0].y + x * cos(angle) - line_func[0].k * (p[0].x + x * sin(angle));
      line_func[1].b =
          p[1].y + y * sin(angle) - line_func[1].k * (p[1].x - y * cos(angle));
      line_func[2].b =
          p[2].y - x * cos(angle) - line_func[2].k * (p[2].x - x * sin(angle));
      line_func[3].b =
          p[3].y - y * sin(angle) - line_func[3].k * (p[3].x + y * cos(angle));
    } else if (p[1].y < p[3].y ||
               (p[1].y == p[3].y && box.size.width < box.size.height)) {
      line_func[0].b =
          p[0].y + y * cos(angle) - line_func[0].k * (p[0].x + y * sin(angle));
      line_func[1].b =
          p[1].y + x * sin(angle) - line_func[1].k * (p[1].x - x * cos(angle));
      line_func[2].b =
          p[2].y - y * cos(angle) - line_func[2].k * (p[2].x - y * sin(angle));
      line_func[3].b =
          p[3].y - x * sin(angle) - line_func[3].k * (p[3].x + x * cos(angle));
    }

    for (int i = 0; i < 4; ++i) {
      expand_rect_points[i].x = (line_func[i].b - line_func[(i + 1) % 4].b) /
                                (line_func[(i + 1) % 4].k - line_func[i].k);
      expand_rect_points[i].y =
          line_func[i].k * expand_rect_points[i].x + line_func[i].b;
    }
  }
  for (int k = 0; k < 4; ++k) {
    cv::line(*img, expand_rect_points[k], expand_rect_points[(k + 1) % 4],
             cv::Scalar(255), 1, 8, 0);
    // cv::line(*result_img_, expand_rect_points[k],
    //          expand_rect_points[(k + 1) % 4], cv::Scalar(255, 0, 0), 3, 8,
    //          0);
  }
}

}  // namespace navigation
}  // namespace sweeper
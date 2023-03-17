#include "planning.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace sweeper {
namespace navigation {

Planning::Planning(ros::NodeHandle &node, ros::NodeHandle &private_nh)
    : mode_(0),
      last_mode_(-1),
      current_type_(-1),
      pub_count_(0),
      pause_(0),
      default_x_(500),
      obstacle_count_(0),
      no_obstacle_count_(0),
      obstacle_expand_width_(0),
      speed_(0.0),
      area_thre_(500),
      last_time_(-1),
      straight_distance_(0.0),
      last_dis_min_x_(FLT_MAX),
      last_dis_min_y_(FLT_MAX),
      last_road_type_(-1),
      current_road_type_(-1),
      estop_(false),
      traffic_light_stop_(false),
      dis_mutation_flag_(false),
      avoid_obstacle_flag_(false),
      find_road_edge_flag_(true),
      lidar_trace_flag_(false),
      only_trace_flag_(false),
      last_have_obstacle_flag_(false),
      trace_good_flag_(false),
      result_img_(cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255))),
      initialized_(false) {
  std::string lidar_topic;
  double filter_min_dis, filter_slope;
  private_nh.param<int>("start_row", start_row_, 0);
  private_nh.param<int>("end_row", end_row_, 333);
  private_nh.param<int>("fit_mode", fit_mode_, 1);
  private_nh.param<int>("order", order_, 3);
  private_nh.param<int>("height_img_dilate", height_img_dilate_, 40);
  private_nh.param<int>("sparse_factor", sparse_factor_, 40);
  private_nh.param<int>("curb_sweep_mode", curb_sweep_mode_, 1);
  private_nh.param<int>("show_img_debug", show_img_debug_, 0);
  private_nh.param<int>("left_trans_thre", left_trans_thre_, 20);
  private_nh.param<int>("right_trans_thre", right_trans_thre_, 20);
  private_nh.param<int>("height", height_, 1000);
  private_nh.param<int>("width", width_, 1000);

  private_nh.param<double>("obj_expand_y", obj_expand_y_, 7.2);
  private_nh.param<double>("obstacle_x", obstacle_x_, 8.0);
  private_nh.param<double>("obstacle_y", obstacle_y_, 5.0);
  private_nh.param<double>("erode_x", erode_x_, 0.2);
  private_nh.param<double>("erode_y", erode_y_, 0.6);
  private_nh.param<double>("e_stop_x", e_stop_x_, 1.2);
  private_nh.param<double>("e_stop_y", e_stop_y_, 0.4);
  private_nh.param<double>("car_width", car_width_, 0.8);
  private_nh.param<double>("car_length", car_length_, 1.2);
  private_nh.param<double>("sweep_width", sweep_width_, 1.8);
  private_nh.param<double>("wj_height", wj_height_, 1.5);
  private_nh.param<double>("resolution", resolution_, 0.02);
  private_nh.param<double>("road_edge_thickness", road_edge_thickness_, 0.2);
  private_nh.param<double>("road_edge_thre", road_edge_thre_, 0.3);
  private_nh.param<double>("low_obstacle_thre", low_obstacle_thre_, 0.18);
  private_nh.param<double>("high_obstacle_thre", high_obstacle_thre_, 1.0);
  private_nh.param<double>("abs_ground_height", abs_ground_height_, 1.32);
  private_nh.param<double>("straight_distance_thre", straight_distance_thre_,
                           1.0);
  private_nh.param<double>("slope_angle", slope_angle_, 60);
  private_nh.param<double>("filter_slop_thre", filter_slop_thre_, 60);
  private_nh.param<double>("delta_dis_min_x", delta_dis_min_x_, 50);
  private_nh.param<double>("delta_dis_min_y", delta_dis_min_y_, 50);
  private_nh.param<double>("front_dis", front_dis_, 10);

  private_nh.param<string>("lidar_topic", lidar_topic,
                           std::string("/livox/lidar"));

  private_nh.param<double>("filter_min_dis", filter_min_dis, 1.4);
  private_nh.param<double>("filter_slope", filter_slope, 2.0);
  result_img_ = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(255, 255, 255));
  center_.x = round(width_ / 2.0);
  center_.y = round(front_dis_ / resolution_);
  default_x_ = center_.x;
  area_thre_ = (erode_x_ / resolution_) * (erode_y_ / resolution_) + 100;

  object_tracker_ = new ObjectTracker();
  object_tracker_->Init();

  road_edge_detect_ =
      new RoadEdgeDetect(private_nh, filter_min_dis, filter_slope, center_);
  double_circle_method_ = new DoubleCircleMethod(
      private_nh, car_width_, e_stop_x_, e_stop_y_, road_edge_detect_);

  global_path_sub_ = node.subscribe("/sweeper/navigation/reference/line", 1,
                                    &Planning::ReceiveGlobalPath, this);
  chassis_sub_ = node.subscribe("/sweeper/chassis/detail", 1,
                                &Planning::ChassisCallback, this);
  obu_sub_ =
      node.subscribe("/sweeper/sensor/obu", 1, &Planning::ObuCallback, this);
  point_cloud_sub_ =
      node.subscribe(lidar_topic, 1, &Planning::CloudCallback, this);
  sweep_mission_sub_ = node.subscribe("/sweeper/sweep_mode", 1,
                                      &Planning::SweepModeCallback, this);

  image_transport::ImageTransport it(node);
  planning_img_pub_ = it.advertise("/sweeper/planning/image", 1);
  path_pub_ = node.advertise<nav_msgs::Path>("path", 10);
  state_code_pub_ = node.advertise<sweeper_msgs::SensorFaultInformation>(
      "/sweeper/common/diagnose", 1);

  livox_ground_height_ = abs_ground_height_;

  // std::thread t_run(&Planning::Run, this);
  // t_run.detach();

  plan_timer =
      node.createTimer(ros::Duration(0.1), &Planning::TimerCallback, this);
  initialized_ = true;
}

Planning::~Planning() {
  delete road_edge_detect_;
  delete object_tracker_;
  delete double_circle_method_;
}

bool Planning::Init() {
  slope_ = tan(slope_angle_ * DEGREE_TO_RAD);
  return initialized_;
}

void Planning::Run() {}

void Planning::Reset() {
  mode_ = 0;
  last_mode_ = -1;
  current_type_ = -1;
  pause_ = 0;
  default_x_ = center_.x;
  obstacle_count_ = 0;
  no_obstacle_count_ = 0;
  last_road_type_ = -1;
  current_road_type_ = -1;
  last_time_ = -1.0;
  last_dis_min_x_ = FLT_MAX;
  last_dis_min_y_ = FLT_MAX;
  straight_distance_ = 0.0;
  estop_ = false;
  traffic_light_stop_ = false;
  lidar_trace_flag_ = false;
  find_road_edge_flag_ = true;
  dis_mutation_flag_ = false;
  only_trace_flag_ = false;
  last_have_obstacle_flag_ = false;
  trace_good_flag_ = true;
  road_edge_boundary_.clear();
  img_path_points_center_.clear();
  img_road_edge_points_.clear();
  track_objects_.clear();
  result_img_ = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(255, 255, 255));
  double_circle_method_->Reset();
}

void Planning::SweepModeCallback(
    const sweeper_msgs::SweepMission &sweep_mission) {
  watchdog_sweep_mode_.UpdataNow();

  // // 获取左右贴边模式
  // if (sweep_mission.side == 0)
  //   curb_sweep_mode_ = 1;  // 右边清扫
  // else if (sweep_mission.side == 1)
  //   curb_sweep_mode_ = 0;  // 左边清扫

  // 判断是否为激光寻迹，激光寻迹模式下不进行避障
  if (sweep_mission.mode == 5)
    lidar_trace_flag_ = true;
  else
    lidar_trace_flag_ = false;

  // 获取贴边精度参数
  road_edge_thickness_ = sweep_mission.trim_accuracy * 2 * 0.01 - car_width_;

  sweep_frame_id_ = sweep_mission.header.frame_id;
  // 获取清扫模式相关信息
  if (sweep_mission.mode == 4 || sweep_mission.mode == 5) {
    if (sweep_mission.start_cmd == 1) {
      if (sweep_mission.header.frame_id == "curb_driving_state") {
        pause_ = 0;
        ADEBUG << "Swith trace mode to curb mode.";
      } else if (sweep_mission.header.frame_id == "gnss_trace_state") {
        pause_ = 0;
      } else if (sweep_mission.header.frame_id == "manual_driving_state") {
        pause_ = 2;
        Reset();
      } else if (sweep_mission.header.frame_id == "lidar_trace_state") {
        pause_ = 0;
        current_type_ = 0;
      }
    } else if (sweep_mission.start_cmd == 0) {
      Reset();
    } else if (sweep_mission.start_cmd == 2) {
      pause_ = 1;
      Reset();
    }
  } else if (sweep_mission.mode == 7) {
    ADEBUG << "Curb mode!";
    // 获取左右贴边模式
    if (sweep_mission.side == 0)
      curb_sweep_mode_ = 1;  // 右边清扫
    else if (sweep_mission.side == 1)
      curb_sweep_mode_ = 0;  // 左边清扫

    if (sweep_mission.start_cmd == 1) {
      pause_ = 0;
      mode_ = 2;
      if (curb_sweep_mode_ == 0)
        current_type_ = 3;
      else if (curb_sweep_mode_ == 1) {
        current_type_ = 2;
      }

    } else if (sweep_mission.start_cmd == 0)
      Reset();
    else if (sweep_mission.start_cmd == 2)
      pause_ = 1;
  } else
    Reset();
}

void Planning::TimerCallback(const ros::TimerEvent &event) {
  if (++pub_count_ % UINT_MAX == 0) pub_count_ = 0;
  state_code_.header.frame_id = "planning";
  state_code_.state_code.clear();
  if (!watchdog_cloud_.DogIsOk(3) && mode_ != 0) {
    state_code_.state_code.push_back(3201);
    ADEBUG << "In curb sweep mode,not recieve point cloud.";
  }
  if (!watchdog_path_.DogIsOk(3) && mode_ == 1) {
    ADEBUG << "In trace mode, not recieve global path.";
    state_code_.state_code.push_back(3202);
  }
  if (!watchdog_obu_.DogIsOk(5)) {
    ADEBUG << "Not recieve obu msg.";
    sweeper_msgs::Obu obu_null;
    obu_msg_ = obu_null;
  }

  if (!watchdog_sweep_mode_.DogIsOk(5) && mode_ != 0) {
    AWARN << "Not recieve sweep mode code.";
    mode_ = 0;
  }

  // 获取本地规划路径
  GetLocalPath();

  if (state_code_.state_code.empty()) {
    state_code_.state_code.push_back(3200);
  }
  state_code_.header.stamp = ros::Time::now();
  if (pub_count_ % 2 == 0) state_code_pub_.publish(state_code_);
}

void Planning::GetLocalPath() {
  double start = ros::Time::now().toSec();
  // AWARN << "Mode: " << mode_;
  // estop_ = false;
  std::vector<cv::Point> final_path;
  if (mode_ != 0) {
    std::vector<cv::Point> obstacle_line;

    // 获取规划路径参数
    PlanParam plan_param;
    GetPlanParam(&plan_param);

    // 获取规划轨迹，此规划轨迹在参考线基础上考虑了以下几个因素获得
    // 1. 考虑了障碍物的边界，速度等信息；
    // 2. 考虑了轨迹的运动学模型信息；
    // 3. 考虑了在避障时障碍物的感知范围；
    if (mode_ == 1 || (mode_ == 2 && !dis_mutation_flag_)) {
      double_circle_method_->GetPlanningTrajectory(
          plan_param, &estop_, &result_img_, &obstacle_line, &state_code_);

      // 对轨迹点进行过滤
      std::vector<cv::Point> filter_line;
      if (mode_ == 2)
        FilterLine(obstacle_line, &filter_line);
      else
        filter_line = obstacle_line;

      // 对轨迹点进行拟合
      GetImgGlobalPath(filter_line, &final_path);

      // 对轨迹点进行曲线拟合
      // PolyFit(order_, sparse_line, &final_path);
    }

    if (mode_ == 2 && dis_mutation_flag_) {
      final_path = img_road_edge_points_;
    }

    // 判断是否触发急停标志，触发则急停
    if (estop_) {
      final_path.clear();
      if (fabs(speed_) < 0.1) state_code_.state_code.push_back(3207);
    }

    //红绿灯停车
    if (mode_ == 1 && traffic_light_stop_) {
      AWARN << "Traffic light stop.";
      final_path.clear();
      if (fabs(speed_) < 0.1) state_code_.state_code.push_back(3211);
    }

    // 在马路沿模式，一定范围内没有寻找到马路沿
    if (!find_road_edge_flag_ && mode_ == 2) {
      final_path.clear();
      if (fabs(speed_) < 0.1) state_code_.state_code.push_back(3208);
    }

    if (pause_ != 0) {
      final_path.clear();
    }

    for (size_t i = 0; i < final_path.size(); ++i) {
      cv::Point tmp = final_path[i];
      cv::circle(result_img_, tmp, 2, cv::Scalar(0, 255, 0), -1, 5);
    }

    if (mode_ == 1)
      cv::polylines(result_img_, img_path_points_center_, false,
                    cv::Scalar(200, 100, 0), 1, 8, 0);
    else if (mode_ == 2) {
      cv::polylines(result_img_, img_road_edge_points_, false,
                    cv::Scalar(100, 100, 0), 1, 8, 0);
    }

    // 在图像标记紧急避障区域
    cv::rectangle(result_img_,
                  cv::Point(center_.x - e_stop_y_ / resolution_,
                            center_.y - e_stop_x_ / resolution_),
                  cv::Point(center_.x + e_stop_y_ / resolution_, center_.y),
                  cv::Scalar(0, 0, 255), 1, cv::LINE_8, 0);
  }

  std::string text;
  if (mode_ == 0)
    text = "Null mode";
  else if (mode_ == 1)
    text = "Trace mode";
  else if (mode_ == 2)
    text = "Curb mode";

  int font_face = cv::FONT_HERSHEY_COMPLEX;
  int baseline;
  cv::Size text_size = cv::getTextSize(text, font_face, 0.5, 0.5, &baseline);
  cv::Point origin(0, text_size.height);
  cv::putText(result_img_, text, origin, font_face, 1, cv::Scalar(0, 255, 0), 1,
              8, 0);

  if (estop_) {
    char name_buf[50];
    snprintf(name_buf, sizeof(name_buf), "estop");
    cv::putText(result_img_, name_buf, cv::Point(width_ / 2, height_ / 2),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1, 8);
  }

  if (traffic_light_stop_ && mode_ == 1) {
    char name_buf[50];
    snprintf(name_buf, sizeof(name_buf), "traffic light stop");
    cv::putText(result_img_, name_buf, cv::Point(width_ / 2, height_ / 2),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1, 8);
  }

  if (!find_road_edge_flag_ && mode_ == 2) {
    char name_buf[50];
    snprintf(name_buf, sizeof(name_buf), "not road edge");
    cv::putText(result_img_, name_buf, cv::Point(width_ / 2, height_ / 2),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1, 8);
  }

  if (pause_ == 1) {
    char name_buf[50];
    snprintf(name_buf, sizeof(name_buf), "Pause");
    cv::putText(result_img_, name_buf, cv::Point(width_ / 2, height_ / 2),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1, 8);
  } else if (pause_ == 2) {
    char name_buf[50];
    snprintf(name_buf, sizeof(name_buf), "Manual stop");
    cv::putText(result_img_, name_buf, cv::Point(width_ / 2, height_ / 2),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1, 8);
  }

  // 6. 将路径从图像坐标转换到车身坐标，并发布出去
  nav_msgs::Path path;
  // path.header.frame_id = "aft_mapped";
  if (sweep_frame_id_ == "lidar_trace_state") current_type_ = 0;
  if (obu_msg_.traffic_light.size() == 3) {
    if (obu_msg_.traffic_light[0].data_valid_flag && current_type_ == 9)
      current_type_ = 6;
    if (obu_msg_.traffic_light[1].data_valid_flag && current_type_ == 8)
      current_type_ = 6;
    if (obu_msg_.traffic_light[2].data_valid_flag && current_type_ == 10)
      current_type_ = 6;
  }

  path.header.frame_id = std::to_string(current_type_);
  path.header.stamp = ros::Time::now();
  std::string pose_frame_id;
  if (FindStateCode(3209))
    pose_frame_id = "left";
  else if (FindStateCode(3210))
    pose_frame_id = "right";
  else
    pose_frame_id = "normal";
  for (int i = 0; i < final_path.size(); i++) {
    geometry_msgs::PoseStamped poses;
    // poses.header.frame_id = "aft_mapped";
    poses.header.frame_id = pose_frame_id;
    poses.header.stamp = ros::Time::now();
    poses.pose.position.x = (center_.y - final_path[i].y) * resolution_;
    poses.pose.position.y = (center_.x - final_path[i].x) * resolution_;
    path.poses.push_back(poses);
  }
  path_pub_.publish(path);
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_img_).toImageMsg();
  msg->header.frame_id = "aft_mapped";
  planning_img_pub_.publish(msg);
  if (show_img_debug_ == 1) {
    cv::imshow("result_img", result_img_);
    cv::waitKey(1);
  }

  ADEBUG << "Total time: " << SET_PRECISION(ros::Time::now().toSec() - start);
}

void Planning::ReceiveGlobalPath(nav_msgs::Path global_path) {
  watchdog_path_.UpdataNow();
  img_path_points_center_.clear();
  trace_patch_path_.clear();
  std::vector<cv::Point> trace_path;
  int last_point_type = -1;
  int path_count = 0;
  int change_type_num = 0;
  int change_type = -1;
  int change_last_type = -1;
  double dis_min = FLT_MAX;
  bool type_is_changed = false;
  cv::Point change_point(-1, -1);
  cv::Point origin_point(INT_MAX, INT_MAX);
  for (size_t i = 0; i < global_path.poses.size(); ++i) {
    cv::Point2f global_point;
    global_point = cv::Point2f(global_path.poses[i].pose.position.x,
                               global_path.poses[i].pose.position.y);
    cv::Point point;
    point.y = static_cast<int>(round(center_.y - global_point.x / resolution_));
    point.x = static_cast<int>(round(center_.x - global_point.y / resolution_));
    if (global_point.x > 0.1 && global_point.x < 9.9 && global_point.y > -4.9 &&
        global_point.y < 4.9) {
      path_count++;
      double dis =
          sqrt(pow((point.x - center_.x), 2) + pow((point.y - center_.y), 2));
      if (dis < dis_min) {
        dis_min = dis;
        origin_point = point;
      }
      std::string frame_id = global_path.poses[i].header.frame_id;
      int point_type = atoi(frame_id.c_str());
      if (last_point_type != -1 && point_type != last_point_type &&
          !type_is_changed) {
        type_is_changed = true;
        change_point = point;
        change_type = point_type;
        change_last_type = last_point_type;
      }
      if (type_is_changed) {
        if (change_type == point_type)
          change_type_num++;
        else
          change_type_num = 0;
      }
      trace_path.push_back(point);
      last_point_type = point_type;
    }
  }

  if (type_is_changed && path_count != 0 && path_count < 10 &&
      change_type_num >= 4) {
    current_type_ = change_type;
  } else if (type_is_changed) {
    current_type_ = change_last_type;
  }
  if (!type_is_changed && path_count > 0) {
    current_type_ = last_point_type;
  } else if (!type_is_changed) {
    AWARN << "Invalid path.";
    return;
  }

  if (current_type_ == 2 || current_type_ == 3)
    mode_ = 2;
  else if (current_type_ != -1)
    mode_ = 1;
  else {
    AWARN << "Invalid path type.";
    return;
  }

  if (mode_ == 1) {
    if (current_type_ == 9)
      curb_sweep_mode_ = 0;
    else if (current_type_ == 10)
      curb_sweep_mode_ = 1;
  }

  if (mode_ == 2) {
    if (current_type_ == 2)
      curb_sweep_mode_ = 1;
    else if (current_type_ == 3)
      curb_sweep_mode_ = 0;
  }

  if (abs(origin_point.x - center_.x) < 0.5 / resolution_)
    trace_good_flag_ = true;
  else
    trace_good_flag_ = false;

  if (!trace_path.empty()) {
    switch (current_type_) {
      case NORMAL_TRACE_MODE:
        only_trace_flag_ = false;
        traffic_light_stop_ = false;
        break;
      case MANUAL_MODE:
        traffic_light_stop_ = false;
        break;
      case RIGHT_CURB_MODE:
        traffic_light_stop_ = false;
        break;
      case LEFT_CURB_MODE:
        traffic_light_stop_ = false;
        break;
      case U_TURN_MODE:
        traffic_light_stop_ = false;
        if (trace_good_flag_) only_trace_flag_ = true;
        break;
      case LOW_SPEED_MODE:
        traffic_light_stop_ = false;
        only_trace_flag_ = false;
        break;
      case HIGH_SPEED_MODE:
        if (trace_good_flag_) only_trace_flag_ = true;
        traffic_light_stop_ = false;
        break;
      case CROSSING_MODE: {
        if (trace_good_flag_) only_trace_flag_ = true;
        sweeper_msgs::Light light;
        if (change_type == CROSSING_LEFT_MODE) {
          light = obu_msg_.traffic_light[0];
          if (light.data_valid_flag && light.light_direction == 1) {
            if (light.light_color == 'R' || light.light_color == 'Y' ||
                (light.light_color == 'G' && light.countdown <= 2)) {
              if (change_point.x != -1 && change_point.y >= 8.5 / resolution_) {
                traffic_light_stop_ = true;
              } else
                traffic_light_stop_ = false;
            } else
              traffic_light_stop_ = false;
          }
        } else if (change_type == CROSSING_STRAIGHT_MODE) {
          light = obu_msg_.traffic_light[1];
          if (light.data_valid_flag && light.light_direction == 2) {
            if (light.light_color == 'R' || light.light_color == 'Y' ||
                (light.light_color == 'G' && light.countdown <= 2)) {
              if (change_point.x != -1 && change_point.y >= 8.5 / resolution_) {
                traffic_light_stop_ = true;
              } else
                traffic_light_stop_ = false;
            } else
              traffic_light_stop_ = false;
          }
        } else if (change_type == CROSSING_RIGHT_MODE) {
          light = obu_msg_.traffic_light[2];
          if (light.data_valid_flag && light.light_direction == 3) {
            if (light.light_color == 'R' || light.light_color == 'Y' ||
                (light.light_color == 'G' && light.countdown <= 2)) {
              if (change_point.x != -1 && change_point.y >= 8.5 / resolution_) {
                traffic_light_stop_ = true;
              } else
                traffic_light_stop_ = false;
            } else
              traffic_light_stop_ = false;
          }
        }
      } break;
      case CROSSING_STRAIGHT_MODE: {
        bool light_valid_flag = false;
        if (obu_msg_.traffic_light.empty()) traffic_light_stop_ = false;
        for (size_t i = 0; i < obu_msg_.traffic_light.size(); ++i) {
          sweeper_msgs::Light light = obu_msg_.traffic_light[1];
          if (light.data_valid_flag && light.light_direction == 2) {
            light_valid_flag = true;
            if (light.light_color == 'R' || light.light_color == 'Y') {
              traffic_light_stop_ = true;
            } else
              traffic_light_stop_ = false;
          }
        }
        if (!light_valid_flag)
          only_trace_flag_ = false;
        else
          only_trace_flag_ = true;
      } break;
      case CROSSING_LEFT_MODE: {
        bool light_valid_flag = false;
        if (obu_msg_.traffic_light.empty()) traffic_light_stop_ = false;
        for (size_t i = 0; i < obu_msg_.traffic_light.size(); ++i) {
          sweeper_msgs::Light light = obu_msg_.traffic_light[0];
          if (light.data_valid_flag && light.light_direction == 1) {
            light_valid_flag = true;
            if (light.light_color == 'R' || light.light_color == 'Y') {
              traffic_light_stop_ = true;
            } else
              traffic_light_stop_ = false;
          }
        }
        if (!light_valid_flag)
          only_trace_flag_ = false;
        else
          only_trace_flag_ = true;
      } break;
      case CROSSING_RIGHT_MODE: {
        bool light_valid_flag = false;
        if (obu_msg_.traffic_light.empty()) traffic_light_stop_ = false;
        for (size_t i = 0; i < obu_msg_.traffic_light.size(); ++i) {
          sweeper_msgs::Light light = obu_msg_.traffic_light[2];
          if (light.data_valid_flag && light.light_direction == 2) {
            light_valid_flag = true;
            if (light.light_color == 'R' || light.light_color == 'Y') {
              traffic_light_stop_ = true;
            } else
              traffic_light_stop_ = false;
          }
        }

        if (!light_valid_flag)
          only_trace_flag_ = false;
        else
          only_trace_flag_ = true;
      } break;
      default:
        break;
    }
    last_mode_ = current_type_;
  }

  cv::Mat trace_path_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
  cv::polylines(trace_path_img, trace_path, false, cv::Scalar(255), 1, 8, 0);
  ContoursVector trace_path_contour;
  cv::findContours(trace_path_img, trace_path_contour, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_NONE);
  int size = static_cast<int>(trace_path_contour.size());
  if (size == 1) {
    double near_dis_min = FLT_MAX;
    double far_dis_min = FLT_MAX;
    int near_index = 0;
    int far_index = 0;
    cv::Point start_point = trace_path.front();
    cv::Point end_point = trace_path.back();
    for (size_t i = 0; i < trace_path_contour[0].size(); ++i) {
      int x = trace_path_contour[0][i].x;
      int y = trace_path_contour[0][i].y;

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

    std::vector<cv::Point> left_line, right_line, temp_vec;
    bool left_flag = false, right_flag = false;
    for (int i = 0; i < trace_path_contour[0].size(); ++i) {
      cv::Point point = trace_path_contour[0][i];
      if (i == near_index) {
        left_flag = false;
        right_flag = true;
      }
      if (i == far_index) {
        left_flag = true;
        right_flag = false;
      }
      if (left_flag) left_line.push_back(point);
      if (right_flag) right_line.push_back(point);
      if (!left_flag && !right_flag) temp_vec.push_back(point);
    }

    if (near_index >= far_index) {
      std::reverse(left_line.begin(), left_line.end());
      right_line.insert(right_line.end(), temp_vec.begin(), temp_vec.end());
    } else {
      left_line.insert(left_line.end(), temp_vec.begin(), temp_vec.end());
      std::reverse(left_line.begin(), left_line.end());
    }
    img_path_points_center_ = left_line;
  }
}

void Planning::ObuCallback(const sweeper_msgs::Obu &obu) {
  watchdog_obu_.UpdataNow();
  obu_msg_ = obu;
}

void Planning::CloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  watchdog_cloud_.UpdataNow();
  if (mode_ == 0) return;
  cv::Mat height_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
  cv::Mat high_obstacle_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
  cv::Mat gray_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(255));
  cv::Mat road_edge_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));

  PointCloudPtr pcl_cloud(new PointCloud);
  PointCloudPtr correctioned_cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *pcl_cloud);
  double t1 = ros::Time::now().toSec();
  // Todo: 需要测试在寻迹模式下是否需要对点云进行矫正
  road_edge_detect_->CorrectionCloud(mode_, curb_sweep_mode_, pcl_cloud,
                                     correctioned_cloud, &abs_ground_height_);
  road_edge_detect_->PointcloudToImg(correctioned_cloud, only_trace_flag_,
                                     &height_img, &high_obstacle_img, &gray_img,
                                     &avoid_obstacle_flag_);
  road_edge_detect_->RemoveDiscretePoints(height_img, height_img, 8, 8);
  int erode_x = static_cast<int>(erode_x_ / resolution_);
  int erode_y = static_cast<int>(erode_y_ / resolution_);
  road_edge_detect_->DilateImg(height_img, height_img, erode_x, erode_y + 10);
  road_edge_detect_->DilateImg(high_obstacle_img, high_obstacle_img, erode_x,
                               erode_y);
  cv::Mat structure_element =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_x, erode_y));
  cv::erode(gray_img, gray_img, structure_element);
  road_edge_detect_->GetRoadEdgeImg(gray_img, road_edge_img);

  // 获取马路沿边界路径
  ContoursVector object_contours;
  if (mode_ == 1) {
    ContoursVector road_edge_contours;
    // cv::findContours(road_edge_img, road_edge_contours, CV_RETR_EXTERNAL,
    //                  CV_CHAIN_APPROX_NONE);
    // if (!only_trace_flag_)
    //   cv::drawContours(height_img, road_edge_contours, -1, cv::Scalar(255),
    //   1,
    //                    8);
    // cv::bitwise_or(high_obstacle_img, height_img, height_img);
    cv::cvtColor(height_img, result_img_, CV_GRAY2RGB);
    cv::findContours(height_img, object_contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_NONE);
  } else if (mode_ == 2) {
    ContoursVector other_contours;
    ContoursVector height_contours;
    ContoursVector high_contours;
    cv::bitwise_or(road_edge_img, height_img, road_edge_img);
    GetRoadEdgePath(road_edge_img, &other_contours);
    cv::cvtColor(road_edge_img, result_img_, CV_GRAY2RGB);
    // cv::bitwise_or(height_img, high_obstacle_img, height_img);
    if (!other_contours.empty())
      cv::drawContours(height_img, other_contours, -1, cv::Scalar(255), -1, 8);
    cv::findContours(height_img, object_contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_NONE);
  }
  // cv::cvtColor(road_edge_img, result_img_, CV_GRAY2RGB);
  // cv::imshow("test", height_img);
  // 对所有边界轮廓进行跟踪，获得速度信息
  ContoursTracker(*correctioned_cloud, object_contours);
  ADEBUG << "Point cloud callback time: "
         << SET_PRECISION(ros::Time::now().toSec() - t1);
}

void Planning::ContoursTracker(const PointCloud &cloud_in,
                               const ContoursVector &contours) {
  ContoursVector track_contours;
  cv::Mat road_edge_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
  if (!road_edge_boundary_.empty()) {
    cv::drawContours(road_edge_img, road_edge_boundary_, -1, cv::Scalar(255),
                     -1, 8);
  }

  std::vector<bool> inside_road_edge_vec;
  std::vector<cv::Point> dis_x_y_vec, bottom_left_vec, bottom_right_vec;
  std::vector<int> obj_width_vec;
  estop_ = false;
  for (size_t i = 0; i < contours.size(); ++i) {
    std::shared_ptr<Object> out_obj(new Object);
    int controur_area = cv::contourArea(contours[i]);
    if (controur_area < area_thre_) continue;
    bool y_flag = false;
    bool inside_road_edge_flag = false;
    std::vector<cv::Point> sparse_contour;
    double obstacle_dis_x = FLT_MAX, obstacle_dis_y = FLT_MAX;
    double bottom_left = FLT_MAX;
    double bottom_right = FLT_MAX;
    double bottom_left_dis_min = FLT_MAX;
    double bottom_right_dis_min = FLT_MAX;
    double distance_min = FLT_MAX;
    cv::Point bottom_left_point, bottom_right_point;
    for (size_t j = 0; j < contours[i].size(); ++j) {
      int x = contours[i][j].x;
      int y = contours[i][j].y;
      if (road_edge_img.at<uchar>(y, x) == 255) {
        inside_road_edge_flag = true;
      }

      double distance, bottom_left_dis, bottom_right_dis;
      distance = sqrt(pow(x - center_.x, 2) + pow(y - center_.y, 2));
      bottom_left_dis = sqrt(pow(x, 2) + pow(y - center_.y, 2));
      bottom_right_dis = sqrt(pow(x - width_, 2) + pow(y - center_.y, 2));
      if (distance < distance_min) {
        distance_min = distance;
        obstacle_dis_x = x;
        obstacle_dis_y = y;
      }
      if (bottom_left_dis < bottom_left_dis_min) {
        bottom_left_dis_min = bottom_left_dis;
        bottom_left_point.x = x;
        bottom_left_point.y = y;
      }

      if (bottom_right_dis < bottom_right_dis_min) {
        bottom_right_dis_min = bottom_right_dis;
        bottom_right_point.x = x;
        bottom_right_point.y = y;
      }

      if (j % 10 != 0) continue;
      sparse_contour.push_back(contours[i][j]);

      // 有障碍物在紧急停车范围内，发出estop指令
      if (fabs(x - center_.x) < (e_stop_y_ / resolution_) &&
          y > (center_.y - (e_stop_x_ / resolution_)) && y < center_.y) {
        estop_ = true;
        // AWARN << "The obstacle is existed in emergency area,Estop!";
        return;
      }

      if (y > (center_.y - obstacle_x_ / resolution_)) y_flag = true;
    }
    if (!y_flag) continue;
    int road_edge_width =
        (int)((car_width_ + road_edge_thickness_ + erode_x_) / resolution_);

    cv::Mat current_contour_img =
        cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
    int obstacle_expand_width;
    if (mode_ == 1) {
      if (current_type_ == 0)
        obstacle_expand_width = (car_width_ - 0.2) / resolution_;
      else
        obstacle_expand_width = (car_width_ - 0.2) / resolution_;
    } else if (mode_ == 2) {
      if (inside_road_edge_flag) {
        obstacle_expand_width =
            road_edge_width - (erode_x_ - 0.06) / resolution_;
      } else
        obstacle_expand_width =
            (sweep_width_ / resolution_ - road_edge_width / 2) * 2 -
            erode_x_ / resolution_;
    }
    if (obstacle_expand_width < 1) obstacle_expand_width = 1;
    if (!sparse_contour.empty())
      cv::polylines(current_contour_img, sparse_contour, true, cv::Scalar(255),
                    obstacle_expand_width, 8, 0);
    ContoursVector temp_contours;
    cv::findContours(current_contour_img, temp_contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_NONE);
    if (temp_contours.size() == 1) {
      track_contours.push_back(temp_contours[0]);
      inside_road_edge_vec.push_back(inside_road_edge_flag);
      dis_x_y_vec.push_back(cv::Point(obstacle_dis_x, obstacle_dis_y));
      bottom_left_vec.push_back(bottom_left_point);
      bottom_right_vec.push_back(bottom_right_point);
      obj_width_vec.push_back(obstacle_expand_width);
    }
  }

  // 对障碍物进行跟踪，计算障碍物的速度
  std::vector<std::shared_ptr<Object>> objects;
  for (size_t i = 0; i < track_contours.size(); ++i) {
    std::shared_ptr<Object> out_obj(new Object);
    int controur_area = cv::contourArea(track_contours[i]);
    cv::RotatedRect box = cv::minAreaRect(cv::Mat(track_contours[i]));
    box.points(out_obj->rect_points);

    out_obj->box = box;
    out_obj->inside_road_edge_flag = inside_road_edge_vec[i];
    out_obj->dis_x_y = dis_x_y_vec[i];
    out_obj->bottom_left_point = bottom_left_vec[i];
    out_obj->bottom_right_point = bottom_right_vec[i];
    out_obj->obstacle_expand_width = obj_width_vec[i];
    out_obj->center[0] = (center_.y - box.center.y) * resolution_;
    out_obj->center[1] = (center_.x - box.center.x) * resolution_;
    out_obj->center[2] = -1;

    out_obj->width = box.size.width * resolution_;
    out_obj->length = box.size.height * resolution_;
    out_obj->height = 1;
    out_obj->contour = track_contours[i];
    out_obj->area = controur_area;
    out_obj->score = 1;
    out_obj->anchor_point[2] = 0;
    objects.push_back(out_obj);
  }

  for (size_t i = 0; i < cloud_in.size(); ++i) {
    for (size_t j = 0; j < objects.size(); ++j) {
      cv::Point p;
      p.y = center_.y - round(cloud_in[i].x / resolution_);
      p.x = center_.x - round(cloud_in[i].y / resolution_);
      if (IsPointInMatrix(objects[j]->rect_points, p)) {
        objects[j]->cloud.push_back(cloud_in[i]);
        objects[j]->height_max =
            std::max(cloud_in[i].z, objects[j]->height_max);
        objects[j]->height_min =
            std::min(cloud_in[i].z, objects[j]->height_min);
        break;
      }
    }
  }

  // object tracker
  track_objects_.clear();
  double timestamp = ros::Time::now().toSec();
  object_tracker_->Track(objects, timestamp, &track_objects_);

  // 对速度进行修正
  for (size_t i = 0; i < objects.size(); ++i) {
    for (size_t j = 0; j < objects.size(); ++j) {
      if (i == j) continue;
      bool inside_flag = true;
      for (int k = 0; k < 4; ++k) {
        cv::Point p;
        p.x = objects[i]->rect_points[k].x;
        p.y = objects[i]->rect_points[k].y;
        if (!IsPointInMatrix(objects[j]->rect_points, p)) {
          inside_flag = false;
          break;
        }
      }
      if (!inside_flag) continue;
      objects[i]->velocity = objects[j]->velocity;
      break;
    }
  }

  // for (size_t i = 0; i < track_objects_.size(); ++i) {
  //   char name_buf[50];
  //   double height = track_objects_[i]->height_max + abs_ground_height_;
  //   // int track_id = track_objects_[i]->track_id;
  //   snprintf(name_buf, sizeof(name_buf), "%.2f(%.2f,%.2f)", height,
  //            track_objects_[i]->velocity[0], track_objects_[i]->velocity[1]);
  //   cv::Point center_point;
  //   center_point.y = static_cast<int>(center_.y -
  //   track_objects_[i]->center[0] /
  //                                                     resolution_);
  //   center_point.x = static_cast<int>(center_.x -
  //   track_objects_[i]->center[1] /
  //                                                     resolution_);
  //   cv::putText(result_img_, name_buf, center_point,
  //   cv::FONT_HERSHEY_SIMPLEX,
  //               0.8, cv::Scalar(0, 0, 255), 2, 8);
  // }
}

// 计算 |p1 p2| X |p1 p|
double Planning::GetCross(cv::Point2f *p1, cv::Point2f *p2,
                          const cv::Point &p) {
  return (p2->x - p1->x) * (p.y - p1->y) - (p.x - p1->x) * (p2->y - p1->y);
}

bool Planning::IsPointInMatrix(cv::Point2f *rp, const cv::Point &p) {
  bool isPointIn = GetCross(rp, rp + 1, p) * GetCross(rp + 2, rp + 3, p) >= 0 &&
                   GetCross(rp + 1, rp + 2, p) * GetCross(rp + 3, rp, p) >= 0;
  return isPointIn;
}

bool Planning::FindStateCode(const int code) {
  std::vector<int>::iterator result =
      find(state_code_.state_code.begin(), state_code_.state_code.end(), code);
  if (result == state_code_.state_code.end())
    return false;
  else
    return true;
}

void Planning::GetRoadEdgePath(const cv::Mat &road_edge_img,
                               ContoursVector *object_contours) {
  road_edge_boundary_.clear();
  ContoursVector contours;
  cv::findContours(road_edge_img, contours, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_NONE);

  ContoursVector contours_road_edge;
  int contours_area_max = 0;
  double distance_min = FLT_MAX;
  double dis_min_x = FLT_MAX;
  double dis_min_y = FLT_MAX;
  int distance_index = -1, contour_area_index = -1;
  int contours_size = static_cast<int>(contours.size());
  std::vector<bool> exclude_flags(contours_size, true);
  std::vector<bool> end_flags(contours_size, true);

  ContoursVector sparse_contours;
  // 根据不同模式找出左右两侧面积最大和离车最近的轮廓
  for (size_t i = 0; i < contours_size; ++i) {
    int controur_area = cv::contourArea(contours[i]);
    std::vector<cv::Point> sparse_contour;
    for (size_t j = 0; j < contours[i].size(); ++j) {
      if (j % 20 != 0) continue;
      sparse_contour.push_back(contours[i][j]);
      int x = contours[i][j].x;
      int y = contours[i][j].y;
      // 过滤在紧急停车范围内的轮廓
      if (abs(x - center_.x) < e_stop_y_ / resolution_ &&
          y > center_.y - e_stop_x_ / resolution_ && y < center_.y) {
        last_dis_min_x_ = FLT_MAX;
        last_dis_min_y_ = FLT_MAX;
        return;
      }

      // if (y < 400 && x > 10 && x < 990) end_flags[i] = false;
      if (y > 3.0 / resolution_) exclude_flags[i] = false;
      if (curb_sweep_mode_ == 0 &&
          x < (center_.x - car_width_ / (2.0 * resolution_)) &&
          y > 5.0 / resolution_ && y < center_.y) {
        double distance = sqrt(pow(width_ - x, 2) + pow(height_ - y, 2));
        if (distance < distance_min && controur_area > area_thre_) {
          distance_min = distance;
          distance_index = i;
          // dis_min_x = x;
          // dis_min_y = y;
        }
        if (controur_area > contours_area_max) {
          contours_area_max = controur_area;
          contour_area_index = i;
        }
      } else if (curb_sweep_mode_ == 1 &&
                 x > (center_.x + car_width_ / (2.0 * resolution_)) &&
                 y > 5 / resolution_ && y < center_.y) {
        double distance = sqrt(pow(x, 2) + pow(height_ - y, 2));
        if (distance < distance_min && controur_area > area_thre_) {
          distance_min = distance;
          distance_index = i;
          // dis_min_x = x;
          // dis_min_y = y;
        }

        if (controur_area > contours_area_max) {
          contours_area_max = controur_area;
          contour_area_index = i;
        }
      }
    }
    sparse_contours.push_back(sparse_contour);
  }

  // road_edge_boundary_:马路牙子边界轮廓，比马路牙子轮廓略小，在用双园法规划时，
  // 判断规划轨迹是否与其相交；
  // contours_road_edge：马路牙子轮廓，主要用来生成马路牙子轮廓；
  for (int i = 0; i < contours_size; ++i) {
    if (distance_index == contour_area_index && distance_index != -1) {
      if (i == distance_index) {
        contours_road_edge.push_back(sparse_contours[i]);
        road_edge_boundary_.push_back(contours[i]);
      } else {
        if (!exclude_flags[i]) object_contours->push_back(contours[i]);
      }
    } else if (distance_index != contour_area_index && distance_index != -1 &&
               contour_area_index != -1) {
      if (i == distance_index || i == contour_area_index) {
        contours_road_edge.push_back(sparse_contours[i]);
        road_edge_boundary_.push_back(contours[i]);
      } else {
        if (!exclude_flags[i]) object_contours->push_back(contours[i]);
      }
    } else {
      if (!exclude_flags[i]) object_contours->push_back(contours[i]);
    }
  }

  cv::Mat img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
  if (distance_index != -1) {
    double end_distance = FLT_MAX;
    int end_index = -1;
    double mutation_dis_min = FLT_MAX;
    for (size_t i = 0; i < contours[distance_index].size(); ++i) {
      if (i % 10 != 0) continue;
      double mutation_distance;
      int x = contours[distance_index][i].x;
      int y = contours[distance_index][i].y;
      if (curb_sweep_mode_ == 0)
        mutation_distance = sqrt(pow(x - width_, 2) + pow(y - height_, 2));
      else
        mutation_distance = sqrt(pow(x, 2) + pow(y - width_, 2));
      if (mutation_distance < mutation_dis_min) {
        mutation_dis_min = mutation_distance;
        dis_min_x = x;
        dis_min_y = y;
      }

      double distance =
          sqrt(pow(center_.x - x, 2) + pow(3.5 / resolution_ - y, 2));
      if (distance < end_distance) {
        end_distance = distance;
        end_index = i;
      }
    }
    if (contours[distance_index][end_index].y < 4.5 / resolution_)
      end_flags[distance_index] = false;
    cv::circle(result_img_, contours[distance_index][end_index], 5,
               cv::Scalar(0, 255, 0), -1, 8);
  }

  int road_edge_size = static_cast<int>(road_edge_boundary_.size());
  int expand_x = (car_width_ + road_edge_thickness_) / resolution_;
  int expand_y = (obj_expand_y_ - erode_y_) / resolution_;
  int patch_expand_width = expand_x;
  if (distance_index != -1 && end_flags[distance_index]) {
    cv::drawContours(img, contours, distance_index, cv::Scalar(255), 1, 8);
    road_edge_detect_->DilateImg(img, img, expand_x + (int)(0.6 / resolution_),
                                 expand_y);
    if (contour_area_index != -1 && distance_index != contour_area_index) {
      double_circle_method_->SubsectionExpand(
          expand_x, expand_y, patch_expand_width, contours[contour_area_index],
          &img);
    }
  } else {
    for (size_t i = 0; i < road_edge_size; ++i) {
      double_circle_method_->SubsectionExpand(
          expand_x, expand_y, patch_expand_width, road_edge_boundary_[i], &img);
    }
  }
  // if (distance_index != -1) {
  //   cv::drawContours(img, contours, distance_index, cv::Scalar(255), 1, 8);
  //   road_edge_detect_->DilateImg(img, img, expand_x, expand_y);
  // }

  // cv::imshow("test", img);
  // cv::bitwise_or(line_img, boundary_img, img);

  contours_road_edge.clear();
  img_road_edge_points_.clear();
  cv::findContours(img, contours_road_edge, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_NONE);

  // current_road_type:
  // 0: 没有马路牙子;
  // 1: 有马路沿但马路沿没有从进到远的突变;
  // 2: 有马路沿但马路沿有从近到远的突变；
  int index = -1;
  int start_index = 0;
  int road_edges_size = static_cast<int>(contours_road_edge.size());
  double dis_min = FLT_MAX;

  std::vector<int> near_index(road_edges_size, 0);
  std::vector<int> far_index(road_edges_size, 0);
  std::vector<double> near_dis(road_edges_size, FLT_MAX);
  std::vector<double> far_dis(road_edges_size, FLT_MAX);

  //　寻找马路沿轮廓与车最近的一点
  for (size_t i = 0; i < contours_road_edge.size(); ++i) {
    for (size_t j = 0; j < contours_road_edge[i].size(); ++j) {
      int x = contours_road_edge[i][j].x;
      int y = contours_road_edge[i][j].y;
      if (j % 10 == 0) {
        double near_distance =
            sqrt(pow(x - center_.x, 2) + pow(y - center_.y, 2));
        double far_distance;
        if (curb_sweep_mode_ == 0) {
          far_distance = sqrt(pow(width_ - x, 2) + pow(y, 2));
          if (near_distance < dis_min) {
            index = i;
            dis_min = near_distance;
          }
        } else if (curb_sweep_mode_ == 1) {
          far_distance = sqrt(pow(x, 2) + pow(y, 2));
          if (near_distance < dis_min) {
            index = i;
            dis_min = near_distance;
          }
        }

        if (near_distance < near_dis[i]) {
          near_index[i] = j;
          near_dis[i] = near_distance;
        }

        if (far_distance < far_dis[i]) {
          far_index[i] = j;
          far_dis[i] = far_distance;
        }
      }
    }
  }
  bool have_obstacle_flag = false;
  if (distance_index != -1 && end_flags[distance_index]) {
    obstacle_count_++;
    if (obstacle_count_ > 2) no_obstacle_count_ = 0;
  } else {
    no_obstacle_count_++;
    if (no_obstacle_count_ > 2)
      obstacle_count_ = 0;
    else {
      dis_min_y = last_dis_min_y_;
      dis_min_x = last_dis_min_x_;
    }
  }

  if (obstacle_count_ > 2) {
    have_obstacle_flag = true;
  }

  if (index != -1) {
    cv::circle(result_img_, contours_road_edge[index][near_index[index]], 10,
               cv::Scalar(255, 0, 0), -1, 5);

    current_road_type_ = 1;
    find_road_edge_flag_ = true;
    int start, end;
    if (near_index[index] < far_index[index]) {
      if (curb_sweep_mode_ == 0) {
        start = near_index[index];
        end = far_index[end];
      } else {
        start = 0;
        end = near_index[index];
      }
    } else {
      if (curb_sweep_mode_ == 0) {
        start = near_index[index];
        end = contours_road_edge[index].size() - 1;
      } else {
        start = far_index[index];
        end = near_index[index];
      }
    }

    // 根据清扫模式(靠左或靠右)来选取马路沿轮廓的点
    if (curb_sweep_mode_ == 0) {
      double delta_x = (last_dis_min_x_ - dis_min_x) * resolution_;
      double delta_y = (last_dis_min_y_ - dis_min_y) * resolution_;
      double delta_dis = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
      bool delta_mut_flag = false;
      if (delta_x >= 0.2 && delta_dis > 0.55) {
        delta_mut_flag = true;
        AWARN << "tmp_dis: " << delta_dis;
      }
      if (last_dis_min_x_ != FLT_MAX && last_dis_min_y_ != FLT_MAX &&
          dis_min_x != FLT_MAX && dis_min_y != FLT_MAX) {
        double mut_x_thre = fabs((last_dis_min_x_ - center_.x) * resolution_);
        double mut_y_thre = fabs((last_dis_min_y_ - center_.y) * resolution_);
        if (center_.x - last_dis_min_x_ > e_stop_y_ / resolution_ &&
            mut_x_thre < 2.3 && mut_y_thre < 4.5 &&
            ((last_have_obstacle_flag_ && !have_obstacle_flag &&
              last_dis_min_x_ - dis_min_x >= 0.1 / resolution_) ||
             ((delta_x > delta_dis_min_x_ || delta_mut_flag) &&
              delta_y > delta_dis_min_y_)) &&
            last_dis_min_y_ >= 6 / resolution_ && last_dis_min_y_ < center_.y &&
            !dis_mutation_flag_ && speed_ > 0.1) {
          AWARN << "dis mutation flag is true.";
          dis_mutation_flag_ = true;
          default_x_ = last_dis_min_x_ + 1.2 / resolution_;
          if (default_x_ > center_.x + 2 / resolution_)
            default_x_ = center_.x + 2 / resolution_;
          else if (default_x_ < center_.x - 2 / resolution_)
            default_x_ = center_.x - 2 / resolution_;
          straight_distance_thre_ = (center_.y - last_dis_min_y_) * resolution_;
          if (straight_distance_thre_ > 3.0) straight_distance_thre_ = 3.0;

          AWARN << "last dis min x: " << last_dis_min_x_;
          AWARN << "last dis min y: " << last_dis_min_y_;
          AWARN << "default_x_: " << default_x_;
          AWARN << "Delta x: " << delta_x;
          AWARN << "Delta y: " << delta_y;
          AWARN << "straight distance thre: " << straight_distance_thre_;
        }
        for (int k = start; k <= end; ++k) {
          img_road_edge_points_.push_back(contours_road_edge[index][k]);
        }
      }
    } else if (curb_sweep_mode_ == 1) {
      double delta_x = (dis_min_x - last_dis_min_x_) * resolution_;
      double delta_y = (last_dis_min_y_ - dis_min_y) * resolution_;
      double delta_dis = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
      bool delta_mut_flag = false;
      if (delta_x >= 0.2 && delta_dis > 0.55) {
        delta_mut_flag = true;
        AWARN << "tmp_dis: " << delta_dis;
      }
      if (last_dis_min_x_ != FLT_MAX && last_dis_min_y_ != FLT_MAX &&
          dis_min_x != FLT_MAX && dis_min_y != FLT_MAX) {
        double mut_x_thre = fabs((last_dis_min_x_ - center_.x) * resolution_);
        double mut_y_thre = fabs((last_dis_min_y_ - center_.y) * resolution_);
        if (last_dis_min_x_ - center_.x > e_stop_y_ / resolution_ &&
            mut_x_thre < 2.3 && mut_y_thre < 4.5 &&
            ((last_have_obstacle_flag_ && !have_obstacle_flag &&
              dis_min_x - last_dis_min_x_ > 0.1 / resolution_) ||
             ((delta_x > delta_dis_min_x_ || delta_mut_flag) &&
              delta_y > delta_dis_min_y_)) &&
            last_dis_min_y_ >= 6 / resolution_ && !dis_mutation_flag_ &&
            speed_ > 0.1) {
          AWARN << "dis mutation flag is true.";
          dis_mutation_flag_ = true;
          default_x_ = last_dis_min_x_ - 1.2 / resolution_;
          if (default_x_ > center_.x + 2 / resolution_)
            default_x_ = center_.x + 2 / resolution_;
          else if (default_x_ < center_.x - 2 / resolution_)
            default_x_ = center_.x - 2 / resolution_;
          straight_distance_thre_ = (center_.y - last_dis_min_y_) * resolution_;
          if (straight_distance_thre_ > 3.0) straight_distance_thre_ = 3.0;
          AWARN << "last dis min x: " << last_dis_min_x_;
          AWARN << "last dis min y: " << last_dis_min_y_;
          AWARN << "default_x_: " << default_x_;
          AWARN << "Delta x: " << delta_x;
          AWARN << "Delta y: " << delta_y;
          AWARN << "straight distance thre: " << straight_distance_thre_;
        }
        for (int k = start; k <= end; ++k) {
          img_road_edge_points_.push_back(contours_road_edge[index][k]);
        }
        std::reverse(img_road_edge_points_.begin(),
                     img_road_edge_points_.end());
      }
    }
  } else {
    if (current_road_type_ != 0) {
      AWARN << "Not detect road edge.";
      current_road_type_ = 0;
      if (!dis_mutation_flag_) {
        if (last_dis_min_y_ != FLT_MAX)
          straight_distance_thre_ = (center_.y - last_dis_min_y_) * resolution_;
        else
          straight_distance_thre_ = 2.0;
        if (straight_distance_thre_ > 2.0) straight_distance_thre_ = 2.0;
      }
      if (last_dis_min_x_ != FLT_MAX) {
        if (curb_sweep_mode_ == 0)
          default_x_ = last_dis_min_x_ + 1.2 / resolution_;
        else if (curb_sweep_mode_ == 1)
          default_x_ = last_dis_min_x_ - 1.2 / resolution_;
        if (default_x_ > center_.x + 2 / resolution_)
          default_x_ = center_.x + 2 / resolution_;
        else if (default_x_ < center_.x - 2 / resolution_)
          default_x_ = center_.x - 2 / resolution_;
      } else
        default_x_ = center_.x;
      AWARN << "last dis min x: " << last_dis_min_x_;
      AWARN << "last dis min y: " << last_dis_min_y_;
      AWARN << "default_x_: " << default_x_;
      AWARN << "straight distance thre: " << straight_distance_thre_;
    }
  }

  if (dis_mutation_flag_) {
    img_road_edge_points_.clear();
    current_road_type_ = 2;
    avoid_obstacle_flag_ = false;
    // 在马路沿发生突变时，要将马路沿轮廓列为障碍物轮廓
    if (distance_index != -1)
      object_contours->push_back(contours[distance_index]);
  }

  last_dis_min_x_ = dis_min_x;
  last_dis_min_y_ = dis_min_y;

  /* 在没有马路沿或者马路沿发生由近到远的突变，此时车默认沿当前方向往前行驶一段距离*/
  /* 这种做法是为了解决由于感知在近处不能检测到比较低的物体，在避障时不碰到障碍物.*/
  switch (current_road_type_) {
    case 0: {
      if (last_road_type_ != current_road_type_) {
        last_time_ = -1;
        straight_distance_ = 0;
      }
      double current_time = ros::Time::now().toSec();
      if (last_time_ > 0.0001) {
        straight_distance_ += (current_time - last_time_) * speed_;
      }
      if (straight_distance_ <= 1.0) {
        for (size_t i = 0; i < 5 / resolution_; i++) {
          img_road_edge_points_.push_back(cv::Point(center_.x, center_.y - i));
        }
      } else if (straight_distance_ < 5.0) {
        for (size_t i = 0; i < 5 / resolution_; i++) {
          if (curb_sweep_mode_ == 0) {
            int y = center_.y - i;
            int x = round(center_.x - i / slope_);
            img_road_edge_points_.push_back(cv::Point(x, y));
          } else if (curb_sweep_mode_ == 1) {
            int y = center_.y - i;
            int x = round(center_.x + i / slope_);
            img_road_edge_points_.push_back(cv::Point(x, y));
          }
        }
      } else {
        AWARN << "Can not find road edge,stop";
        find_road_edge_flag_ = false;
      }
      last_time_ = current_time;
      last_road_type_ = current_road_type_;
    } break;

    case 1: {
      last_road_type_ = current_road_type_;
    } break;

    case 2: {
      if (last_road_type_ != current_road_type_) {
        last_time_ = -1;
        straight_distance_ = 0;
      }
      double current_time = ros::Time::now().toSec();
      if (last_time_ > 0.0001) {
        straight_distance_ += (current_time - last_time_) * speed_;
      }
      if (straight_distance_ <= straight_distance_thre_) {
        for (size_t i = 0; i < 5 / resolution_; i++) {
          img_road_edge_points_.push_back(cv::Point(default_x_, center_.y - i));
        }
      } else {
        AWARN << "set mutation flag to false.";
        dis_mutation_flag_ = false;
      }
      last_time_ = current_time;
      last_road_type_ = current_road_type_;
    } break;
    default:
      break;
  }
  if (have_obstacle_flag && no_obstacle_count_ > 0 && no_obstacle_count_ <= 2)
    img_road_edge_points_ = last_road_edge_points_;
  last_have_obstacle_flag_ = have_obstacle_flag;
  last_road_edge_points_ = img_road_edge_points_;
}

void Planning::FilterLine(const std::vector<cv::Point> &line,
                          std::vector<cv::Point> *filter_line) {
  filter_line->push_back(cv::Point(center_.x, center_.y));
  for (size_t i = 0; i < line.size(); ++i) {
    if (line[i].y < 1 / resolution_ || line[i].y > 8 / resolution_) continue;
    int x = center_.y - line[i].y;
    int y = center_.x - line[i].x;
    double theta = atan2(y, x);
    if (curb_sweep_mode_ == 0 && theta < filter_slop_thre_ * DEGREE_TO_RAD) {
      filter_line->push_back(line[i]);
    } else if (curb_sweep_mode_ == 1 &&
               theta > -filter_slop_thre_ * DEGREE_TO_RAD) {
      filter_line->push_back(line[i]);
    }
  }
}

void Planning::SparseLine(const std::vector<cv::Point> &line,
                          std::vector<cv::Point> *sparse_line) {
  for (size_t i = 0; i < line.size(); ++i) {
    if (i % sparse_factor_ != 0) continue;
    if (mode_ == 2) {
      if (line[i].y < 1 / resolution_) continue;
      sparse_line->push_back(line[i]);

      // 过滤相对车身斜率比较大的点
      // int x = 1000 - line[i].y;
      // int y = 500 - line[i].x;
      // double theta = atan2(y, x);
      // if (curb_sweep_mode_ == 0 && theta < PI / 4) {
      //   sparse_line->push_back(line[i]);
      // } else if (curb_sweep_mode_ == 1 && theta > -PI / 4) {
      //   sparse_line->push_back(line[i]);
      // }

    } else if (mode_ == 1) {
      if (line[i].y < 1 / resolution_) continue;
      sparse_line->push_back(line[i]);
    }
  }
}

void Planning::GetImgGlobalPath(const std::vector<cv::Point> &path,
                                std::vector<cv::Point> *patch_line) {
  if (path.empty()) return;
  cv::Mat trace_path_img = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(0));
  cv::polylines(trace_path_img, path, false, cv::Scalar(255), 1, 8, 0);
  ContoursVector trace_path_contour;
  cv::findContours(trace_path_img, trace_path_contour, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_NONE);
  int size = static_cast<int>(trace_path_contour.size());
  if (size == 1) {
    double near_dis_min = FLT_MAX;
    double far_dis_min = FLT_MAX;
    int near_index = 0;
    int far_index = 0;
    cv::Point start_point = path.front();
    cv::Point end_point = path.back();
    for (size_t i = 0; i < trace_path_contour[0].size(); ++i) {
      int x = trace_path_contour[0][i].x;
      int y = trace_path_contour[0][i].y;

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

    std::vector<cv::Point> left_line, right_line, temp_vec;
    bool left_flag = false, right_flag = false;
    for (int i = 0; i < trace_path_contour[0].size(); ++i) {
      cv::Point point = trace_path_contour[0][i];
      if (i == near_index) {
        left_flag = false;
        right_flag = true;
      }
      if (i == far_index) {
        left_flag = true;
        right_flag = false;
      }
      if (left_flag) left_line.push_back(point);
      if (right_flag) right_line.push_back(point);
      if (!left_flag && !right_flag) temp_vec.push_back(point);
    }

    if (near_index >= far_index) {
      std::reverse(left_line.begin(), left_line.end());
      right_line.insert(right_line.end(), temp_vec.begin(), temp_vec.end());
    } else {
      left_line.insert(left_line.end(), temp_vec.begin(), temp_vec.end());
      std::reverse(left_line.begin(), left_line.end());
    }
    SparseLine(left_line, patch_line);
    // *patch_line = left_line;
  }
  // AWARN << "img_global_path_size:" << img_global_path_.size();
}

void Planning::PolyFit(const int order, const std::vector<cv::Point> &line,
                       std::vector<cv::Point> *fit_line) {
  int size = line.size();
  if (size < 4) return;
  if (0 == fit_mode_) {
    *fit_line = line;
  } else if (1 == fit_mode_) {
    std::vector<cv::Point2f> line_bspline;
    cv::Point line_point;
    line_bspline = bspline_.CoutVector(1 / resolution_, line);
    for (int i = 0; i < line_bspline.size(); i++) {
      line_point.x = line_bspline[i].x;
      line_point.y = line_bspline[i].y;
      fit_line->push_back(cv::Point(line_bspline[i].x, line_bspline[i].y));
    }
  } else if (2 == fit_mode_) {
    int x_num = order + 1;
    //构造矩阵U和Y
    cv::Mat mat_u(size, x_num, CV_64F);
    cv::Mat mat_y(size, 1, CV_64F);
    for (int i = 0; i < mat_u.rows; ++i)
      for (int j = 0; j < mat_u.cols; ++j) {
        mat_u.at<double>(i, j) = pow(line[i].y, j);
      }
    for (int i = 0; i < mat_y.rows; ++i) {
      mat_y.at<double>(i, 0) = line[i].x;
    }

    //矩阵运算，获得系数矩阵K
    cv::Mat mat_k(x_num, 1, CV_64F);
    mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
    for (size_t i = 0; i < line.size(); ++i) {
      cv::Point point(0, line[i].y);
      double x = 0.0;
      for (size_t j = 0; j < x_num; ++j) {
        x += mat_k.at<double>(j, 0) * pow(line[i].y, j);
      }
      point.x = round(x);
      fit_line->push_back(point);
    }
  }
}

void Planning::GetPlanParam(PlanParam *plan_param) {
  plan_param->mode = mode_;
  plan_param->curb_sweep_mode = curb_sweep_mode_;
  plan_param->road_edge_thickness = road_edge_thickness_;
  plan_param->obstacle_expand_width = obstacle_expand_width_;
  plan_param->avoid_obstacle_flag = avoid_obstacle_flag_;
  plan_param->only_trace_flag = only_trace_flag_;
  plan_param->trace_good_flag = trace_good_flag_;
  plan_param->vehicle_speed = speed_;
  plan_param->obj = track_objects_;
  plan_param->current_type = current_type_;
  plan_param->straight_distance_thre = straight_distance_thre_;
  plan_param->road_edge_boundary = road_edge_boundary_;
  if (mode_ == 1) {
    plan_param->reference_line = img_path_points_center_;
  } else if (mode_ == 2) {
    plan_param->reference_line = img_road_edge_points_;
    if (current_road_type_ == 0)
      plan_param->road_edge_flag = false;
    else if (current_road_type_ == 1)
      plan_param->road_edge_flag = true;
  }
}

void Planning::ChassisCallback(
    const sweeper_msgs::SweeperChassisDetail &chassis) {
  speed_ = chassis.vehicle_speed_output / 3.6;
}

void Planning::ConvertPathToImg(std::vector<cv::Point> &input_line,
                                int *img_path) {
  cv::Mat img = cv::Mat(1000, 1000, CV_8UC1, cv::Scalar(0));
  cv::polylines(img, input_line, false, cv::Scalar(255), 1, 8, 0);
  for (size_t i = 0; i < 1000; ++i) {
    for (size_t j = 0; j < 1000; ++j) {
      if (img.at<uchar>(i, j) != 255) continue;
      if (0 == *(img_path + i * 2)) {
        *(img_path + i * 2) = j;
      } else {
        if (*(img_path + i * 2) > j) {
          int j_tmp = *(img_path + (i)*2);
          *(img_path + i * 2) = j;
          *(img_path + i * 2 + 1) = j_tmp;
        } else if (*(img_path + i * 2) < j) {
          *(img_path + i * 2 + 1) = j;
        }
      }
    }
  }
}
}  // namespace navigation
}  // namespace sweeper

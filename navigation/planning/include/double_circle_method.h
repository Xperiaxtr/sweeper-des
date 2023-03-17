#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "../../../common/log.h"
#include "road_edge_detect.h"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "tracker/object.h"

#define PI 3.1415926
#define RAD_TO_DEGREE (180.0 / PI)
#define DEGREE_TO_RAD (PI / 180.0)

namespace sweeper {
namespace navigation {

struct LinePara {
  float k;
  float b;
};

typedef struct Point {
  int index;
  int location;
  int flag = -1;
  float slop_angle;
  float curvity;
  cv::Point point;
  double r;
} PathPoint;

typedef std::vector<std::shared_ptr<Object>> Objects;
typedef std::vector<std::vector<cv::Point>> ContoursVector;

typedef struct PlanParam {
  int mode;
  int curb_sweep_mode;
  int road_edge_thickness;
  int current_type;
  int obstacle_expand_width;
  bool avoid_obstacle_flag;
  bool only_trace_flag;
  bool trace_good_flag;
  bool road_edge_flag;
  double vehicle_speed;
  double straight_distance_thre;
  Objects obj;
  ContoursVector road_edge_boundary;
  std::vector<cv::Point> reference_line;
} PlanParam;

class DoubleCircleMethod {
 public:
  DoubleCircleMethod(ros::NodeHandle &private_nh, double car_width,
                     double e_stop_x, double e_stop_y,
                     RoadEdgeDetect *road_edge_detect);
  ~DoubleCircleMethod();
  void GetPlanningTrajectory(const PlanParam &plan_param, bool *estop,
                             cv::Mat *result_img,
                             std::vector<cv::Point> *output_line,
                             sweeper_msgs::SensorFaultInformation *state_code);
  void SubsectionExpand(const int obj_expand_x, const int obj_expand_y,
                        const int patch_expand_width,
                        const std::vector<cv::Point> &contour,
                        cv::Mat *expand_img);
  void SplitContour(const std::vector<cv::Point> &contours,
                    std::vector<cv::Point> *left_line,
                    std::vector<cv::Point> *right_line,
                    std::vector<cv::Point> *up_line,
                    std::vector<cv::Point> *down_line, bool hull_flag = false);
  void Reset();

 private:
  void GetOptimalPath(const cv::Mat &obstacle_img, const cv::Mat &boundary_img,
                      const bool have_obstacle_flag,
                      std::vector<cv::Point> *output_line, bool *right_flag);
  void GetPathPoint(const std::vector<cv::Point> &contours,
                    std::vector<cv::Point> *left_line,
                    std::vector<cv::Point> *right_line);
  bool OptimizePathKinematic(const cv::Mat &reference_img,
                             const cv::Mat &obstacle_img,
                             std::vector<cv::Point> *patch_line,
                             std::vector<cv::Point> *final_path,
                             int interval_num = 20);
  void CalculateRadiusAndLocation(PathPoint *path_point);
  double CalculateCurvity(const cv::Point &last, const cv::Point &current,
                          const cv::Point &next);
  bool DoubleCircleMethodPlan(const PathPoint &current_point,
                              const cv::Mat &obstacle_img,
                              std::vector<cv::Point> *patch_points);
  bool IsIntersectBoundary(const std::vector<cv::Point> &line,
                           const cv::Mat boundary_img);
  void ExpandRotatedRect(const int x, const int y,
                         const std::vector<cv::Point> &contour, cv::Mat *img);
  void CorrectPointSlop(const int interval_num, const int path_index,
                        const std::vector<PathPoint> &path_points,
                        PathPoint *point);
  void SparseLine(const int sparse_factor, const std::vector<cv::Point> &line,
                  std::vector<cv::Point> *sparse_line);

  bool avoid_obstacle_flag_;
  bool double_circle_flag_;
  bool last_obstacle_flag_;
  bool dis_mutation_flag_;
  bool last_have_obstacle_flag_;
  bool pause_flag_;
  bool start_in_obj_flag_;
  bool end_in_obj_flag_;
  bool only_trace_flag_;

  int mode_;
  int default_x_;

  int avoid_obstacle_direction_;
  int curb_sweep_mode_;
  int obstacle_count_;
  int no_obstacle_count_;
  int current_type_;
  int last_obj_speed_x_;
  int height_, width_;

  double speed_thre_;
  double front_dis_;
  double broadcast_thre_;
  double min_turn_radius_;
  double resolution_;
  double sweep_width_;
  double car_width_;
  double turn_obstacle_radius_;
  double barrier_min_height_;
  double last_time_;
  double last_stop_time_;
  double obstacle_thickness_;
  double last_obstacle_dis_x_;
  double last_obstacle_dis_y_;
  double straight_distance_;
  double delta_dis_min_trace_;
  double straight_distance_thre_;
  double accumulation_time_;
  double current_stop_time_;
  double e_stop_x_, e_stop_y_;
  double erode_x_, erode_y_;
  double obstacle_x_, obstacle_y_;
  double obj_expand_x_, obj_expand_y_;
  double intersect_x_, intersect_y_;
  double trace_velocity_x_thre_, trace_velocity_y_thre_;
  double curb_velocity_x_thre_, curb_velocity_y_thre_;
  double curb_obstacle_min_, curb_obstacle_max_;
  double trace_obstacle_min_, trace_obstacle_max_;

  int last_line_times_;
  std::vector<cv::Point> last_line_;
  std::vector<cv::Point> last_trace_line_;
  std::vector<cv::Point> reference_line_;
  std::unordered_map<int, int> obstacle_v_x_;
  std::unordered_map<int, int> continuous_v_x_;
  std::unordered_map<int, int> continuous_v_x_1_;
  std::unordered_map<int, int> continuous_v_y_;
  std::unordered_map<int, int> continuous_v_y_1_;

  cv::Point center_;
  cv::Mat *result_img_;
  ContoursVector road_edge_boundary_;
  RoadEdgeDetect *road_edge_detect_;
  sweeper_msgs::SensorFaultInformation *state_code_;
};

}  // namespace navigation
}  // namespace sweeper
#ifndef MAPPING_TOOLS_TOOLS_HPP_
#define MAPPING_TOOLS_TOOLS_HPP_

#include <math.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "../../../../common/log.h"
// template <typename T>
namespace mapping {
class MathCalculation {
 public:
  static Eigen::Isometry3d Matrix4dToIsometry3d(Eigen::Matrix4d pose);
  static Eigen::Matrix4d Isometry3dToMatrix4d(Eigen::Isometry3d pose_iso);
  // template <typename T>
  static void GetYamlParam(ros::NodeHandle &nh,
                           const std::string parameter_name, double &parameter,
                           double default_val);

  static bool GetCalibYamlParam(const std::string file_path,
                                Eigen::Quaterniond &rotation,
                                Eigen::Vector3d &translation);

  static bool GetFixedGpsPoint(const std::string file_path,
                        Eigen::Vector3d &translation);
  static double GetMatcherCov(double matcher_score, double matcher_cost,
                              int avail_num);

  static double RpyToSignRpy(double rpy);

  static double CalcDiffForRadian(const double now_rad,
                                  const double before_rad);

  static void SensorToCenter(const Eigen::Quaterniond &tran_quater,
                             const Eigen::Vector3d &tran_pos,
                             Eigen::Quaterniond &sensor_quater,
                             Eigen::Vector3d &sensor_pos);

  static void SaveGpsToMapYaml(std::string &yaml_path,
                               Eigen::Vector3d &tran_pos,
                               Eigen::Quaterniond &tran_quater,
                               Eigen::Vector3d &fixed_point);
  static Eigen::MatrixXd CalInformationMatrix(const double &matcher_score);
  static double weight(double a, double max_x, double min_y, double max_y,
                       double x);
  static void SaveMapToimageYaml(std::string &yaml_path, double dis_x,
                                 double dis_y, double resolution);
};
}  // namespace mapping

#endif
#include "tools/math_calculation.hpp"

namespace mapping {
Eigen::Isometry3d MathCalculation::Matrix4dToIsometry3d(Eigen::Matrix4d pose) {
  Eigen::Isometry3d pose_iso;
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      pose_iso(i, j) = pose(i, j);
    }
  }
  return pose_iso;
}
Eigen::Matrix4d MathCalculation::Isometry3dToMatrix4d(
    Eigen::Isometry3d pose_iso) {
  Eigen::Matrix4d pose;
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      pose(i, j) = pose_iso(i, j);
    }
  }
  return pose;
}

// template <typename T>
void MathCalculation::GetYamlParam(ros::NodeHandle &nh,
                                   const std::string parameter_name,
                                   double &parameter, double default_val) {
  // nh.param<double>(parameter_name.c_str(), parameter, default_val);
  // nh.getParam(parameter_name, parameter);
  ros::param::get(parameter_name, parameter);
  AWARN << "[mapping]: " << parameter_name << " ==> " << parameter << std::endl;
}

//得到标定参数
bool MathCalculation::GetCalibYamlParam(const std::string file_path,
                                        Eigen::Quaterniond &rotation,
                                        Eigen::Vector3d &translation) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["transform"]) {
      return false;
    }
    if (config["transform"]["translation"] && config["transform"]["rotation"]) {
      double tx = config["transform"]["translation"]["x"].as<double>();
      double ty = config["transform"]["translation"]["y"].as<double>();
      double tz = config["transform"]["translation"]["z"].as<double>();

      double qx = config["transform"]["rotation"]["x"].as<double>();
      double qy = config["transform"]["rotation"]["y"].as<double>();
      double qz = config["transform"]["rotation"]["z"].as<double>();
      double qw = config["transform"]["rotation"]["w"].as<double>();

      rotation = Eigen::Quaterniond(qw, qx, qy, qz);
      translation = Eigen::Vector3d(tx, ty, tz);
      AWARN << "qx, qy, qz, qw : " << qx << " " << qy << " " << qz << " " << qw;
      AWARN << "tx, ty, tz : " << tx << " " << ty << " " << tz;
    }
  } catch (const YAML::Exception &e) {
    AERROR << "load extrinsics: " << file_path
           << " failed! error: " << e.what();
    rotation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    translation = Eigen::Vector3d(0.0, 0.0, 0.0);
    return false;
  }
  return true;
}

//得到标定参数
bool MathCalculation::GetFixedGpsPoint(const std::string file_path,
                                       Eigen::Vector3d &translation) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["fixed"]) {
      AERROR << "load extrinsics: " << file_path << " not fixed";
      translation = Eigen::Vector3d(0.0, 0.0, 0.0);
      return false;
    }
    double tx = config["fixed"]["x"].as<double>();
    double ty = config["fixed"]["y"].as<double>();
    double tz = config["fixed"]["z"].as<double>();

    translation = Eigen::Vector3d(tx, ty, tz);
    AWARN << "tx, ty, tz : " << tx << " " << ty << " " << tz;
  } catch (const YAML::Exception &e) {
    AERROR << "load extrinsics: " << file_path
           << " failed! error: " << e.what();
    translation = Eigen::Vector3d(0.0, 0.0, 0.0);
    return false;
  }
  return true;
}

double MathCalculation::GetMatcherCov(double matcher_score, double matcher_cost,
                                      int avail_num) {
  // AWARN << "raw param : " << matcher_score << " " << matcher_cost << " "
  //       << avail_num;
  double matcher_cov;
  if (matcher_score < 0.006) {
    matcher_cov = 0.01;
  } else {
    matcher_cov = fabs(matcher_score - 0.004) * 100;
  }

  double matcher_cost_cal = matcher_cost * 10000 / avail_num;
  if (avail_num < 100 || matcher_cost < 0) matcher_cost_cal = 500.0;

  // AWARN << "get matcher cov : " << matcher_cov;
  // AWARN << "cov param: " << matcher_cost_cal;
  return matcher_cost_cal;
}

double MathCalculation::RpyToSignRpy(double rpy) {
  if (rpy >= M_PI) {
    rpy -= 2.0 * M_PI;
  }
  return rpy;
}

double MathCalculation::CalcDiffForRadian(const double now_rad,
                                          const double before_rad) {
  double diff_rad = now_rad - before_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI) {
    diff_rad = diff_rad + 2 * M_PI;
  }
  return diff_rad;
}

void MathCalculation::SaveGpsToMapYaml(std::string &yaml_path,
                                       Eigen::Vector3d &tran_pos,
                                       Eigen::Quaterniond &tran_quater,
                                       Eigen::Vector3d &fixed_point) {
  std::ofstream fout(yaml_path + "/gps_to_map.yaml");
  YAML::Node config;
  config["header"]["frame_id"] = "map";
  config["transform"]["translation"]["x"] = tran_pos.x();
  config["transform"]["translation"]["y"] = tran_pos.y();
  config["transform"]["translation"]["z"] = tran_pos.z();
  config["transform"]["rotation"]["x"] = tran_quater.x();
  config["transform"]["rotation"]["y"] = tran_quater.y();
  config["transform"]["rotation"]["z"] = tran_quater.z();
  config["transform"]["rotation"]["w"] = tran_quater.w();

  config["fixed"]["x"] = fixed_point.x();
  config["fixed"]["y"] = fixed_point.y();
  config["fixed"]["z"] = fixed_point.z();

  config["child_frame_id"] = "gps";
  fout << config;
  fout.close();
}

//传感器矫正
void MathCalculation::SensorToCenter(const Eigen::Quaterniond &tran_quater,
                                     const Eigen::Vector3d &tran_pos,
                                     Eigen::Quaterniond &sensor_quater,
                                     Eigen::Vector3d &sensor_pos) {
  Eigen::Quaterniond sensor_quat = sensor_quater;
  Eigen::Vector3d sensor_position = sensor_pos;
  sensor_quater = tran_quater * sensor_quat;
  sensor_pos = sensor_quat * tran_pos + sensor_position;
}

double MathCalculation::weight(double a, double max_x, double min_y,
                               double max_y, double x) {
  double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
  return min_y + (max_y - min_y) * y;
}

Eigen::MatrixXd MathCalculation::CalInformationMatrix(
    const double &matcher_score) {
  // if(use_const_inf_matrix) {
  //   Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
  //   inf.topLeftCorner(3, 3).array() /= const_stddev_x;
  //   inf.bottomRightCorner(3, 3).array() /= const_stddev_q;
  //   return inf;
  // }
  // double fitness_score = calc_fitness_score(cloud1, cloud2, relpose);
  double min_stddev_x = 0.1;
  double max_stddev_x = 5.0;
  double min_stddev_q = 0.05;
  double max_stddev_q = 0.2;
  double fitness_score_thresh = 2.5;
  double var_gain_a = 20.0;

  double min_var_x = std::pow(min_stddev_x, 2);
  double max_var_x = std::pow(max_stddev_x, 2);
  double min_var_q = std::pow(min_stddev_q, 2);
  double max_var_q = std::pow(max_stddev_q, 2);

  float w_x = weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x,
                     matcher_score);
  float w_q = weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q,
                     matcher_score);

  Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
  inf.topLeftCorner(3, 3).array() /= w_x;
  inf.bottomRightCorner(3, 3).array() /= w_q;
  return inf;
}

void MathCalculation::SaveMapToimageYaml(std::string &yaml_path, double dis_x,
                                         double dis_y, double resolution) {
  // std::ofstream fout(yaml_path + "/map_to_image.yaml");
  // YAML::Node config;
  // config["header"]["frame_id"] = "image";
  // config["header"]["child_frame_id"] = "map";

  // Eigen::Matrix<double, 2, 3> tran_matrix;
  // tran_matrix << 0, -1.0 / resolution, dis_y / resolution, -1.0 / resolution,
  // 0,
  //     dis_x / resolution;
  // cv::Mat map_to_image =
  //     (cv::Mat_<double>(2, 3) << 0, -1.0 / resolution, dis_y / resolution,
  //      -1.0 / resolution, 0, dis_x / resolution);
  // Eigen::Vector2d tran_vector = [dis_y/resolution, dis_x/resolution];
  // config["matrix"] = map_to_image;
  // config["matrix"] = {

  cv::FileStorage fs(yaml_path + "/map_to_image.yaml", cv::FileStorage::WRITE);
  cv::Mat map_to_image =
      (cv::Mat_<double>(2, 3) << 0, -1.0 / resolution, dis_y / resolution,
       -1.0 / resolution, 0, dis_x / resolution);
  fs << "map_to_image" << map_to_image;
  fs.release();
}
}  // namespace mapping
#include "pose_util.h"

#include <yaml-cpp/yaml.h>
#include "log.h"

namespace sweeper {
namespace common {
bool ReadPoseFileMat12(const std::string &filename, Eigen::Matrix4d *pose,
                       int *frame_id, double *time_stamp) {
  *pose = Eigen::Matrix4d::Identity();
  std::ifstream ifs(filename.c_str());
  if (!ifs.is_open()) {
    AERROR << "Failed to open file " << filename;
    return false;
  }
  ifs >> *frame_id >> *time_stamp;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ifs >> (*pose)(i, j);
    }
  }
  return true;
}

bool LoadExtrinsic(const std::string &file_path, Eigen::Affine3d *extrinsic) {
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
      *extrinsic =
          Eigen::Translation3d(tx, ty, tz) * Eigen::Quaterniond(qw, qx, qy, qz);
    }
  } catch (const YAML::Exception &e) {
    AERROR << "load extrinsics: " << file_path
           << " failed! error: " << e.what();
    return false;
  }
  return true;
}

}  // namespace common
}  // namespace sweeper

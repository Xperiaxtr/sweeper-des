#pragma once

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Dense"

namespace sweeper {
namespace common {

bool ReadPoseFileMat12(const std::string& filename, Eigen::Matrix4d* pose,
                       int* frame_id, double* time_stamp);

// @brief: Load the velodyne extrinsic from a YAML file.
// @param [in]: path to yaml file
// @param [out]: extrinsic transform
bool LoadExtrinsic(const std::string& file_path, Eigen::Affine3d* extrinsic);

}  // namespace common
}  // namespace sweeper

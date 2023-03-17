#ifndef MAPPING_FILE_MANAGER_FILE_MANAGER_HPP_
#define MAPPING_FILE_MANAGER_FILE_MANAGER_HPP_

#include <fstream>
#include <iostream>
#include <string>

#include "../../../../common/log.h"
#include "sensor_data/cloud_data.hpp"

namespace mapping {
class FileManager {
 public:
  static bool CreateFile(std::ofstream& ofs, std::string file_path);
  static bool InitDirectory(std::string directory_path, std::string use_for);
  static bool CreateDirectory(std::string directory_path, std::string use_for);
};
}  // namespace mapping
#endif
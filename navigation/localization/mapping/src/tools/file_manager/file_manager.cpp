#include "tools/file_manager/file_manager.hpp"

#include <boost/filesystem.hpp>
namespace mapping {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
  ofs.close();
  boost::filesystem::remove(file_path.c_str());

  ofs.open(file_path.c_str(), std::ios::out);
  if (!ofs) {
    AWARN << "无法生成文件: "
          << "/n" << file_path;
    return false;
  }
  return true;
}
bool FileManager::InitDirectory(std::string directory_path,
                                std::string use_for) {
  if (boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::remove_all(directory_path);
  }
  return CreateDirectory(directory_path, use_for);
}

bool FileManager::CreateDirectory(std::string directory_path,
                                  std::string use_for) {
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }

  if (!boost::filesystem::is_directory(directory_path)) {
    AWARN << "无法创建文件夹: "
          << "\n"
          << directory_path;
    return false;
  }

  std::cout << use_for << "存放地址："
            << "\n"
            << directory_path;
  return true;
}

}  // namespace mapping
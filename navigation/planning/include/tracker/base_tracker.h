#pragma once

#include <memory>
#include <string>
#include <vector>

#include "object.h"

namespace sweeper {
namespace navigation {

struct TrackerOptions {
  TrackerOptions() = default;
  explicit TrackerOptions(Eigen::Matrix4d *pose) : velodyne_trans(pose) {}
  std::shared_ptr<Eigen::Matrix4d> velodyne_trans;
};

class BaseTracker {
 public:
  BaseTracker() {}
  virtual ~BaseTracker() {}

  virtual bool Init() = 0;

  // @brief: tracking objects.
  // @param [in]: current frame object list.
  // @param [in]: timestamp.
  // @param [in]: options.
  // @param [out]: current tracked objects.
  virtual bool Track(
      const std::vector<std::shared_ptr<Object>> &objects,
      double timestamp, const TrackerOptions &options,
      std::vector<std::shared_ptr<Object>> *tracked_objects) = 0;

  virtual std::string name() const = 0;
};

}  // namespace navigation
}  // namespace sweeper

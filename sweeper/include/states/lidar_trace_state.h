#pragma once

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "../../../common/watch_dog.h"
#include "navigation/find_global_path.h"
#include "sweeper_states.h"

using sweeper::common::WatchDog;

namespace sweeper {
class LidarTraceState : public SweeperStates {
 public:
  LidarTraceState(SweepDataCenter *sweep_data_center);
  ~LidarTraceState();

  void Enter(int currentState);
  void Run(void);
  void Quit(void);
  void ForceQuit(void);
  void Reset(void);

 private:
  bool read_entire_path_flag_;
  ros::Publisher reference_line_pub_;
  ros::Publisher boundary_line_pub_;

  FindGlobalPath *find_global_path_;
};
}  // namespace sweeper

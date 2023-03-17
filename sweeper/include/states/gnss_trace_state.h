#pragma once

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "../../../common/watch_dog.h"
#include "navigation/find_global_path.h"
#include "sweeper_states.h"

using sweeper::common::WatchDog;

namespace sweeper {
class GnssTraceState : public SweeperStates {
 public:
  GnssTraceState(SweepDataCenter *sweep_data_center);
  ~GnssTraceState();

  void Enter(int currentState);
  void Run(void);
  void Quit(void);
  void ForceQuit(void);
  void Reset(void);

 private:
  FindGlobalPath *find_global_path_;
  bool read_entire_path_flag_;

  ros::Publisher reference_line_pub_;
  ros::Publisher boundary_line_pub_;
};
}  // namespace sweeper

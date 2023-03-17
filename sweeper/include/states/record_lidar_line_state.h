#pragma once

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <fstream>
#include <iostream>

#include "../../../common/watch_dog.h"
#include "navigation/find_global_path.h"
#include "sweeper_states.h"

namespace sweeper {
class RecordLidarLineState : public SweeperStates {
 public:
  RecordLidarLineState(SweepDataCenter *sweep_data_center);
  ~RecordLidarLineState();

  void Enter(int currentState);
  void Run(void);
  void Quit(void);
  void ForceQuit(void);
  void Reset(void);

 private:
  int lidar_last_start_cmd_;
  FindGlobalPath *find_global_path_;
};
}  // namespace sweeper
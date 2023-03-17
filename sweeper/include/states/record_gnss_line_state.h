#pragma once

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <fstream>
#include <iostream>

#include "../../../common/watch_dog.h"
#include "navigation/find_global_path.h"
#include "sweeper_states.h"
#include "../../src/matlab/gpsrecordline/gps_record_line_model.h"

using sweeper::common::WatchDog;

namespace sweeper {
class RecordGnssLineState : public SweeperStates {
 public:
  RecordGnssLineState(SweepDataCenter *sweep_data_center);
  ~RecordGnssLineState();

  void Enter(int currentState);
  void Run(void);
  void Quit(void);
  void ForceQuit(void);
  void Reset(void);

 private:
  void GnssCallback(const nav_msgs::Odometry &gnss_location);
  void SavePath(const std::string &line);
  void GpsRecordLineModelInput(void);
  int crossing_name_;
  int position_not_fix_count_;
  int gnss_last_start_cmd_;
  double gps_point_distance_;

  LineType crossing_line_;
  LineType entire_line_;
  FindGlobalPath *find_global_path_;
  gps_record_line_modelModelClass gps_record_line;
};
}  // namespace sweeper

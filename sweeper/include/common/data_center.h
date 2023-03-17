#pragma once

#include <stdint.h>

#include <cstdint>
#include <string>
#include <vector>
class SweepDataCenter {
 public:
  SweepDataCenter();
  ~SweepDataCenter();

  void DataReset() {
    sweep_request_mode = -1;
    sweep_start_cmd = -1;
    mode_match_flag = -1;
    sweep_map = "";
    sweep_line = "";
    if (!state_code.empty()) state_code.clear();
  }

 public:
  int sweep_request_mode;
  int sweep_start_cmd;
  int sweep_side_mode;
  int sweep_point_attribute;
  int sweep_right_trim_accuracy;
  int sweep_left_trim_accuracy;
  int sweep_real_side_mode;
  int sweep_chassis_stop;
  int mode_match_flag;
  std::string sweep_map;
  std::string sweep_line;
  std::vector<int> state_code;
};
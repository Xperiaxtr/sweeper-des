#pragma once

#include "sweeper_states.h"
#include <fstream>
#include <iostream>

namespace sweeper {
class UpdateState : public SweeperStates {
 public:
  UpdateState(SweepDataCenter *sweep_data_center);
  ~UpdateState();

  void Enter(int currentState);
  void Run(void);
  void Quit(void);
  void ForceQuit(void);
  void Reset(void);

  private:
  std::vector<std::string> StringSplit(const std::string &str, const std::string &delim);
  void AddStateCode(const int code); 
  void UpdateSoftware(void);
  bool update_run_command_;
  int  update_network_status_;
  int  update_program_status_;
  int  update_parameter_status_;
};
}  // namespace sweeper

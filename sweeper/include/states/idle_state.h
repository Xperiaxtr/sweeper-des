#pragma once

#include <dirent.h>
#include <sys/stat.h>

#include "navigation/find_global_path.h"
#include "sweeper_states.h"

namespace sweeper {
class IdleState : public SweeperStates {
 public:
  IdleState(SweepDataCenter *sweep_data_center);
  ~IdleState();

  void Enter(int currentState);
  void Run(void);
  void Quit(void);
  void ForceQuit(void);
  void Reset(void);

 private:
  int RemoveDir(const char *dir);
  std::string last_line_;
  FindGlobalPath *find_global_path_;
};
}  // namespace sweeper

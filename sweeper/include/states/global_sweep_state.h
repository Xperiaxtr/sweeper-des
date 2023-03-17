#pragma once

#include "sweeper_states.h"

namespace sweeper {
class GlobalSweepState : public SweeperStates {
 public:
  GlobalSweepState(SweepDataCenter *sweep_data_center);
  ~GlobalSweepState();

  void Enter(int currentState);
  void Run(void);
  void Quit(void);
  void ForceQuit(void);
  void Reset(void);
};

}  // namespace sweeper
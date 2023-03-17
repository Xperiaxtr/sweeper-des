#pragma once

#include "sweeper_states.h"

namespace sweeper {
class InitState : public SweeperStates {
 public:
  InitState(SweepDataCenter *sweep_data_center);
  ~InitState();

  void Enter(int currentState);
  void Run(void);
  void Quit(void);
  void ForceQuit(void);
  void Reset(void);
};
}  // namespace sweeper

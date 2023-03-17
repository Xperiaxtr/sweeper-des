#pragma once

#include "sweeper_states.h"

namespace sweeper {
class ChargingState : public SweeperStates {
 public:
  ChargingState(SweepDataCenter *sweep_data_center);
  ~ChargingState();

  void Enter(int currentState);
  void Run(void);
  void Quit(void);
  void ForceQuit(void);
  void Reset(void);
};

}  // namespace sweeper

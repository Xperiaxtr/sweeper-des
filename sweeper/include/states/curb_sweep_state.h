#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "sweeper_states.h"

namespace sweeper {
class CurbSweepState : public SweeperStates {
 public:
  CurbSweepState(SweepDataCenter* sweep_data_center);
  ~CurbSweepState();

  void Enter(int currentState);
  void Run(void);
  void Quit(void);
  void ForceQuit(void);
  void Reset(void);

 private:
  Eigen::Affine3d bynav_livox_extrinsic_;
};
}  // namespace sweeper
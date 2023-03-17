//
// File: SweeperDriveModeControl.h
//
// Code generated for Simulink model 'SweeperDriveModeControl'.
//
// Model version                  : 1.83
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Tue Aug 18 13:57:31 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 7
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_SweeperDriveModeControl_h_
#define RTW_HEADER_SweeperDriveModeControl_h_
#include <cmath>
#include <string.h>
#include <stddef.h>
#ifndef SweeperDriveModeControl_COMMON_INCLUDES_
# define SweeperDriveModeControl_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // SweeperDriveModeControl_COMMON_INCLUDES_ 

#include "SweeperDriveModeControl_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block states (default storage) for system '<Root>'
typedef struct {
  uint8_T is_active_c3_SweeperDriveModeCo;// '<Root>/Algorithm'
  uint8_T is_mode;                     // '<Root>/Algorithm'
  uint8_T is_side;                     // '<Root>/Algorithm'
  uint8_T is_drive;                    // '<Root>/Algorithm'
  uint8_T is_mode_e;                   // '<Root>/Algorithm'
  uint8_T is_command;                  // '<Root>/Algorithm'
  uint8_T is_response;                 // '<Root>/Algorithm'
  uint8_T is_response_o;               // '<Root>/Algorithm'
  uint8_T is_mode_m;                   // '<Root>/Algorithm'
  uint8_T is_command_l;                // '<Root>/Algorithm'
  uint8_T temporalCounter_i1;          // '<Root>/Algorithm'
  uint8_T temporalCounter_i2;          // '<Root>/Algorithm'
} DW_SweeperDriveModeControl_T;

// External inputs (root inport signals with default storage)
typedef struct {
  real_T ipAppStartCmd;                // '<Root>/ipAppStartCmd'
  real_T ipAppWorkMode;                // '<Root>/ipAppWorkMode'
  real_T ipDcuWorkModeRequest;         // '<Root>/ipDcuWorkModeRequest'
  real_T ipDcuWorkStatus;              // '<Root>/ipDcuWorkStatus'
  real_T ipAccPedalOverride;           // '<Root>/ipAccPedalOverride'
  real_T ipBrakePedalOverride;         // '<Root>/ipBrakePedalOverride'
  real_T ipSteerPedalOverride;         // '<Root>/ipSteerPedalOverride'
  real_T ipCsuFaultLevel;              // '<Root>/ipCsuFaultLevel'
  real_T ipDcuFaultLevel;              // '<Root>/ipDcuFaultLevel'
  real_T ipAppSideMode;                // '<Root>/ipAppSideMode'
  real_T ipDcuSideModeRequest;         // '<Root>/ipDcuSideModeRequest'
  real_T ipChassisVehicleSpeed;        // '<Root>/ipChassisVehicleSpeed'
  real_T ipDcuWorkMode;                // '<Root>/ipDcuWorkMode'
  real_T ipDcuSideMode;                // '<Root>/ipDcuSideMode'
  real_T ipAppSpeedMode;               // '<Root>/ipAppSpeedMode'
  real_T ipDcuSpeedMode;               // '<Root>/ipDcuSpeedMode'
  real_T ipRealSideMode;               // '<Root>/ipRealSideMode'
} ExtU_SweeperDriveModeControl_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real_T opCsuWorkStatus;              // '<Root>/opCsuWorkStatus'
  real_T opCsuWorkModeRequest;         // '<Root>/opCsuWorkModeRequest'
  real_T opCsuWorkModeResponse;        // '<Root>/opCsuWorkModeResponse'
  int32_T opCsuSweepMode;              // '<Root>/opCsuSweepMode'
  int32_T opCsuSweepCmd;               // '<Root>/opCsuSweepCmd'
  int32_T opCsuSweepSide;              // '<Root>/opCsuSweepSide'
  int32_T opCsuSweepStatus;            // '<Root>/opCsuSweepStatus'
  int32_T opCsuSweepSpeed;             // '<Root>/opCsuSweepSpeed'
} ExtY_SweeperDriveModeControl_T;

// Real-time Model Data Structure
struct tag_RTM_SweeperDriveModeContr_T {
  const char_T * volatile errorStatus;
};

// Class declaration for model SweeperDriveModeControl
class SweeperDriveModeControlModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_SweeperDriveModeControl_T SweeperDriveModeControl_U;

  // External outputs
  ExtY_SweeperDriveModeControl_T SweeperDriveModeControl_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  SweeperDriveModeControlModelClass();

  // Destructor
  ~SweeperDriveModeControlModelClass();

  // Real-Time Model get method
  RT_MODEL_SweeperDriveModeCont_T * getRTM();

  // private data and function members
 private:
  // Block states
  DW_SweeperDriveModeControl_T SweeperDriveModeControl_DW;

  // Real-Time Model
  RT_MODEL_SweeperDriveModeCont_T SweeperDriveModeControl_M;

  // private member function(s) for subsystem '<Root>'
  void SweeperDriveModeControl_mode(const real_T *Abs);
  void SweeperDriveModeControl_command(void);
  void Sweep_exit_internal_manualdrive(void);
  void SweeperDriveModeCon_manualdrive(const real_T *Abs);
  void SweeperDriveMod_idel_sweep_mode(const real_T *Abs);
  void SweeperDriveModeControl_mode_a(const real_T *Abs);
  void SweeperDriveModeContr_command_o(void);
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'SweeperDriveModeControl'
//  '<S1>'   : 'SweeperDriveModeControl/Algorithm'

#endif                                 // RTW_HEADER_SweeperDriveModeControl_h_

//
// File trailer for generated code.
//
// [EOF]
//

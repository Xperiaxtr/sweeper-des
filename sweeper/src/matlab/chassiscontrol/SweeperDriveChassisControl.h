//
// File: SweeperDriveChassisControl.h
//
// Code generated for Simulink model 'SweeperDriveChassisControl'.
//
// Model version                  : 1.79
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Wed Nov 18 17:52:11 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 7
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_SweeperDriveChassisControl_h_
#define RTW_HEADER_SweeperDriveChassisControl_h_
#include <string.h>
#include <stddef.h>
#ifndef SweeperDriveChassisControl_COMMON_INCLUDES_
# define SweeperDriveChassisControl_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // SweeperDriveChassisControl_COMMON_INCLUDES_ 

#include "SweeperDriveChassisControl_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// External inputs (root inport signals with default storage)
typedef struct {
  real_T ipChassisWorkStatus;          // '<Root>/ipChassisWorkStatus'
  real_T ipSweeperWorkStatus;          // '<Root>/ipSweeperWorkStatus'
  real_T ipChassisDriveMode;           // '<Root>/ipChassisDriveMode'
  real_T ipSweeperDriveMode;           // '<Root>/ipSweeperDriveMode'
  real_T ipSweeperSpeedValue;          // '<Root>/ipSweeperSpeedValue'
  real_T ipSweeperSteeringAngle;       // '<Root>/ipSweeperSteeringAngle'
  real_T ipSweeperSteeringAngleSpeed;  // '<Root>/ipSweeperSteeringAngleSpeed'
  boolean_T ipChassisSteeringEnable;   // '<Root>/ipChassisSteeringEnable'
  boolean_T ipChassisDriveModeEnable;  // '<Root>/ipChassisDriveModeEnable'
  real_T ipAppCurrentCommand;          // '<Root>/ipAppCurrentCommand'
  real_T ipAppCurrentMode;             // '<Root>/ipAppCurrentMode'
} ExtU_SweeperDriveChassisContr_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  boolean_T opSweeperDriveModeEnable;  // '<Root>/opSweeperDriveModeEnable'
  boolean_T opSweeperSteeringEnable;   // '<Root>/opSweeperSteeringEnable'
  real_T opSweeperSpeedValue;          // '<Root>/opSweeperSpeedValue'
  real_T opSweeperSteeringAngle;       // '<Root>/opSweeperSteeringAngle'
  real_T opSweeperSteeringAngleSpeed;  // '<Root>/opSweeperSteeringAngleSpeed'
  boolean_T opSweeperSweepEnable;      // '<Root>/opSweeperSweepEnable'
} ExtY_SweeperDriveChassisContr_T;

// Real-time Model Data Structure
struct tag_RTM_SweeperDriveChassisCo_T {
  const char_T * volatile errorStatus;
};

// Class declaration for model SweeperDriveChassisControl
class SweeperDriveChassisControlModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_SweeperDriveChassisContr_T SweeperDriveChassisControl_U;

  // External outputs
  ExtY_SweeperDriveChassisContr_T SweeperDriveChassisControl_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  SweeperDriveChassisControlModelClass();

  // Destructor
  ~SweeperDriveChassisControlModelClass();

  // Real-Time Model get method
  RT_MODEL_SweeperDriveChassisC_T * getRTM();

  // private data and function members
 private:
  // Real-Time Model
  RT_MODEL_SweeperDriveChassisC_T SweeperDriveChassisControl_M;
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
//  '<Root>' : 'SweeperDriveChassisControl'
//  '<S1>'   : 'SweeperDriveChassisControl/Compare To Constant1'
//  '<S2>'   : 'SweeperDriveChassisControl/Compare To Constant2'
//  '<S3>'   : 'SweeperDriveChassisControl/Compare To Constant3'
//  '<S4>'   : 'SweeperDriveChassisControl/Compare To Constant4'
//  '<S5>'   : 'SweeperDriveChassisControl/Compare To Constant5'
//  '<S6>'   : 'SweeperDriveChassisControl/DriveMode'
//  '<S7>'   : 'SweeperDriveChassisControl/OutputControl'
//  '<S8>'   : 'SweeperDriveChassisControl/WorkStatus'
//  '<S9>'   : 'SweeperDriveChassisControl/DriveMode/Compare To Constant1'
//  '<S10>'  : 'SweeperDriveChassisControl/DriveMode/Compare To Constant4'
//  '<S11>'  : 'SweeperDriveChassisControl/WorkStatus/Compare To Constant1'
//  '<S12>'  : 'SweeperDriveChassisControl/WorkStatus/Compare To Constant4'

#endif                                 // RTW_HEADER_SweeperDriveChassisControl_h_ 

//
// File trailer for generated code.
//
// [EOF]
//

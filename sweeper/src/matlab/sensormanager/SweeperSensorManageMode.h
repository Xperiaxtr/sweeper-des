//
// File: SweeperSensorManageMode.h
//
// Code generated for Simulink model 'SweeperSensorManageMode'.
//
// Model version                  : 1.39
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Sun Mar 22 19:54:02 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 7
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_SweeperSensorManageMode_h_
#define RTW_HEADER_SweeperSensorManageMode_h_
#include <string.h>
#include <stddef.h>
#ifndef SweeperSensorManageMode_COMMON_INCLUDES_
# define SweeperSensorManageMode_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // SweeperSensorManageMode_COMMON_INCLUDES_ 

#include "SweeperSensorManageMode_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// External inputs (root inport signals with default storage)
typedef struct {
  real_T ipDjlidarProcess;             // '<Root>/ipDjlidarProcess'
  real_T ipDjlidarResetTime;           // '<Root>/ipDjlidarResetTime'
  real_T ipDjlidarRefreshTime;         // '<Root>/ipDjlidarRefreshTime'
  int32_T ipDjlidarFaultCode;          // '<Root>/ipDjlidarFaultCode'
  boolean_T ipDjlidarRelayState;       // '<Root>/ipDjlidarRelayState'
  real_T ipDjlidarRelayResetTime;      // '<Root>/ipDjlidarRelayResetTime'
  real_T ipWjlidarProcess;             // '<Root>/ipWjlidarProcess'
  real_T ipWjlidarResetTime;           // '<Root>/ipWjlidarResetTime'
  real_T ipWjlidarRefreshTime;         // '<Root>/ipWjlidarRefreshTime'
  int32_T ipWjlidarFaultCode;          // '<Root>/ipWjlidarFaultCode'
  boolean_T ipWjlidarRelayState;       // '<Root>/ipWjlidarRelayState'
  real_T ipWjlidarRelayResetTime;      // '<Root>/ipWjlidarRelayResetTime'
} ExtU_SweeperSensorManageMode_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  boolean_T opDjlidarSensorReset;      // '<Root>/opDjlidarSensorReset'
  boolean_T opWjlidarSensorReset;      // '<Root>/opWjlidarSensorReset'
} ExtY_SweeperSensorManageMode_T;

// Real-time Model Data Structure
struct tag_RTM_SweeperSensorManageMo_T {
  const char_T * volatile errorStatus;
};

// Class declaration for model SweeperSensorManageMode
class SweeperSensorManageModeModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_SweeperSensorManageMode_T SweeperSensorManageMode_U;

  // External outputs
  ExtY_SweeperSensorManageMode_T SweeperSensorManageMode_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  SweeperSensorManageModeModelClass();

  // Destructor
  ~SweeperSensorManageModeModelClass();

  // Real-Time Model get method
  RT_MODEL_SweeperSensorManageM_T * getRTM();

  // private data and function members
 private:
  // Real-Time Model
  RT_MODEL_SweeperSensorManageM_T SweeperSensorManageMode_M;
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
//  '<Root>' : 'SweeperSensorManageMode'
//  '<S1>'   : 'SweeperSensorManageMode/CyclicTask'
//  '<S2>'   : 'SweeperSensorManageMode/CyclicTask/DjlidarSensorResetManager'
//  '<S3>'   : 'SweeperSensorManageMode/CyclicTask/WjlidarSensorResetManager'
//  '<S4>'   : 'SweeperSensorManageMode/CyclicTask/DjlidarSensorResetManager/Compare To Constant'
//  '<S5>'   : 'SweeperSensorManageMode/CyclicTask/DjlidarSensorResetManager/Compare To Constant6'
//  '<S6>'   : 'SweeperSensorManageMode/CyclicTask/WjlidarSensorResetManager/Compare To Constant'
//  '<S7>'   : 'SweeperSensorManageMode/CyclicTask/WjlidarSensorResetManager/Compare To Constant6'

#endif                                 // RTW_HEADER_SweeperSensorManageMode_h_

//
// File trailer for generated code.
//
// [EOF]
//

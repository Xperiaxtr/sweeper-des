//
// File: SweeperNodeManageMode.h
//
// Code generated for Simulink model 'SweeperNodeManageMode'.
//
// Model version                  : 1.222
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri Oct 30 14:38:54 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 7
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_SweeperNodeManageMode_h_
#define RTW_HEADER_SweeperNodeManageMode_h_
#include <string.h>
#include <stddef.h>
#ifndef SweeperNodeManageMode_COMMON_INCLUDES_
# define SweeperNodeManageMode_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // SweeperNodeManageMode_COMMON_INCLUDES_ 

#include "SweeperNodeManageMode_types.h"

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
  boolean_T ipDjlidarRelayState;       // '<Root>/ipDjlidarRelayState'
  real_T ipDjlidarRelayResetTime;      // '<Root>/ipDjlidarRelayResetTime'
  int32_T ipDjlidarFaultCode;          // '<Root>/ipDjlidarFaultCode'
  real_T ipPlanningProcess;            // '<Root>/ipPlanningProcess'
  real_T ipPlanningResetTime;          // '<Root>/ipPlanningResetTime'
  real_T ipPlanningRefreshTime;        // '<Root>/ipPlanningRefreshTime'
  real_T ipControlProcess;             // '<Root>/ipControlProcess'
  real_T ipControlResetTime;           // '<Root>/ipControlResetTime'
  real_T ipControlRefreshTime;         // '<Root>/ipControlRefreshTime'
  real_T ipFusionLidarProcess;         // '<Root>/ipFusionLidarProcess'
  real_T ipFusionLidarResetTime;       // '<Root>/ipFusionLidarResetTime'
  real_T ipFusionLidarRefreshTime;     // '<Root>/ipFusionLidarRefreshTime'
  real_T ipWjlidarProcess;             // '<Root>/ipWjlidarProcess'
  real_T ipWjlidarResetTime;           // '<Root>/ipWjlidarResetTime'
  real_T ipWjlidarRefreshTime;         // '<Root>/ipWjlidarRefreshTime'
  boolean_T ipWjlidarRelayState;       // '<Root>/ipWjlidarRelayState'
  real_T ipWjlidarRelayResetTime;      // '<Root>/ipWjlidarRelayResetTime'
  int32_T ipWjlidarFaultCode;          // '<Root>/ipWjlidarFaultCode'
  real_T ipGnssProcess;                // '<Root>/ipGnssProcess'
  real_T ipGnssResetTime;              // '<Root>/ipGnssResetTime'
  real_T ipGnssRefreshTime;            // '<Root>/ipGnssRefreshTime'
  int32_T ipGnssFaultCode;             // '<Root>/ipGnssFaultCode'
  real_T ipImuProcess;                 // '<Root>/ipImuProcess'
  real_T ipImuResetTime;               // '<Root>/ipImuResetTime'
  real_T ipImuRefreshTime;             // '<Root>/ipImuRefreshTime'
  int32_T ipImuFaultCode;              // '<Root>/ipImuFaultCode'
  real_T ipMappingProcess;             // '<Root>/ipMappingProcess'
  real_T ipMappingResetTime;           // '<Root>/ipMappingResetTime'
  real_T ipMappingRefreshTime;         // '<Root>/ipMappingRefreshTime'
  real_T ipLocationProcess;            // '<Root>/ipLocationProcess'
  real_T ipLocationResetTime;          // '<Root>/ipLocationResetTime'
  real_T ipLocationRefreshTime;        // '<Root>/ipLocationRefreshTime'
  real_T ipAppProcess;                 // '<Root>/ipAppProcess'
  real_T ipAppResetTime;               // '<Root>/ipAppResetTime'
  real_T ipAppRefreshTime;             // '<Root>/ipAppRefreshTime'
  int32_T ipAppFaultCode;              // '<Root>/ipAppFaultCode'
  real_T ipV2XProcess;                 // '<Root>/ipV2XProcess'
  real_T ipV2XResetTime;               // '<Root>/ipV2XResetTime'
  real_T ipV2XRefreshTime;             // '<Root>/ipV2XRefreshTime'
  real_T ipMatcherProcess;             // '<Root>/ipMatcherProcess'
  real_T ipFusionGpsImuProcess;        // '<Root>/ipFusionGpsImuProcess'
  real_T ipFusionImuProcess;           // '<Root>/ipFusionImuProcess'
  boolean_T ipNetState;                // '<Root>/ipNetState'
} ExtU_SweeperNodeManageMode_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  boolean_T opDjlidarReset;            // '<Root>/opDjlidarReset'
  boolean_T opPlanningReset;           // '<Root>/opPlanningReset'
  boolean_T opControlReset;            // '<Root>/opControlReset'
  boolean_T opFusionLidarReset;        // '<Root>/opFusionLidarReset'
  boolean_T opWjlidarReset;            // '<Root>/opWjlidarReset'
  boolean_T opGnssReset;               // '<Root>/opGnssReset'
  boolean_T opV2XReset;                // '<Root>/opV2XReset'
  boolean_T opImuReset;                // '<Root>/opImuReset'
  boolean_T opMappingReset;            // '<Root>/opMappingReset'
  boolean_T opLocationReset;           // '<Root>/opLocationReset'
  boolean_T opAppReset;                // '<Root>/opAppReset'
  boolean_T opLidarOdomReset;          // '<Root>/opLidarOdomReset'
  boolean_T opFusionGpsImuReset;       // '<Root>/opFusionGpsImuReset'
  boolean_T opFusionImuReset;          // '<Root>/opFusionImuReset'
} ExtY_SweeperNodeManageMode_T;

// Real-time Model Data Structure
struct tag_RTM_SweeperNodeManageMode_T {
  const char_T * volatile errorStatus;
};

// Class declaration for model SweeperNodeManageMode
class SweeperNodeManageModeModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_SweeperNodeManageMode_T SweeperNodeManageMode_U;

  // External outputs
  ExtY_SweeperNodeManageMode_T SweeperNodeManageMode_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  SweeperNodeManageModeModelClass();

  // Destructor
  ~SweeperNodeManageModeModelClass();

  // Real-Time Model get method
  RT_MODEL_SweeperNodeManageMod_T * getRTM();

  // private data and function members
 private:
  // Real-Time Model
  RT_MODEL_SweeperNodeManageMod_T SweeperNodeManageMode_M;
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
//  '<Root>' : 'SweeperNodeManageMode'
//  '<S1>'   : 'SweeperNodeManageMode/AppProcessCheck'
//  '<S2>'   : 'SweeperNodeManageMode/ControlProcessCheck'
//  '<S3>'   : 'SweeperNodeManageMode/DjlidarProcessCheck'
//  '<S4>'   : 'SweeperNodeManageMode/FusionGpsImuProcessCheck'
//  '<S5>'   : 'SweeperNodeManageMode/FusionImuProcessCheck'
//  '<S6>'   : 'SweeperNodeManageMode/FusionProcessCheck'
//  '<S7>'   : 'SweeperNodeManageMode/FusionProcessCheck1'
//  '<S8>'   : 'SweeperNodeManageMode/GnssProcessCheck'
//  '<S9>'   : 'SweeperNodeManageMode/ImuProcessCheck'
//  '<S10>'  : 'SweeperNodeManageMode/LidarOdomProcessCheck'
//  '<S11>'  : 'SweeperNodeManageMode/LocationProcessCheck'
//  '<S12>'  : 'SweeperNodeManageMode/MappingProcessCheck'
//  '<S13>'  : 'SweeperNodeManageMode/PlanningProcessCheck'
//  '<S14>'  : 'SweeperNodeManageMode/WjlidarProcessCheck'
//  '<S15>'  : 'SweeperNodeManageMode/AppProcessCheck/Compare To Constant1'
//  '<S16>'  : 'SweeperNodeManageMode/AppProcessCheck/Compare To Constant6'
//  '<S17>'  : 'SweeperNodeManageMode/ControlProcessCheck/Compare To Constant'
//  '<S18>'  : 'SweeperNodeManageMode/DjlidarProcessCheck/Compare To Constant'
//  '<S19>'  : 'SweeperNodeManageMode/DjlidarProcessCheck/Compare To Constant6'
//  '<S20>'  : 'SweeperNodeManageMode/FusionProcessCheck/Compare To Constant1'
//  '<S21>'  : 'SweeperNodeManageMode/FusionProcessCheck1/Compare To Constant1'
//  '<S22>'  : 'SweeperNodeManageMode/GnssProcessCheck/Compare To Constant'
//  '<S23>'  : 'SweeperNodeManageMode/GnssProcessCheck/Compare To Constant1'
//  '<S24>'  : 'SweeperNodeManageMode/GnssProcessCheck/Compare To Constant6'
//  '<S25>'  : 'SweeperNodeManageMode/ImuProcessCheck/Compare To Constant1'
//  '<S26>'  : 'SweeperNodeManageMode/ImuProcessCheck/Compare To Constant2'
//  '<S27>'  : 'SweeperNodeManageMode/LocationProcessCheck/Compare To Constant1'
//  '<S28>'  : 'SweeperNodeManageMode/MappingProcessCheck/Compare To Constant1'
//  '<S29>'  : 'SweeperNodeManageMode/PlanningProcessCheck/Compare To Constant'
//  '<S30>'  : 'SweeperNodeManageMode/WjlidarProcessCheck/Compare To Constant'
//  '<S31>'  : 'SweeperNodeManageMode/WjlidarProcessCheck/Compare To Constant1'

#endif                                 // RTW_HEADER_SweeperNodeManageMode_h_

//
// File trailer for generated code.
//
// [EOF]
//

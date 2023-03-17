//
// File: SweeperFaultManageMode.h
//
// Code generated for Simulink model 'SweeperFaultManageMode'.
//
// Model version                  : 1.200
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Sat Dec  5 09:22:16 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 7
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_SweeperFaultManageMode_h_
#define RTW_HEADER_SweeperFaultManageMode_h_
#include <string.h>
#include <stddef.h>
#ifndef SweeperFaultManageMode_COMMON_INCLUDES_
# define SweeperFaultManageMode_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // SweeperFaultManageMode_COMMON_INCLUDES_ 

#include "SweeperFaultManageMode_types.h"

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
  real_T ipDjlidarRefreshTime;         // '<Root>/ipDjlidarRefreshTime'
  int32_T ipDjlidarFaultCode;          // '<Root>/ipDjlidarFaultCode'
  boolean_T ipDjlidarRelayState;       // '<Root>/ipDjlidarRelayState'
  real_T ipPlanningProcess;            // '<Root>/ipPlanningProcess'
  real_T ipPlanningRefreshTime;        // '<Root>/ipPlanningRefreshTime'
  real_T ipControlProcess;             // '<Root>/ipControlProcess'
  real_T ipControlRefreshTime;         // '<Root>/ipControlRefreshTime'
  real_T ipFusionLidarProcess;         // '<Root>/ipFusionLidarProcess'
  real_T ipFusionLidarRefreshTime;     // '<Root>/ipFusionLidarRefreshTime'
  real_T ipWjlidarProcess;             // '<Root>/ipWjlidarProcess'
  real_T ipWjlidarRefreshTime;         // '<Root>/ipWjlidarRefreshTime'
  boolean_T ipWjlidarRelayState;       // '<Root>/ipWjlidarRelayState'
  boolean_T ipWjlidarFaultCode;        // '<Root>/ipWjlidarFaultCode'
  real_T ipGnssProcess;                // '<Root>/ipGnssProcess'
  real_T ipGnssRefreshTime;            // '<Root>/ipGnssRefreshTime'
  int32_T ipGnssFaultCode;             // '<Root>/ipGnssFaultCode'
  real_T ipImuProcess;                 // '<Root>/ipImuProcess'
  real_T ipImuRefreshTime;             // '<Root>/ipImuRefreshTime'
  int32_T ipImuFaultCode;              // '<Root>/ipImuFaultCode'
  real_T ipMappingProcess;             // '<Root>/ipMappingProcess'
  real_T ipMappingRefreshTime;         // '<Root>/ipMappingRefreshTime'
  real_T ipLocationProcess;            // '<Root>/ipLocationProcess'
  real_T ipLocationRefreshTime;        // '<Root>/ipLocationRefreshTime'
  real_T ipAppProcess;                 // '<Root>/ipAppProcess'
  real_T ipAppRefreshTime;             // '<Root>/ipAppRefreshTime'
  real_T ipV2XProcess;                 // '<Root>/ipV2XProcess'
  real_T ipV2XRefreshTime;             // '<Root>/ipV2XRefreshTime'
  real_T ipMatcherProcess;             // '<Root>/ipMatcherProcess'
  real_T ipFusionGpsImuProcess;        // '<Root>/ipFusionGpsImuProcess'
  real_T ipFusionImuProcess;           // '<Root>/ipFusionImuProcess'
  real_T ipCmdRefreshTime;             // '<Root>/ipCmdRefreshTime'
  real_T ipCanRefreshTime;             // '<Root>/ipCanRefreshTime'
  real_T ipDcuFaultState;              // '<Root>/ipDcuFaultState'
  real_T ipEmergencyStop;              // '<Root>/ipEmergencyStop'
  boolean_T ipNetState;                // '<Root>/ipNetState'
  real_T ipRunMode;                    // '<Root>/ipRunMode'
} ExtU_SweeperFaultManageMode_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  boolean_T opAutoDriveReady;          // '<Root>/opAutoDriveReady'
  real_T opCSUFaultLevel;              // '<Root>/opCSUFaultLevel'
  real_T opCSUFaultCode[32];           // '<Root>/opCSUFaultCode'
  real_T opDjlidarFaultCode;           // '<Root>/opDjlidarFaultCode'
  real_T opGnssFaultCode;              // '<Root>/opGnssFaultCode'
  real_T opWjlidarLeftFaultCode;       // '<Root>/opWjlidarLeftFaultCode'
  real_T opWjlidarRightFaultCode;      // '<Root>/opWjlidarRightFaultCode'
  real_T opImuFaultCode;               // '<Root>/opImuFaultCode'
} ExtY_SweeperFaultManageMode_T;

// Real-time Model Data Structure
struct tag_RTM_SweeperFaultManageMod_T {
  const char_T * volatile errorStatus;
};

// Class declaration for model SweeperFaultManageMode
class SweeperFaultManageModeModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_SweeperFaultManageMode_T SweeperFaultManageMode_U;

  // External outputs
  ExtY_SweeperFaultManageMode_T SweeperFaultManageMode_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  SweeperFaultManageModeModelClass();

  // Destructor
  ~SweeperFaultManageModeModelClass();

  // Real-Time Model get method
  RT_MODEL_SweeperFaultManageMo_T * getRTM();

  // private data and function members
 private:
  // Real-Time Model
  RT_MODEL_SweeperFaultManageMo_T SweeperFaultManageMode_M;
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
//  '<Root>' : 'SweeperFaultManageMode'
//  '<S1>'   : 'SweeperFaultManageMode/AutoDriveReady'
//  '<S2>'   : 'SweeperFaultManageMode/CommunicationFault'
//  '<S3>'   : 'SweeperFaultManageMode/FaultCode'
//  '<S4>'   : 'SweeperFaultManageMode/HardFault'
//  '<S5>'   : 'SweeperFaultManageMode/ProcessCheck'
//  '<S6>'   : 'SweeperFaultManageMode/SensorFualtCode'
//  '<S7>'   : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant1'
//  '<S8>'   : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant10'
//  '<S9>'   : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant11'
//  '<S10>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant2'
//  '<S11>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant28'
//  '<S12>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant29'
//  '<S13>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant3'
//  '<S14>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant30'
//  '<S15>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant31'
//  '<S16>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant32'
//  '<S17>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant33'
//  '<S18>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant34'
//  '<S19>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant4'
//  '<S20>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant5'
//  '<S21>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant6'
//  '<S22>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant7'
//  '<S23>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant8'
//  '<S24>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Constant9'
//  '<S25>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Zero1'
//  '<S26>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Zero10'
//  '<S27>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Zero11'
//  '<S28>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Zero8'
//  '<S29>'  : 'SweeperFaultManageMode/AutoDriveReady/Compare To Zero9'
//  '<S30>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem'
//  '<S31>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1'
//  '<S32>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem2'
//  '<S33>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3'
//  '<S34>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4'
//  '<S35>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Constant14'
//  '<S36>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Constant19'
//  '<S37>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Constant40'
//  '<S38>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Constant41'
//  '<S39>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Constant42'
//  '<S40>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Constant43'
//  '<S41>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Constant44'
//  '<S42>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Constant45'
//  '<S43>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Constant46'
//  '<S44>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Zero13'
//  '<S45>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Zero14'
//  '<S46>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Zero15'
//  '<S47>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Zero16'
//  '<S48>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem/Compare To Zero3'
//  '<S49>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Constant13'
//  '<S50>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Constant16'
//  '<S51>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Constant17'
//  '<S52>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Constant2'
//  '<S53>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Constant26'
//  '<S54>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Constant5'
//  '<S55>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Constant7'
//  '<S56>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Zero'
//  '<S57>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Zero1'
//  '<S58>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Zero2'
//  '<S59>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Zero3'
//  '<S60>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem1/Compare To Zero6'
//  '<S61>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem2/Compare To Constant22'
//  '<S62>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem2/Compare To Constant25'
//  '<S63>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem2/Compare To Zero5'
//  '<S64>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Constant1'
//  '<S65>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Constant13'
//  '<S66>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Constant16'
//  '<S67>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Constant17'
//  '<S68>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Constant2'
//  '<S69>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Constant26'
//  '<S70>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Constant3'
//  '<S71>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Constant4'
//  '<S72>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Constant5'
//  '<S73>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Constant7'
//  '<S74>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Zero'
//  '<S75>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Zero1'
//  '<S76>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Zero2'
//  '<S77>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Zero3'
//  '<S78>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Zero4'
//  '<S79>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Zero5'
//  '<S80>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Zero6'
//  '<S81>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Zero7'
//  '<S82>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Zero8'
//  '<S83>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem3/Compare To Zero9'
//  '<S84>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Constant1'
//  '<S85>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Constant2'
//  '<S86>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Constant25'
//  '<S87>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Constant3'
//  '<S88>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Constant4'
//  '<S89>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Constant46'
//  '<S90>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Zero1'
//  '<S91>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Zero2'
//  '<S92>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Zero3'
//  '<S93>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Zero5'
//  '<S94>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Zero6'
//  '<S95>'  : 'SweeperFaultManageMode/AutoDriveReady/Subsystem4/Compare To Zero7'
//  '<S96>'  : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant'
//  '<S97>'  : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant1'
//  '<S98>'  : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant10'
//  '<S99>'  : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant11'
//  '<S100>' : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant12'
//  '<S101>' : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant2'
//  '<S102>' : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant3'
//  '<S103>' : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant4'
//  '<S104>' : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant5'
//  '<S105>' : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant6'
//  '<S106>' : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant7'
//  '<S107>' : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant8'
//  '<S108>' : 'SweeperFaultManageMode/CommunicationFault/Compare To Constant9'
//  '<S109>' : 'SweeperFaultManageMode/SensorFualtCode/Compare To Constant1'
//  '<S110>' : 'SweeperFaultManageMode/SensorFualtCode/Compare To Constant2'
//  '<S111>' : 'SweeperFaultManageMode/SensorFualtCode/Compare To Constant3'
//  '<S112>' : 'SweeperFaultManageMode/SensorFualtCode/Compare To Constant31'
//  '<S113>' : 'SweeperFaultManageMode/SensorFualtCode/Compare To Constant4'
//  '<S114>' : 'SweeperFaultManageMode/SensorFualtCode/Compare To Zero1'
//  '<S115>' : 'SweeperFaultManageMode/SensorFualtCode/Compare To Zero2'
//  '<S116>' : 'SweeperFaultManageMode/SensorFualtCode/Compare To Zero3'
//  '<S117>' : 'SweeperFaultManageMode/SensorFualtCode/Compare To Zero4'
//  '<S118>' : 'SweeperFaultManageMode/SensorFualtCode/Compare To Zero5'
//  '<S119>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar'
//  '<S120>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_gnss'
//  '<S121>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_imu'
//  '<S122>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_wjlidar'
//  '<S123>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_wjlidar1'
//  '<S124>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero1'
//  '<S125>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero10'
//  '<S126>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero11'
//  '<S127>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero12'
//  '<S128>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero13'
//  '<S129>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero14'
//  '<S130>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero15'
//  '<S131>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero16'
//  '<S132>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero6'
//  '<S133>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero7'
//  '<S134>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero8'
//  '<S135>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_djlidar/Compare To Zero9'
//  '<S136>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_gnss/Compare To Zero10'
//  '<S137>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_gnss/Compare To Zero11'
//  '<S138>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_gnss/Compare To Zero12'
//  '<S139>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_gnss/Compare To Zero13'
//  '<S140>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_gnss/Compare To Zero16'
//  '<S141>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_gnss/Compare To Zero7'
//  '<S142>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_gnss/Compare To Zero8'
//  '<S143>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_gnss/Compare To Zero9'
//  '<S144>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_imu/Compare To Zero1'
//  '<S145>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_imu/Compare To Zero16'
//  '<S146>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_wjlidar/Compare To Zero1'
//  '<S147>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_wjlidar/Compare To Zero16'
//  '<S148>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_wjlidar/Compare To Zero6'
//  '<S149>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_wjlidar1/Compare To Zero1'
//  '<S150>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_wjlidar1/Compare To Zero16'
//  '<S151>' : 'SweeperFaultManageMode/SensorFualtCode/code2value_wjlidar1/Compare To Zero6'

#endif                                 // RTW_HEADER_SweeperFaultManageMode_h_

//
// File trailer for generated code.
//
// [EOF]
//

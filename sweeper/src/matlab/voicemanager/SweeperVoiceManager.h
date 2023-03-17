//
// File: SweeperVoiceManager.h
//
// Code generated for Simulink model 'SweeperVoiceManager'.
//
// Model version                  : 1.55
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Tue Dec  8 13:48:47 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Emulation hardware selection:
//    Differs from embedded hardware (MATLAB Host)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_SweeperVoiceManager_h_
#define RTW_HEADER_SweeperVoiceManager_h_
#include <string.h>
#include <stddef.h>
#ifndef SweeperVoiceManager_COMMON_INCLUDES_
# define SweeperVoiceManager_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // SweeperVoiceManager_COMMON_INCLUDES_

#include "SweeperVoiceManager_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block states (default storage) for system '<Root>'
typedef struct {
  uint8_T is_active_c3_SweeperVoiceManage;// '<Root>/SweeperVoiceManagerModel'
  uint8_T is_VoiceState;               // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_VoiceManager;             // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_Side;                     // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_gps;                      // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_Drive;                    // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_AutoDriving;              // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_SweeperWalkStatus;        // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_Run;                      // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_edge_status;              // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_width_status;             // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_left_front;               // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_right_front;              // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_left;                     // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_right;                    // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_far;                      // '<Root>/SweeperVoiceManagerModel'
  uint8_T is_lidar_s;                  // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i1;          // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i2;          // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i3;          // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i4;          // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i5;          // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i6;          // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i7;          // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i8;          // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i9;          // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i10;         // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i11;         // '<Root>/SweeperVoiceManagerModel'
  uint8_T temporalCounter_i12;         // '<Root>/SweeperVoiceManagerModel'
  boolean_T voice_finish_status;       // '<Root>/SweeperVoiceManagerModel'
} DW_SweeperVoiceManager_T;

// External inputs (root inport signals with default storage)
typedef struct {
  real_T ip_cus_drive_mode;            // '<Root>/ip_cus_drive_mode'
  real_T ip_dcu_drive_mode;            // '<Root>/ip_dcu_drive_mode'
  real_T ip_sweeper_speed;             // '<Root>/ip_sweeper_speed'
  real_T ip_sweeper_mode;              // '<Root>/ip_sweeper_mode'
  real_T ip_sweeper_ready;             // '<Root>/ip_sweeper_ready'
  real_T ip_fault_level;               // '<Root>/ip_fault_level'
  real_T ip_sweeper_width;             // '<Root>/ip_sweeper_width'
  real_T ip_sweeper_edge;              // '<Root>/ip_sweeper_edge'
  real_T ip_sweeper_turn;              // '<Root>/ip_sweeper_turn'
  real_T ip_sweeper_stop;              // '<Root>/ip_sweeper_stop'
  real_T ip_sweeper_side;              // '<Root>/ip_sweeper_side'
  real_T ip_sweeper_left_front;        // '<Root>/ip_sweeper_left_front'
  real_T ip_sweeper_right_front;       // '<Root>/ip_sweeper_right_front'
  real_T ip_sweeper_left;              // '<Root>/ip_sweeper_left'
  real_T ip_sweeper_right;             // '<Root>/ip_sweeper_right'
  real_T ip_sweeper_far;               // '<Root>/ip_sweeper_far'
  real_T ip_sweeper_lidar_s;           // '<Root>/ip_sweeper_lidar_s'
  int32_T ip_sweeper_code;             // '<Root>/ip_sweeper_code'
} ExtU_SweeperVoiceManager_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real_T op_voice_mode;                // '<Root>/op_voice_mode'
} ExtY_SweeperVoiceManager_T;

// Real-time Model Data Structure
struct tag_RTM_SweeperVoiceManager_T {
  const char_T * volatile errorStatus;
};

// Class declaration for model SweeperVoiceManager
class SweeperVoiceManagerModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_SweeperVoiceManager_T SweeperVoiceManager_U;

  // External outputs
  ExtY_SweeperVoiceManager_T SweeperVoiceManager_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  SweeperVoiceManagerModelClass();

  // Destructor
  ~SweeperVoiceManagerModelClass();

  // Real-Time Model get method
  RT_MODEL_SweeperVoiceManager_T * getRTM();

  // private data and function members
 private:
  // Block states
  DW_SweeperVoiceManager_T SweeperVoiceManager_DW;

  // Real-Time Model
  RT_MODEL_SweeperVoiceManager_T SweeperVoiceManager_M;

  // private member function(s) for subsystem '<Root>'
  void SweeperVoi_enter_internal_Nomal(void);
  void SweeperVoiceM_SweeperWalkStatus(void);
  void SweeperVoic_exit_internal_Nomal(void);
  void SweeperVoiceManager_Nomal(void);
  void SweeperVoiceManager_Run(const boolean_T *LogicalOperator5);
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
//  '<Root>' : 'SweeperVoiceManager'
//  '<S1>'   : 'SweeperVoiceManager/SweeperVoiceManagerModel'
//  '<S2>'   : 'SweeperVoiceManager/gps_status'
//  '<S3>'   : 'SweeperVoiceManager/gps_status/Compare To Constant1'
//  '<S4>'   : 'SweeperVoiceManager/gps_status/Compare To Constant2'
//  '<S5>'   : 'SweeperVoiceManager/gps_status/Compare To Constant3'
//  '<S6>'   : 'SweeperVoiceManager/gps_status/Compare To Constant6'

#endif                                 // RTW_HEADER_SweeperVoiceManager_h_

//
// File trailer for generated code.
//
// [EOF]
//

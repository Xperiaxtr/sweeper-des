//
// File: gps_record_line_model.h
//
// Code generated for Simulink model 'gps_record_line_model'.
//
// Model version                  : 1.99
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Mon Aug 10 15:34:48 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_gps_record_line_model_h_
#define RTW_HEADER_gps_record_line_model_h_
#include <cmath>
#include <string.h>
#include <stddef.h>
#ifndef gps_record_line_model_COMMON_INCLUDES_
# define gps_record_line_model_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // gps_record_line_model_COMMON_INCLUDES_ 

#include "gps_record_line_model_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block states (default storage) for system '<Root>'
typedef struct {
  real_T Delay1_DSTATE;                // '<S2>/Delay1'
} DW_gps_record_line_model_T;

// External inputs (root inport signals with default storage)
typedef struct {
  real_T ip_gps_cmd;                   // '<Root>/ip_gps_cmd'
  boolean_T ip_gps_signal_status;      // '<Root>/ip_gps_signal_status'
  real_T ip_gps_point_distance;        // '<Root>/ip_gps_point_distance'
} ExtU_gps_record_line_model_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real_T op_gps_point_type;            // '<Root>/op_gps_point_type'
  boolean_T op_gps_save_cmd;           // '<Root>/op_gps_save_cmd'
  uint16_T op_gps_crossing_num;        // '<Root>/op_gps_crossing_num'
} ExtY_gps_record_line_model_T;

// Real-time Model Data Structure
struct tag_RTM_gps_record_line_model_T {
  const char_T * volatile errorStatus;
};

// Class declaration for model gps_record_line_model
class gps_record_line_modelModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_gps_record_line_model_T gps_record_line_model_U;

  // External outputs
  ExtY_gps_record_line_model_T gps_record_line_model_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  gps_record_line_modelModelClass();

  // Destructor
  ~gps_record_line_modelModelClass();

  // Real-Time Model get method
  RT_MODEL_gps_record_line_mode_T * getRTM();

  // private data and function members
 private:
  // Block states
  DW_gps_record_line_model_T gps_record_line_model_DW;

  // Real-Time Model
  RT_MODEL_gps_record_line_mode_T gps_record_line_model_M;
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
//  '<Root>' : 'gps_record_line_model'
//  '<S1>'   : 'gps_record_line_model/cmd_module'
//  '<S2>'   : 'gps_record_line_model/crossing_module'
//  '<S3>'   : 'gps_record_line_model/point_module'
//  '<S4>'   : 'gps_record_line_model/save_module'
//  '<S5>'   : 'gps_record_line_model/cmd_module/Compare To Constant1'
//  '<S6>'   : 'gps_record_line_model/cmd_module/Compare To Constant2'
//  '<S7>'   : 'gps_record_line_model/save_module/Compare To Constant'
//  '<S8>'   : 'gps_record_line_model/save_module/Compare To Constant1'

#endif                                 // RTW_HEADER_gps_record_line_model_h_

//
// File trailer for generated code.
//
// [EOF]
//

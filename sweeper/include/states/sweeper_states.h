/*
 *文件名称:sweeper_states.h
 *文件说明:所有状态类的基类
 */
#pragma once

#include <ros/ros.h>

#include <string>

#include "../../common/log.h"
#include "../common/data_center.h"
#include "sweeper_msgs/SensorFaultInformation.h"
#include "sweeper_msgs/SweepMission.h"

namespace sweeper {

class SweeperStates {
 public:  //状态定义
  enum STATE_NAME {
    INIT_STATE,
    IDLE_STATE,
    RECORD_GNSS_LINE_STATE,
    RECORD_LIDAR_LINE_STATE,
    GNSS_TRACE_STATE,
    LIDAR_TRACE_STATE,
    GLOBAL_SWEEP_STATE,
    CURB_SWEEP_STATE,
  };

  enum STARTCMD {
    CANCEL,          //　取消
    START,           //　开始
    PAUSE,           //　暂停
    SAVE,            //  保存
    DELETE_CURRENT,  //  取消当前路口轨迹
    DELETE_ENTIRE,   //  删除已有路径
  };

 public:
  SweeperStates(SweepDataCenter *sweep_data_center)
      : sweep_data_center_(sweep_data_center), last_cmd_(-1) {}

  bool IsChange(int state) {
    if (state != last_state_) {
      return true;
    } else {
      return false;
    }
  }

  /*
   *函数说明  :状态入口函数，每次切换状态需调用此函数
   *输入参数  :currentState：调用此函数之前处于的状态
   *输出参数  :无
   *备注     :
   */
  virtual void Enter(int state) { last_state_ = state; }

  /*
   *函数说明  :跟随main函数循环运行的函数，主循环循环一次该函数运行一次
   *输入参数  :无
   *输出参数  :无
   *备注     :
   */
  virtual void Run(void) {}

  /*
   *函数说明  :退出本循环函数，需要推出本循环时需先置bChangeStateFlag为true
   *          然后由main调用quit函数
   *输入参数  :无
   *输出参数  :要切换到的状态
   *备注     :进入此函数后应先置bChangeStateFlag为false
   */
  virtual void Quit(void) {}

  /*
   *函数说明  :执行此函数后，主控强行退出当前状态
   *输入参数  :无
   *输出参数  :无
   *备注
   */
  virtual void ForceQuit(void) {}

  /*
   *函数说明  :执行此函数，恢复该状态全局变量默认值
   *输入参数  :无
   *输出参数  :无
   *备注
   */
  virtual void Reset(void) {}

 protected:
  int last_state_;   // 上一个状态
  int last_cmd_;   // 上一清扫命令
  SweepDataCenter *sweep_data_center_;
  ros::Publisher mode_pub_;
};
}  // namespace sweeper

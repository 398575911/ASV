/*
***********************************************************************
* PLCdata.h:
* header file to define the constant and real-time data for
* communication to stm32
* This header file can be read by both C and C++ compilers
*
*  by Hu.ZH (CrossOcean.ai)
***********************************************************************
*/

#ifndef _PLCDATA_H_
#define _PLCDATA_H_

#include <string>
#include "common/communication/include/linkdata.h"

namespace ASV::messages {

enum class PLCSTATUS {
  STANDBY = 0,    // 等待PC连接
  INITE,          // 初始化
  RUNNING,        // 正常运行
  MANUAL,         // 手动控制
  AUTO,           // 自动模式
  ALARM,          // 报警
  EMERGENCY_STOP  // 急停
};

struct PLCdata {
  /********* from IPC to PLC **********/
  // motor
  int16_t Thruster_A_rpm_command;      // rpm
  int16_t Thruster_A_azimuth_command;  // deg
  int16_t Thruster_B_rpm_command;      // rpm
  int16_t Thruster_B_azimuth_command;  // deg

  // GPS
  float latitude;   // deg
  float longitude;  // deg
  float heading;    // deg

  // wind
  float wind_speed;
  int16_t wind_direction;

  /********* from PLC to IPC **********/
  // motor
  uint8_t Thruster_A_status;
  int16_t Thruster_A_rpm_feedback;      // rpm
  int16_t Thruster_A_azimuth_feedback;  // deg
  uint8_t Thruster_B_status;
  int16_t Thruster_B_rpm_feedback;      // rpm
  int16_t Thruster_B_azimuth_feedback;  // deg

  // power generator
  uint16_t power;  // W

  // PLCstatus
  PLCSTATUS feedback_stm32status;

  // link status
  common::LINKSTATUS linkstatus;

};  // PLCdata

}  // namespace ASV::messages

#endif /* _PLCDATA_H_ */
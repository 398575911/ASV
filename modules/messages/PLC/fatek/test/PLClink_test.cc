/*
*******************************************************************************
* PLClink_test.cc:
* unit test for PLC communication
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <chrono>
#include <iostream>
#include "../include/PLC_link.h"
#include "common/timer/include/timecounter.h"

using namespace ASV::messages;

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  // real time wind sensor
  PLCdata _PLCdata{
      0,                                     // Thruster_port_rpm_command
      0,                                     // Thruster_port_azimuth_command
      0,                                     // Thruster_star_rpm_command
      0,                                     // Thruster_star_azimuth_command
      0,                                     // latitude
      0,                                     // longitude
      0,                                     // heading
      0,                                     // speed
      0,                                     // wind_speed
      0,                                     // wind_direction
      0,                                     // Thruster_port_status
      0,                                     // Thruster_port_rpm_feedback
      0,                                     // Thruster_port_azimuth_feedback
      0,                                     // Thruster_star_status
      0,                                     // Thruster_star_rpm_feedback
      0,                                     // Thruster_star_azimuth_feedback
      0,                                     // power
      ASV::common::LINKSTATUS::DISCONNECTED  // linkstatus
  };

  ASV::common::timecounter _timer;
  long int totaltime = 0;

  try {
    PLC_link _PLC_link(_PLCdata, 115200, "/dev/ttyUSB0");  // zone 30n

    for (int i = 0; i != 100; i) {
      static int speed = 10;
      if (speed < 1000) speed += 10;
      _PLC_link
          .setupPLCdata(speed, -400, 30, 300, 114.2323232, 51.87342, 110.1,
                        -4.9, 1.2, 5)
          .PLConestep();
      _PLCdata = _PLC_link.getPLCdata();
      long int et = _timer.timeelapsed();

      printf("%02x\n", _PLCdata.Thruster_port_status);
      printf("%d\n", _PLCdata.Thruster_port_rpm_feedback);
      printf("%d\n", _PLCdata.Thruster_port_azimuth_feedback);
      printf("%02x\n", _PLCdata.Thruster_star_status);
      printf("%d\n", _PLCdata.Thruster_star_rpm_feedback);
      printf("%d\n", _PLCdata.Thruster_star_azimuth_feedback);
      printf("%d\n", _PLCdata.power);
      printf("%d\n", static_cast<int>(_PLCdata.linkstatus));

      std::cout << "sample time: " << et << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}

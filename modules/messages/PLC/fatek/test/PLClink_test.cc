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
      0,  // speed
      0   // orientation
  };

  ASV::common::timecounter _timer;
  long int totaltime = 0;

  try {
    PLC_link _PLC_link(_PLCdata, 115200, "/dev/ttyUSB0");  // zone 30n

    for (int i = 0; i != 100; i) {
      // static int speed = 10;
      // if (speed < 2000) speed += 10;
      _PLC_link
          .setupPLCdata(speed, 400, 30, 300, 134.2323232, 31.87342, 270.1, 1.2,
                        6)
          .PLConestep();
      long int et = _timer.timeelapsed();
      std::cout << "sample time: " << et << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}

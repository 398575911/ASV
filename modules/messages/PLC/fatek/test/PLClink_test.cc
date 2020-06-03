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

using namespace ASV::messages;

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  // real time wind sensor
  PLCdata _PLCdata{
      0,  // speed
      0   // orientation
  };

  try {
    PLC_link _PLC_link(_PLCdata, 9600, "/dev/ttyr3");  // zone 30n

    while (1) {
      _PLC_link.setupPLCdata(0, 0, 0, 0, 0, 0, 0).PLConestep();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}

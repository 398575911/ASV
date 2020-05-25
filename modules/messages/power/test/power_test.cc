/*
*******************************************************************************
* power_test.cc:
* unit test for power monitoring
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <chrono>
#include <iostream>
#include "../include/PowerMonitor.h"

using namespace ASV::messages;

int main() {
  // real time wind sensor
  ThreePhaseRTdata _ThreePhaseRTdata{
      0,  // U_power
      0,  // V_power
      0,  // W_power
      0,  // U_voltage
      0,  // V_voltage
      0,  // W_voltage
      0,  // U_current
      0,  // V_current
      0,  // W_current
  };

  try {
    PowerMonitor _PowerMonitor(57600, "/dev/ttyr7");  // zone 30n

    while (1) {
      _ThreePhaseRTdata = _PowerMonitor.ParsePowerMeter().GetThreePhaseRTdata();
      // std::cout << "wind speed: " << _windRTdata.speed << std::endl;
      // std::cout << "wind orientation: " << _windRTdata.orientation <<
      // std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}
/*
*******************************************************************************
* testwind.cc:
* unit test for serial communication and UTM projection for GPS/IMU
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <chrono>
#include <iostream>
#include "../include/wind.h"

using namespace ASV::messages;

int main() {
  // real time wind sensor
  windRTdata _windRTdata{
      0,  // speed
      0   // orientation
  };

  try {
    wind _wind(9600, "/dev/ttyr7");  // zone 30n

    while (1) {
      _windRTdata = _wind.readwind().getwindRTdata();
      std::cout << "wind speed: " << _windRTdata.speed << std::endl;
      std::cout << "wind orientation: " << _windRTdata.orientation << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}

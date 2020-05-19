/*
*******************************************************************************
* serial_test.cc:
* unit test for serial port
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/
#include <unistd.h>
#include <iostream>
#include "../include/serialport.h"
int main() {
  ASV::common::serialport serialport_("/dev/ttyr7", 115200, 100);

  for (int i = 0; i != 100; ++i) {
    auto read = serialport_.readline();
    // std::cout << "recieve: " << read << std::endl;
  }

  for (int i = 0; i != 100; ++i) {
    serialport_.writeline("testserial\n");
    // std::cout << "recieve: " << read << std::endl;
    sleep(1);
  }
}
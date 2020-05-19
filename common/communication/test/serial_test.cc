/*
*******************************************************************************
* serial_test.cc:
* unit test for serial port
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/
#include "../include/serialport.h"

int main() { ASV::common::serialport serialport_("/dev/ttyUSB0", 115200, 100); }
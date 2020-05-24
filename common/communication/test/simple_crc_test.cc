

/*
*******************************************************************************
* testcrc.cc:
* unit test for crc checking
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#define CRCPP_INCLUDE_ESOTERIC_CRC_DEFINITIONS

#include "../include/CRC.h"

#include <iomanip>   // Includes ::std::hex, ::std::setw
#include <iostream>  // Includes ::std::cerr, ::std::endl
#include <string>    // Includes ::std::string

int main() {
  static unsigned char CRC_CHECK_DATA[] = {0x02, 0x03, 0x00, 0x2A,
                                           0x00, 0x02, 0xE5, 0xF0};
  uint16_t computedCRC =
      CRC::Calculate<uint16_t, 16>(CRC_CHECK_DATA, 6, CRC::CRC_16_MODBUS());
  printf("%04x\n", computedCRC >> 8);
  printf("%04x\n", computedCRC & 0x00FF);
}
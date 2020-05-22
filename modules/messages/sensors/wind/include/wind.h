/*
 *************************************************
 *wind.h
 *function for read and parse the data from wind sensor
 *the header file can be read by C++ compilers
 *
 *by ZH.Hu, Ziheng Yang (CrossOcean.ai)
 *************************************************
 */

#ifndef _WIND_H_
#define _WIND_H_

#include <cmath>
#include "common/communication/include/serialport.h"
#include "winddata.h"

namespace ASV::messages {

class wind {
 public:
  explicit wind(int baud,  // baudrate
                const std::string& port = "/dev/ttyUSB0")
      : ser_wind_(port, baud, 1000) {}
  wind() = delete;
  ~wind() {}

  wind& readwind() {
    unsigned char buff_send_wind[] = {0x02, 0x03, 0x00, 0x2A,
                                      0x00, 0x02, 0xE5, 0xF0};

    char buff_rec[7];

    ser_wind_.writeline(buff_send_wind);
    ser_wind_.readline(buff_rec, 7);

    float data_wind = (buff_rec[4] + buff_rec[3] * 256) / 10.0;
    float data_angle = (buff_rec[6] + buff_rec[5] * 256) / 10.0;

    return *this;
  }  // readwind

  windRTdata getwindRTdata() const { return _windRTdata; }

 private:
  // serial data
  common::serialport ser_wind_;
  windRTdata _windRTdata;

  void restrictdata(int& _wspeed, int& _orientation) {
    if ((_wspeed < 0) || (_wspeed > 300)) _wspeed = 0;
    if ((_orientation < 0) || (_orientation >= 360)) _orientation = 0;
  }
};  // end class wind

}  // namespace ASV::messages

#endif /*_WIND_H_*/

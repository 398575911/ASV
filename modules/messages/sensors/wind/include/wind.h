/*
 *************************************************
 *wind.h
 *function for read and parse the data from wind sensor
 *the header file can be read by C++ compilers
 *
 *by ZH.Hu (SJTU)
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
    char buff_send_wind[8];
    char buff_send_angle[8];
    buff_send_wind[0] = 0x02;
    buff_send_wind[1] = 0x03;
    buff_send_wind[2] = 0x00;
    buff_send_wind[3] = 0x2A;
    buff_send_wind[4] = 0x00;
    buff_send_wind[5] = 0x01;
    buff_send_wind[6] = 0xA5;
    buff_send_wind[7] = 0xF1;
    buff_send_angle[0] = 0x02;
    buff_send_angle[1] = 0x03;
    buff_send_angle[2] = 0x00;
    buff_send_angle[3] = 0x2B;
    buff_send_angle[4] = 0x00;
    buff_send_angle[5] = 0x01;
    buff_send_angle[6] = 0xF4;
    buff_send_angle[7] = 0x31;

    char buff_rec[7];

    ser_wind_.writeline(buff_send_wind);
    ser_wind_.readline(buff_rec, 7);

    float data_wind = (buff_rec[4] + buff_rec[3] * 256) / 10.0;

    ser_wind_.writeline(buff_send_angle);
    ser_wind_.readline(buff_rec, 7);

    float data_angle = (buff_rec[4] + buff_rec[3] * 256) / 10.0;

    return *this;
  }  // readwind

  windRTdata getwindRTdata() const { return _windRTdata; }

 private:
  // serial data
  common::serialport ser_wind_;
  windRTdata _windRTdata;
  uint8_t s_buffer[7];

  void restrictdata(int& _wspeed, int& _orientation) {
    if ((_wspeed < 0) || (_wspeed > 300)) _wspeed = 0;
    if ((_orientation < 0) || (_orientation >= 360)) _orientation = 0;
  }
};  // end class wind

}  // namespace ASV::messages

#endif /*_WIND_H_*/

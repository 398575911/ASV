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

#include "common/communication/include/CRC.h"
#include "common/communication/include/serialport.h"
#include "winddata.h"

namespace ASV::messages {

class wind {
 public:
  explicit wind(int baud,  // baudrate
                const std::string& port = "/dev/ttyUSB0")
      : ser_wind_(port, baud, 1000), windRTdata_({0, 0}) {}
  wind() = delete;
  ~wind() {}

  wind& readwind() {
    static unsigned char buff_send_wind[] = {0x02, 0x03, 0x00, 0x2A,
                                             0x00, 0x02, 0xE5, 0xF0};

    unsigned char buff_rec[9];

    ser_wind_.writeline(buff_send_wind, 8);
    ser_wind_.readline(buff_rec, 9);

    uint16_t crc_result = ASV::common::CRC::Calculate<uint16_t, 16>(
        buff_rec, 7, ASV::common::CRC::CRC_16_MODBUS());

    if ((buff_rec[7] == (crc_result >> 8)) &&
        (buff_rec[8] == (crc_result & 0x00FF))) {
      printf("check success\n");
    } else {
      printf("crc fail\n");
    }

    // printf("%04x\n", c1);

    windRTdata_.speed = 0.1 * (buff_rec[4] + buff_rec[3] * 256);
    windRTdata_.orientation = 0.1 * (buff_rec[6] + buff_rec[5] * 256);

    return *this;
  }  // readwind

  windRTdata getwindRTdata() const { return windRTdata_; }

 private:
  // serial data
  common::serialport ser_wind_;
  windRTdata windRTdata_;

  void restrictdata(int& _wspeed, int& _orientation) {
    if ((_wspeed < 0) || (_wspeed > 300)) _wspeed = 0;
    if ((_orientation < 0) || (_orientation >= 360)) _orientation = 0;
  }
};  // end class wind

}  // namespace ASV::messages

#endif /* _WIND_H_ */

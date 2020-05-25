/*
 *************************************************
 * PowerMonitor.h
 * function for monitor
 * the header file can be read by C++ compilers
 *
 *by ZH.Hu, Ziheng Yang (CrossOcean.ai)
 *************************************************
 */

#ifndef _POWERMONITOR_H_
#define _POWERMONITOR_H_

#include "common/communication/include/CRC.h"
#include "common/communication/include/serialport.h"

namespace ASV::messages {

struct ThreePhaseRTdata {
  float U_power;    // W
  float V_power;    // W
  float W_power;    // W
  float U_voltage;  // V
  float V_voltage;  // V
  float W_voltage;  // V
  float U_current;  // A
  float V_current;  // A
  float W_current;  // A

};  // ThreePhaseRTdata

class PowerMonitor {
 public:
  explicit PowerMonitor(int baud,  // baudrate
                        const std::string& port = "/dev/ttyUSB0")
      : serial_PM_(port, baud, 1000),
        ThreePhaseRTdata_({
            0,  // U_power
            0,  // V_power
            0,  // W_power
            0,  // U_voltage
            0,  // V_voltage
            0,  // W_voltage
            0,  // U_current
            0,  // V_current
            0,  // W_current
        }) {}
  PowerMonitor() = delete;
  virtual ~PowerMonitor() = default;

  PowerMonitor& ParsePowerMeter() {
    static unsigned char buff_send[] = {0x01, 0x03, 0x00, 0x1C,
                                        0x00, 0x40, 0x85, 0xFC};

    unsigned char buff_rec[134];

    serial_PM_.writeline(buff_send, 8);
    serial_PM_.readline(buff_rec, 134);

    uint16_t crc_result = ASV::common::CRC::Calculate<uint16_t, 16>(
        buff_rec, 131, ASV::common::CRC::CRC_16_MODBUS());

    if ((buff_rec[131] == (crc_result >> 8)) &&
        (buff_rec[132] == (crc_result & 0x00FF))) {
      printf("check success\n");
    } else {
      printf("crc fail\n");
    }

    // printf("%04x\n", c1);

    // windRTdata_.speed = 0.1 * (buff_rec[4] + buff_rec[3] * 256);
    // windRTdata_.orientation = 0.1 * (buff_rec[6] + buff_rec[5] * 256);

    return *this;
  }  // ParsePowerMeter

  ThreePhaseRTdata GetThreePhaseRTdata() const noexcept {
    return ThreePhaseRTdata_;
  }

 private:
  // serial data
  common::serialport serial_PM_;
  ThreePhaseRTdata ThreePhaseRTdata_;

};  // end class PowerMonitor

}  // namespace ASV::messages

#endif /* _POWERMONITOR_H_ */

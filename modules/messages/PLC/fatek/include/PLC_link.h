/*
***********************************************************************
* PLC_link.h: link with PLC, to motor control, error report, etc
* This header file can be read by C++ compilers
*
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _PLC_LINK_H_
#define _PLC_LINK_H_

#include <chrono>
#include <thread>
#include "PLCdata.h"
#include "common/communication/include/CRC.h"
#include "common/communication/include/dataserialization.h"
#include "common/communication/include/serialport.h"
#include "common/logging/include/easylogging++.h"

namespace ASV::messages {

class PLC_link {
 public:
  explicit PLC_link(const PLCdata& _PLCdata,       //
                    unsigned long _baud = 115200,  // baudrate
                    const std::string& _port = "/dev/ttyUSB0")
      : PLCdata_(_PLCdata),
        PLC_serial_(_port, _baud, 100),
        send_buffer_(""),
        recv_buffer_(""),
        bytes_send_(0),
        bytes_reci_(0),
        connection_count_(0) {
    checkserialstatus();
  }
  virtual ~PLC_link() = default;

  // communication with PLC
  PLC_link& PLConestep() {
    checkconnection(connection_count_, PLCdata_);
    // senddata2PLC(PLCdata_);
    // std::this_thread::sleep_for(
    //     std::chrono::milliseconds(1));  // 对于单片机，最好不要sleep
    if (parsedata_from_PLC(PLCdata_)) {
      // parse successfully
      connection_count_ = std::min(connection_count_ + 1, 10);
    } else {
      // fail to parse
      connection_count_ = std::max(connection_count_ - 1, 0);
    }
    return *this;
  }  // PLConestep

  PLC_link& setupPLCdata(const int Thruster_A_rpm_command,
                         const int Thruster_A_azimuth_command,
                         const int Thruster_B_rpm_command,
                         const int Thruster_B_azimuth_command,
                         const double latitude, const double longitude,
                         const double heading) {
    PLCdata_.Thruster_A_rpm_command =
        static_cast<int16_t>(Thruster_A_rpm_command);
    PLCdata_.Thruster_A_azimuth_command =
        static_cast<int16_t>(Thruster_A_azimuth_command);
    PLCdata_.Thruster_B_rpm_command =
        static_cast<int16_t>(Thruster_B_rpm_command);
    PLCdata_.Thruster_B_azimuth_command =
        static_cast<int16_t>(Thruster_B_azimuth_command);
    PLCdata_.latitude = static_cast<float>(latitude);
    PLCdata_.longitude = static_cast<float>(longitude);
    PLCdata_.heading = static_cast<float>(heading);

    return *this;
  }  // setupPLCdata

  auto getPLCdata() const noexcept { return PLCdata_; }
  std::string recv_buffer() const noexcept { return recv_buffer_; }
  std::string send_buffer() const noexcept { return send_buffer_; }

 private:
  PLCdata PLCdata_;
  /** serial data **/
  ASV::common::serialport PLC_serial_;
  std::string send_buffer_;
  std::string recv_buffer_;
  std::size_t bytes_send_;
  std::size_t bytes_reci_;

  int connection_count_;

  void checkconnection(const int connection_count, PLCdata& _PLCdata) {
    if (connection_count < 2)
      _PLCdata.linkstatus = common::LINKSTATUS::DISCONNECTED;
    else if (2 <= connection_count && connection_count < 8)
      _PLCdata.linkstatus = common::LINKSTATUS::CONNECTING;
    else
      _PLCdata.linkstatus = common::LINKSTATUS::CONNECTED;
  }

  void checkserialstatus() {
    if (PLC_serial_.isOpen())
      CLOG(INFO, "PLC-serial") << "serial port open successful!";
    else
      CLOG(INFO, "PLC-serial") << "serial port open failure!";
  }

  bool parsedata_from_PLC(PLCdata& _PLCdata) {
    unsigned char read_buffer[100];

    PLC_serial_.readline(read_buffer, 100);

    uint16_t crc_result = ASV::common::CRC::Calculate<uint16_t, 16>(
        read_buffer, 10, ASV::common::CRC::CRC_16_MODBUS());

    if ((read_buffer[8] == (crc_result >> 8)) &&
        (read_buffer[7] == (crc_result & 0x00FF))) {
      // windRTdata_.speed = 0.1 * (buff_rec[4] + buff_rec[3] * 256);
      // windRTdata_.orientation = 0.1 * (buff_rec[6] + buff_rec[5] * 256);
      return true;
    } else {
      CLOG(INFO, "PLC-serial") << " checksum error!";
    }

    return false;
  }  // parsedata_from_PLC

  // void senddata2PLC(const stm32data& _stm32data) {
  //   send_buffer.clear();
  //   send_buffer = "STM";
  //   convert2string(_stm32data, send_buffer);

  //   uint16_t crc = ASV::common::CRC::Calculate<uint16_t, 16>(
  //       send_buffer.c_str(), send_buffer.length(),
  //       ASV::common::CRC::CRC_16_MODBUS());

  //   send_buffer = "$" + send_buffer + "*" + std::to_string(crc) + "\n";
  //   bytes_send = stm32_serial.writeline(send_buffer);
  // }  // senddata2PLC

};  // end class PLC_link

}  // namespace ASV::messages

#endif /* _PLC_LINK_H_ */
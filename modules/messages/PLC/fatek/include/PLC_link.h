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
    senddata2PLC(PLCdata_);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(1));  // 对于单片机，最好不要sleep
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
    recv_buffer_ = PLC_serial_.readline(100, "\n");

    std::size_t pos = recv_buffer.find("$");
    if (pos != std::string::npos) {
      // remove string before "$"
      recv_buffer = recv_buffer.substr(pos + 1);
      std::size_t rpos = recv_buffer.rfind("*");
      if (rpos != std::string::npos) {
        // compute the expected crc checksum value
        std::string expected_crc = recv_buffer.substr(rpos + 1);
        expected_crc.pop_back();
        recv_buffer = recv_buffer.substr(0, rpos);

        if (std::to_string(ASV::common::CRC::Calculate<uint16_t, 16>(
                recv_buffer.c_str(), rpos,
                ASV::common::CRC::CRC_16_MODBUS())) == expected_crc) {
          int _stm32status = 0;
          sscanf(recv_buffer.c_str(),
                 "PC,"
                 "%d,"   // stm32status
                 "%lf,"  // voltage_b1
                 "%lf,"  // voltage_b2
                 "%lf,"  // voltage_b3
                 "%lf,"  // feedback_u1
                 "%lf,"  // feedback_u2
                 "%d,"   // feedback_pwm1
                 "%d,"   // feedback_pwm2
                 "%lf,"  // RC_X
                 "%lf,"  // RC_Y
                 "%lf"   // RC_Mz
                 ,
                 &_stm32status,                // int
                 &(_stm32data.voltage_b1),     // double
                 &(_stm32data.voltage_b2),     // double
                 &(_stm32data.voltage_b3),     // double
                 &(_stm32data.feedback_u1),    // double
                 &(_stm32data.feedback_u2),    // double
                 &(_stm32data.feedback_pwm1),  // int
                 &(_stm32data.feedback_pwm2),  // int
                 &(_stm32data.RC_X),           // double
                 &(_stm32data.RC_Y),           // double
                 &(_stm32data.RC_Mz)           // double
          );

          _stm32data.feedback_stm32status =
              static_cast<STM32STATUS>(_stm32status);
          return true;

        } else {
          CLOG(INFO, "stm32-serial") << " checksum error!";
        }
      }
    }
    return false;
  }  // parsedata_from_stm32

  void senddata2stm32(const stm32data& _stm32data) {
    send_buffer.clear();
    send_buffer = "STM";
    convert2string(_stm32data, send_buffer);

    uint16_t crc = ASV::common::CRC::Calculate<uint16_t, 16>(
        send_buffer.c_str(), send_buffer.length(),
        ASV::common::CRC::CRC_16_MODBUS());

    send_buffer = "$" + send_buffer + "*" + std::to_string(crc) + "\n";
    bytes_send = stm32_serial.writeline(send_buffer);
  }  // senddata2stm32

  void convert2string(const stm32data& _stm32data, std::string& _str) {
    _str += ",";
    _str += _stm32data.UTC_time;
    _str += ",";
    _str += std::to_string(_stm32data.command_u1);
    _str += ",";
    _str += std::to_string(_stm32data.command_u2);
    _str += ",";
    _str += std::to_string(static_cast<int>(_stm32data.command_stm32status));
  }  // convert2string

};  // end class PLC_link

}  // namespace ASV::messages

#endif /* _PLC_LINK_H_ */
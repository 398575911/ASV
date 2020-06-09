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
        connection_count_(0) {
    checkserialstatus();
  }
  virtual ~PLC_link() = default;

  // communication with PLC
  PLC_link& PLConestep() {
    checkconnection(connection_count_, PLCdata_);
    senddata2PLC(PLCdata_);

    if (parsedata_from_PLC(PLCdata_)) {
      // parse successfully
      connection_count_ = std::min(connection_count_ + 1, 10);
    } else {
      // fail to parse
      connection_count_ = std::max(connection_count_ - 1, 0);
    }
    return *this;
  }  // PLConestep

  PLC_link& setupPLCdata(const int Thruster_port_rpm_command,
                         const int Thruster_star_rpm_command,
                         const int Thruster_port_azimuth_command,
                         const int Thruster_star_azimuth_command,
                         const double longitude, const double latitude,
                         const double heading, const double speed,
                         const float wind_speed, const uint8_t wind_direction) {
    PLCdata_.Thruster_port_rpm_command =
        static_cast<int16_t>(Thruster_port_rpm_command);
    PLCdata_.Thruster_star_rpm_command =
        static_cast<int16_t>(Thruster_star_rpm_command);
    PLCdata_.Thruster_port_azimuth_command =
        static_cast<int16_t>(Thruster_port_azimuth_command);
    PLCdata_.Thruster_star_azimuth_command =
        static_cast<int16_t>(Thruster_star_azimuth_command);
    PLCdata_.longitude = static_cast<float>(longitude);
    PLCdata_.latitude = static_cast<float>(latitude);
    PLCdata_.heading = static_cast<float>(heading);
    PLCdata_.speed = static_cast<float>(speed);
    PLCdata_.wind_speed = static_cast<float>(wind_speed);
    PLCdata_.wind_direction = static_cast<int16_t>(wind_direction);

    return *this;
  }  // setupPLCdata

  auto getPLCdata() const noexcept { return PLCdata_; }

 private:
  PLCdata PLCdata_;
  ASV::common::serialport PLC_serial_;
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
    // read buffer from serial port
    static const size_t buffer_size = 46;
    unsigned char read_buffer[buffer_size] = {0x00};
    PLC_serial_.readline(read_buffer, buffer_size);

    // crc check
    uint16_t crc_result = ASV::common::CRC::Calculate<uint16_t, 16>(
        read_buffer, buffer_size - 2, ASV::common::CRC::CRC_16_MODBUS());

    if ((read_buffer[buffer_size - 1] == (crc_result >> 8)) &&
        (read_buffer[buffer_size - 2] == (crc_result & 0x00FF))) {
      uint16_t Thruster_port_rpm_feedback = 0;
      uint16_t Thruster_star_rpm_feedback = 0;
      uint16_t Thruster_port_azimuth_feedback = 0;
      uint16_t Thruster_star_azimuth_feedback = 0;
      uint16_t A_voltagte = 0;
      uint16_t B_voltagte = 0;
      uint16_t C_voltagte = 0;
      uint16_t AB_voltagte = 0;
      uint16_t BC_voltagte = 0;
      uint16_t CA_voltagte = 0;
      uint16_t A_current = 0;
      uint16_t B_current = 0;
      uint16_t C_current = 0;
      uint16_t A_Power = 0;
      uint16_t B_Power = 0;
      uint16_t C_Power = 0;
      uint16_t A_ACPower = 0;
      uint16_t B_ACPower = 0;
      uint16_t C_ACPower = 0;
      uint16_t total_ACPower = 0;
      uint8_t port_register_status1 = 0;
      uint8_t port_register_status2 = 0;
      uint8_t star_register_status1 = 0;
      uint8_t star_register_status2 = 0;
      ASV::common::unpack(read_buffer, "HHHHHHHHHHHHHHHHHHHHCCCC",
                          &Thruster_port_rpm_feedback,      // uint16_t
                          &Thruster_star_rpm_feedback,      // uint16_t
                          &Thruster_port_azimuth_feedback,  // uint16_t
                          &Thruster_star_azimuth_feedback,  // uint16_t
                          &A_voltagte,                      // uint16_t
                          &B_voltagte,                      // uint16_t
                          &C_voltagte,                      // uint16_t
                          &AB_voltagte,                     // uint16_t
                          &BC_voltagte,                     // uint16_t
                          &CA_voltagte,                     // uint16_t
                          &A_current,                       // uint16_t
                          &B_current,                       // uint16_t
                          &C_current,                       // uint16_t
                          &A_Power,                         // uint16_t
                          &B_Power,                         // uint16_t
                          &C_Power,                         // uint16_t
                          &A_ACPower,                       // uint16_t
                          &B_ACPower,                       // uint16_t
                          &C_ACPower,                       // uint16_t
                          &total_ACPower,                   // uint16_t
                          &port_register_status1,           // uint8_t
                          &port_register_status2,           // uint8_t
                          &star_register_status1,           // uint8_t
                          &star_register_status2            // uint8_t
      );

      _PLCdata.power = total_ACPower;
      _PLCdata.Thruster_port_azimuth_feedback =
          static_cast<int16_t>(Thruster_port_azimuth_feedback) - 180;
      _PLCdata.Thruster_star_azimuth_feedback =
          static_cast<int16_t>(Thruster_star_azimuth_feedback) - 180;

      switch (port_register_status1 & 0xf) {
        case 0x1:
          _PLCdata.Thruster_port_status = 0x01;
          _PLCdata.Thruster_port_rpm_feedback =
              static_cast<int16_t>(Thruster_port_rpm_feedback);
          break;
        case 0x2:
          _PLCdata.Thruster_port_status = 0x02;
          _PLCdata.Thruster_port_rpm_feedback =
              -static_cast<int16_t>(Thruster_port_rpm_feedback);
          break;
        case 0x3:
          _PLCdata.Thruster_port_status = 0x03;
          _PLCdata.Thruster_port_rpm_feedback = 0;
          break;
        case 0x4:
          _PLCdata.Thruster_port_status = 0x04;
          _PLCdata.Thruster_port_rpm_feedback = 0;
          break;
        default:
          _PLCdata.Thruster_port_status = 0x00;
          _PLCdata.Thruster_port_rpm_feedback = 0;
          break;
      };
      switch (star_register_status1 & 0xf) {
        case 0x1:
          _PLCdata.Thruster_star_status = 0x01;
          _PLCdata.Thruster_star_rpm_feedback =
              static_cast<int16_t>(Thruster_star_rpm_feedback);
          break;
        case 0x2:
          _PLCdata.Thruster_star_status = 0x02;
          _PLCdata.Thruster_star_rpm_feedback =
              -static_cast<int16_t>(Thruster_star_rpm_feedback);
          break;
        case 0x3:
          _PLCdata.Thruster_star_status = 0x03;
          _PLCdata.Thruster_star_rpm_feedback = 0;
          break;
        case 0x4:
          _PLCdata.Thruster_star_status = 0x04;
          _PLCdata.Thruster_star_rpm_feedback = 0;
          break;
        default:
          _PLCdata.Thruster_star_status = 0x00;
          _PLCdata.Thruster_star_rpm_feedback = 0;
          break;
      };

      return true;
    } else {
      CLOG(INFO, "PLC-serial") << " checksum error!";
    }

    return false;
  }  // parsedata_from_PLC

  void senddata2PLC(const PLCdata& _PLCdata) {
    static const size_t buffer_size = 32;

    unsigned char t_send_buf[buffer_size] = {0x00};
    ASV::common::pack(t_send_buf, "hhhhdddddh",
                      _PLCdata.Thruster_port_rpm_command,      // int16_t
                      _PLCdata.Thruster_star_rpm_command,      // int16_t
                      _PLCdata.Thruster_port_azimuth_command,  // int16_t
                      _PLCdata.Thruster_star_azimuth_command,  // int16_t
                      _PLCdata.longitude,                      // float
                      _PLCdata.latitude,                       // float
                      _PLCdata.heading,                        // float
                      _PLCdata.speed,                          // float
                      _PLCdata.wind_speed,                     // float
                      _PLCdata.wind_direction                  // int16_t
    );

    uint16_t crc_result = ASV::common::CRC::Calculate<uint16_t, 16>(
        t_send_buf, buffer_size - 2, ASV::common::CRC::CRC_16_MODBUS());
    t_send_buf[buffer_size - 2] = crc_result & 0x00FF;
    t_send_buf[buffer_size - 1] = crc_result >> 8;

    PLC_serial_.writeline(t_send_buf, buffer_size);
  }  // senddata2PLC

};  // end class PLC_link

}  // namespace ASV::messages

#endif /* _PLC_LINK_H_ */
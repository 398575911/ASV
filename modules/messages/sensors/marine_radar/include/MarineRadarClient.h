/*
****************************************************************************
* MarineRadarClient.h:
* Marine radar for spoke updating, PPI display, target tracking
* and guard zone alarm, etc.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _MARINERADARCLIENT_H_
#define _MARINERADARCLIENT_H_

#include <cassert>
#include <chrono>
#include <cstring>
#include <thread>

#include "MarineRadarData.h"
#include "common/communication/include/dataserialization.h"
#include "common/communication/include/tcpclient.h"

namespace ASV::messages {

class MarineRadarClient : public ASV::common::tcpclient {
 public:
  MarineRadarClient(const std::string &_ip, const std::string &_port)
      : ASV::common::tcpclient(_ip, _port), {}
  ~MarineRadarClient() {}

  void DataTransmission() {
    static const int recv_size = 1024;
    static const int send_size = 50;

    char recv_buffer[recv_size] = {0x00};
    char send_buffer[send_size] = {0x00};

    ASV::common::tcpclient::TrytoConnect();
    ASV::common::tcpclient::TransmitData(recv_buffer, send_buffer, recv_size,
                                         send_size);
  }

 private:
};  //  end class MarineRadarClient
}  // namespace ASV::messages

#endif /* _MARINERADARCLIENT_H_ */
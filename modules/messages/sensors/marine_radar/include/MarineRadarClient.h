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
      : ASV::common::tcpclient(_ip, _port) {
    ASV::common::tcpclient::TrytoConnect();
  }
  ~MarineRadarClient() {}

  void DataTransmission() {
    static const size_t recv_size = 100;
    static const size_t send_size = 50;

    unsigned char recv_buffer[recv_size] = {0x00};
    unsigned char send_buffer[send_size] = {0x00};

    ASV::common::tcpclient::TransmitData(recv_buffer, send_buffer, recv_size,
                                         send_size);

    double azimuth = 0;
    ASV::common::unpack(recv_buffer + 80, "d", &azimuth);
    printf("azimuth: %f\n", azimuth);
    // for (int i = 0; i != 512; ++i) printf("%02x\n", recv_buffer[i]);
  }

 private:
};  //  end class MarineRadarClient
}  // namespace ASV::messages

#endif /* _MARINERADARCLIENT_H_ */
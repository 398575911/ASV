/*
*******************************************************************************
* testtcpserver.cc:
* unit test for socket server using tcp/ip
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <cmath>
#include <iostream>
#include "../include/dataserialization.h"
#include "../include/tcpserver.h"

#define SAMPLES_PER_SPOKE 20

void testtcpserver() {
  ASV::common::tcpserver _tcpserver("9340");

  const size_t recv_size = 10;
  const size_t send_size = 4 + SAMPLES_PER_SPOKE / 2;

  static float count = 0;
  unsigned char recv_buffer[recv_size];
  uint8_t spokedata[SAMPLES_PER_SPOKE / 2] = {0xff, 0xff, 0x09, 0xff, 0xff,
                                              0xfa, 0xff, 0xac, 0x10, 0x16};
  unsigned char sendmsg[send_size];
  while (1) {
    count += 1;
    // ASV::common::pack(sendmsg, "d", count);
    std::memcpy(sendmsg, spokedata, SAMPLES_PER_SPOKE / 2);

    ASV::common::pack(sendmsg + SAMPLES_PER_SPOKE / 2, "f", count);

    // for (int i = 0; i != 44; ++i) sendmsg[i] = _radarmsg.header4[i];
    // for (int i = 0; i != (SAMPLES_PER_SPOKE / 2); ++i)
    //   sendmsg[i + 44] = (char)spokedata[i];

    // strcpy(sendmsg, _radarmsg.header4);
    // strcat(sendmsg, (char*)spokedata);

    _tcpserver.selectserver(recv_buffer, sendmsg, recv_size, send_size);
    printf("The buffer recived: %s\n", recv_buffer);
    printf("The socket status: %d\n", _tcpserver.getsocketresults());
    printf("The clients connected: %d\n", _tcpserver.getconnectioncount());
  }
}

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";
  testtcpserver();
  LOG(INFO) << "Shutting down.";
}
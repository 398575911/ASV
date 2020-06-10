/*
*******************************************************************************
* testtcpclient.cc:
* unit test for socket client using tcp/ip
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <chrono>
#include <thread>
#include "../include/dataserialization.h"
#include "../include/tcpclient.h"

#define SAMPLES_PER_SPOKE 20

void test() {
  ASV::common::tcpclient _tcpclient("127.0.0.1", "9340");
  const size_t recv_size = 4 + SAMPLES_PER_SPOKE / 2;
  const size_t send_size = 10;
  unsigned char send_buffer[send_size] = "socket";
  unsigned char recv_buffer[recv_size];
  uint8_t spokedata[SAMPLES_PER_SPOKE / 2];

  for (int i = 0; i != 1000; i++) {
    _tcpclient.TrytoConnect();

    _tcpclient.TransmitData(recv_buffer, send_buffer, recv_size, send_size);

    float azimuth = 0;
    // unsigned char s_azimuth[4];
    // for (int j = 0; j != 4; j++)
    //   s_azimuth[j] = recv_buffer[j + SAMPLES_PER_SPOKE / 2];
    ASV::common::unpack(recv_buffer + SAMPLES_PER_SPOKE / 2, "f", &azimuth);
    printf("azimuth: %f\n", azimuth);
    for (int j = 0; j != SAMPLES_PER_SPOKE / 2; ++j)
      printf("%02x\n", recv_buffer[j]);
    // for (int i = 0; i != 44; ++i) _radarmsg.header4[i] = recv_buffer[i];
    // for (int i = 0; i != (SAMPLES_PER_SPOKE / 2); ++i) {
    //   spokedata[i] = (uint8_t)recv_buffer[i + 44];
    //   printf("%u\n", spokedata[i]);
    // }

    printf("The socket status: %d\n", _tcpclient.getsocketresults());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";
  test();
}
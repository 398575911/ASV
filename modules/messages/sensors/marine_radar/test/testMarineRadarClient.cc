/*
****************************************************************************
* testMarineRadarClient.cc:
* example for data transmission for marine radar
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include "modules/messages/sensors/marine_radar/include/MarineRadarClient.h"

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  ASV::messages::MarineRadarClient _MarineRadarClient("127.0.0.1", "9340");

  while (1) {
    _MarineRadarClient.DataTransmission();

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

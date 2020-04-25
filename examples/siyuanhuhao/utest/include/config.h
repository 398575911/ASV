/*
***********************************************************************
* config.h: constexpr number
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <pthread.h>

#include <experimental/filesystem>
#include "StateMonitor.h"
#include "common/communication/include/tcpserver.h"
#include "common/fileIO/include/jsonparse.h"
#include "common/fileIO/recorder/include/datarecorder.h"
#include "common/logging/include/easylogging++.h"
#include "common/timer/include/timecounter.h"
#include "modules/controller/include/controller.h"
#include "modules/controller/include/trajectorytracking.h"
#include "modules/estimator/include/estimator.h"
#include "modules/messages/GUILink/include/guilink.h"
#include "modules/messages/sensors/gpsimu/include/gps.h"
#include "modules/messages/sensors/marine_radar/include/MarineRadar.h"
#include "modules/messages/stm32/include/stm32_link.h"
#include "modules/perception/marine_radar/include/TargetTracking.h"
#include "modules/planner/path_planning/lanefollow/include/LatticePlanner.h"
#include "modules/planner/path_planning/openspace/include/OpenSpacePlanner.h"
#include "modules/planner/route_planning/include/RoutePlanning.h"
#include "modules/simulator/include/simulator.h"

namespace ASV {

const std::string parameter_json_path = "./../../properties/property.json";
constexpr int num_thruster = 2;
constexpr int dim_controlspace = 3;
constexpr localization::USEKALMAN indicator_kalman =
    localization::USEKALMAN::KALMANOFF;
constexpr control::ACTUATION indicator_actuation =
    control::ACTUATION::UNDERACTUATED;
constexpr int max_num_targets = 20;

// constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_DP;
// constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_LOS;
// constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_FRENET;
// constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_AVOIDANCE;
constexpr common::TESTMODE testmode = common::TESTMODE::SIMULATION_DOCKING;

// constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_DP;
// constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_LOS;
// constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_FRENET;
// constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_AVOIDANCE;
// constexpr common::TESTMODE testmode = common::TESTMODE::EXPERIMENT_DOCKING;

void WriteConstConfig2File(const std::string &filepath) {
  // prepare the config
  std::string j_indicator_kalman = "on";
  if (indicator_kalman == localization::USEKALMAN::KALMANOFF)
    j_indicator_kalman = "off";

  std::string j_indicator_actuation = "Fully-actuated";
  if (indicator_actuation == control::ACTUATION::UNDERACTUATED)
    j_indicator_actuation = "Under-actuated";

  std::string j_testmode = "";
  switch (testmode) {
    case common::TESTMODE::SIMULATION_DP:
      j_testmode = "SIMULATION_DP";
      break;
    case common::TESTMODE::SIMULATION_LOS:
      j_testmode = "SIMULATION_LOS";
      break;
    case common::TESTMODE::SIMULATION_FRENET:
      j_testmode = "SIMULATION_FRENET";
      break;
    case common::TESTMODE::SIMULATION_AVOIDANCE:
      j_testmode = "SIMULATION_AVOIDANCE";
      break;
    case common::TESTMODE::SIMULATION_DOCKING:
      j_testmode = "SIMULATION_DOCKING";
      break;
    case common::TESTMODE::EXPERIMENT_DP:
      j_testmode = "EXPERIMENT_DP";
      break;
    case common::TESTMODE::EXPERIMENT_LOS:
      j_testmode = "EXPERIMENT_LOS";
      break;
    case common::TESTMODE::EXPERIMENT_FRENET:
      j_testmode = "EXPERIMENT_FRENET";
      break;
    case common::TESTMODE::EXPERIMENT_AVOIDANCE:
      j_testmode = "EXPERIMENT_AVOIDANCE";
      break;
    case common::TESTMODE::EXPERIMENT_DOCKING:
      j_testmode = "EXPERIMENT_DOCKING";
      break;
    default:
      break;
  };

  //  write the json file
  nlohmann::json j = {
      {"num_thruster", num_thruster},                //
      {"dim_controlspace", dim_controlspace},        //
      {"Kalman", j_indicator_kalman},                //
      {"control_actuation", j_indicator_actuation},  //
      {"max_num_targets", max_num_targets},          //
      {"Test_mode", j_testmode},                     //
  };

  // write prettified JSON to another file
  std::ofstream output_file(filepath + "cconfig.json");
  output_file << std::setw(4) << j << std::endl;
}  // WriteConstConfig2File

}  // namespace ASV

#endif /* _CONFIG_H_ */

/*
*******************************************************************************
* testthrust_osqp.cc:
* unit test for thrust allocation
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/thrustallocation_osqp.h"

using namespace ASV::control;
using namespace ASV::common;

// test thrust allocation for 4 propellers (fully actuated)
void testonestepthrustallocation() {
  const int m = 4;
  const int n = 3;
  constexpr ACTUATION index_actuation = ACTUATION::FULLYACTUATED;

  std::vector<int> index_thrusters{1, 1, 2, 2};

  int num_tunnel =
      std::count(index_thrusters.begin(), index_thrusters.end(), 1);

  int num_azimuth =
      std::count(index_thrusters.begin(), index_thrusters.end(), 2);

  int num_mainrudder =
      std::count(index_thrusters.begin(), index_thrusters.end(), 3);

  int num_twinfixed =
      std::count(index_thrusters.begin(), index_thrusters.end(), 4);

  thrustallocationdata _thrustallocationdata{
      500,             // Q_surge
      500,             // Q_sway
      1000,            // Q_yaw
      num_tunnel,      // num_tunnel
      num_azimuth,     // num_azimuth
      num_mainrudder,  // num_mainrudder
      num_twinfixed,   // num_twinfixed
      index_thrusters  // index_thrusters
  };

  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  v_tunnelthrusterdata.reserve(num_tunnel);
  v_tunnelthrusterdata.push_back({
      1.9,   // lx
      0,     // ly
      1e-6,  // K_positive
      2e-6,  // K_negative
      50,    // max_delta_rotation
      1000,  // max_rotation
      1,     // max_thrust_positive
      2      // max_thrust_negative
  });
  v_tunnelthrusterdata.push_back({
      1,     // lx
      0,     // ly
      1e-6,  // K_positive
      2e-6,  // K_negative
      50,    // max_delta_rotation
      1000,  // max_rotation
      1,     // max_thrust_positive
      2      // max_thrust_negative
  });

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      -1.893,         // lx
      -0.216,         // ly
      2e-5,           // K
      10,             // max_delta_rotation
      1000,           // max rotation
      10,             // min_rotation
      0.1277,         // max_delta_alpha
      M_PI / 6,       // max_alpha
      -7 * M_PI / 6,  // min_alpha
      20,             // max_thrust
      2e-3            // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -1.893,        // lx
      0.216,         // ly
      2e-5,          // K
      10,            // max_delta_rotation
      1000,          // max rotation
      10,            // min_rotation
      0.1277,        // max_delta_alpha
      7 * M_PI / 6,  // max_alpha
      -M_PI / 6,     // min_alpha
      20,            // max_thrust
      2e-3           // min_thrust
  });
  std::vector<ruddermaindata> v_ruddermaindata;
  std::vector<twinfixedthrusterdata> v_twinfixeddata;

  controllerRTdata<m, n> _controllerRTdata{
      STATETOGGLE::IDLE,                                      // state_toggle
      (Eigen::Matrix<double, n, 1>() << 0, 0, 1).finished(),  // tau
      Eigen::Matrix<double, n, 1>::Zero(),                    // BalphaU
      (Eigen::Matrix<double, m, 1>() << 0, 0.5, 0, 1).finished(),  // command_u
      // vectormi()::Zero(),
      (Eigen::Matrix<int, m, 1>() << 100, 500, 400, 300)
          .finished(),  // command_rotation
      (Eigen::Matrix<double, m, 1>() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3, 0)
          .finished(),                   // command_alpha
      Eigen::Matrix<int, m, 1>::Zero(),  // command_alpha_deg
      (Eigen::Matrix<double, m, 1>() << 0, 0.5, 0, 1).finished(),  // feedback_u
      // vectormi()::Zero(),                    // feedback_rotation
      (Eigen::Matrix<int, m, 1>() << 100, 500, 400, 300).finished(),
      (Eigen::Matrix<double, m, 1>() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3, 0)
          .finished(),                  // feedback_alpha
      Eigen::Matrix<int, m, 1>::Zero()  // feedback_alpha_deg

  };

  thrustallocation<m, index_actuation, n> _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata,
      v_ruddermaindata, v_twinfixeddata);
  _thrustallocation.initializapropeller(_controllerRTdata);
  _thrustallocation.setQ(CONTROLMODE::DYNAMICPOSITION);

  _thrustallocation.onestepthrustallocation(_controllerRTdata);

  std::cout << "command_alpha: \n"
            << _controllerRTdata.command_alpha << std::endl;
  std::cout << "upper_delta_alpha: \n"
            << _thrustallocation.getupper_delta_alpha() << std::endl;
  std::cout << "lower_delta_alpha: \n"
            << _thrustallocation.getlower_delta_alpha() << std::endl;
  std::cout << "upper_delta_u: \n"
            << _thrustallocation.getupper_delta_u() << std::endl;
  std::cout << "lower_delta_u: \n"
            << _thrustallocation.getlower_delta_u() << std::endl;
  std::cout << "Q: \n" << _thrustallocation.getQ() << std::endl;
  std::cout << "Omega: \n" << _thrustallocation.getOmega() << std::endl;
  std::cout << "Q_deltau: \n" << _thrustallocation.getQ_deltau() << std::endl;
  std::cout << "g_deltau: \n" << _thrustallocation.getg_deltau() << std::endl;
  std::cout << "d_rho: \n" << _thrustallocation.getd_rho() << std::endl;
  std::cout << "B_alpha: \n" << _thrustallocation.getB_alpha() << std::endl;
  std::cout << "d_Balpha_u: \n"
            << _thrustallocation.getd_Balpha_u() << std::endl;
  std::cout << "lx: \n" << _thrustallocation.getlx() << std::endl;
}  // testonestepthrustallocation

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  testonestepthrustallocation();
  // test_multiplethrusterallocation();
  // testrudder();
  // test_twinfixed();
  // testbiling();
  // testoutboard();

  LOG(INFO) << "Shutting down.";
  return 0;
}
/*
***********************************************************************
* simulator.h:
* 3DoF motion simulator for USV
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include "common/math/NumericalAnalysis/include/odesolver.h"
#include "common/math/miscellaneous/include/math_utils.h"
#include "simulatordata.h"

namespace ASV::simulation {

// vessel motion used in simulation
struct VesselSimulator {
  using state_type = Eigen::Matrix<double, 6, 1>;

  // constructor
  VesselSimulator(const common::vessel& _vessel)
      : A_(Eigen::Matrix<double, 6, 6>::Zero()),
        B_(Eigen::Matrix<double, 6, 3>::Zero()),
        u_(Eigen::Vector3d::Zero()) {
    initializeAB(_vessel.AddedMass + _vessel.Mass, _vessel.LinearDamping);
  }

  state_type FirstDerivative(const double t, const state_type& x) const {
    (void)t;
    return A_ * x + B_ * u_;
  }  // FirstDerivative

  // update the A matrix in state space model
  void updateA(double theta) {
    // update the transformation matrix
    double cvalue = std::cos(theta);
    double svalue = std::sin(theta);
    Eigen::Matrix3d Transform = Eigen::Matrix3d::Identity();
    Transform(0, 0) = cvalue;
    Transform(0, 1) = -svalue;
    Transform(1, 0) = svalue;
    Transform(1, 1) = cvalue;

    //
    A_.block(0, 3, 3, 3) = Transform;
  }

  // update the input
  void updateu(const Eigen::Vector3d& u) { u_ = u; }

  void initializeAB(const Eigen::Matrix3d& M, const Eigen::Matrix3d& D) {
    auto M_inv = M.inverse();
    A_.block(3, 3, 3, 3) = -M_inv * D;
    B_.block(3, 0, 3, 3) = M_inv;
  }

  state_type X;

  Eigen::Matrix<double, 6, 6> A_;
  Eigen::Matrix<double, 6, 3> B_;
  Eigen::Vector3d u_;  // input defined in the body-fixed coordinate frame

};  // end struct VesselSimulator

class simulator {
  using state_type = Eigen::Matrix<double, 6, 1>;

 public:
  simulator(const double sample_time, const common::vessel& vessel,
            const state_type& x = state_type::Zero())
      : sample_time_(sample_time),
        vesselsimulator_(vessel),
        RTdata_({common::STATETOGGLE::READY, x}) {}
  virtual ~simulator() = default;

  // perform one step of ode solver
  simulator& do_step(const double desired_heading,
                     const Eigen::Vector3d& thrust,
                     const Eigen::Vector3d& seaload = Eigen::Vector3d::Zero()) {
    double _theta = 0.0;
    if (std::abs(common::math::Normalizeheadingangle(
            RTdata_.X(2) - desired_heading)) < M_PI / 36) {
      // use the fixed setpoint orientation to prevent measurement noise
      _theta = desired_heading;
    } else {
      // if larger than 5 deg, we use the realtime orientation
      _theta = RTdata_.X(2);
    }

    vesselsimulator_.updateA(_theta);  // update the transform matrix and A
    vesselsimulator_.updateu(thrust + seaload);  // update the input
    RTdata_.X = rk4.rk4vec(0.0, sample_time_, RTdata_.X, vesselsimulator_);
    RTdata_.X(2) = common::math::Normalizeheadingangle(RTdata_.X(2));

    return *this;
  }  // do_step

  void setX(const state_type& _x) { RTdata_.X = _x; }
  auto X() const noexcept { return RTdata_.X; }
  double sampletime() const noexcept { return sample_time_; }

 private:
  const double sample_time_;  // second
  VesselSimulator vesselsimulator_;
  simulatorRTdata RTdata_;

  // 4-order Runge-Kutta methods
  ASV::common::math::OdeSolver<VesselSimulator> rk4;
};  // end class simulator

}  // namespace ASV::simulation

#endif /* _SIMULATOR_H_ */
/*
***********************************************************************
* rk4_test.cc:
* Utility test to solve equation of lorenz
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <array>
#include <iostream>
#include "../include/odesolver.h"

struct sys {
  using state_type = std::array<double, 2>;

  state_type FirstDerivative(const double t, const state_type &u) const {
    state_type uprime;

    uprime[0] = -K / M * u[1] + T / M;
    uprime[1] = u[0];
    (void)t;
    return uprime;
  }

  void setK(double _K) { K = _K; }
  void setT(double _T) { T = _T; }

  state_type x;
  double K = 1;
  double M = 1;
  double T = 0;
};

int main() {
  std::array<double, 2> x = {0, 3};
  ASV::common::math::OdeSolver<sys> test;
  sys mysys_;

  int total_step = 100;
  for (int i = 0; i != total_step; ++i) {
    mysys_.setT(1);
    x = test.rk4vec(0, 0.1, x, mysys_);
  }
}

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
#include <cmath>
#include <iostream>
#include "../include/odesolver.h"

struct sys {
  using state_type = std::array<double, 1>;

  state_type FirstDerivative(const double t, const state_type &u) const {
    state_type uprime;

    uprime[0] = u[0] - t * t + 1;
    (void)t;
    return uprime;
  }

  state_type X;
};

int main() {
  std::array<double, 1> x = {0.5};
  double t = 0;
  ASV::common::math::OdeSolver<sys> test;
  sys mysys_;

  int total_step = 5;
  double dt = 0.5;
  for (int i = 0; i != total_step; ++i) {
    x = test.rk4vec(t, dt, x, mysys_);
    t += dt;
    std::cout << x[0] << ", " << t * t + 2 * t + 1 - 0.5 * std::exp(t)
              << std::endl;
  }
}

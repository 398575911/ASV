/*
***********************************************************************
* lorenz_test.cc:
* Utility test to solve equation of lorenz
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <array>
#include "../include/odesolver.h"
#include "common/plotting/include/gnuplot-iostream.h"

struct lorenz {
  using state_type = std::array<double, 3>;

  state_type FirstDerivative(const double t, const state_type& u) const {
    state_type uprime;
    const double sigma = 10.0;
    const double R = 28.0;
    const double b = 8.0 / 3.0;
    uprime[0] = sigma * (u[1] - u[0]);
    uprime[1] = R * u[0] - u[1] - u[0] * u[2];
    uprime[2] = -b * u[2] + u[0] * u[1];
    return uprime;
  }

  state_type x;
  // std::array<double, 3> x;
};

int main() {
  std::array<double, 3> u0 = {10, 10, 10};
  ASV::common::math::OdeSolver<lorenz> test;
  lorenz mylorenz_;
  auto x = u0;

  std::vector<boost::tuple<double, double, double>> pts;

  int total_step = 1000;
  for (int i = 0; i != total_step; ++i) {
    x = test.rk4vec(0, 0.005, x, mylorenz_);
    std::cout << x[0] << ", " << x[1] << ", " << x[2] << std::endl;
    pts.push_back(boost::make_tuple(x[0], x[1], x[2]));
  }

  Gnuplot gp;
  gp << "splot ";
  // gp << "set zrange [-1:1]\n";

  gp << gp.binFile1d(pts, "record") << "with lines title 'lorenz'";
  gp << std::endl;
}

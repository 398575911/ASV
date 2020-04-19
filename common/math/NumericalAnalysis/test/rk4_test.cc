/*
***********************************************************************
* rk4_test.cc:
* Utility test to solve equation of lorenz
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <iostream>
#include "../include/odesolver.h"
#include "common/plotting/include/gnuplot-iostream.h"

struct mysys {
  std::array<double, 2> GetSuccessors(const double t,
                                      const std::array<double, 2> &u) const {
    std::array<double, 2> uprime;

    uprime[0] = -K / M * u[1] + T / M;
    uprime[1] = u[0];
    (void)t;
    return uprime;
  }

  void setK(double _K) { K = _K; }
  void setT(double _T) { T = _T; }

  std::array<double, 2> x;
  double K = 1;
  double M = 1;
  double T = 0;
};

int main() {
  std::array<double, 2> x = {0, 3};
  ASV::common::math::OdeSolver<mysys> test;
  mysys mysys_;

  int total_step = 100;
  std::vector<std::pair<double, double> > xy_pts_A;

  for (int i = 0; i != total_step; ++i) {
    mysys_.setT(1);
    x = test.rk4vec(0, 0.1, x, mysys_);

    xy_pts_A.push_back(std::make_pair(i, x.at(1)));
  }

  Gnuplot gp;
  gp << "set title 'ODE'\n";
  gp << "set xlabel 'sampling instant'\n";
  gp << "set ylabel 'x'\n";
  gp << "plot"
        " '-' with lines lw 2\n";
  gp.send1d(xy_pts_A);
}

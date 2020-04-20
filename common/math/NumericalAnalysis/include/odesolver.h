/*
***********************************************************************
* odesolver.h:
* ordinary differential equation solver, with support to
* Eigen::Matrix<double>, std::array, std::vector
* Given a system of ordinary differential equations of the form
*    X' = f(t,X)
*    X(t0) = X0
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _ODESOLVER_H_
#define _ODESOLVER_H_

namespace ASV::common::math {

template <class UserState>
class OdeSolver {
  using user_state_type = decltype(UserState::X);

 public:
  OdeSolver() = default;
  ~OdeSolver() = default;

  // Runge–Kutta 4th order method
  user_state_type rk4vec(const double t0, const double dt,
                         const user_state_type &X0,
                         const UserState &user_state) const {
    auto dimension = X0.size();

    //  Get four sample values of the derivative.
    auto f0 = user_state.FirstDerivative(t0, X0);

    auto t1 = t0 + 0.5 * dt;
    user_state_type x1;
    for (decltype(dimension) i = 0; i < dimension; i++) {
      x1[i] = X0[i] + 0.5 * dt * f0[i];
    }
    auto f1 = user_state.FirstDerivative(t1, x1);

    auto t2 = t0 + 0.5 * dt;
    user_state_type x2;
    for (decltype(dimension) i = 0; i < dimension; i++) {
      x2[i] = X0[i] + 0.5 * dt * f1[i];
    }
    auto f2 = user_state.FirstDerivative(t2, x2);

    auto t3 = t0 + dt;
    user_state_type x3;
    for (decltype(dimension) i = 0; i < dimension; i++) {
      x3[i] = X0[i] + dt * f2[i];
    }
    auto f3 = user_state.FirstDerivative(t3, x3);

    //  Combine them to estimate the solution.
    user_state_type X;
    for (decltype(dimension) i = 0; i < dimension; i++) {
      X[i] = X0[i] + dt * (f0[i] + 2.0 * f1[i] + 2.0 * f2[i] + f3[i]) / 6.0;
    }

    return X;
  }  // rk4vec
};   // end class OdeSolver

}  // namespace ASV::common::math

#endif /* _ODESOLVER_H_ */
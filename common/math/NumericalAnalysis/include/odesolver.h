/*
***********************************************************************
* odesolver.h:
* ordinary differential equation solver
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
  using user_state_type = decltype(UserState::x);

 public:
  OdeSolver() = default;
  ~OdeSolver() = default;

  // Runge–Kutta 4th order method
  user_state_type rk4vec(const double t0, const double dt,
                         const user_state_type &u0,
                         const UserState &user_state) const {
    auto dimension = u0.size();

    //  Get four sample values of the derivative.
    auto f0 = user_state.GetSuccessors(t0, u0);
    auto t1 = t0 + 0.5 * dt;
    user_state_type u1;
    for (decltype(dimension) i = 0; i < dimension; i++) {
      u1[i] = u0[i] + 0.5 * dt * f0[i];
    }
    auto f1 = user_state.GetSuccessors(t1, u1);

    auto t2 = t0 + 0.5 * dt;
    user_state_type u2;
    for (decltype(dimension) i = 0; i < dimension; i++) {
      u2[i] = u0[i] + 0.5 * dt * f1[i];
    }
    auto f2 = user_state.GetSuccessors(t2, u2);

    auto t3 = t0 + dt;
    user_state_type u3;
    for (decltype(dimension) i = 0; i < dimension; i++) {
      u3[i] = u0[i] + dt * f2[i];
    }
    auto f3 = user_state.GetSuccessors(t3, u3);

    //  Combine them to estimate the solution.
    user_state_type u;
    for (decltype(dimension) i = 0; i < dimension; i++) {
      u[i] = u0[i] + dt * (f0[i] + 2.0 * f1[i] + 2.0 * f2[i] + f3[i]) / 6.0;
    }

    return u;
  }  // rk4vec
};   // end class OdeSolver

}  // namespace ASV::common::math

#endif /* _ODESOLVER_H_ */
/*
***********************************************************************
* Reeds_Shepp.h:
* Generate the Reeds Shepp curves from starting state to ending state.
* The comments, variable names, etc. use the nomenclature
* from the Reeds & Shepp paper.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _REEDS_SHEPP_H_
#define _REEDS_SHEPP_H_

#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace ASV::common::math {

enum ReedsSheppPathSegmentType {
  RS_NOP = 0,
  RS_LEFT = 1,
  RS_STRAIGHT = 2,
  RS_RIGHT = 3
};

const ReedsSheppPathSegmentType reedsSheppPathType[18][5] = {
    {RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP},         // 0
    {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP},        // 1
    {RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP},       // 2
    {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP},       // 3
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 4
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 5
    {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},    // 6
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},   // 7
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 8
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 9
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},   // 10
    {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},    // 11
    {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},     // 12
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},     // 13
    {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},      // 14
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},    // 15
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT},  // 16
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT}   // 17
};

struct ReedsSheppPath {
  ReedsSheppPath(const ReedsSheppPathSegmentType *type = reedsSheppPathType[0],
                 double t = std::numeric_limits<double>::max(), double u = 0.,
                 double v = 0., double w = 0., double x = 0.)
      : type_(type) {
    length_[0] = t;
    length_[1] = u;
    length_[2] = v;
    length_[3] = w;
    length_[4] = x;
    totalLength_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
  }

  double length() const { return totalLength_; }
  const ReedsSheppPathSegmentType *type_;
  double length_[5];
  double totalLength_;
};  // end struct ReedsSheppPath

class ReedsSheppStateSpace {
 public:
  ReedsSheppStateSpace(double turningRadius = 1.0)
      : rho_(turningRadius),
        ZERO(10 * std::numeric_limits<double>::epsilon()),
        RS_EPS(1e-6),
        twopi(2. * M_PI) {}
  virtual ~ReedsSheppStateSpace() = default;

  double distance(double q0[3], double q1[3]) {
    return rho_ * reedsShepp(q0, q1).length();
  }

  std::array<int, 5> rs_type(double q0[3], double q1[3]) {
    ReedsSheppPath path = reedsShepp(q0, q1);
    std::array<int, 5> types;
    for (int i = 0; i < 5; ++i) {
      types[i] = (int(path.type_[i]));
      // std::cout<<path.type_[i]<<std::endl;
    }
    return types;
  }  // rs_type

  std::vector<std::array<double, 3> > rs_state(double q0[3], double q1[3],
                                               double step_size) {
    ReedsSheppPath path = reedsShepp(q0, q1);
    double dist = rho_ * path.length();
    std::vector<std::array<double, 3> > result;

    for (double seg = 0.0; seg <= dist; seg += step_size) {
      double qnew[3] = {};
      interpolate(q0, path, seg / rho_, qnew);
      std::array<double, 3> temp;
      std::copy(std::begin(qnew), std::end(qnew), std::begin(temp));
      result.emplace_back(temp);
    }

    return result;
  }  // rs_state

  ReedsSheppPath reedsShepp(double x, double y, double phi) {
    ReedsSheppPath path;
    CSC(x, y, phi, path);
    CCC(x, y, phi, path);
    CCCC(x, y, phi, path);
    CCSC(x, y, phi, path);
    CCSCC(x, y, phi, path);
    return path;
  }  // reedsShepp

  ReedsSheppPath reedsShepp(double q0[3], double q1[3]) {
    double dx = q1[0] - q0[0], dy = q1[1] - q0[1], dth = q1[2] - q0[2];
    double c = cos(q0[2]), s = sin(q0[2]);
    double x = c * dx + s * dy, y = -s * dx + c * dy;

    return reedsShepp(x / rho_, y / rho_, dth);
  }  // reedsShepp

  void interpolate(double q0[3], ReedsSheppPath &path, double seg,
                   double s[3]) {
    if (seg < 0.0) seg = 0.0;
    if (seg > path.length()) seg = path.length();

    double phi, v;

    s[0] = s[1] = 0.0;
    s[2] = q0[2];

    for (unsigned int i = 0; i < 5 && seg > 0; ++i) {
      if (path.length_[i] < 0) {
        v = std::max(-seg, path.length_[i]);
        seg += v;
      } else {
        v = std::min(seg, path.length_[i]);
        seg -= v;
      }
      phi = s[2];
      switch (path.type_[i]) {
        case RS_LEFT:
          s[0] += (sin(phi + v) - sin(phi));
          s[1] += (-cos(phi + v) + cos(phi));
          s[2] = phi + v;
          break;
        case RS_RIGHT:
          s[0] += (-sin(phi - v) + sin(phi));
          s[1] += (cos(phi - v) - cos(phi));
          s[2] = phi - v;
          break;
        case RS_STRAIGHT:
          s[0] += (v * cos(phi));
          s[1] += (v * sin(phi));
          break;
        case RS_NOP:
          break;
      }
    }

    s[0] = s[0] * rho_ + q0[0];
    s[1] = s[1] * rho_ + q0[1];
  }  // interpolate

 protected:
  double rho_;  // TURNNING RADIUS

 private:
  const double ZERO;
  const double RS_EPS;
  const double twopi;

  inline double mod2pi(double x) {
    double v = fmod(x, twopi);
    if (v < -M_PI)
      v += twopi;
    else if (v > M_PI)
      v -= twopi;
    return v;
  }  // mod2pi

  inline void polar(double x, double y, double &r, double &theta) {
    r = sqrt(x * x + y * y);
    theta = atan2(y, x);
  }  // polar

  inline void tauOmega(double u, double v, double xi, double eta, double phi,
                       double &tau, double &omega) {
    double delta = mod2pi(u - v), A = sin(u) - sin(delta),
           B = cos(u) - cos(delta) - 1.;
    double t1 = atan2(eta * A - xi * B, xi * A + eta * B),
           t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
    tau = (t2 < 0) ? mod2pi(t1 + M_PI) : mod2pi(t1);
    omega = mod2pi(tau - u + v - phi);
  }  // tauOmega

  // formula 8.1 in Reeds-Shepp paper
  inline bool LpSpLp(double x, double y, double phi, double &t, double &u,
                     double &v) {
    polar(x - sin(phi), y - 1. + cos(phi), u, t);
    if (t >= -ZERO) {
      v = mod2pi(phi - t);
      if (v >= -ZERO) {
        assert(fabs(u * cos(t) + sin(phi) - x) < RS_EPS);
        assert(fabs(u * sin(t) - cos(phi) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t + v - phi)) < RS_EPS);
        return true;
      }
    }
    return false;
  }  // LpSpLp

  // formula 8.2
  inline bool LpSpRp(double x, double y, double phi, double &t, double &u,
                     double &v) {
    double t1, u1;
    polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
    u1 = u1 * u1;
    if (u1 >= 4.) {
      double theta;
      u = sqrt(u1 - 4.);
      theta = atan2(2., u);
      t = mod2pi(t1 + theta);
      v = mod2pi(t - phi);
      assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < RS_EPS);
      assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < RS_EPS);
      assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
      return t >= -ZERO && v >= -ZERO;
    }
    return false;
  }  // LpSpRp

  void CSC(double x, double y, double phi, ReedsSheppPath &path) {
    double t, u, v, Lmin = path.length(), L;
    if (LpSpLp(x, y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[14], t, u, v);
      Lmin = L;
    }
    if (LpSpLp(-x, y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[14], -t, -u, -v);
      Lmin = L;
    }
    if (LpSpLp(x, -y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[15], t, u, v);
      Lmin = L;
    }
    if (LpSpLp(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[15], -t, -u, -v);
      Lmin = L;
    }
    if (LpSpRp(x, y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[12], t, u, v);
      Lmin = L;
    }
    if (LpSpRp(-x, y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[12], -t, -u, -v);
      Lmin = L;
    }
    if (LpSpRp(x, -y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[13], t, u, v);
      Lmin = L;
    }
    if (LpSpRp(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
      path = ReedsSheppPath(reedsSheppPathType[13], -t, -u, -v);
  }  // CSC

  // formula 8.3 / 8.4  *** TYPO IN PAPER ***
  inline bool LpRmL(double x, double y, double phi, double &t, double &u,
                    double &v) {
    double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
    polar(xi, eta, u1, theta);
    if (u1 <= 4.) {
      u = -2. * asin(0.25 * u1);
      t = mod2pi(theta + 0.5 * u + M_PI);
      v = mod2pi(phi - t + u);
      assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < RS_EPS);
      assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < RS_EPS);
      assert(fabs(mod2pi(t - u + v - phi)) < RS_EPS);
      return t >= -ZERO && u <= ZERO;
    }
    return false;
  }  // LpRmL

  void CCC(double x, double y, double phi, ReedsSheppPath &path) {
    double t, u, v, Lmin = path.length(), L;
    if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[0], t, u, v);
      Lmin = L;
    }
    if (LpRmL(-x, y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[0], -t, -u, -v);
      Lmin = L;
    }
    if (LpRmL(x, -y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[1], t, u, v);
      Lmin = L;
    }
    if (LpRmL(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[1], -t, -u, -v);
      Lmin = L;
    }

    // backwards
    double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
    if (LpRmL(xb, yb, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[0], v, u, t);
      Lmin = L;
    }
    if (LpRmL(-xb, yb, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[0], -v, -u, -t);
      Lmin = L;
    }
    if (LpRmL(xb, -yb, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[1], v, u, t);
      Lmin = L;
    }
    if (LpRmL(-xb, -yb, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
      path = ReedsSheppPath(reedsSheppPathType[1], -v, -u, -t);
  }  // CCC

  // formula 8.7
  inline bool LpRupLumRm(double x, double y, double phi, double &t, double &u,
                         double &v) {
    double xi = x + sin(phi), eta = y - 1. - cos(phi),
           rho = 0.25 * (2. + sqrt(xi * xi + eta * eta));
    if (rho <= 1.) {
      u = acos(rho);
      tauOmega(u, -u, xi, eta, phi, t, v);
      assert(fabs(2 * (sin(t) - sin(t - u) + sin(t - 2 * u)) - sin(phi) - x) <
             RS_EPS);
      assert(fabs(2 * (-cos(t) + cos(t - u) - cos(t - 2 * u)) + cos(phi) + 1 -
                  y) < RS_EPS);
      assert(fabs(mod2pi(t - 2 * u - v - phi)) < RS_EPS);
      return t >= -ZERO && v <= ZERO;
    }
    return false;
  }  // LpRupLumRm

  // formula 8.8
  inline bool LpRumLumRp(double x, double y, double phi, double &t, double &u,
                         double &v) {
    double xi = x + sin(phi), eta = y - 1. - cos(phi),
           rho = (20. - xi * xi - eta * eta) / 16.;
    if (rho >= 0 && rho <= 1) {
      u = -acos(rho);
      if (u >= -0.5 * M_PI) {
        tauOmega(u, u, xi, eta, phi, t, v);
        assert(fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x) < RS_EPS);
        assert(fabs(-4 * cos(t) + 2 * cos(t - u) + cos(phi) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
        return t >= -ZERO && v >= -ZERO;
      }
    }
    return false;
  }  // LpRumLumRp

  void CCCC(double x, double y, double phi, ReedsSheppPath &path) {
    double t, u, v, Lmin = path.length(), L;
    if (LpRupLumRm(x, y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[2], t, u, -u, v);
      Lmin = L;
    }
    if (LpRupLumRm(-x, y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[2], -t, -u, u, -v);
      Lmin = L;
    }
    if (LpRupLumRm(x, -y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[3], t, u, -u, v);
      Lmin = L;
    }
    if (LpRupLumRm(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[3], -t, -u, u, -v);
      Lmin = L;
    }

    if (LpRumLumRp(x, y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[2], t, u, u, v);
      Lmin = L;
    }
    if (LpRumLumRp(-x, y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[2], -t, -u, -u, -v);
      Lmin = L;
    }
    if (LpRumLumRp(x, -y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[3], t, u, u, v);
      Lmin = L;
    }
    if (LpRumLumRp(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
      path = ReedsSheppPath(reedsSheppPathType[3], -t, -u, -u, -v);
  }  // CCCC

  // formula 8.9
  inline bool LpRmSmLm(double x, double y, double phi, double &t, double &u,
                       double &v) {
    double xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
    polar(xi, eta, rho, theta);
    if (rho >= 2.) {
      double r = sqrt(rho * rho - 4.);
      u = 2. - r;
      t = mod2pi(theta + atan2(r, -2.));
      v = mod2pi(phi - 0.5 * M_PI - t);
      assert(fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x) < RS_EPS);
      assert(fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y) <
             RS_EPS);
      assert(fabs(mod2pi(t + 0.5 * M_PI + v - phi)) < RS_EPS);
      return t >= -ZERO && u <= ZERO && v <= ZERO;
    }
    return false;
  }  // LpRmSmLm

  // formula 8.10
  inline bool LpRmSmRm(double x, double y, double phi, double &t, double &u,
                       double &v) {
    double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
    polar(-eta, xi, rho, theta);
    if (rho >= 2.) {
      t = theta;
      u = 2. - rho;
      v = mod2pi(t + 0.5 * M_PI - phi);
      assert(fabs(2 * sin(t) - cos(t - v) - u * sin(t) - x) < RS_EPS);
      assert(fabs(-2 * cos(t) - sin(t - v) + u * cos(t) + 1 - y) < RS_EPS);
      assert(fabs(mod2pi(t + 0.5 * M_PI - v - phi)) < RS_EPS);
      return t >= -ZERO && u <= ZERO && v <= ZERO;
    }
    return false;
  }  // LpRmSmRm

  void CCSC(double x, double y, double phi, ReedsSheppPath &path) {
    double t, u, v, Lmin = path.length() - .5 * M_PI, L;
    if (LpRmSmLm(x, y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[4], t, -.5 * M_PI, u, v);
      Lmin = L;
    }
    if (LpRmSmLm(-x, y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[4], -t, .5 * M_PI, -u, -v);
      Lmin = L;
    }
    if (LpRmSmLm(x, -y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[5], t, -.5 * M_PI, u, v);
      Lmin = L;
    }
    if (LpRmSmLm(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[5], -t, .5 * M_PI, -u, -v);
      Lmin = L;
    }

    if (LpRmSmRm(x, y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[8], t, -.5 * M_PI, u, v);
      Lmin = L;
    }
    if (LpRmSmRm(-x, y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[8], -t, .5 * M_PI, -u, -v);
      Lmin = L;
    }
    if (LpRmSmRm(x, -y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[9], t, -.5 * M_PI, u, v);
      Lmin = L;
    }
    if (LpRmSmRm(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[9], -t, .5 * M_PI, -u, -v);
      Lmin = L;
    }

    // backwards
    double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
    if (LpRmSmLm(xb, yb, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[6], v, u, -.5 * M_PI, t);
      Lmin = L;
    }
    if (LpRmSmLm(-xb, yb, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[6], -v, -u, .5 * M_PI, -t);
      Lmin = L;
    }
    if (LpRmSmLm(xb, -yb, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[7], v, u, -.5 * M_PI, t);
      Lmin = L;
    }
    if (LpRmSmLm(-xb, -yb, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[7], -v, -u, .5 * M_PI, -t);
      Lmin = L;
    }

    if (LpRmSmRm(xb, yb, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[10], v, u, -.5 * M_PI, t);
      Lmin = L;
    }
    if (LpRmSmRm(-xb, yb, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[10], -v, -u, .5 * M_PI, -t);
      Lmin = L;
    }
    if (LpRmSmRm(xb, -yb, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[11], v, u, -.5 * M_PI, t);
      Lmin = L;
    }
    if (LpRmSmRm(-xb, -yb, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
      path = ReedsSheppPath(reedsSheppPathType[11], -v, -u, .5 * M_PI, -t);
  }  // CCSC

  // formula 8.11 *** TYPO IN PAPER ***
  inline bool LpRmSLmRp(double x, double y, double phi, double &t, double &u,
                        double &v) {
    double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
    polar(xi, eta, rho, theta);
    if (rho >= 2.) {
      u = 4. - sqrt(rho * rho - 4.);
      if (u <= ZERO) {
        t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
        v = mod2pi(t - phi);
        assert(fabs(4 * sin(t) - 2 * cos(t) - u * sin(t) - sin(phi) - x) <
               RS_EPS);
        assert(fabs(-4 * cos(t) - 2 * sin(t) + u * cos(t) + cos(phi) + 1 - y) <
               RS_EPS);
        assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
        return t >= -ZERO && v >= -ZERO;
      }
    }
    return false;
  }  // LpRmSLmRp

  void CCSCC(double x, double y, double phi, ReedsSheppPath &path) {
    double t, u, v, Lmin = path.length() - M_PI, L;
    if (LpRmSLmRp(x, y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
      path = ReedsSheppPath(reedsSheppPathType[16], t, -0.5 * M_PI, u,
                            -0.5 * M_PI, v);
      Lmin = L;
    }
    if (LpRmSLmRp(-x, y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
      path = ReedsSheppPath(reedsSheppPathType[16], -t, 0.5 * M_PI, -u,
                            0.5 * M_PI, -v);
      Lmin = L;
    }
    if (LpRmSLmRp(x, -y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
      path = ReedsSheppPath(reedsSheppPathType[17], t, -0.5 * M_PI, u,
                            -0.5 * M_PI, v);
      Lmin = L;
    }
    if (LpRmSLmRp(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
      path = ReedsSheppPath(reedsSheppPathType[17], -t, 0.5 * M_PI, -u,
                            0.5 * M_PI, -v);
  }  // CCSCC
};   //  end class ReedsSheppStateSpace

}  // namespace ASV::common::math

#endif /* _REEDS_SHEPP_H_ */
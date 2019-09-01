/*
*******************************************************************************
* lineofsight.h:
* path following using line of sight algorithm
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _TRAJECTORYTRACKING_H_
#define _TRAJECTORYTRACKING_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "controllerdata.h"
#include "easylogging++.h"

class lineofsight {
 public:
  explicit lineofsight(double _los_radius, double _capture_radius = 0)
      : los_radius(_los_radius),
        capture_radius(_capture_radius),
        desired_theta(0),
        trackerror(Eigen::Vector2d::Zero()),
        R(Eigen::Matrix2d::Zero()) {}
  lineofsight() = delete;
  virtual ~lineofsight() = default;

  bool judgewaypoint(const Eigen::Vector2d &_vesselposition,
                     const Eigen::Vector2d &_wp1) {
    Eigen::Vector2d _error = _vesselposition - _wp1;
    double _distance =
        std::sqrt(std::pow(_error(0), 2) + std::pow(_error(1), 2));
    if (_distance < capture_radius) return true;
    return false;
  }

  // compute the orientation of LOS vector and cross-track error
  void computelospoint(const Eigen::Vector2d &_vesselposition,
                       const Eigen::Vector2d &_p0, const Eigen::Vector2d &_p1) {
    auto delta_pos = _p1 - _p0;
    double thetaK = std::atan2(delta_pos(1), delta_pos(0));
    // rotation matrix
    computeR(thetaK);

    // track error
    trackerror = R.transpose() * (_vesselposition - _p0);

    double e = trackerror(1);  // cross-track error
    double thetar = 0;
    if (e > los_radius)
      thetar = -M_PI / 2;
    else if (e < -los_radius)
      thetar = M_PI / 2;
    else
      thetar = std::asin(-e / los_radius);

    desired_theta = thetar + thetaK;

  }  // computelospoint

  double getdesired_theta() const noexcept { return desired_theta; }
  auto gettrackerror() const noexcept { return trackerror; }

  void setcaptureradius(double _captureradius) {
    capture_radius = _captureradius;
  }
  void setlosradius(double _radius) { los_radius = _radius; }

 protected:
  double los_radius;
  double capture_radius;
  double desired_theta;
  Eigen::Vector2d trackerror;
  Eigen::Matrix2d R;

 private:
  // compute the rotation matrix
  void computeR(double theta) {
    double svalue = std::sin(theta);
    double cvalue = std::cos(theta);
    R(0, 0) = cvalue;
    R(0, 1) = -svalue;
    R(1, 0) = svalue;
    R(1, 1) = cvalue;
  }  // computeR
};

class trajectroytracking final : public lineofsight {
 public:
  trajectroytracking(double _los_radius, double _capture_radius,
                     const trackerRTdata &_TrackerRTdata, double _sampletime)
      : lineofsight(_los_radius, _capture_radius),
        TrackerRTdata(_TrackerRTdata),
        sample_time(_sampletime) {}

  ~trajectroytracking() = default;

  // Eigen::MatrixXd followcircle(const Eigen::Vector2d &_startposition,
  //                              const Eigen::Vector2d &_endposition,
  //                              double _radius, double _vesselheading,
  //                              double _desiredspeed) {
  //   Eigen::Vector2d delta_pos = _endposition - _startposition;
  //   double length = computevectorlength(delta_pos(1), delta_pos(0));
  //   double thetaK = computevectororientation(delta_pos(1), delta_pos(0));
  //   Eigen::MatrixXd waypoints(2, 2);
  //   if (length > 2 * _radius) {
  //     waypoints.col(0) = _startposition;
  //     waypoints.col(1) = _endposition;
  //   } else {
  //     double gamma = std::acos(length / (2 * _radius));
  //     int n = static_cast<int>(std::floor((M_PI - 2 * gamma) * 6));
  //     waypoints.resize(Eigen::NoChange, n);

  //     double _clockwise_angle = thetaK + gamma;
  //     double _anticlockwise_angle = thetaK - gamma;

  //     double delta_clockwise_angle = std::abs(
  //         restrictheadingangle(_vesselheading - _clockwise_angle + 0.5 *
  //         M_PI));
  //     double delta_anticlockwise_angle = std::abs(restrictheadingangle(
  //         _vesselheading - _anticlockwise_angle - 0.5 * M_PI));
  //     if (delta_clockwise_angle < delta_anticlockwise_angle) {
  //       // follow the clockwise circle
  //       waypoints = generatecirclepoints(
  //           _startposition + _radius *
  //           computevectorlocation(_clockwise_angle), _radius, n,
  //           _clockwise_angle - M_PI, _anticlockwise_angle);
  //     } else {
  //       // follow the anti-clockwise circle
  //       waypoints = generatecirclepoints(
  //           _startposition +
  //               _radius * computevectorlocation(_anticlockwise_angle),
  //           _radius, n, _clockwise_angle, _anticlockwise_angle + M_PI);
  //     }
  //   }
  //   return waypoints;
  // }

  // follow a circular arc using LOS
  trajectroytracking &CircularArcLOS(double _curvature, double _desired_u,
                                     const Eigen::Vector2d &_vesselposition,
                                     const Eigen::Vector2d &_rp0,
                                     const Eigen::Vector2d &_rp1) {
    // desired u and r
    double _rot = _curvature * _desired_u;
    TrackerRTdata.v_setpoint(0) = _desired_u;
    TrackerRTdata.v_setpoint(2) = _rot;

    // desired heading
    lineofsight::computelospoint(_vesselposition, _rp0, _rp1);
    TrackerRTdata.setpoint(2) = desired_theta;

    return *this;
  }  // CircularArcLOS

  auto gettrackerRTdata() const noexcept { return TrackerRTdata; }

 private:
  trackerRTdata TrackerRTdata;
  const double sample_time;  // sample time of controller
};

#endif /* _TRAJECTORYTRACKING_H_ */
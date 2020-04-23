/*
*******************************************************************************
* trajectorytracking.h:
* The trajectory tracker: including path following using line of sight,
* and path tracking using direct method
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _TRAJECTORYTRACKING_H_
#define _TRAJECTORYTRACKING_H_

#include <stdexcept>

#include "common/logging/include/easylogging++.h"
#include "common/math/miscellaneous/include/math_utils.h"
#include "controllerdata.h"

// TODO:
// move the trajectory tracker into planner

namespace ASV::control {

// path following using line of sight method
class lineofsight {
 public:
  explicit lineofsight(double los_radius, double capture_radius = 0)
      : los_radius_(los_radius),
        capture_radius_(capture_radius),
        desired_theta_(0),
        trackerror_(Eigen::Vector2d::Zero()),
        R_(Eigen::Matrix2d::Zero()) {}
  lineofsight() = delete;
  virtual ~lineofsight() = default;

  // Does the vessel enter the capture radius of waypoint
  bool IsEnterCaptureRadius(const Eigen::Vector2d &_vesselposition,
                            const Eigen::Vector2d &_wp) {
    Eigen::Vector2d _error = _vesselposition - _wp;
    double distance_square = _error(0) * _error(0) + _error(1) * _error(1);
    if (distance_square <= capture_radius_ * capture_radius_) return true;
    return false;
  }  // IsEnterCaptureRadius

  // compute the orientation of LOS vector and cross-track error
  void computelospoint(const Eigen::Vector2d &_vesselposition,
                       const Eigen::Vector2d &_p0, const Eigen::Vector2d &_p1) {
    auto delta_pos = _p1 - _p0;
    double thetaK = std::atan2(delta_pos(1), delta_pos(0));
    // rotation matrix
    computeR(thetaK);

    // track error
    trackerror_ = R_.transpose() * (_vesselposition - _p0);

    double e = trackerror_(1);  // cross-track error
    double thetar = 0;
    if (e > los_radius_)
      thetar = -M_PI / 2;
    else if (e < -los_radius_)
      thetar = M_PI / 2;
    else
      thetar = std::asin(-e / los_radius_);

    desired_theta_ = thetar + thetaK;

  }  // computelospoint

  double desired_theta() const noexcept { return desired_theta_; }
  auto trackerror() const noexcept { return trackerror_; }

  void setcaptureradius(double captureradius) noexcept {
    capture_radius_ = captureradius;
  }
  void setlosradius(double radius) noexcept { los_radius_ = radius; }

 private:
  double los_radius_;
  double capture_radius_;
  double desired_theta_;
  Eigen::Vector2d trackerror_;
  Eigen::Matrix2d R_;
  // compute the rotation matrix
  void computeR(double theta) {
    double svalue = std::sin(theta);
    double cvalue = std::cos(theta);
    R_(0, 0) = cvalue;
    R_(0, 1) = -svalue;
    R_(1, 0) = svalue;
    R_(1, 1) = cvalue;
  }  // computeR
};   // end class lineofsight

class trajectorytracking final : public lineofsight {
 public:
  trajectorytracking(const controllerdata &_controllerdata,
                     const trackerRTdata &_TrackerRTdata)
      : lineofsight(_controllerdata.los_radius,
                    _controllerdata.los_capture_radius),
        TrackerRTdata(_TrackerRTdata),
        sample_time(_controllerdata.sample_time),
        desired_speed(1.0),
        grid_points_index(0),
        grid_points_x((Eigen::VectorXd(2) << 0, 1).finished()),
        grid_points_y((Eigen::VectorXd(2) << 1, 0).finished()) {}

  ~trajectorytracking() = default;

  // follow a set of points like grid using LOS
  void Grid_LOS(const Eigen::Vector2d &_vesselposition) {
    Eigen::Vector2d current_wp0 =
        (Eigen::Vector2d() << grid_points_x(grid_points_index),
         grid_points_y(grid_points_index))
            .finished();
    Eigen::Vector2d current_wp1 =
        (Eigen::Vector2d() << grid_points_x(grid_points_index + 1),
         grid_points_y(grid_points_index + 1))
            .finished();

    if (grid_points_x.size() > 2) {  // multiple waypoints
      // switch waypoints
      if (lineofsight::IsEnterCaptureRadius(_vesselposition, current_wp1)) {
        if (grid_points_index == grid_points_x.size() - 2) {
          TrackerRTdata.trackermode = TRACKERMODE::FINISHED;
          CLOG(INFO, "LOS") << "reach the last waypoint!";
          return;
        } else
          ++grid_points_index;
      }
      // reduce speed based on turning angles
      if (lineofsight::IsEnterCaptureRadius(_vesselposition, current_wp0) &&
          (grid_points_index > 0)) {
        double reduced_desired_speed =
            (1 - std::abs(turning_angles(grid_points_index - 1)) / M_PI) *
            desired_speed;
        CircularArcLOS(0, reduced_desired_speed, _vesselposition, current_wp0,
                       current_wp1);
      } else
        CircularArcLOS(0, desired_speed, _vesselposition, current_wp0,
                       current_wp1);

    } else {  // only two waypoints
      if (lineofsight::IsEnterCaptureRadius(_vesselposition, current_wp1)) {
        TrackerRTdata.trackermode = TRACKERMODE::FINISHED;
        CLOG(INFO, "LOS") << "reach the last waypoint!";
        return;
      }
      CircularArcLOS(0, desired_speed, _vesselposition, current_wp0,
                     current_wp1);
    }

    TrackerRTdata.trackermode = TRACKERMODE::TRACKING;
    return;

  }  // Grid_LOS

  // follow a circular arc without LOS
  trajectorytracking &FollowCircularArc(double desired_curvature,
                                        double desired_speed,
                                        double desired_heading) {
    // desired u and r
    double _rot = desired_curvature * desired_speed;
    TrackerRTdata.v_setpoint(0) = desired_speed;
    TrackerRTdata.v_setpoint(2) = _rot;

    // desired heading
    TrackerRTdata.setpoint(2) =
        common::math::Normalizeheadingangle(desired_heading);

    return *this;
  }  // FollowCircularArc

  // set the waypoints for LOS
  void set_grid_points(const Eigen::VectorXd &_grid_points_x,
                       const Eigen::VectorXd &_grid_points_y,
                       double _desired_speed, double _captureradius) {
    assert(_grid_points_x.size() == _grid_points_y.size());
    assert(_grid_points_x.size() >= 2);
    assert(_desired_speed > 0);

    if (_grid_points_x.size() > 2) {
      turning_angles = compute_turning_angles(_grid_points_x, _grid_points_y);
      lineofsight::setcaptureradius(_captureradius);
    }

    desired_speed = _desired_speed;
    grid_points_x = _grid_points_x;
    grid_points_y = _grid_points_y;
    grid_points_index = 0;  // reset the index

    TrackerRTdata.trackermode = TRACKERMODE::STARTED;

  }  // set_grid_points

  auto gettrackerRTdata() const noexcept { return TrackerRTdata; }

 private:
  trackerRTdata TrackerRTdata;
  const double sample_time;  // sample time of controller

  double desired_speed;
  int grid_points_index;
  Eigen::VectorXd grid_points_x;  // a set of points like grid
  Eigen::VectorXd grid_points_y;
  Eigen::VectorXd
      turning_angles;  // a set of turning angles based on grid points

  // follow a circular arc using LOS
  void CircularArcLOS(double _curvature, double _desired_u,
                      const Eigen::Vector2d &_vesselposition,
                      const Eigen::Vector2d &_rp0,
                      const Eigen::Vector2d &_rp1) {
    // desired u and r
    double _rot = _curvature * _desired_u;
    TrackerRTdata.v_setpoint(0) = _desired_u;
    TrackerRTdata.v_setpoint(2) = _rot;

    // desired heading
    lineofsight::computelospoint(_vesselposition, _rp0, _rp1);
    TrackerRTdata.setpoint(2) = lineofsight::desired_theta();

  }  // CircularArcLOS

  // compute the turning angles of each set of waypoints
  Eigen::VectorXd compute_turning_angles(
      const Eigen::VectorXd &_grid_points_x,
      const Eigen::VectorXd &_grid_points_y) {
    std::size_t num_grid_points = _grid_points_x.size();
    Eigen::VectorXd t_turning_angles =
        Eigen::VectorXd::Zero(num_grid_points - 2);  // assert size > 2

    for (std::size_t i = 0; i != (num_grid_points - 2); ++i) {
      t_turning_angles(i) = common::math::VectorAngle_2d(
          _grid_points_x(i + 1) - _grid_points_x(i),      // x1
          _grid_points_y(i + 1) - _grid_points_y(i),      // y1
          _grid_points_x(i + 2) - _grid_points_x(i + 1),  // x2
          _grid_points_y(i + 2) - _grid_points_y(i + 1)   // y2
      );
    }
    return t_turning_angles;
  }  // compute_turning_angles

};  // end class trajectorytracking
}  // namespace ASV::control

#endif /* _TRAJECTORYTRACKING_H_ */
/*
***********************************************************************
* PathSmoothing.h:
* improve the smoothness of path, using conjugate-gradeient descent
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _PATHSMOOTHING_H_
#define _PATHSMOOTHING_H_

#include "CollisionChecking.h"

namespace ASV::planning {

class PathSmoothing {
  using vec2d = ASV::common::math::Vec2d;
  using vec4t = std::vector<std::tuple<double, double, double, bool>>;

 public:
  explicit PathSmoothing(const SmootherConfig &smootherconfig)
      : dmax_(smootherconfig.d_max),
        x_resolution_(0.01),
        y_resolution_(0.01),
        theta_resolution_(0.01),
        omega_o_(0.05),
        omega_s_(2) {}
  virtual ~PathSmoothing() = default;

  PathSmoothing &SetupCoarsePath(const vec4t &path) {
    // TODO: assert
    assert(path.size() > 3);

    // remove the closed vertex and find the forward/reverse switch vertex
    FindForwardReverseSwitch(path, coarse_vec2d_, coarse_theta_,
                             coarse_isforward_);

    return *this;
  }  // SetupCoarsePath

  // Smoothing for the trajectory of center of vessel box
  PathSmoothing &PerformSmoothing(
      const CollisionChecking_Astar &collision_checker) {
    smooth_path_ = coarse_vec2d_;
    for (std::size_t seg = 0; seg != coarse_vec2d_.size(); seg++) {
      smooth_path_[seg] = OneSegmentSmoothing(
          collision_checker, coarse_vec2d_[seg], dmax_, omega_o_, omega_s_);
    }
    fine_path_ =
        CombineFineTrajectory(smooth_path_, coarse_theta_, coarse_isforward_);

    return *this;
  }  // PerformSmoothing

  auto coarse_vec2d() const noexcept { return coarse_vec2d_; }
  auto coarse_theta() const noexcept { return coarse_theta_; }
  auto coarse_isforward() const noexcept { return coarse_isforward_; }
  auto smooth_path() const noexcept { return smooth_path_; }
  auto fine_path() const noexcept { return fine_path_; }

 private:
  const double dmax_;              // m
  const double x_resolution_;      // m
  const double y_resolution_;      // m
  const double theta_resolution_;  // rad
  const double omega_o_;           // penality of obstacle term
  const double omega_s_;           // penality of smoothing term

  // coarse vertex and index of forward/reverse switch point in coarse path
  mutable std::vector<std::vector<vec2d>> coarse_vec2d_;
  mutable std::vector<std::array<double, 2>> coarse_theta_;
  mutable std::vector<bool> coarse_isforward_;

  // fine path
  mutable std::vector<std::vector<vec2d>> smooth_path_;
  mutable std::vector<std::array<double, 3>> fine_path_;

  // perform path smoothing on one segment
  std::vector<vec2d> OneSegmentSmoothing(
      const CollisionChecking_Astar &collision_checker,
      const std::vector<vec2d> &coarse_path, const double dmax,
      const double omega_o, const double omega_s) {
    if (coarse_path.size() >= 3) {  // ensure the size
      auto smooth_path_ing = coarse_path;
      for (int i = 0; i != 1; ++i) {
        std::vector<std::vector<vec2d>> _nearest_obstacles =
            GenerateNearestNeighbors(collision_checker, smooth_path_ing, dmax);

        // compute the gradient of cost function
        std::vector<vec2d> _gradient = GenerateGradient(
            _nearest_obstacles, smooth_path_ing, dmax, omega_o, omega_s);

        double _norm2_gradient = ComputeL2NormSquare(_gradient);

        // backtracking line search
        static double _alpha = 0.2;
        static double _beta = 0.5;
        double _gamma = 1;  // 0 < gamma
        double _cost = GenerateCost(_nearest_obstacles, smooth_path_ing, dmax,
                                    omega_o, omega_s);

        std::size_t ls_count = 0;  // max iteration in line search
        while (1) {
          auto _smooth_path_ing =
              UpdateTrajectory(smooth_path_ing, _gradient, _gamma);
          if ((_cost - GenerateCost(_nearest_obstacles, _smooth_path_ing, dmax,
                                    omega_o, omega_s) >
               _alpha * _gamma * _norm2_gradient) ||
              (ls_count > 10)) {
            smooth_path_ing = _smooth_path_ing;
            break;
          }
          ls_count++;
          _gamma *= _beta;
        }  // end while loop
      }    // end for loop
      return smooth_path_ing;
    }
    return coarse_path;

  }  // OneSegmentSmoothing

  // compute the nearest neighbors of each vertex on one segment
  std::vector<std::vector<vec2d>> GenerateNearestNeighbors(
      const CollisionChecking_Astar &collision_checker,
      const std::vector<vec2d> &path, const double dmax) const {
    std::size_t size_coarse_path = path.size();
    std::vector<std::vector<vec2d>> _nearest_obstacles(size_coarse_path,
                                                       {{0, 0}});
    // gradient at start/end and fixed node are all zero
    for (std::size_t index = 1; index != (size_coarse_path - 1); ++index) {
      // find the nearest neighbors around the current node
      auto i_nearest_obstacles = collision_checker.FindNearestNeighbors(
          path[index].x(), path[index].y(), dmax);
      _nearest_obstacles[index] = i_nearest_obstacles;
    }
    return _nearest_obstacles;
  }  // GenerateNearestNeighbors

  // compute the gradient of each vertex on one segment
  std::vector<vec2d> GenerateGradient(
      const std::vector<std::vector<vec2d>> &nearest_obstacles,
      const std::vector<vec2d> &path, const double dmax, const double omega_o,
      const double omega_s) const {
    std::size_t size_coarse_path = path.size();

    // compute the gradient of cost function
    std::vector<vec2d> _gradient(size_coarse_path, {0, 0});
    std::vector<vec2d> _Xim1_Xi_Xip1(size_coarse_path, {0, 0});

    for (std::size_t index = 1; index != (size_coarse_path - 1); ++index) {
      _Xim1_Xi_Xip1[index] =
          path[index + 1] + path[index - 1] - path[index] * 2.0;
    }

    // gradient at start/end and fixed node are all zero
    for (std::size_t index = 1; index != (size_coarse_path - 1); ++index) {
      // gradient obstacle
      vec2d i_gradient_obstacle(0, 0);
      for (const auto &nearest_obstacle : nearest_obstacles[index]) {
        // add the obstacle potential in the nearst neighbors
        auto x2o = path[index] - nearest_obstacle;
        // assume the x2o is not very small
        double k = 1.0 - dmax / x2o.Length();
        i_gradient_obstacle += (x2o * k);
      }

      // gradient smoothing
      vec2d i_gradient_smooth = _Xim1_Xi_Xip1[index - 1] +
                                _Xim1_Xi_Xip1[index + 1] -
                                _Xim1_Xi_Xip1[index] * 2;

      // add all gradients
      _gradient[index] =
          i_gradient_smooth * omega_s + i_gradient_obstacle * omega_o;

    }  // end for loop
    return _gradient;
  }  // GenerateGradient

  // compute the cost value given the current trajectory
  double GenerateCost(const std::vector<std::vector<vec2d>> &nearest_obstacles,
                      const std::vector<vec2d> &path, const double dmax,
                      const double omega_o, const double omega_s) const {
    double _obstacle_cost = 0.0;
    double _smooth_cost = 0.0;

    std::size_t size_coarse_path = path.size();

    // gradient at start/end and fixed node are all zero
    for (std::size_t index = 1; index != (size_coarse_path - 1); ++index) {
      // cost obstacle
      double i_obstacle = 0;
      for (const auto &nearest_obstacle : nearest_obstacles[index]) {
        // add the obstacle potential in the nearst neighbors
        auto x2o = path[index] - nearest_obstacle;
        i_obstacle += std::pow(dmax - x2o.Length(), 2);
      }
      _obstacle_cost += i_obstacle;

      // cost smoothing
      auto Xi_Xim1 = path[index] - path[index - 1];
      auto Xip1_Xi = path[index + 1] - path[index];
      auto i_Xim1_Xi_Xip1 = Xip1_Xi - Xi_Xim1;
      _smooth_cost += i_Xim1_Xi_Xip1.LengthSquare();

    }  // end for loop
    return _obstacle_cost * omega_o + _smooth_cost * omega_s;

  }  // GenerateCost

  // update the coarse path
  std::vector<vec2d> UpdateTrajectory(const std::vector<vec2d> &path,
                                      const std::vector<vec2d> &gradient,
                                      const double gamma) const {
    auto new_path = path;
    for (std::size_t index = 0; index != new_path.size(); ++index)
      new_path[index] -= (gradient[index] * gamma);
    return new_path;
  }  // UpdateTrajectory

  // generate the fine trajectory from the fine path
  std::vector<std::array<double, 3>> CombineFineTrajectory(
      const std::vector<std::vector<vec2d>> &fine_path,
      const std::vector<std::array<double, 2>> &fine_theta,
      const std::vector<bool> &isforward) const {
    std::vector<std::array<double, 3>> fine_trajectory;

    for (std::size_t seg = 0; seg != fine_path.size(); ++seg) {
      auto segment_fine_path = fine_path[seg];
      auto segment_theta = fine_theta[seg];
      bool segment_isforward = isforward[seg];

      std::size_t num_segment = segment_fine_path.size();
      if (num_segment > 2) {
        fine_trajectory.push_back({segment_fine_path[0].x(),
                                   segment_fine_path[0].y(),
                                   segment_theta.at(0)});
        for (std::size_t index = 1; index != (num_segment - 1); ++index) {
          vec2d delta_vec2d(0, 0);
          if (segment_isforward)
            delta_vec2d =
                (segment_fine_path[index + 1] - segment_fine_path[index - 1]) *
                0.5;
          else
            delta_vec2d =
                (segment_fine_path[index - 1] - segment_fine_path[index + 1]) *
                0.5;
          double heading = delta_vec2d.Angle();
          fine_trajectory.push_back({segment_fine_path[index].x(),
                                     segment_fine_path[index].y(), heading});
        }
        fine_trajectory.push_back({segment_fine_path.back().x(),
                                   segment_fine_path.back().y(),
                                   segment_theta.at(1)});
      }
      if (num_segment == 2) {
        fine_trajectory.push_back({segment_fine_path[0].x(),
                                   segment_fine_path[0].y(),
                                   segment_theta.at(0)});
        fine_trajectory.push_back({segment_fine_path[1].x(),
                                   segment_fine_path[1].y(),
                                   segment_theta.at(1)});
      }
    }

    return fine_trajectory;
  }  // CombineFineTrajectory

  // find the trajectory node when the forward/reverse switch occurs
  void FindForwardReverseSwitch(
      const vec4t &coarse_trajectory,
      std::vector<std::vector<vec2d>> &coarse_vec2d,
      std::vector<std::array<double, 2>> &coarse_theta,
      std::vector<bool> &coarse_isforward) const {
    vec4t processed_coarse_trajectory;

    coarse_vec2d.clear();
    coarse_theta.clear();
    coarse_isforward.clear();

    // start point
    processed_coarse_trajectory.push_back(coarse_trajectory[0]);

    coarse_vec2d.push_back({vec2d(std::get<0>(coarse_trajectory[0]),
                                  std::get<1>(coarse_trajectory[0]))});
    coarse_theta.push_back({std::get<2>(coarse_trajectory[0]),
                            std::get<2>(coarse_trajectory.back())});
    coarse_isforward.push_back(std::get<3>(coarse_trajectory[0]));

    for (const auto &current_state : coarse_trajectory) {
      auto processed_coarse_last = processed_coarse_trajectory.back();
      if (IsSameNode(std::get<0>(processed_coarse_last),
                     std::get<1>(processed_coarse_last),
                     std::get<2>(processed_coarse_last),
                     std::get<0>(current_state), std::get<1>(current_state),
                     std::get<2>(current_state)) &&
          (std::get<3>(processed_coarse_last) ==
           std::get<3>(current_state))) {  // same state
        continue;
      }

      if (std::get<3>(processed_coarse_last) != std::get<3>(current_state)) {
        // new a list of vertex
        processed_coarse_trajectory.push_back(current_state);
        coarse_vec2d.push_back(
            {vec2d(std::get<0>(current_state), std::get<1>(current_state))});

        coarse_theta.back().at(1) = std::get<2>(current_state);
        coarse_theta.push_back({std::get<2>(current_state),
                                std::get<2>(coarse_trajectory.back())});
        coarse_isforward.push_back(std::get<3>(current_state));
      } else {
        // continue to push_back the node on the current trajectory
        processed_coarse_trajectory.push_back(current_state);
        coarse_vec2d.back().push_back(
            vec2d(std::get<0>(current_state), std::get<1>(current_state)));
      }
    }
  }  // FindForwardReverseSwitch

  // check if two node is closed
  bool IsSameNode(const double lhs_x, const double lhs_y,
                  const double lhs_theta, const double rhs_x,
                  const double rhs_y, const double rhs_theta) const {
    return ((std::abs(lhs_x - rhs_x) <= x_resolution_) &&
            (std::abs(lhs_y - rhs_y) <= y_resolution_) &&
            (std::abs(ASV::common::math::Normalizeheadingangle(
                 lhs_theta - rhs_theta)) <= theta_resolution_));
  }  // IsSameNode

  double ComputeL2NormSquare(const std::vector<vec2d> &x) {
    double l2norm = 0;
    for (const auto &v : x) l2norm += v.LengthSquare();
    return l2norm;
  }  // ComputeL2NormSquare

};  // end class PathSmoothing
}  // namespace ASV::planning

#endif /* _PATHSMOOTHING_H_ */
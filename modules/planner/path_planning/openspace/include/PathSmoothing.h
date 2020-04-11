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

#include <iostream>
#include "CollisionChecking.h"

namespace ASV::planning {

class PathSmoothing {
  using vec2d = ASV::common::math::Vec2d;
  using vec4t = std::vector<std::tuple<double, double, double, bool>>;
  using vec3t = std::vector<std::tuple<vec2d, bool>>;

 public:
  explicit PathSmoothing(const SmootherConfig &smootherconfig)
      : dmax_(smootherconfig.d_max),
        kappa_max_(0),
        x_resolution_(0.01),
        y_resolution_(0.01),
        theta_resolution_(0.01),
        omega_o_(1),
        omega_k_(1),
        omega_s_(1) {}
  virtual ~PathSmoothing() = default;

  PathSmoothing &SetupCoarsePath(const vec4t &path) {
    // TODO: assert
    assert(path.size() > 3);
    coarse_path_ = FindForwardReverseSwitch(path);
    assert(coarse_path_.size() > 3);
    return *this;
  }

  vec4t PerformSmoothing() {}

  auto coarse_path() const noexcept { return coarse_path_; }

 private:
  const double dmax_;              // m
  const double kappa_max_;         // 1/m
  const double x_resolution_;      // m
  const double y_resolution_;      // m
  const double theta_resolution_;  // rad
  const double omega_o_;           // penality of obstacle term
  const double omega_k_;           // penality of curvature term
  const double omega_s_;           // penality of smoothing term

  // coarse vertex and index of forward/reverse switch point in coarse path
  mutable vec3t coarse_path_;

  // fine path
  mutable std::vector<vec2d> fine_path_;

  // compute the gradient of obstacle term
  std::vector<vec2d> GenerateObstacleTerm(
      const CollisionChecking_Astar &collision_checker, const vec3t &path) {
    std::size_t total_num = path.size();
    std::vector<vec2d> obstacleTerm(total_num, {0, 0});

    // obstacleTerm at start/end and fixed node are all zero
    for (std::size_t index = 1; index != (total_num - 1); ++index) {
      if (!std::get<1>(path[index])) {  // not fixed node
        vec2d current_obstacle_term(0, 0);
        auto current_state_vec2d = std::get<0>(path[index]);

        // find the nearest neighbors around the current node
        auto nearest_obstacles = collision_checker.FindNearestNeighbors(
            current_state_vec2d.x(), current_state_vec2d.y(), dmax_);

        // add the obstacle potential in the nearst neighbors
        for (const auto &nearest_obstacle : nearest_obstacles) {
          auto x2o = current_state_vec2d -
                     vec2d(nearest_obstacle.x(), nearest_obstacle.y());
          // assume the x2o is not very small
          double k = 2 * omega_o_ * (1.0 - dmax_ / x2o.Length());
          current_obstacle_term += (x2o * k);
        }

        obstacleTerm[index] = current_obstacle_term;
      }  // end if
    }    // end for loop
    return obstacleTerm;
  }  // GenerateObstacleTerm

  // compute the gradient of smoothing term
  std::vector<vec2d> GenerateSmoothTerm(const vec3t &path) {
    std::size_t total_num = path.size();
    std::vector<vec2d> SmoothTerm(total_num, {0, 0});
    std::vector<vec2d> Xim1_Xi_Xip1(total_num, {0, 0});

    for (std::size_t index = 1; index != (total_num - 1); ++index) {
      Xim1_Xi_Xip1[index] = std::get<0>(path[index - 1]) +
                            std::get<0>(path[index + 1]) -
                            std::get<0>(path[index]) * 2;
    }

    // SmoothTerm at start/end and fixed node are all zero
    for (std::size_t index = 1; index != (total_num - 1); ++index) {
      if (!std::get<1>(path[index])) {  // not fixed node
        vec2d current_smooth_term = Xim1_Xi_Xip1[index - 1] +
                                    Xim1_Xi_Xip1[index + 1] -
                                    Xim1_Xi_Xip1[index] * 2;
        current_smooth_term *= (2 * omega_s_);
        SmoothTerm[index] = current_smooth_term;
      }  // end if
    }    // end for loops
    return SmoothTerm;

  }  // GenerateSmoothTerm

  // compute the gradient of curvature term ()
  std::vector<vec2d> GenerateCurvatureTerm(const vec3t &path) {
    std::size_t total_num = path.size();
    std::vector<vec2d> CurvatureTerm(total_num, {0, 0});
    std::vector<vec2d> P1i(total_num, {0, 0});
    std::vector<vec2d> P2i(total_num, {0, 0});
    std::vector<double> Delta_Thetai(total_num, 0.0);

    // prepare
    for (std::size_t index = 1; index != (total_num - 1); ++index) {
      auto Xi_Xim1 = std::get<0>(path[index]) - std::get<0>(path[index - 1]);
      auto Xip1_Xi = std::get<0>(path[index + 1]) - std::get<0>(path[index]);
      auto n_Xip1_Xi = Xip1_Xi * (-1);

      auto _P1i = Xi_Xim1.OrthogonalComplementNormal(n_Xip1_Xi);
      auto _P2i = n_Xip1_Xi.OrthogonalComplementNormal(Xi_Xim1);
      double _Delta_Thetai = vec2d::AngleTwoVectors(Xi_Xim1, Xip1_Xi);

      double _reciprocal_length = 1.0 / Xi_Xim1.Length();
      double _Delta_Thetai_sin = std::sin(_Delta_Thetai);
      double _reciprocal_length_sin = 0;
      if (_Delta_Thetai_sin <= theta_resolution_)
        _reciprocal_length_sin = _reciprocal_length / theta_resolution_;
      else
        _reciprocal_length_sin = _reciprocal_length / _Delta_Thetai_sin;

      P1i[index] = _P1i * _reciprocal_length_sin;
      P2i[index] = _P2i * _reciprocal_length_sin;
      Delta_Thetai[index] = _Delta_Thetai * _reciprocal_length;
    }

    // CurvatureTerm at start/end and fixed node are all zero
    for (std::size_t index = 1; index != (total_num - 1); ++index) {
      if (!std::get<1>(path[index])) {  // not fixed node

        auto Xi_Xim1 = std::get<0>(path[index]) - std::get<0>(path[index - 1]);
        auto Xip1_Xi = std::get<0>(path[index + 1]) - std::get<0>(path[index]);

        vec2d current_curvature_term =
            (Xip1_Xi * (Delta_Thetai[index + 1] / Xip1_Xi.LengthSquare()) -
             P2i[index + 1]) *
                Delta_Thetai[index + 1] +
            (P1i[index] + P2i[index] -
             Xi_Xim1 * (Delta_Thetai[index] / Xi_Xim1.LengthSquare())) *
                Delta_Thetai[index] -
            P1i[index - 1] * Delta_Thetai[index - 1];
        current_curvature_term *= (2 * omega_k_);
        CurvatureTerm[index] = current_curvature_term;
      }  // end if
    }    // end for loops

    return CurvatureTerm;
  }  // GenerateCurvatureTerm

  // find the trajectory node when the forward/reverse switch occurs
  vec3t FindForwardReverseSwitch(const vec4t &coarse_trajectory) const {
    vec4t processed_coarse_trajectory;
    vec3t processed_coarse_vertex;

    // start point
    processed_coarse_trajectory.push_back(coarse_trajectory[0]);
    processed_coarse_vertex.push_back({vec2d(std::get<0>(coarse_trajectory[0]),
                                             std::get<1>(coarse_trajectory[0])),
                                       false});

    for (const auto &current_state : coarse_trajectory) {
      auto processed_coarse_last = processed_coarse_trajectory.back();
      if ((std::abs(std::get<0>(processed_coarse_last) -
                    std::get<0>(current_state)) <= x_resolution_) &&
          (std::abs(std::get<1>(processed_coarse_last) -
                    std::get<1>(current_state)) <= y_resolution_) &&
          (std::abs(ASV::common::math::Normalizeheadingangle(
               std::get<2>(processed_coarse_last) -
               std::get<2>(current_state))) <=
           theta_resolution_)) {  // same state
        if (std::get<3>(processed_coarse_last) == std::get<3>(current_state)) {
          std::get<1>(processed_coarse_vertex.back()) = false;
        } else
          std::get<1>(processed_coarse_vertex.back()) = true;
      } else {
        // different state
        processed_coarse_trajectory.push_back(current_state);
        processed_coarse_vertex.push_back(
            {vec2d(std::get<0>(current_state), std::get<1>(current_state)),
             false});
      }
    }
    return processed_coarse_vertex;
  }  // FindForwardReverseSwitch

};  // end class PathSmoothing
}  // namespace ASV::planning

#endif /* _PATHSMOOTHING_H_ */
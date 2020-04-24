/*
*******************************************************************************
* OpenSpacePlanner.h:
* Path planner used in the low-speed vessel, including hybrid A star,
* trajectory smoother and collision checking. This planner is designed to
* be used in both fully-actuated and underactuated vessels.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _OPENSPACEPLANNER_H_
#define _OPENSPACEPLANNER_H_

#include "HybridAStar.h"
#include "PathSmoothing.h"

#include <iostream>

namespace ASV::planning {

class OpenSpacePlanner {
 public:
  enum HybridAStarStatus {
    FAILURE = 0,
    START_END_CLOSED,
    SUCCESS,
  };

  OpenSpacePlanner(const CollisionData &collisiondata,
                   const HybridAStarConfig &hybridastarconfig,
                   const SmootherConfig &smootherconfig)
      : sample_time_(0.5),
        move_length_(hybridastarconfig.move_length),
        status_(FAILURE),
        collision_checker_(collisiondata),
        Hybrid_AStar_(collisiondata, hybridastarconfig),
        path_smoother_(smootherconfig),
        Planning_State_({
            0,  // x
            0,  // y
            0,  // theta
            0,  // speed
            0   // kappa
        }) {}
  virtual ~OpenSpacePlanner() = default;

  OpenSpacePlanner &update_obstacles(
      const std::vector<Obstacle_Vertex_Config> &Obstacles_Vertex,
      const std::vector<Obstacle_LineSegment_Config> &Obstacles_LineSegment,
      const std::vector<Obstacle_Box2d_Config> &Obstacles_Box2d) {
    collision_checker_.set_all_obstacls(Obstacles_Vertex, Obstacles_LineSegment,
                                        Obstacles_Box2d);

    return *this;
  }  // update_obstacles

  // setup the start and end points of the center of vessel box, uses the
  // coordinate of CoG of vessel as input
  OpenSpacePlanner &update_start_end(
      const std::array<double, 3> &start_point_cog,
      const std::array<double, 3> &end_point_cog) {
    // status
    status_ = FAILURE;

    // transformation
    auto start_point = collision_checker_.Transform2Center(start_point_cog);
    auto end_point = collision_checker_.Transform2Center(end_point_cog);

    if (Hybrid_AStar_.setup_start_end(static_cast<float>(start_point.at(0)),
                                      static_cast<float>(start_point.at(1)),
                                      static_cast<float>(start_point.at(2)),
                                      static_cast<float>(end_point.at(0)),
                                      static_cast<float>(end_point.at(1)),
                                      static_cast<float>(end_point.at(2)))) {
      CLOG(INFO, "Hybrid_Astar") << "start and end points are too closed!";
      status_ = SUCCESS;
    };

    return *this;
  }  // update_start_end

  OpenSpacePlanner &GenerateTrajectory() {
    auto coarse_path_direction =
        Hybrid_AStar_.perform_4dnode_search(collision_checker_);
    path_smoother_.SetupCoarsePath(coarse_path_direction);
    auto center_fine_path =
        path_smoother_.PerformSmoothing(collision_checker_).fine_path();

    std::vector<std::array<double, 3>> center_coarse_path;
    for (const auto &value : coarse_path_direction)
      center_coarse_path.push_back(
          {std::get<0>(value), std::get<1>(value), std::get<2>(value)});
    cog_coarse_path_ = collision_checker_.Transform2CoG(center_coarse_path);
    // cog_coarse_path_ = center_coarse_path;
    cog_fine_path_ = collision_checker_.Transform2CoG(center_fine_path);

    return *this;

  }  // GenerateTrajectory

  OpenSpacePlanner &GenerateTrajectory(
      const std::array<double, 3> &start_point_cog,
      const std::array<double, 3> &end_point_cog) {
    update_start_end(start_point_cog, end_point_cog);

    if (status_ != SUCCESS) {
      auto coarse_path_direction =
          Hybrid_AStar_.perform_4dnode_search(collision_checker_);
      if (coarse_path_direction.size() < 2) {
        Planning_State_ = {
            start_point_cog.at(0),  // x
            start_point_cog.at(1),  // y
            start_point_cog.at(2),  // theta
            0,                      // speed
            0,                      // kappa
        };
      } else {
        std::cout << "search success\n";
        for (const auto &value : coarse_path_direction)
          std::cout << std::get<0>(value) << ", " << std::get<1>(value) << ", "
                    << std::get<2>(value) << ", " << std::get<3>(value)
                    << std::endl;

        auto first_center_state = coarse_path_direction[0];
        auto second_center_state = coarse_path_direction[1];

        double t_speed = std::get<3>(first_center_state) ? 0.5 : -0.5;

        auto second_cog_state = collision_checker_.Transform2CoG(
            {std::get<0>(second_center_state), std::get<1>(second_center_state),
             std::get<2>(second_center_state)});
        Planning_State_ = {
            second_cog_state.at(0),  // x
            second_cog_state.at(1),  // y
            second_cog_state.at(2),  // theta
            t_speed,                 // speed
            (std::get<2>(second_center_state) -
             std::get<2>(first_center_state)) /
                move_length_,  // kappa
        };
      }
    }  // end if(status)

    return *this;
  }  // GenerateTrajectory

  auto sampletime() const noexcept { return sample_time_; }
  auto status() const noexcept { return status_; }
  auto coarse_path() const noexcept { return cog_coarse_path_; }
  auto cog_path() const noexcept { return cog_fine_path_; }
  auto Planning_State() const noexcept { return Planning_State_; }

 private:
  const double sample_time_;  // s
  const double move_length_;  // m

  HybridAStarStatus status_;

  CollisionChecking_Astar collision_checker_;
  HybridAStar Hybrid_AStar_;
  PathSmoothing path_smoother_;

  OpenSpace_Planning_State Planning_State_;
  std::vector<std::array<double, 3>> cog_coarse_path_;
  std::vector<std::array<double, 3>> cog_fine_path_;

};  // namespace ASV::planning

}  // namespace ASV::planning

#endif /* _OPENSPACEPLANNER_H_ */
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
  OpenSpacePlanner(const CollisionData &collisiondata,
                   const HybridAStarConfig &hybridastarconfig,
                   const SmootherConfig &smootherconfig)
      : collision_checker_(collisiondata),
        Hybrid_AStar_(collisiondata, hybridastarconfig),
        path_smoother_(smootherconfig) {}
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
    // transformation
    auto start_point = collision_checker_.Transform2Center(start_point_cog);
    auto end_point = collision_checker_.Transform2Center(end_point_cog);

    Hybrid_AStar_.setup_start_end(static_cast<float>(start_point.at(0)),
                                  static_cast<float>(start_point.at(1)),
                                  static_cast<float>(start_point.at(2)),
                                  static_cast<float>(end_point.at(0)),
                                  static_cast<float>(end_point.at(1)),
                                  static_cast<float>(end_point.at(2)));
    return *this;
  }  // update_start_end

  OpenSpacePlanner &GenerateTrajectory() {
    Hybrid_AStar_.perform_4dnode_search(collision_checker_);
    auto coarse_path_direction = Hybrid_AStar_.hybridastar_trajecotry();
    // path_smoother_.SetupCoarsePath(coarse_path_direction);
    // auto center_fine_path =
    //     path_smoother_.PerformSmoothing(collision_checker_).fine_path();

    std::vector<std::array<double, 3>> center_coarse_path;
    for (const auto &value : coarse_path_direction)
      center_coarse_path.push_back(
          {std::get<0>(value), std::get<1>(value), std::get<2>(value)});
    cog_coarse_path_ = collision_checker_.Transform2CoG(center_coarse_path);
    // cog_coarse_path_ = center_coarse_path;
    // cog_fine_path_ = collision_checker_.Transform2CoG(center_fine_path);

    std::cout << Hybrid_AStar_.status() << std::endl;

    return *this;

  }  // GenerateTrajectory

  OpenSpace_Planning_State GenerateTrajectory(
      const std::array<double, 3> &start_point_cog,
      const std::array<double, 3> &end_point_cog) {
    update_start_end(start_point_cog, end_point_cog);
    Hybrid_AStar_.perform_4dnode_search(collision_checker_);
    auto coarse_path_direction = Hybrid_AStar_.hybridastar_trajecotry();

    if (Hybrid_AStar_.status() == HybridAStar::SUCCESS) {
      std::cout << "search success\n";
      for (const auto &value : coarse_path_direction)
        std::cout << std::get<0>(value) << ", " << std::get<1>(value) << ", "
                  << std::get<2>(value) << ", " << std::get<3>(value)
                  << std::endl;
    } else if (Hybrid_AStar_.status() == HybridAStar::FAILURE) {
      std::cout << "failure\n";
      for (const auto &value : coarse_path_direction)
        std::cout << std::get<0>(value) << ", " << std::get<1>(value) << ", "
                  << std::get<2>(value) << ", " << std::get<3>(value)
                  << std::endl;
    } else {
      std::cout << "closed\n";
      for (const auto &value : coarse_path_direction)
        std::cout << std::get<0>(value) << ", " << std::get<1>(value) << ", "
                  << std::get<2>(value) << ", " << std::get<3>(value)
                  << std::endl;
    }
    // cog_coarse_path_ = collision_checker_.Transform2CoG(center_coarse_path);
    // cog_coarse_path_ = center_coarse_path;

    OpenSpace_Planning_State _next_Planning_State{0, 0, 0};

    return _next_Planning_State;
  }  // GenerateTrajectory

  auto coarse_path() const noexcept { return cog_coarse_path_; }
  auto cog_path() const noexcept { return cog_fine_path_; }

 private:
  CollisionChecking_Astar collision_checker_;
  HybridAStar Hybrid_AStar_;
  PathSmoothing path_smoother_;

  std::vector<std::array<double, 3>> cog_coarse_path_;
  std::vector<std::array<double, 3>> cog_fine_path_;

};  // end class OpenSpacePlanner

}  // namespace ASV::planning

#endif /* _OPENSPACEPLANNER_H_ */
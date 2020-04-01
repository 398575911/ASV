/////////////////////////////////////////////////////////////////////////

// Hybrid A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

//////////////////////////////////////////////////////////////////////////

#include "../include/HybridAStar.h"
#include <iostream>
#include "common/plotting/include/gnuplot-iostream.h"

constexpr int DEBUG_LISTS = 0;
constexpr int DEBUG_LIST_LENGTHS_ONLY = 0;

using namespace ASV;

planning::CollisionData _collisiondata{
    4,     // MAX_SPEED
    4.0,   // MAX_ACCEL
    -3.0,  // MIN_ACCEL
    2.0,   // MAX_ANG_ACCEL
    -2.0,  // MIN_ANG_ACCEL
    0.2,   // MAX_CURVATURE
    4,     // HULL_LENGTH
    2,     // HULL_WIDTH
    2,     // HULL_BACK2COG
    3.3    // ROBOT_RADIUS
};

double ego_length = _collisiondata.HULL_LENGTH;
double ego_width = _collisiondata.HULL_WIDTH;
double ego_center_local_x = 0.5 * ego_length - _collisiondata.HULL_BACK2COG;
double ego_center_local_y = 0.0;

// illustrate the hybrid A* planner at a time instant
void rtplotting_bestpath(
    Gnuplot &_gp, const std::vector<std::array<double, 3>> &trajectory,
    const std::vector<planning::Obstacle_Vertex> &Obstacles_Vertex,
    const std::vector<planning::Obstacle_LineSegment> &Obstacles_LineSegment,
    const std::vector<planning::Obstacle_Box2d> &Obstacles_Box2d) {
  std::vector<std::pair<double, double>> xy_pts_A;

  for (std::size_t i = 0; i != trajectory.size(); i++) {
    auto state = trajectory[i];
    double cvalue = std::cos(state.at(2));
    double svalue = std::sin(state.at(2));
    double ego_center_x =
        state.at(0) + cvalue * ego_center_local_x - svalue * ego_center_local_y;
    double ego_center_y =
        state.at(1) + svalue * ego_center_local_x + cvalue * ego_center_local_y;

    common::math::Box2d ego_box(
        (Eigen::Vector2d() << ego_center_x, ego_center_y).finished(),
        state.at(2), ego_length, ego_width);

    auto allcorners = ego_box.GetAllCorners();

    xy_pts_A.push_back(std::make_pair(state.at(0), state.at(1)));

    //
    _gp << "set object " + std::to_string(i) + " polygon from";
    for (int j = 0; j != 4; ++j) {
      _gp << " " << allcorners(0, j) << ", " << allcorners(1, j) << " to";
    }
    _gp << " " << allcorners(0, 0) << ", " << allcorners(1, 0) << "\n";
    _gp << "set object " + std::to_string(i) +
               " fc rgb 'blue' fillstyle solid 0.2 noborder\n";
  }

  // trajectory
  _gp << "plot ";
  _gp << _gp.file1d(xy_pts_A)
      << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
         "pointsize 1 title 'best path',";

  // obstacle
  xy_pts_A.clear();
  // for (std::size_t i = 0; i != _cart_obstacle_x.size(); ++i) {
  //   xy_pts_A.push_back(
  //       std::make_pair(-_cart_obstacle_y[i], _cart_obstacle_x[i]));
  // }
  // _gp << _gp.file1d(xy_pts_A)
  //     << " with points pt 9 ps 3 lc rgb '#2166AC' title 'obstacles', ";

  _gp.flush();

}  // rtplotting_bestpath

// Main
int main() {
  planning::HybridAStarConfig _HybridAStarConfig{
      1,    // move_length
      1.5,  // penalty_turning
      2,    // penalty_reverse
      3,    // penalty_switch
  };

  // obstacles
  std::vector<planning::Obstacle_Vertex> Obstacles_Vertex{{
      10,  // x
      2    // y
  }};
  std::vector<planning::Obstacle_LineSegment> Obstacles_lS{{
                                                               4,  // start_x
                                                               4,  // start_y
                                                               4,  // end_x
                                                               -4  // end_y
                                                           },
                                                           {
                                                               4,   // start_x
                                                               -4,  // start_y
                                                               -4,  // end_x
                                                               -4   // end_y
                                                           },
                                                           {
                                                               -4,  // start_x
                                                               -4,  // start_y
                                                               -4,  // end_x
                                                               4    // end_y
                                                           }};
  std::vector<planning::Obstacle_Box2d> Obstacles_Box{{
      10,  // center_x
      10,  // center_y
      4,   // length
      2,   // width
      0    // heading
  }};

  float start_x = 5;
  float start_y = 6;
  float start_theta = 0;
  float end_x = 11;
  float end_y = 24;
  float end_theta = 0;

  planning::HybridAStar Hybrid_AStar(_collisiondata, _HybridAStarConfig);
  Hybrid_AStar.update_obstacles(Obstacles_Vertex, Obstacles_lS, Obstacles_Box)
      .setup_start_end(start_x, start_y, start_theta, end_x, end_y, end_theta);

  Hybrid_AStar.performsearch();

  auto hr = Hybrid_AStar.hybridastar_trajecotry();

  Gnuplot gp;
  gp << "set terminal x11 size 1100, 1200 0\n";
  gp << "set title 'A star search'\n";

  rtplotting_bestpath(gp, hr, Obstacles_Vertex, Obstacles_lS, Obstacles_Box);

  return 0;
}
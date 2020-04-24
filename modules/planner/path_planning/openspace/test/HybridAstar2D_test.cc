/////////////////////////////////////////////////////////////////////////

// Hybrid A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

//////////////////////////////////////////////////////////////////////////

#include "../include/HybridAStar2D.h"
#include <chrono>
#include <iostream>
#include <thread>
#include "DataFactory.hpp"
#include "common/plotting/include/gnuplot-iostream.h"

using namespace ASV::planning;

// illustrate the hybrid A* planner at a time instant
void rtplotting_2dbestpath(
    Gnuplot &_gp, const std::array<double, 3> &start_point,
    const std::array<double, 3> &end_point, const std::array<double, 3> &state,
    const std::vector<std::array<double, 3>> &trajectory,
    const std::vector<Obstacle_Vertex_Config> &Obstacles_Vertex,
    const std::vector<Obstacle_LineSegment_Config> &Obstacles_LineSegment,
    const std::vector<Obstacle_Box2d_Config> &Obstacles_Box2d) {
  std::vector<std::pair<double, double>> xy_pts_A;

  plot_vessel(_gp, start_point.at(0), start_point.at(1), start_point.at(2), 1);
  plot_vessel(_gp, end_point.at(0), end_point.at(1), end_point.at(2), 2);
  plot_vessel(_gp, state.at(0), state.at(1), state.at(2), 3);

  // obstacle (box)
  for (std::size_t i = 0; i != Obstacles_Box2d.size(); ++i) {
    ASV::common::math::Box2d ob_box(
        ASV::common::math::Vec2d(Obstacles_Box2d[i].center_x,
                                 Obstacles_Box2d[i].center_y),
        Obstacles_Box2d[i].heading, Obstacles_Box2d[i].length,
        Obstacles_Box2d[i].width);
    auto allcorners = ob_box.GetAllCorners();

    //
    _gp << "set object " + std::to_string(i + 5) + " polygon from";
    for (int j = 0; j != 4; ++j) {
      _gp << " " << allcorners[j].x() << ", " << allcorners[j].y() << " to";
    }
    _gp << " " << allcorners[0].x() << ", " << allcorners[0].y() << "\n";
    _gp << "set object " + std::to_string(i + 5) +
               " fc rgb '#000000' fillstyle solid lw 0\n";
  }

  _gp << "plot ";
  // trajectory
  for (const auto &ps : trajectory) {
    xy_pts_A.push_back(std::make_pair(ps.at(0), ps.at(1)));
  }
  _gp << _gp.file1d(xy_pts_A)
      << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
         "pointsize 1 title 'best path',";

  // obstacle (vertex)
  xy_pts_A.clear();
  for (std::size_t i = 0; i != Obstacles_Vertex.size(); ++i) {
    xy_pts_A.push_back(
        std::make_pair(Obstacles_Vertex[i].x, Obstacles_Vertex[i].y));
  }
  _gp << _gp.file1d(xy_pts_A)
      << " with points pt 9 ps 3 lc rgb '#000000' title 'obstacles_v', ";

  // obstacle (Line Segment)
  for (std::size_t i = 0; i != Obstacles_LineSegment.size(); ++i) {
    xy_pts_A.clear();
    xy_pts_A.push_back(std::make_pair(Obstacles_LineSegment[i].start_x,
                                      Obstacles_LineSegment[i].start_y));
    xy_pts_A.push_back(std::make_pair(Obstacles_LineSegment[i].end_x,
                                      Obstacles_LineSegment[i].end_y));
    _gp << _gp.file1d(xy_pts_A)
        << " with linespoints linetype 1 lw 2 lc rgb '#000000' pointtype 7 "
           "pointsize 1 notitle, ";
  }
  _gp << "\n";

  _gp.flush();

}  // rtplotting_2dbestpath

// Main
int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  SearchConfig2D _SearchConfig2D{
      1.05,  // move_length
      4,     //
  };

  int test_scenario = 3;

  // obstacles
  std::vector<Obstacle_Vertex_Config> Obstacles_Vertex;
  std::vector<Obstacle_LineSegment_Config> Obstacles_LS;
  std::vector<Obstacle_Box2d_Config> Obstacles_Box;
  std::array<double, 3> start_point;
  std::array<double, 3> end_point;
  generate_obstacle_map(Obstacles_Vertex, Obstacles_LS, Obstacles_Box,
                        start_point, end_point, test_scenario);

  // Collision Checking
  CollisionChecking_Astar collision_checker_(_collisiondata);
  collision_checker_.set_all_obstacls(Obstacles_Vertex, Obstacles_LS,
                                      Obstacles_Box);

  // Hybrid A* search
  HybridAStar2D Hybrid_AStar(_SearchConfig2D);
  Hybrid_AStar.setup_2d_start_end(start_point.at(0), start_point.at(1),
                                  start_point.at(2), end_point.at(0),
                                  end_point.at(1), end_point.at(2));

  Hybrid_AStar.perform_2dnode_search(collision_checker_);

  auto hr2d = Hybrid_AStar.hybridastar_2dtrajecotry();

  std::cout << "coarse\n";
  for (const auto &value : hr2d) {
    std::cout << value[0] << ", " << value[1] << ", " << value[2] << std::endl;
  }

  Gnuplot gp1;
  gp1 << "set terminal x11 size 1100, 1100 1\n";
  gp1 << "set title 'A star search (2d)'\n";
  gp1 << "set xrange [0:40]\n";
  gp1 << "set yrange [0:40]\n";
  gp1 << "set size ratio -1\n";

  for (std::size_t i = 0; i != hr2d.size(); ++i) {
    rtplotting_2dbestpath(
        gp1, {start_point.at(0), start_point.at(1), start_point.at(2)},
        {end_point.at(0), end_point.at(1), end_point.at(2)}, hr2d[i], hr2d,
        Obstacles_Vertex, Obstacles_LS, Obstacles_Box);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return 0;
}
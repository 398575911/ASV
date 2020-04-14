/*
*******************************************************************************
* OpenSpace_test.cc:
* unit test for openspace planner
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <thread>
#include "../include/OpenSpacePlanner.h"
#include "DataFactory.hpp"

using namespace ASV::planning;

// illustrate the hybrid A* planner at a time instant
void compare_bestpath(
    Gnuplot &_gp, const std::array<double, 3> &start_point,
    const std::array<double, 3> &end_point,
    const std::vector<std::array<double, 3>> &coarse_trajectory,
    const std::vector<std::array<double, 3>> &fine_trajectory,
    const std::vector<Obstacle_Vertex_Config> &Obstacles_Vertex,
    const std::vector<Obstacle_LineSegment_Config> &Obstacles_LineSegment,
    const std::vector<Obstacle_Box2d_Config> &Obstacles_Box2d) {
  std::vector<std::pair<double, double>> xy_pts_A;

  plot_vessel(_gp, start_point.at(0), start_point.at(1), start_point.at(2), 1);
  plot_vessel(_gp, end_point.at(0), end_point.at(1), end_point.at(2), 2);

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
  // coarse trajectory
  for (const auto &ps : coarse_trajectory) {
    xy_pts_A.push_back(std::make_pair(ps.at(0), ps.at(1)));
  }
  _gp << _gp.file1d(xy_pts_A)
      << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
         "pointsize 1 title 'coarse',";

  // fine trajectory
  xy_pts_A.clear();
  for (const auto &ps : fine_trajectory) {
    xy_pts_A.push_back(std::make_pair(ps.at(0), ps.at(1)));
  }
  _gp << _gp.file1d(xy_pts_A)
      << " with linespoints linetype 1 lw 2 lc rgb '#b5367a' pointtype 7 "
         "pointsize 1 title 'fine',";

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

int main() {
  // obstacles
  int test_scenario = 7;

  std::vector<Obstacle_Vertex_Config> Obstacles_Vertex;
  std::vector<Obstacle_LineSegment_Config> Obstacles_LS;
  std::vector<Obstacle_Box2d_Config> Obstacles_Box;
  std::array<double, 3> start_point;
  std::array<double, 3> end_point;
  generate_obstacle_map(Obstacles_Vertex, Obstacles_LS, Obstacles_Box,
                        start_point, end_point, test_scenario);

  HybridAStarConfig _HybridAStarConfig{
      1.0,  // move_length
      1.2,  // penalty_turning
      1.2,  // penalty_reverse
      1.5   // penalty_switch
            // 5,    // num_interpolate
  };
  SmootherConfig smoothconfig{
      5,  // d_max
  };

  OpenSpacePlanner openspace(_collisiondata, _HybridAStarConfig, smoothconfig);
  openspace.update_obstacles(Obstacles_Vertex, Obstacles_LS, Obstacles_Box);
  openspace.update_start_end(start_point, end_point);
  openspace.GenerateTrajectory();

  auto hr_plot = openspace.coarse_path();
  auto hr_plot_fine = openspace.fine_path();

  Gnuplot gp;
  gp << "set terminal x11 size 1100, 1100 0\n";
  gp << "set title 'A star search (4d)'\n";
  gp << "set xrange [0:40]\n";
  gp << "set yrange [0:40]\n";

  // hr_plot = {{1, 20, 0}};

  compare_bestpath(gp,
                   {start_point.at(0), start_point.at(1), start_point.at(2)},
                   {end_point.at(0), end_point.at(1), end_point.at(2)}, hr_plot,
                   hr_plot_fine, Obstacles_Vertex, Obstacles_LS, Obstacles_Box);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
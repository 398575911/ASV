/////////////////////////////////////////////////////////////////////////

// Hybrid A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

//////////////////////////////////////////////////////////////////////////

#include "../include/HybridAStar.h"
#include <chrono>
#include <iostream>
#include <thread>
#include "common/plotting/include/gnuplot-iostream.h"
using namespace ASV;

planning::CollisionData _collisiondata{
    4,     // MAX_SPEED
    4.0,   // MAX_ACCEL
    -3.0,  // MIN_ACCEL
    2.0,   // MAX_ANG_ACCEL
    -2.0,  // MIN_ANG_ACCEL
    0.3,   // MAX_CURVATURE
    4,     // HULL_LENGTH
    2,     // HULL_WIDTH
    1,     // HULL_BACK2COG
    3.3    // ROBOT_RADIUS
};

double ego_length = _collisiondata.HULL_LENGTH;
double ego_width = _collisiondata.HULL_WIDTH;
double ego_center_local_x = 0.5 * ego_length - _collisiondata.HULL_BACK2COG;
double ego_center_local_y = 0.0;

// illustrate the hybrid A* planner at a time instant
void rtplotting_bestpath(
    Gnuplot &_gp, const std::array<double, 3> &state,
    const std::vector<planning::Obstacle_Vertex> &Obstacles_Vertex,
    const std::vector<planning::Obstacle_LineSegment> &Obstacles_LineSegment,
    const std::vector<planning::Obstacle_Box2d> &Obstacles_Box2d) {
  std::vector<std::pair<double, double>> xy_pts_A;

  double cvalue = std::cos(state.at(2));
  double svalue = std::sin(state.at(2));
  double ego_center_x =
      state.at(0) + cvalue * ego_center_local_x - svalue * ego_center_local_y;
  double ego_center_y =
      state.at(1) + svalue * ego_center_local_x + cvalue * ego_center_local_y;

  common::math::Box2d ego_box(
      (Eigen::Vector2d() << ego_center_x, ego_center_y).finished(), state.at(2),
      ego_length, ego_width);

  auto allcorners = ego_box.GetAllCorners();

  xy_pts_A.push_back(std::make_pair(state.at(0), state.at(1)));

  //
  _gp << "set object 1 polygon from";
  for (int j = 0; j != 4; ++j) {
    _gp << " " << allcorners(0, j) << ", " << allcorners(1, j) << " to";
  }
  _gp << " " << allcorners(0, 0) << ", " << allcorners(1, 0) << "\n";
  _gp << "set object 1 fc rgb 'blue' fillstyle solid 0.1 noborder\n";

  // obstacle (box)
  for (std::size_t i = 0; i != Obstacles_Box2d.size(); ++i) {
    ASV::common::math::Box2d ob_box(
        (Eigen::Vector2d() << Obstacles_Box2d[i].center_x,
         Obstacles_Box2d[i].center_y)
            .finished(),
        Obstacles_Box2d[i].heading, Obstacles_Box2d[i].length,
        Obstacles_Box2d[i].width);
    auto allcorners = ob_box.GetAllCorners();

    //
    _gp << "set object " + std::to_string(i + 2) + " polygon from";
    for (int j = 0; j != 4; ++j) {
      _gp << " " << allcorners(0, j) << ", " << allcorners(1, j) << " to";
    }
    _gp << " " << allcorners(0, 0) << ", " << allcorners(1, 0) << "\n";
    _gp << "set object " + std::to_string(i + 2) +
               " fc rgb '#000000' fillstyle solid lw 0\n";
  }

  _gp << "plot ";
  // trajectory
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
           "pointsize 1 title 'obstacles_l', ";
  }
  _gp << "\n";

  // _gp.flush();

}  // rtplotting_bestpath

// generate map and endpoint
void generate_obstacle_map(
    std::vector<planning::Obstacle_Vertex> &Obstacles_Vertex,
    std::vector<planning::Obstacle_LineSegment> &Obstacles_LS,
    std::vector<planning::Obstacle_Box2d> &Obstacles_Box,
    std::array<float, 3> &end_point, int type) {
  // type:
  // 0: parking lot
  //

  switch (type) {
    case 0:
      // vertex
      Obstacles_Vertex.push_back({10, 4});
      // linesegment
      Obstacles_LS.push_back({
          10,  // start_x
          10,  // start_y
          10,  // end_x
          20   // end_y
      });
      Obstacles_LS.push_back({
          10,   // start_x
          20,   // start_y
          -10,  // end_x
          20    // end_y
      });
      Obstacles_LS.push_back({
          -10,  // start_x
          20,   // start_y
          -10,  // end_x
          10    // end_y
      });
      // box
      Obstacles_Box.push_back({
          10,  // center_x
          10,  // center_y
          4,   // length
          1,   // width
          0    // heading
      });

      // end point
      end_point = {20, 10, 0.0 * M_PI};
      break;
    case 1:
      // vertex
      Obstacles_Vertex.push_back({10, 4});
      // linesegment
      Obstacles_LS.push_back({
          1.5,  // start_x
          0,    // start_y
          1.5,  // end_x
          5     // end_y
      });
      Obstacles_LS.push_back({
          1.5,   // start_x
          5,     // start_y
          -1.5,  // end_x
          5      // end_y
      });
      Obstacles_LS.push_back({
          -1.5,  // start_x
          5,     // start_y
          -1.5,  // end_x
          0      // end_y
      });
      // box
      Obstacles_Box.push_back({
          10,  // center_x
          10,  // center_y
          4,   // length
          1,   // width
          0    // heading
      });

      // end point
      end_point = {0, 0.5, 0.5 * M_PI};
      break;
    case 2:
      // vertex
      Obstacles_Vertex.push_back({10, 4});
      // linesegment
      Obstacles_LS.push_back({
          0,   // start_x
          4,   // start_y
          24,  // end_x
          4    // end_y
      });
      Obstacles_LS.push_back({
          8,  // start_x
          4,  // start_y
          8,  // end_x
          -1  // end_y
      });
      Obstacles_LS.push_back({
          24,  // start_x
          8,   // start_y
          24,  // end_x
          2    // end_y
      });
      Obstacles_LS.push_back({
          24,  // start_x
          -8,  // start_y
          24,  // end_x
          -2   // end_y
      });
      Obstacles_LS.push_back({
          0,   // start_x
          -4,  // start_y
          20,  // end_x
          -4   // end_y
      });
      Obstacles_LS.push_back({
          16,  // start_x
          -4,  // start_y
          16,  // end_x
          1    // end_y
      });
      // box
      Obstacles_Box.push_back({
          10,  // center_x
          10,  // center_y
          4,   // length
          1,   // width
          0    // heading
      });

      // end point
      end_point = {23, 0.5, 0.0 * M_PI};
      break;
    default:
      break;
  };

}  // generate_obstacle_map

// Main
int main() {
  planning::HybridAStarConfig _HybridAStarConfig{
      1,    // move_length
      1.2,  // penalty_turning
      1.2,  // penalty_reverse
      1.3,  // penalty_switch
            // 5,    // num_interpolate
  };

  // obstacles
  std::vector<planning::Obstacle_Vertex> Obstacles_Vertex;
  std::vector<planning::Obstacle_LineSegment> Obstacles_LS;
  std::vector<planning::Obstacle_Box2d> Obstacles_Box;
  std::array<float, 3> end_point;
  generate_obstacle_map(Obstacles_Vertex, Obstacles_LS, Obstacles_Box,
                        end_point, 2);
  float end_x = end_point.at(0);
  float end_y = end_point.at(1);
  float end_theta = end_point.at(2);

  float start_x = 0;
  float start_y = 0;
  float start_theta = 0.0 * M_PI;
  planning::HybridAStar Hybrid_AStar(_collisiondata, _HybridAStarConfig);
  Hybrid_AStar.update_obstacles(Obstacles_Vertex, Obstacles_LS, Obstacles_Box)
      .setup_start_end(start_x, start_y, start_theta, end_x, end_y, end_theta);

  Hybrid_AStar.performsearch();

  auto hr = Hybrid_AStar.hybridastar_trajecotry();

  Gnuplot gp;
  gp << "set terminal x11 size 1100, 1100 0\n";
  gp << "set title 'A star search'\n";
  gp << "set xrange [-20:20]\n";
  gp << "set yrange [-20:20]\n";
  gp << "set size ratio -1\n";

  for (std::size_t i = 0; i != hr.size(); ++i) {
    rtplotting_bestpath(gp, hr[i], Obstacles_Vertex, Obstacles_LS,
                        Obstacles_Box);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
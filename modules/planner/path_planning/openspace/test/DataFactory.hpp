/*
*******************************************************************************
* DataFactory.hpp:
* generate data used for unit test of open-space planner
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/openspacedata.h"
#include "common/plotting/include/gnuplot-iostream.h"

namespace ASV::planning {

constexpr CollisionData _collisiondata{
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

void plot_vessel(Gnuplot &_gp, const double vessel_x, const double vessel_y,
                 const double vessel_theta, const int id) {
  static const double ego_length = _collisiondata.HULL_LENGTH;
  static const double ego_width = _collisiondata.HULL_WIDTH;
  static const double ego_center_local_x =
      0.5 * ego_length - _collisiondata.HULL_BACK2COG;
  static const double ego_center_local_y = 0.0;

  double cvalue = std::cos(vessel_theta);
  double svalue = std::sin(vessel_theta);
  double ego_center_x =
      vessel_x + cvalue * ego_center_local_x - svalue * ego_center_local_y;
  double ego_center_y =
      vessel_y + svalue * ego_center_local_x + cvalue * ego_center_local_y;

  ASV::common::math::Box2d ego_box(
      ASV::common::math::Vec2d(ego_center_x, ego_center_y), vessel_theta,
      ego_length, ego_width);

  auto allcorners = ego_box.GetAllCorners();

  //
  _gp << "set object " + std::to_string(id) + " polygon from";
  for (int j = 0; j != 4; ++j) {
    _gp << " " << allcorners[j].x() << ", " << allcorners[j].y() << " to";
  }
  _gp << " " << allcorners[0].x() << ", " << allcorners[0].y() << "\n";
  _gp << "set object " + std::to_string(id) +
             " fc rgb 'blue' fillstyle solid 0.1 noborder\n";
}  // plot_vessel

// illustrate the hybrid A* planner at a time instant
void rtplotting_4dbestpath(
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

}  // rtplotting_4dbestpath

// generate map and endpoint
void generate_obstacle_map(
    std::vector<Obstacle_Vertex_Config> &Obstacles_Vertex,
    std::vector<Obstacle_LineSegment_Config> &Obstacles_LS,
    std::vector<Obstacle_Box2d_Config> &Obstacles_Box,
    std::array<double, 3> &start_point, std::array<double, 3> &end_point,
    int type) {
  // type:
  // 0: parking lot
  //

  switch (type) {
    case -1:
      // start point
      start_point = {0, 0, 0.0 * M_PI};
      // end point
      end_point = {20, 0, 0.0 * M_PI};
      break;

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

      // start point
      start_point = {0, 0, 0.5 * M_PI};
      // end point
      end_point = {20, 0, 0.0 * M_PI};
      break;
    case 1:
      // vertex
      Obstacles_Vertex.push_back({30, 24});
      // linesegment
      Obstacles_LS.push_back({
          21.5,  // start_x
          20,    // start_y
          21.5,  // end_x
          25     // end_y
      });
      Obstacles_LS.push_back({
          21.5,  // start_x
          25,    // start_y
          18.5,  // end_x
          25     // end_y
      });
      Obstacles_LS.push_back({
          18.5,  // start_x
          25,    // start_y
          18.5,  // end_x
          20     // end_y
      });
      // box
      Obstacles_Box.push_back({
          30,  // center_x
          30,  // center_y
          4,   // length
          1,   // width
          0    // heading
      });

      // start point
      start_point = {0, 0, 0.0 * M_PI};
      // end point
      end_point = {20, 21.5, 0.5 * M_PI};
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
      // start point
      start_point = {0, 0, 0.0 * M_PI};

      // end point
      end_point = {26, 0.5, 0.0 * M_PI};
      break;
    case 3:
      // vertex
      // Obstacles_Vertex.push_back({10, 4});
      // linesegment
      Obstacles_LS.push_back({
          20,  // start_x
          25,  // start_y
          20,  // end_x
          40   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          25,  // start_y
          23,  // end_x
          25   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          40,  // start_y
          40,  // end_x
          35   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          55,  // start_y
          40,  // end_x
          50   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          55,  // start_y
          20,  // end_x
          65   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          65,  // start_y
          40,  // end_x
          60   // end_y
      });
      Obstacles_LS.push_back({
          40,  // start_x
          60,  // start_y
          40,  // end_x
          50   // end_y
      });
      Obstacles_LS.push_back({
          40,  // start_x
          55,  // start_y
          80,  // end_x
          55   // end_y
      });
      Obstacles_LS.push_back({
          100,  // start_x
          70,   // start_y
          90,   // end_x
          70    // end_y
      });
      Obstacles_LS.push_back({
          90,  // start_x
          30,  // start_y
          90,  // end_x
          70   // end_y
      });
      Obstacles_LS.push_back({
          38,  // start_x
          76,  // start_y
          95,  // end_x
          76   // end_y
      });
      Obstacles_LS.push_back({
          38,  // start_x
          76,  // start_y
          38,  // end_x
          70   // end_y
      });
      Obstacles_LS.push_back({
          38,  // start_x
          70,  // start_y
          20,  // end_x
          73   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          73,  // start_y
          20,  // end_x
          80   // end_y
      });
      Obstacles_LS.push_back({
          38,  // start_x
          76,  // start_y
          20,  // end_x
          80   // end_y
      });
      // box
      Obstacles_Box.push_back({
          4,          // center_x
          7.5,        // center_y
          15,         // length
          8,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          19,         // center_x
          7.5,        // center_y
          15,         // length
          8,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          60,         // center_x
          12.5,       // center_y
          55,         // length
          5,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          45.5,       // center_x
          2,          // center_y
          15,         // length
          4,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          5,          // center_x
          35,         // center_y
          10,         // length
          20,         // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          3.5,        // center_x
          55,         // center_y
          15,         // length
          7,          // width
          0.4 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          58,         // center_x
          30,         // center_y
          70,         // length
          10,         // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          83,         // center_x
          55,         // center_y
          7,          // length
          10,         // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          70,          // center_x
          57,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          65,          // center_x
          57,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          60,          // center_x
          57,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          56,          // center_x
          57,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          53,          // center_x
          57,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          50,          // center_x
          53,          // center_y
          4,           // length
          3,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          60,          // center_x
          53,          // center_y
          4,           // length
          3,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          44,          // center_x
          52,          // center_y
          6,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          67,          // center_x
          53,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          23,         // center_x
          42,         // center_y
          5,          // length
          3,          // width
          0.4 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          30,         // center_x
          40,         // center_y
          4,          // length
          2,          // width
          0.4 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          34,         // center_x
          39,         // center_y
          4,          // length
          2,          // width
          0.4 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          70,          // center_x
          37,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          60,          // center_x
          37,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          40,          // center_x
          37,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          50,          // center_x
          37,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          88,         // center_x
          38,         // center_y
          3,          // length
          6,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          48,         // center_x
          74,         // center_y
          20,         // length
          4,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          68,         // center_x
          74,         // center_y
          2,          // length
          4,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          78,         // center_x
          74,         // center_y
          2,          // length
          4,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          88,         // center_x
          74,         // center_y
          2,          // length
          4,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          10,         // center_x
          74,         // center_y
          4,          // length
          4,          // width
          0.3 * M_PI  // heading
      });
      // start point
      start_point = {5, 20, 0.0 * M_PI};

      // end point
      end_point = {76, 60, -0.5 * M_PI};
      break;

    case 4:
      // vertex
      Obstacles_Vertex.push_back({-10, 4});
      // linesegment
      Obstacles_LS.push_back({
          0,    // start_x
          3,    // start_y
          9.5,  // end_x
          3     // end_y
      });
      Obstacles_LS.push_back({
          9.5,  // start_x
          3,    // start_y
          9.5,  // end_x
          0     // end_y
      });
      Obstacles_LS.push_back({
          9.5,   // start_x
          0,     // start_y
          15.5,  // end_x
          0      // end_y
      });
      Obstacles_LS.push_back({
          15.5,  // start_x
          0,     // start_y
          15.5,  // end_x
          3      // end_y
      });
      Obstacles_LS.push_back({
          15.5,  // start_x
          3,     // start_y
          30,    // end_x
          3      // end_y
      });
      Obstacles_LS.push_back({
          0,   // start_x
          10,  // start_y
          30,  // end_x
          10   // end_y
      });
      // start point
      start_point = {4, 6, 0.3 * M_PI};
      // end point
      end_point = {11.5, 1.5, 0 * M_PI};
      break;

    case 5:
      // vertex
      Obstacles_Vertex.push_back({-10, 4});
      // linesegment
      Obstacles_LS.push_back({
          0,   // start_x
          4,   // start_y
          20,  // end_x
          4    // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          4,   // start_y
          20,  // end_x
          0    // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          0,   // start_y
          23,  // end_x
          0    // end_y
      });
      Obstacles_LS.push_back({
          23,  // start_x
          0,   // start_y
          23,  // end_x
          4    // end_y
      });
      Obstacles_LS.push_back({
          23,  // start_x
          4,   // start_y
          40,  // end_x
          4    // end_y
      });
      // start point
      start_point = {30, 7, 0.0 * M_PI};
      // end point
      end_point = {21.5, 1.5, 0.5 * M_PI};
      break;

    case 6:
      // vertex
      Obstacles_Vertex.push_back({-10, 4});
      // linesegment
      Obstacles_LS.push_back({
          0,   // start_x
          5,   // start_y
          20,  // end_x
          5    // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          5,   // start_y
          20,  // end_x
          0    // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          0,   // start_y
          23,  // end_x
          0    // end_y
      });
      Obstacles_LS.push_back({
          23,  // start_x
          0,   // start_y
          23,  // end_x
          5    // end_y
      });
      Obstacles_LS.push_back({
          23,  // start_x
          5,   // start_y
          40,  // end_x
          5    // end_y
      });
      // start point
      start_point = {30, 7, 0.0 * M_PI};
      // end point
      end_point = {21.5, 3.5, -0.5 * M_PI};
      break;

    case 7:
      // linesegment
      Obstacles_LS.push_back({
          33,  // start_x
          14,  // start_y
          33,  // end_x
          18   // end_y
      });
      Obstacles_LS.push_back({
          33,  // start_x
          18,  // start_y
          29,  // end_x
          18   // end_y
      });
      Obstacles_LS.push_back({
          29,  // start_x
          18,  // start_y
          29,  // end_x
          20   // end_y
      });
      Obstacles_LS.push_back({
          29,  // start_x
          20,  // start_y
          36,  // end_x
          20   // end_y
      });
      Obstacles_LS.push_back({
          18,  // start_x
          20,  // start_y
          21,  // end_x
          20   // end_y
      });
      Obstacles_LS.push_back({
          21,  // start_x
          12,  // start_y
          21,  // end_x
          20   // end_y
      });
      Obstacles_LS.push_back({
          21,  // start_x
          12,  // start_y
          18,  // end_x
          12   // end_y
      });
      Obstacles_Box.push_back({
          11,         // center_x
          2,          // center_y
          4,          // length
          5,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          20,         // center_x
          4,          // center_y
          6,          // length
          2,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          30,         // center_x
          2,          // center_y
          3,          // length
          2,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          31,         // center_x
          4.1,        // center_y
          3.5,        // length
          2,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          31,         // center_x
          6.0,        // center_y
          3.0,        // length
          1.1,        // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          31,         // center_x
          10.5,       // center_y
          3.2,        // length
          2,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          31,         // center_x
          13,         // center_y
          4,          // length
          2,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          31,         // center_x
          30,         // center_y
          6,          // length
          6,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          17,         // center_x
          18,         // center_y
          4,          // length
          2,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          13,         // center_x
          18,         // center_y
          3.7,        // length
          2,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          13,         // center_x
          13,         // center_y
          3.6,        // length
          2,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          10.5,       // center_x
          13,         // center_y
          3,          // length
          2,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          7,          // center_x
          18,         // center_y
          3.7,        // length
          2,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          3.5,        // center_x
          13,         // center_y
          3.6,        // length
          2,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          3.5,        // center_x
          13,         // center_y
          3,          // length
          2,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          2,          // center_x
          18,         // center_y
          3,          // length
          2,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          13,         // center_x
          30,         // center_y
          3,          // length
          2,          // width
          0.5 * M_PI  // heading
      });
      // start point
      end_point = {29, 8, 0.0 * M_PI};
      // end_point = {23.5, 10, -0.5 * M_PI};

      // end point
      start_point = {4.5, 20, -0.5 * M_PI};

      break;

    case 8:
      // linesegment
      Obstacles_LS.push_back({
          9,   // start_x
          5,   // start_y
          22,  // end_x
          5    // end_y
      });
      Obstacles_LS.push_back({
          9,  // start_x
          5,  // start_y
          6,  // end_x
          3   // end_y
      });
      Obstacles_LS.push_back({
          22,  // start_x
          5,   // start_y
          23,  // end_x
          3    // end_y
      });
      Obstacles_LS.push_back({
          10,   // start_x
          7.5,  // start_y
          20,   // end_x
          7.5   // end_y
      });
      Obstacles_LS.push_back({
          10,   // start_x
          7.5,  // start_y
          6.5,  // end_x
          9     // end_y
      });
      Obstacles_LS.push_back({
          20,   // start_x
          7.5,  // start_y
          40,   // end_x
          20    // end_y
      });
      Obstacles_LS.push_back({
          40,  // start_x
          20,  // start_y
          40,  // end_x
          3    // end_y
      });
      Obstacles_LS.push_back({
          40,  // start_x
          3,   // start_y
          23,  // end_x
          3    // end_y
      });
      Obstacles_Box.push_back({
          8,          // center_x
          14,         // center_y
          4,          // length
          5,          // width
          0.5 * M_PI  // heading
      });

      // start point
      end_point = {29.2, 7, 0.0 * M_PI};
      // end_point = {23.5, 10, -0.5 * M_PI};

      // end point
      start_point = {1, 10, -0.5 * M_PI};

      break;

    case 9:

      Obstacles_Box.push_back({
          10,         // center_x
          10,         // center_y
          4,          // length
          4,          // width
          0.5 * M_PI  // heading
      });
      // end point
      end_point = {10, 20, 0.5 * M_PI};

      // start point
      start_point = {10, -4, 0.5 * M_PI};

      break;

    case 10:

      Obstacles_Box.push_back({
          8,          // center_x
          3,          // center_y
          16,         // length
          6,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          19,         // center_x
          1,          // center_y
          6,          // length
          2,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          24,         // center_x
          3,          // center_y
          4,          // length
          6,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          2,          // center_x
          22,         // center_y
          4,          // length
          6,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          7,          // center_x
          24,         // center_y
          6,          // length
          2,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          18,         // center_x
          22,         // center_y
          16,         // length
          6,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          13,         // center_x
          13,         // center_y
          2,          // length
          6,          // width
          0.0 * M_PI  // heading
      });
      // start point
      start_point = {6, 20, 0.0 * M_PI};

      // end point
      end_point = {18, 5, 0.0 * M_PI};

      break;
    case 11:

      // linesegment
      Obstacles_LS.push_back({
          21.5,  // start_x
          20,    // start_y
          21.5,  // end_x
          25     // end_y
      });
      Obstacles_LS.push_back({
          21.5,  // start_x
          25,    // start_y
          18.5,  // end_x
          25     // end_y
      });
      Obstacles_LS.push_back({
          18.5,  // start_x
          25,    // start_y
          18.5,  // end_x
          20     // end_y
      });
      // box
      Obstacles_Box.push_back({
          30,  // center_x
          30,  // center_y
          4,   // length
          1,   // width
          0    // heading
      });

      // start point
      start_point = {0, 0, 0.0 * M_PI};
      // end point
      end_point = {0.5, 0, 0.0 * M_PI};
      break;

    default:
      break;
  };

}  // generate_obstacle_map

}  // namespace ASV::planning

/*
*******************************************************************************
* Smoothing_test.cc:
* unit test for path smoothing
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/PathSmoothing.h"
#include "DataFactory.hpp"

using namespace ASV::planning;

int main() {
  int test_scenario = 4;

  // obstacles
  std::vector<Obstacle_Vertex_Config> Obstacles_Vertex;
  std::vector<Obstacle_LineSegment_Config> Obstacles_LS;
  std::vector<Obstacle_Box2d_Config> Obstacles_Box;
  std::array<float, 3> start_point;
  std::array<float, 3> end_point;
  generate_obstacle_map(Obstacles_Vertex, Obstacles_LS, Obstacles_Box,
                        start_point, end_point, test_scenario);

  CollisionChecking_Astar collision_checker_(_collisiondata);
  collision_checker_.set_all_obstacls(Obstacles_Vertex, Obstacles_LS,
                                      Obstacles_Box);

  // Path Smoothing
  SmootherConfig smoothconfig{
      10,  // d_max
  };

  PathSmoothing pathsmoother(smoothconfig);

  // std::vector<std::tuple<double, double, double, bool>> coarse_path = {
  //     {30, 7, 0, true},
  //     {30.9851, 7.14888, 0.3, true},
  //     {31.8821, 7.58221, 0.6, true},
  //     {32.6111, 8.2613, 0.9, true},
  //     {33.1068, 9.12547, 1.2, true},
  //     {33.325, 10.0975, 1.5, true},
  //     {33.2462, 11.0907, 1.8, true},
  //     {32.8774, 12.0162, 2.1, true},
  //     {32.2515, 12.7913, 2.4, true},
  //     {31.4246, 13.3469, 2.7, true},
  //     {30.4704, 13.6333, 3.0, true},
  //     {29.4742, 13.6249, -2.98319, true},
  //     {28.5249, 13.3225, -2.68319, true},
  //     {27.7074, 12.7531, -2.38319, true},
  //     {28.5249, 11.9675, -2.08319, true},
  //     {25.6646, 10.6125, -2.68319, true},
  //     {24.7153, 10.3101, -2.98319, true},
  //     {24.7153, 10.3101, -2.98319, true},
  //     {24.2293, 10.1946, -2.83319, true},
  //     {23.7661, 10.0077, -2.68319, true},
  //     {23.3359, 9.75374, -2.53319, true},
  //     {22.9486, 9.43832, -2.38319, true},
  //     {22.6127, 9.06855, -2.23319, true},
  //     {22.3359, 8.65274, -2.08319, true},
  //     {22.1243, 8.20024, -1.93319, true},
  //     {21.9827, 7.72119, -1.78319, true},
  //     {21.9096, 7.22683, -1.68618, true},
  //     {21.8521, 6.73015, -1.68618, true},
  //     {21.7945, 6.23348, -1.68618, true},
  //     {21.7369, 5.7368, -1.68618, true},
  //     {21.6794, 5.24013, -1.68618, true},
  //     {21.6218, 4.74375, -1.68618, true},
  //     {21.5642, 4.24678, -1.68618, true},
  //     {21.5094, 3.74982, -1.68618, true},
  //     {21.5, 3.5, -1.64581, true},
  // };

  // std::vector<std::tuple<double, double, double, bool>> coarse_path = {
  //     {30, 7, 0, true},
  //     {29, 7, 8.7e-8, true},
  //     {29, 7, 8.7e-8, false},
  //     {28, 7, 8.7e-8, false},
  //     {27, 7, 8.7e-8, false},
  //     {26, 7, 8.7e-8, false},
  //     {25, 7, 8.7e-8, false},
  //     {24.5019, 6.96257, 0.15, false},
  //     {24.0149, 6.85112, 0.3, false},
  //     {23.5501, 6.66816, 0.45, false},
  //     {23.1179, 6.41779, 0.60, false},
  //     {23.7279, 6.10563, 0.75, false},
  //     {22.3889, 5.7387, 0.90, false},
  //     {22.1086, 5.32524, 1.05, false},
  //     {21.8932, 4.87453, 1.2, false},
  //     {21.7476, 4.39669, 1.35, false},
  //     {21.675, 3.90246, 1.49402, false},
  //     {21.6366, 3.40393, 1.49402, false},
  //     {21.5983, 2.90541, 1.49402, false},
  //     {21.5599, 2.40688, 1.49402, false},
  //     {21.5216, 1.90835, 1.49402, false},
  //     {21.5, 1.5, 1.5708, false},

  // };

  std::vector<std::tuple<double, double, double, bool>> coarse_path = {
      {4, 6, 0.942478, true},
      {4.69945, 6.70943, 0.642478, true},
      {5.57732, 7.18047, 0.342378, true},
      {6.55517, 7.37104, 0.042478, true},
      {7.54567, 7.26413, -0.257522, true},
      {8.46024, 6.86928, -0.557522, true},
      {9.21747, 6.22176, -0.857522, true},
      {9.9746, 5.57424, -0.557522, true},
      {10.8893, 5.17939, -0.257522, true},
      {11.7902, 4.79047, -0.557522, true},
      {12.7049, 4.39562, -0.257522, true},
      {13.6954, 4.28871, 0.0424779, true},
      {14.6732, 4.47928, 0.342478, true},
      {14.6732, 4.47928, 0.342478, false},
      {14.2166, 4.27675, 0.492478, false},
      {13.7954, 4.00824, 0.642478, false},
      {13.419, 3.67981, 0.792478, false},
      {13.0959, 3.29882, 0.942478, false},
      {12.8278, 2.87705, 1.00435, false},
      {12.5289, 2.47681, 0.854345, false},
      {12.1735, 2.12574, 0.704345, false},
      {11.7697, 1.8317, 0.554345, false},
      {11.3265, 1.60132, 0.404345, false},
      {10.8538, 1.43976, 0.254345, false},
      {10.7575, 1.41626, 0.224623, false},
      {10.7575, 1.41626, 0.224623, true},
      {11.2515, 1.49072, 0.074623, true},
      {11.5, 1.5, 0.0, true},

  };
  auto p_coarse = pathsmoother.SetupCoarsePath(coarse_path).coarse_path();
  pathsmoother.PerformSmoothing(collision_checker_);

  // plotting
  Gnuplot gp;
  gp << "set terminal x11 size 1100, 1100 0\n";
  gp << "set title 'Path Smoothing'\n";
  gp << "set xrange [0:50]\n";
  gp << "set yrange [0:50]\n";
  gp << "set size ratio -1\n";

  for (const auto &value : p_coarse) {
    std::cout << std::get<0>(value).x() << ", " << std::get<0>(value).y()
              << ", " << std::get<1>(value) << std::endl;
  }
}
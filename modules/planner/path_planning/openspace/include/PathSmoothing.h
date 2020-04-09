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

 public:
  PathSmoothing() = default;
  ~PathSmoothing() = default;

 private:
  const double dmax;

  std::vector<vec2d> GenerateObstacleTerm(
      const CollisionChecking_Astar &collision_checker,
      const std::vector<vec2d> &path) {
    std::size_t total_num = path.size();

    std::vector<vec2d> obstacleTerm(total_num, vec2d(0, 0));
    auto nearest_obstacles = collision_checker.FindNearstObstacle(path);
    for (std::size_t index = 0; index != total_num; ++index) {
      auto x2o = path[index] - nearest_obstacles[index];

      if (x2o.IsSmall()) obstacleTerm[index] = x2o * (-10000);
      ;
      double k = 2 * (1 - dmax / x2o.Length());
    }

  }  // GenerateObstacleTerm
};   // end class PathSmoothing
}  // namespace ASV::planning

#endif /* _PATHSMOOTHING_H_ */
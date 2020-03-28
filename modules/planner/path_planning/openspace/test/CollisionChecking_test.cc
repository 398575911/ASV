/*
*******************************************************************************
* CollisionChecking_test.cc:
* unit test for collision checking
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/CollisionChecking.h"

using namespace ASV;

int main() {
  planning::CollisionData _collisiondata{
      4,     // MAX_SPEED
      4.0,   // MAX_ACCEL
      -3.0,  // MIN_ACCEL
      2.0,   // MAX_ANG_ACCEL
      -2.0,  // MIN_ANG_ACCEL
      0.2,   // MAX_CURVATURE
      3,     // HULL_LENGTH
      1,     // HULL_WIDTH
      1.5,   // HULL_BACK2COG
      3.3    // ROBOT_RADIUS
  };
  std::vector<planning::obstacle_Vertex> Obstacles_Vertex{
      {},
  };

  OpenSpace_Trajectory _OpenSpace_Trajectory{};

  planning::CollisionChecking _CollisionChecking(_collisiondata);
  _CollisionChecking.set_Obstacles_Vertex();
}

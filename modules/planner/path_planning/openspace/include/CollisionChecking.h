/*
***********************************************************************
* CollisionChecking.h:
* Collision detection, using a 2dbox-shaped vessel
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _CONSTRAINTCHECKING_H_
#define _CONSTRAINTCHECKING_H_

#include "openspacedata.h"

namespace ASV::planning {

class CollisionChecking {
 public:
  explicit CollisionChecking();

  virtual ~CollisionChecking() = default;

  std::vector<Obstacle_Vertex> Obstacles_Vertex() const {
    return Obstacles_Vertex_;
  }
  std::vector<Obstacle_LineSegment> Obstacles_LineSegment_() const {
    return Obstacles_LineSegment_;
  }
  std::vector<Obstacle_Box2d> Obstacles_Box2d_() const {
    return Obstacles_Box2d_;
  }

 private:
  std::vector<Obstacle_Vertex> Obstacles_Vertex_;
  std::vector<Obstacle_LineSegment> Obstacles_LineSegment_;
  std::vector<Obstacle_Box2d> Obstacles_Box2d_;

};  // end class CollisionChecking

}  // namespace ASV::planning

#endif /* _CONSTRAINTCHECKING_H_ */
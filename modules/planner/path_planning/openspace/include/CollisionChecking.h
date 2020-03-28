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

#include "common/math/Geometry/include/box2d.h"
#include "openspacedata.h"

namespace ASV::planning {

class CollisionChecking {
 public:
  explicit CollisionChecking(const CollisionData &_CollisionData)
      : CollisionData_(_CollisionData) {}

  virtual ~CollisionChecking() = default;

  // check collision, return true if collision occurs.
  bool InCollision(const OpenSpace_Trajectory &ego_trajectory) {
    const double ego_center_local_x =
        0.5 * CollisionData_.HULL_LENGTH - CollisionData_.HULL_BACK2COG;
    const double ego_center_local_y = 0.0;

    for (int i = 0; i != ego_trajectory.t.size(); ++i) {
      double ego_theta = ego_trajectory.theta(i);

      // correct CoG trajectory to center
      auto [ego_center_global_x, ego_center_global_y] =
          local2global(ego_center_local_x, ego_center_local_y, ego_theta);
      ego_center_global_x += ego_trajectory.x(i);
      ego_center_global_y += ego_trajectory.y(i);

      // genereat a 2dbox for ego vessel
      ASV::common::math::Box2d ego_box(
          (Eigen::Vector2d() << ego_center_global_x, ego_center_global_y)
              .finished(),
          ego_theta, CollisionData_.HULL_LENGTH, CollisionData_.HULL_WIDTH);

      // check vertex
      for (auto const &vertex : Obstacles_Vertex_) {
        if (ego_box.IsPointIn(
                (Eigen::Vector2d() << vertex.x, vertex.y).finished()))
          return true;
      }
      // check legement
      for (auto const &legment_para : Obstacles_LineSegment_) {
        if (ego_box.HasOverlap(ASV::common::math::LineSegment2d(
                (Eigen::Vector2d() << legment_para.start_x,
                 legment_para.start_y)
                    .finished(),
                (Eigen::Vector2d() << legment_para.end_x, legment_para.end_y)
                    .finished())))
          return true;
      }
      // check box
      for (auto const &box_para : Obstacles_Box2d_) {
        if (ego_box.HasOverlap(ASV::common::math::Box2d(
                (Eigen::Vector2d() << box_para.center_x, box_para.center_y)
                    .finished(),
                box_para.heading, box_para.length, box_para.width)))
          return true;
      }
    }
    return false;
  }  // InCollision

  std::vector<Obstacle_Vertex> Obstacles_Vertex() const {
    return Obstacles_Vertex_;
  }
  std::vector<Obstacle_LineSegment> Obstacles_LineSegment() const {
    return Obstacles_LineSegment_;
  }
  std::vector<Obstacle_Box2d> Obstacles_Box2d() const {
    return Obstacles_Box2d_;
  }

  CollisionChecking &set_Obstacles_Vertex(
      const std::vector<Obstacle_Vertex> &Obstacles_Vertex) {
    Obstacles_Vertex_ = Obstacles_Vertex;
    return *this;
  }

  CollisionChecking &set_Obstacles_LineSegment(
      const std::vector<Obstacle_LineSegment> &Obstacles_LineSegment) {
    Obstacles_LineSegment_ = Obstacles_LineSegment;
    return *this;
  }

  CollisionChecking &set_Obstacles_Box2d(
      const std::vector<Obstacle_Box2d> &Obstacles_Box2d) {
    Obstacles_Box2d_ = Obstacles_Box2d;
    return *this;
  }

 private:
  const CollisionData CollisionData_;
  std::vector<Obstacle_Vertex> Obstacles_Vertex_;
  std::vector<Obstacle_LineSegment> Obstacles_LineSegment_;
  std::vector<Obstacle_Box2d> Obstacles_Box2d_;

  std::tuple<double, double> local2global(const double local_x,
                                          const double local_y,
                                          const double theta) {
    double cvalue = std::cos(theta);
    double svalue = std::sin(theta);
    return {cvalue * local_x - svalue * local_y,
            svalue * local_x + cvalue * local_y};
  }  // local2global

  std::tuple<double, double> global2local(const double global_x,
                                          const double global_y,
                                          const double theta) {
    double cvalue = std::cos(theta);
    double svalue = std::sin(theta);
    return {cvalue * global_x + svalue * global_y,
            cvalue * global_y - svalue * global_x};
  }  // global2local

};  // end class CollisionChecking

}  // namespace ASV::planning

#endif /* _CONSTRAINTCHECKING_H_ */
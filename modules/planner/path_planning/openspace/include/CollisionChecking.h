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
      : ego_length_(_CollisionData.HULL_LENGTH),
        ego_width_(_CollisionData.HULL_WIDTH),
        ego_back2cog_(_CollisionData.HULL_BACK2COG),
        ego_center_local_x_(0.5 * ego_length_ - ego_back2cog_),
        ego_center_local_y_(0.0) {}

  virtual ~CollisionChecking() = default;

  // check collision, return true if collision occurs.
  bool InCollision(
      const std::vector<std::array<double, 3>> &ego_trajectory) const {
    for (auto const &state : ego_trajectory) {
      if (InCollision(state.at(0), state.at(1), state.at(2))) return true;
    }
    return false;

  }  // InCollision

  // check collision, return true if collision occurs.
  bool InCollision(const double ego_x, const double ego_y,
                   const double ego_theta) const {
    // correct CoG trajectory to center
    auto [ego_center_global_x, ego_center_global_y] = local2global(
        this->ego_center_local_x_, this->ego_center_local_y_, ego_theta);
    ego_center_global_x += ego_x;
    ego_center_global_y += ego_y;

    // genereat a 2dbox for ego vessel
    ASV::common::math::Box2d ego_box(
        (Eigen::Vector2d() << ego_center_global_x, ego_center_global_y)
            .finished(),
        ego_theta, this->ego_length_, this->ego_width_);

    // check vertex
    for (auto const &vertex : Obstacles_Vertex_) {
      if (ego_box.IsPointIn(
              (Eigen::Vector2d() << vertex.x, vertex.y).finished())) {
        return true;
      }
    }
    // check legement
    for (auto const &legment_para : Obstacles_LineSegment_) {
      if (ego_box.HasOverlap(ASV::common::math::LineSegment2d(
              (Eigen::Vector2d() << legment_para.start_x, legment_para.start_y)
                  .finished(),
              (Eigen::Vector2d() << legment_para.end_x, legment_para.end_y)
                  .finished()))) {
        return true;
      }
    }
    // check box
    for (auto const &box_para : Obstacles_Box2d_) {
      if (ego_box.HasOverlap(ASV::common::math::Box2d(
              (Eigen::Vector2d() << box_para.center_x, box_para.center_y)
                  .finished(),
              box_para.heading, box_para.length, box_para.width))) {
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
  }  // set_Obstacles_Vertex

  CollisionChecking &set_Obstacles_LineSegment(
      const std::vector<Obstacle_LineSegment> &Obstacles_LineSegment) {
    Obstacles_LineSegment_ = Obstacles_LineSegment;
    return *this;
  }  // set_Obstacles_LineSegment

  CollisionChecking &set_Obstacles_Box2d(
      const std::vector<Obstacle_Box2d> &Obstacles_Box2d) {
    Obstacles_Box2d_ = Obstacles_Box2d;
    return *this;
  }  // set_Obstacles_Box2d

 private:
  const double ego_length_;
  const double ego_width_;
  const double ego_back2cog_;
  const double ego_center_local_x_;
  const double ego_center_local_y_;

  std::vector<Obstacle_Vertex> Obstacles_Vertex_;
  std::vector<Obstacle_LineSegment> Obstacles_LineSegment_;
  std::vector<Obstacle_Box2d> Obstacles_Box2d_;

  std::tuple<double, double> local2global(const double local_x,
                                          const double local_y,
                                          const double theta) const {
    double cvalue = std::cos(theta);
    double svalue = std::sin(theta);
    return {cvalue * local_x - svalue * local_y,
            svalue * local_x + cvalue * local_y};
  }  // local2global

  std::tuple<double, double> global2local(const double global_x,
                                          const double global_y,
                                          const double theta) const {
    double cvalue = std::cos(theta);
    double svalue = std::sin(theta);
    return {cvalue * global_x + svalue * global_y,
            cvalue * global_y - svalue * global_x};
  }  // global2local

};  // end class CollisionChecking

}  // namespace ASV::planning

#endif /* _CONSTRAINTCHECKING_H_ */
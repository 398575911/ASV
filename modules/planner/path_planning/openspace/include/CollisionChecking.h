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

template <int max_vertex = 50, int max_ls = 50, int max_box = 20>
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
  // TODO: multi-thread to finish this
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
        ASV::common::math::Vec2d(ego_center_global_x, ego_center_global_y),
        ego_theta, this->ego_length_, this->ego_width_);

    // check vertex
    for (int i = 0; i != max_vertex; ++i) {
      if (Obstacles_Vertex_.status[i] &&
          ego_box.IsPointIn(Obstacles_Vertex_.vertex[i])) {
        return true;
      }
    }

    // check legement
    for (int i = 0; i != max_ls; ++i) {
      if (Obstacles_LineSegment_.status[i] &&
          ego_box.HasOverlap(Obstacles_LineSegment_.linesegment[i])) {
        return true;
      }
    }

    // check box
    for (int i = 0; i != max_box; ++i) {
      if (Obstacles_Box2d_.status[i] &&
          ego_box.HasOverlap(Obstacles_Box2d_.box2d[i])) {
        return true;
      }
    }

    return false;
  }  // InCollision

  auto Obstacles_Vertex() const noexcept { return Obstacles_Vertex_; }
  auto Obstacles_LineSegment() const noexcept { return Obstacles_LineSegment_; }
  auto Obstacles_Box2d() const noexcept { return Obstacles_Box2d_; }

  CollisionChecking &set_Obstacles_Vertex(
      const std::vector<Obstacle_Vertex_Config> &Obstacles_Vertex) {
    int num_vertex = Obstacles_Vertex.size();
    if (num_vertex > max_vertex) {
      // TODO:: warning
      for (int i = 0; i != max_vertex; ++i) {
        Obstacles_Vertex_.status[i] = true;
        Obstacles_Vertex_.vertex[i] = ASV::common::math::Vec2d(
            Obstacles_Vertex[i].x, Obstacles_Vertex[i].y);
      }

    } else {
      std::fill(Obstacles_Vertex_.status.begin(),
                Obstacles_Vertex_.status.end(), false);
      for (int i = 0; i != num_vertex; ++i) {
        Obstacles_Vertex_.status[i] = true;
        Obstacles_Vertex_.vertex[i] = ASV::common::math::Vec2d(
            Obstacles_Vertex[i].x, Obstacles_Vertex[i].y);
      }
    }
    return *this;
  }  // set_Obstacles_Vertex

  CollisionChecking &set_Obstacles_LineSegment(
      const std::vector<Obstacle_LineSegment_Config> &Obstacles_LineSegment) {
    int num_ls = Obstacles_LineSegment.size();
    if (num_ls > max_ls) {
      // TODO:: warning
      for (int i = 0; i != max_ls; ++i) {
        Obstacles_LineSegment_.status[i] = true;
        Obstacles_LineSegment_.linesegment[i] =
            ASV::common::math::LineSegment2d(
                ASV::common::math::Vec2d(Obstacles_LineSegment[i].start_x,
                                         Obstacles_LineSegment[i].start_y),
                ASV::common::math::Vec2d(Obstacles_LineSegment[i].end_x,
                                         Obstacles_LineSegment[i].end_y));
      }

    } else {
      std::fill(Obstacles_LineSegment_.status.begin(),
                Obstacles_LineSegment_.status.end(), false);
      for (int i = 0; i != num_ls; ++i) {
        Obstacles_LineSegment_.status[i] = true;
        Obstacles_LineSegment_.linesegment[i] =
            ASV::common::math::LineSegment2d(
                ASV::common::math::Vec2d(Obstacles_LineSegment[i].start_x,
                                         Obstacles_LineSegment[i].start_y),
                ASV::common::math::Vec2d(Obstacles_LineSegment[i].end_x,
                                         Obstacles_LineSegment[i].end_y));
      }
    }

    return *this;
  }  // set_Obstacles_LineSegment

  CollisionChecking &set_Obstacles_Box2d(
      const std::vector<Obstacle_Box2d_Config> &Obstacles_Box2d) {
    int num_box = Obstacles_Box2d.size();
    if (num_box > max_box) {
      // TODO:: warning
      for (int i = 0; i != max_box; ++i) {
        auto box_para = Obstacles_Box2d[i];
        Obstacles_Box2d_.status[i] = true;
        Obstacles_Box2d_.box2d[i] = ASV::common::math::Box2d(
            ASV::common::math::Vec2d(box_para.center_x, box_para.center_y),
            box_para.heading, box_para.length, box_para.width);
      }

    } else {
      std::fill(Obstacles_Box2d_.status.begin(), Obstacles_Box2d_.status.end(),
                false);
      for (int i = 0; i != num_box; ++i) {
        auto box_para = Obstacles_Box2d[i];
        Obstacles_Box2d_.status[i] = true;
        Obstacles_Box2d_.box2d[i] = ASV::common::math::Box2d(
            ASV::common::math::Vec2d(box_para.center_x, box_para.center_y),
            box_para.heading, box_para.length, box_para.width);
      }
    }

    return *this;
  }  // set_Obstacles_Box2d

 private:
  const double ego_length_;
  const double ego_width_;
  const double ego_back2cog_;
  const double ego_center_local_x_;
  const double ego_center_local_y_;

  Obstacle_Vertex<max_vertex> Obstacles_Vertex_;
  Obstacle_LineSegment<max_ls> Obstacles_LineSegment_;
  Obstacle_Box2d<max_box> Obstacles_Box2d_;

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
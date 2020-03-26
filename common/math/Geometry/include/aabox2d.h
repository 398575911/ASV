/*
***********************************************************************
* aabox2d.h: (undirected) axes-aligned bounding boxes in 2-D.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _AABOX2D_H_
#define _AABOX2D_H_

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include "common/math/miscellaneous/include/math_utils.h"

namespace ASV::common::math {

class AABox2d {
 public:
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box with given center, length, and width.
   * @param center The center of the box
   * @param length The size of the box along the x-axis
   * @param width The size of the box along the y-axis
   */
  explicit AABox2d(const Eigen::Vector2d &center, const double length,
                   const double width)
      : center_(center),
        length_(length),
        width_(width),
        half_length_(0.5 * length),
        half_width_(0.5 * width) {}
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box from two opposite corners.
   * @param one_corner One corner of the box
   * @param opposite_corner The opposite corner to the first one
   */
  explicit AABox2d(const Eigen::Vector2d &one_corner,
                   const Eigen::Vector2d &opposite_corner) {}
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box containing all points in a given vector.
   * @param points Vector of points to be included inside the box.
   */
  explicit AABox2d(const Eigen::VectorXd &points_x,
                   const Eigen::VectorXd &points_x);

  /**
   * @brief Getter of center_
   * @return Center of the box
   */
  Eigen::Vector2d center() const { return center_; }

  /**
   * @brief Getter of x-component of center_
   * @return x-component of the center of the box
   */
  double center_x() const { return center_.x(); }

  /**
   * @brief Getter of y-component of center_
   * @return y-component of the center of the box
   */
  double center_y() const { return center_.y(); }

  /**
   * @brief Getter of length_
   * @return The length of the box
   */
  double length() const { return length_; }

  /**
   * @brief Getter of width_
   * @return The width of the box
   */
  double width() const { return width_; }

  /**
   * @brief Getter of half_length_
   * @return Half of the length of the box
   */
  double half_length() const { return half_length_; }

  /**
   * @brief Getter of half_width_
   * @return Half of the width of the box
   */
  double half_width() const { return half_width_; }

  /**
   * @brief Getter of length_*width_
   * @return The area of the box
   */
  double area() const { return length_ * width_; }

  /**
   * @brief Returns the minimum x-coordinate of the box
   *
   * @return x-coordinate
   */
  double min_x() const { return center_.x() - half_length_; }

  /**
   * @brief Returns the maximum x-coordinate of the box
   *
   * @return x-coordinate
   */
  double max_x() const { return center_.x() + half_length_; }

  /**
   * @brief Returns the minimum y-coordinate of the box
   *
   * @return y-coordinate
   */
  double min_y() const { return center_.y() - half_width_; }

  /**
   * @brief Returns the maximum y-coordinate of the box
   *
   * @return y-coordinate
   */
  double max_y() const { return center_.y() + half_width_; }

  /**
   * @brief Gets all corners in counter clockwise order.
   *
   * @param corners Output where the corners are written
   */
  void GetAllCorners(std::vector<Vec2d> *const corners) const;

  /**
   * @brief Determines whether a given point is in the box.
   *
   * @param point The point we wish to test for containment in the box
   */
  bool IsPointIn(const Eigen::Vector2d &point) const;

  /**
   * @brief Determines whether a given point is on the boundary of the box.
   *
   * @param point The point we wish to test for boundary membership
   */
  bool IsPointOnBoundary(const Eigen::Vector2d &point) const;

  /**
   * @brief Determines the distance between a point and the box.
   *
   * @param point The point whose distance to the box we wish to determine.
   */
  double DistanceTo(const Eigen::Vector2d &point) const;

  /**
   * @brief Determines the distance between two boxes.
   *
   * @param box Another box.
   */
  double DistanceTo(const AABox2d &box) const;

  /**
   * @brief Determines whether two boxes overlap.
   *
   * @param box Another box
   */
  bool HasOverlap(const AABox2d &box) const;

  /**
   * @brief Shift the center of AABox by the input vector.
   *
   * @param shift_vec The vector by which we wish to shift the box
   */
  void Shift(const Eigen::Vector2d &shift_vec);

  /**
   * @brief Changes box to include another given box, as well as the current
   * one.
   *
   * @param other_box Another box
   */
  void MergeFrom(const AABox2d &other_box);

  /**
   * @brief Changes box to include a given point, as well as the current box.
   *
   * @param other_point Another point
   */
  void MergeFrom(const Eigen::Vector2d &other_point);

 private:
  Eigen::Vector2d center_;
  double length_;
  double width_;
  double half_length_;
  double half_width_;
};  // end class AABox2d

}  // namespace ASV::common::math

#endif /* _AABOX2D_H_ */
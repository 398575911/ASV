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

#include <iostream>

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
                   const Eigen::Vector2d &opposite_corner)
      : AABox2d(0.5 * (one_corner + opposite_corner),
                std::abs(one_corner(0) - opposite_corner(0)),
                std::abs(one_corner(1) - opposite_corner(1))) {}
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box containing all points in a given vector.
   * @param points Vector of points to be included inside the box.
   */
  explicit AABox2d(const Eigen::VectorXd &points_x,
                   const Eigen::VectorXd &points_y)
      : center_(Eigen::Vector2d::Zero()),
        length_(0),
        width_(0),
        half_length_(0),
        half_width_(0) {
    double min_x = points_x.minCoeff();
    double max_x = points_x.maxCoeff();
    double min_y = points_y.minCoeff();
    double max_y = points_y.maxCoeff();
    center_ << 0.5 * (min_x + max_x), 0.5 * (min_y + max_y);
    length_ = max_x - min_x;
    width_ = max_y - min_y;
    half_length_ = length_ / 2.0;
    half_width_ = width_ / 2.0;
  }

  virtual ~AABox2d() = default;

  Eigen::Vector2d center() const { return center_; }

  // Getter of x-component of center_
  double center_x() const noexcept { return center_(0); }

  // Getter of y-component of center_
  double center_y() const noexcept { return center_(1); }

  double length() const noexcept { return length_; }
  double width() const noexcept { return width_; }

  // Getter of half_length_
  double half_length() const noexcept { return half_length_; }

  // Getter of half_width_
  double half_width() const noexcept { return half_width_; }

  // The area of the box
  double area() const noexcept { return length_ * width_; }

  // Returns the minimum x-coordinate of the box
  double min_x() const noexcept { return center_x() - half_length_; }

  // Returns the maximum x-coordinate of the box
  double max_x() const noexcept { return center_x() + half_length_; }

  // Returns the minimum y-coordinate of the box
  double min_y() const noexcept { return center_y() - half_width_; }

  // Returns the maximum y-coordinate of the box
  double max_y() const { return center_y() + half_width_; }

  // Gets all corners in counter clockwise order.
  Eigen::Matrix<double, 4, 2> GetAllCorners() const {
    Eigen::Matrix<double, 4, 2> allcorners =
        Eigen::Matrix<double, 4, 2>::Zero();

    allcorners << max_x(), min_y(),  // first corner
        max_x(), max_y(),            // second corner
        min_x(), max_y(),            // third corner
        min_x(), min_y()             // fourth corner
        ;
    return allcorners;
  }  // GetAllCorners

  // Determines whether a given point is in the box.
  bool IsPointIn(const Eigen::Vector2d &point) const {
    return std::abs(point(0) - center_(0)) <= half_length_ + kMathEpsilon &&
           std::abs(point(1) - center_(1)) <= half_width_ + kMathEpsilon;
  }  // IsPointIn

  // Determines whether a given point is on the boundary of the box.
  bool IsPointOnBoundary(const Eigen::Vector2d &point) const {
    const double dx = std::abs(point(0) - center_(0));
    const double dy = std::abs(point(1) - center_(1));
    return (std::abs(dx - half_length_) <= kMathEpsilon &&
            dy <= half_width_ + kMathEpsilon) ||
           (std::abs(dy - half_width_) <= kMathEpsilon &&
            dx <= half_length_ + kMathEpsilon);

  }  // IsPointOnBoundary

  // Determines the distance between a point and the box.
  double DistanceTo(const Eigen::Vector2d &point) const {
    const double dx = std::abs(point(0) - center_(0)) - half_length_;
    const double dy = std::abs(point(1) - center_(1)) - half_width_;
    if (dx <= 0.0) {
      return std::max(0.0, dy);
    }
    if (dy <= 0.0) {
      return dx;
    }

    return std::hypot(dx, dy);
  }  // DistanceTo

  // brief Determines the distance between two boxes.
  double DistanceTo(const AABox2d &box) const {
    const double dx = std::abs(box.center_x() - center_(0)) -
                      box.half_length() - half_length_;
    const double dy =
        std::abs(box.center_y() - center_(1)) - box.half_width() - half_width_;
    if (dx <= 0.0) {
      return std::max(0.0, dy);
    }
    if (dy <= 0.0) {
      return dx;
    }
    return std::hypot(dx, dy);

  }  // DistanceTo

  // brief Determines whether two boxes overlap.
  bool HasOverlap(const AABox2d &box) const {
    return std::abs(box.center_x() - center_(0)) <=
               box.half_length() + half_length_ &&
           std::abs(box.center_y() - center_(1)) <=
               box.half_width() + half_width_;
  }  // HasOverlap

  // Shift the center of AABox by the input vector.
  void Shift(const Eigen::Vector2d &shift_vec) {
    center_ += shift_vec;
  }  // Shift

  // Changes box to include another given box, as well as the current one.
  void MergeFrom(const AABox2d &other_box) {
    const double x1 = std::min(min_x(), other_box.min_x());
    const double x2 = std::max(max_x(), other_box.max_x());
    const double y1 = std::min(min_y(), other_box.min_y());
    const double y2 = std::max(max_y(), other_box.max_y());
    center_ << 0.5 * (x1 + x2), 0.5 * (y1 + y2);
    length_ = x2 - x1;
    width_ = y2 - y1;
    half_length_ = 0.5 * length_;
    half_width_ = 0.5 * width_;
  }  // MergeFrom

  // Changes box to include a given point, as well as the current box.
  void MergeFrom(const Eigen::Vector2d &other_point) {
    const double x1 = std::min(min_x(), other_point(0));
    const double x2 = std::max(max_x(), other_point(0));
    const double y1 = std::min(min_y(), other_point(1));
    const double y2 = std::max(max_y(), other_point(1));
    center_ << 0.5 * (x1 + x2), 0.5 * (y1 + y2);
    length_ = x2 - x1;
    width_ = y2 - y1;
    half_length_ = 0.5 * length_;
    half_width_ = 0.5 * width_;
  }  // MergeFrom

 private:
  Eigen::Vector2d center_;
  double length_;
  double width_;
  double half_length_;
  double half_width_;
};  // end class AABox2d

}  // namespace ASV::common::math

#endif /* _AABOX2D_H_ */

/**
 * @file
 * @brief Line segment in 2-D.
 */

#ifndef _LINESEGMENT2D_H_
#define _LINESEGMENT2D_H_

#include <algorithm>
#include <cmath>
#include <string>

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include "common/math/miscellaneous/include/math_utils.h"

namespace ASV::common::math {

class LineSegment2d {
 public:
  explicit LineSegment2d(const Eigen::Vector2d &start,
                         const Eigen::Vector2d &end)
      : start_(start),
        end_(end),
        unit_direction_(Eigen::Vector2d::Zero()),
        heading_(0),
        length_(0) {
    const double dx = end_(0) - start_(0);
    const double dy = end_(1) - start_(1);

    length_ = std::sqrt(dx * dx + dy * dy);

    unit_direction_ =
        (length_ <= kMathEpsilon
             ? Eigen::Vector2d::Zero()
             : (Eigen::Vector2d() << dx / length_, dy / length_).finished());

    heading_ = std::atan2(dy, dx);
  }
  virtual ~LineSegment2d() = default;

  Eigen::Vector2d start() const { return start_; }  // start
  Eigen::Vector2d end() const { return end_; }      // end
  Eigen::Vector2d unit_direction() const {
    return unit_direction_;
  }  // unit_direction

  // Get the center of the line segment.
  Eigen::Vector2d center() const { return 0.5 * (start_ + end_); }  // center

  /** @brief Get a new line-segment with the same start point, but rotated
   * counterclock-wise by the given amount.
   * @return The rotated line-segment's end-point.
   */
  Eigen::Vector2d rotate(const double angle) {
    Vec2d diff_vec = end_ - start_;

    double delta_x =
        diff_vec(0) * std::cos(angle) - diff_vec(1) * std::sin(angle);
    double delta_y =
        diff_vec(0) * std::sin(angle) + diff_vec(1) * std::cos(angle);

    return start_ + (Eigen::Vector2d() << delta_x, delta_y).finished();
  }  // rotate

  double heading() const { return heading_; }

  // Get the cosine of the heading.
  double cos_heading() const { return unit_direction_(0); }  // cos_heading

  // @brief Get the sine of the heading.
  double sin_heading() const { return unit_direction_(1); }  // sin_heading

  double length() const { return length_; }
  // Get the square of length of the line segment.
  double length_sqr() const { return length_ * length_; }

  // find the nearst points on the line segment
  Eigen::Vector2d find_nearest_point(const Eigen::Vector2d &point) {
    if (length_ <= kMathEpsilon) {
      return start_;
    }

    double proj = ProjectOntoUnit(point);

    if (proj <= 0.0) {
      return start_;
    } else if (proj >= length_) {
      return end_;
    } else {
      return start_ + unit_direction_ * proj;
    }
  }

  /**
   * @brief Compute perpendicular foot of a point in 2-D on the straight line
   *        expanded from the line segment.
   * @param point The point to compute the perpendicular foot from.
   * @param foot_point The computed perpendicular foot from the input point to
   *        the straight line expanded from the line segment.
   * @return The distance from the input point to the perpendicular foot.
   */
  std::pair<double, Eigen::Vector2d> GetPerpendicularFoot(
      const Eigen::Vector2d &point) const {
    if (length_ <= kMathEpsilon) {
      return DistanceTo(start_);
    }

    double proj = ProjectOntoUnit(point);

    return {std::abs(ProductOntoUnit(point)), start_ + unit_direction_ * proj};
  }  // GetPerpendicularFoot

  // Compute the shortest distance from a point on the line segment to a point
  // in 2-D.
  std::pair<double, Eigen::Vector2d> DistanceTo(
      const Eigen::Vector2d &point) const {
    Eigen::Vector2d nearstpoint = find_nearest_point(point);

    return {(nearstpoint - point).norm(), nearstpoint};
  }  // DistanceTo

  //  Check if a point is within the line segment.
  bool IsPointIn(const Eigen::Vector2d &point) const {
    if (length_ <= kMathEpsilon) {
      return std::abs(point(0) - start_(0)) <= kMathEpsilon &&
             std::abs(point(1) - start_(1)) <= kMathEpsilon;
    }

    double prod = CrossProd(start_ - point, end_ - point);
    if (std::abs(prod) > kMathEpsilon) {
      return false;
    }
    return IsWithin(point(0), start_(0), end_(0)) &&
           IsWithin(point(1), start_(1), end_(1));
  }

  // Check if the line segment has an intersect with another 2d line segment
  bool HasIntersect(const LineSegment2d &other_segment) const {
    auto [IsIntersect, point] = GetIntersect(other_segment);
    return IsIntersect;
  }  // HasIntersect

  /**
   * @brief Compute the intersect with another line segment in 2-D if any.
   * @param other_segment The line segment to compute the intersect.
   * @param point the computed intersect between the line segment and
   *        the input other_segment.
   * @return Whether the line segment has an intersect
   *         with the input other_segment.
   */
  std::pair<bool, Eigen::Vector2d> GetIntersect(
      const LineSegment2d &other_segment) const {
    if (length_ <= kMathEpsilon || other_segment.length() <= kMathEpsilon) {
      return {false, Eigen::Vector2d::Zero()};
    }
    if (IsPointIn(other_segment.start())) {
      return {true, other_segment.start()};
    }
    if (IsPointIn(other_segment.end())) {
      return {true, other_segment.end()};
    }
    if (other_segment.IsPointIn(start_)) {
      return {true, start_};
    }
    if (other_segment.IsPointIn(end_)) {
      return {true, end_};
    }

    const double cc1 = CrossProd(end_ - start_, other_segment.start() - start_);
    const double cc2 = CrossProd(end_ - start_, other_segment.end() - start_);
    if (cc1 * cc2 >= -kMathEpsilon) {
      return {false, Eigen::Vector2d::Zero()};
    }
    const double cc3 = CrossProd(other_segment.end() - other_segment.start(),
                                 start_ - other_segment.start());
    const double cc4 = CrossProd(other_segment.end() - other_segment.start(),
                                 end_ - other_segment.start());
    if (cc3 * cc4 >= -kMathEpsilon) {
      return {false, Eigen::Vector2d::Zero()};
    }

    const double ratio = cc4 / (cc4 - cc3);
    return {true,
            (Eigen::Vector2d() << start_(0) * ratio + end_(0) * (1.0 - ratio),
             start_(1) * ratio + end_(1) * (1.0 - ratio))
                .finished()};

  }  // GetIntersect

  // Compute the projection of a vector onto the line segment,
  // which is from the start point of the line segment to the input point,
  // onto the unit direction
  double ProjectOntoUnit(const Eigen::Vector2d &point) const {
    auto vectorps = point - start_;
    return InnerProd(vectorps(0), vectorps(1), unit_direction_(0),
                     unit_direction_(1));
  }

  // Compute the cross product of a vector onto the line segment,
  // which is from the start point of the line segment to the input point.
  double ProductOntoUnit(const Eigen::Vector2d &point) const {
    auto vectorps = point - start_;
    return CrossProd(unit_direction_(0), unit_direction_(1), vectorps(0),
                     vectorps(1));
  }

 private:
  Eigen::Vector2d start_;
  Eigen::Vector2d end_;
  // the unit direction from the start point to the end point.
  Eigen::Vector2d unit_direction_;
  double heading_;
  double length_;
};

}  // namespace ASV::common::math

#endif /* _LINESEGMENT2D_H_ */
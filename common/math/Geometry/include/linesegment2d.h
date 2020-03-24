
/**
 * @file
 * @brief Line segment in 2-D.
 */

#ifndef _LINESEGMENT2D_H_
#define _LINESEGMENT2D_H_

#include <string>

#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>

namespace ASV::common::math {

class LineSegment2d {
 public:
  LineSegment2d() unit_direction_((Eigen::Vector2d() << 1, 0).finished()) {}

  LineSegment2d(const Eigen::Vector2d &start, const Eigen::Vector2d &end);
  virtual ~LineSegment2d() = default;

  Eigen::Vector2d start() const { return start_; }
  Eigen::Vector2d end() const { return end_; }
  Eigen::Vector2d unit_direction() const { return unit_direction_; }

  /**
   * @brief Get the center of the line segment.
   * @return The center of the line segment.
   */
  Vec2d center() const { return 0.5 * (start_ + end_); }

  /** @brief Get a new line-segment with the same start point, but rotated
   * counterclock-wise by the given amount.
   * @return The rotated line-segment's end-point.
   */
  Vec2d rotate(const double angle);

  double heading() const { return heading_; }

  /**
   * @brief Get the cosine of the heading.
   * @return The cosine of the heading.
   */
  double cos_heading() const { return unit_direction_.x(); }

  /**
   * @brief Get the sine of the heading.
   * @return The sine of the heading.
   */
  double sin_heading() const { return unit_direction_.y(); }

  /**
   * @brief Get the length of the line segment.
   * @return The length of the line segment.
   */
  double length() const;

  /**
   * @brief Get the square of length of the line segment.
   * @return The square of length of the line segment.
   */
  double length_sqr() const;

  /**
   * @brief Compute the shortest distance from a point on the line segment
   *        to a point in 2-D.
   * @param point The point to compute the distance to.
   * @return The shortest distance from points on the line segment to point.
   */
  double DistanceTo(const Vec2d &point) const;

  /**
   * @brief Compute the shortest distance from a point on the line segment
   *        to a point in 2-D, and get the nearest point on the line segment.
   * @param point The point to compute the distance to.
   * @param nearest_pt The nearest point on the line segment
   *        to the input point.
   * @return The shortest distance from points on the line segment
   *         to the input point.
   */
  double DistanceTo(const Vec2d &point, Vec2d *const nearest_pt) const;

  /**
   * @brief Compute the square of the shortest distance from a point
   *        on the line segment to a point in 2-D.
   * @param point The point to compute the squared of the distance to.
   * @return The square of the shortest distance from points
   *         on the line segment to the input point.
   */
  double DistanceSquareTo(const Vec2d &point) const;

  /**
   * @brief Compute the square of the shortest distance from a point
   *        on the line segment to a point in 2-D,
   *        and get the nearest point on the line segment.
   * @param point The point to compute the squared of the distance to.
   * @param nearest_pt The nearest point on the line segment
   *        to the input point.
   * @return The shortest distance from points on the line segment
   *         to the input point.
   */
  double DistanceSquareTo(const Vec2d &point, Vec2d *const nearest_pt) const;

  /**
   * @brief Check if a point is within the line segment.
   * @param point The point to check if it is within the line segment.
   * @return Whether the input point is within the line segment or not.
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Check if the line segment has an intersect
   *        with another line segment in 2-D.
   * @param other_segment The line segment to check if it has an intersect.
   * @return Whether the line segment has an intersect
   *         with the input other_segment.
   */
  bool HasIntersect(const LineSegment2d &other_segment) const;

  /**
   * @brief Compute the intersect with another line segment in 2-D if any.
   * @param other_segment The line segment to compute the intersect.
   * @param point the computed intersect between the line segment and
   *        the input other_segment.
   * @return Whether the line segment has an intersect
   *         with the input other_segment.
   */
  bool GetIntersect(const LineSegment2d &other_segment,
                    Vec2d *const point) const;

  /**
   * @brief Compute the projection of a vector onto the line segment.
   * @param point The end of the vector (starting from the start point of the
   *        line segment) to compute the projection onto the line segment.
   * @return The projection of the vector, which is from the start point of
   *         the line segment to the input point, onto the unit direction.
   */
  double ProjectOntoUnit(const Vec2d &point) const;

  /**
   * @brief Compute the cross product of a vector onto the line segment.
   * @param point The end of the vector (starting from the start point of the
   *        line segment) to compute the cross product onto the line segment.
   * @return The cross product of the unit direction and
   *         the vector, which is from the start point of
   *         the line segment to the input point.
   */
  double ProductOntoUnit(const Vec2d &point) const;

  /**
   * @brief Compute perpendicular foot of a point in 2-D on the straight line
   *        expanded from the line segment.
   * @param point The point to compute the perpendicular foot from.
   * @param foot_point The computed perpendicular foot from the input point to
   *        the straight line expanded from the line segment.
   * @return The distance from the input point to the perpendicular foot.
   */
  double GetPerpendicularFoot(const Vec2d &point,
                              Vec2d *const foot_point) const;

  /**
   * @brief Get the debug string including the essential information.
   * @return Information of the line segment for debugging.
   */
  std::string DebugString() const;

 private:
  Eigen::Vector2d start_;
  Eigen::Vector2d end_;
  // the unit direction from the start point to the end point.
  Eigen::Vector2d unit_direction_;
  double heading_ = 0.0;
  double length_ = 0.0;
};

}  // namespace ASV::common::math

#endif /* _LINESEGMENT2D_H_ */
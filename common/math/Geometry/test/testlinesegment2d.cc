/*
***********************************************************************
* testlinesegment2d.cc: test for Line segment in 2-D.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <boost/test/included/unit_test.hpp>
#include "../include/linesegment2d.h"

BOOST_AUTO_TEST_CASE(Accessors) {
  ASV::common::math::LineSegment2d ls((Eigen::Vector2d() << 1, 2).finished(),
                                      (Eigen::Vector2d() << 5, 4).finished());
  BOOST_CHECK_CLOSE(ls.length(), std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(ls.length_sqr(), 20.0, 1e-6);
  BOOST_CHECK_CLOSE(ls.center()[0], 3, 1e-6);
  BOOST_CHECK_CLOSE(ls.center()[1], 3, 1e-6);
  BOOST_CHECK_CLOSE(ls.heading(), std::atan2(2, 4), 1e-6);
  BOOST_CHECK_CLOSE(ls.cos_heading(), 4.0 / std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(ls.sin_heading(), 2.0 / std::sqrt(20.0), 1e-6);
}

BOOST_AUTO_TEST_CASE(Distance) {
  ASV::common::math::LineSegment2d ls((Eigen::Vector2d() << 1, 2).finished(),
                                      (Eigen::Vector2d() << 5, 4).finished());
  auto [distance1, nearest_pt1] =
      ls.DistanceTo((Eigen::Vector2d() << 0, 0).finished());

  BOOST_CHECK_CLOSE(distance1, std::sqrt(5.0), 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt1[0], 1, 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt1[1], 2, 1e-6);

  auto [distance2, nearest_pt2] =
      ls.DistanceTo((Eigen::Vector2d() << 10, 10).finished());
  BOOST_CHECK_CLOSE(distance2, std::sqrt(61.0), 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt2[0], 5, 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt2[1], 4, 1e-6);

  auto [distance3, nearest_pt3] =
      ls.DistanceTo((Eigen::Vector2d() << 3, 3).finished());
  BOOST_CHECK_CLOSE(distance3, 0, 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt3[0], 3, 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt3[1], 3, 1e-6);

  auto [distance4, nearest_pt4] =
      ls.DistanceTo((Eigen::Vector2d() << 4, 4).finished());
  BOOST_CHECK_CLOSE(distance4, 2.0 / std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt4[0], 4.2, 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt4[1], 3.6, 1e-6);
}

BOOST_AUTO_TEST_CASE(PerpendicularFoot) {
  ASV::common::math::LineSegment2d ls((Eigen::Vector2d() << 1, 2).finished(),
                                      (Eigen::Vector2d() << 5, 4).finished());
  auto [distance1, foot_pt1] =
      ls.GetPerpendicularFoot((Eigen::Vector2d() << 0, 0).finished());
  BOOST_CHECK_CLOSE(distance1, 0.6 * std::sqrt(5.0), 1e-6);
  BOOST_CHECK_CLOSE(foot_pt1[0], -0.6, 1e-6);
  BOOST_CHECK_CLOSE(foot_pt1[1], 1.2, 1e-6);

  auto [distance2, foot_pt2] =
      ls.GetPerpendicularFoot((Eigen::Vector2d() << 3, 3).finished());
  BOOST_CHECK_CLOSE(distance2, 0, 1e-6);
  BOOST_CHECK_CLOSE(foot_pt2[0], 3, 1e-6);
  BOOST_CHECK_CLOSE(foot_pt2[1], 3, 1e-6);

  auto [distance3, foot_pt3] =
      ls.DistanceTo((Eigen::Vector2d() << 4, 4).finished());
  BOOST_CHECK_CLOSE(distance3, 2.0 / std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(foot_pt3[0], 4.2, 1e-6);
  BOOST_CHECK_CLOSE(foot_pt3[1], 3.6, 1e-6);
}

BOOST_AUTO_TEST_CASE(ProjectOntoUnit) {
  ASV::common::math::LineSegment2d ls((Eigen::Vector2d() << 1, 2).finished(),
                                      (Eigen::Vector2d() << 5, 4).finished());
  BOOST_CHECK_CLOSE(ls.ProjectOntoUnit((Eigen::Vector2d() << 1, 2).finished()),
                    0.0, 1e-6);
  BOOST_CHECK_CLOSE(ls.ProjectOntoUnit((Eigen::Vector2d() << 5, 4).finished()),
                    std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(
      ls.ProjectOntoUnit((Eigen::Vector2d() << -2, -3).finished()),
      -22.0 / std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(ls.ProjectOntoUnit((Eigen::Vector2d() << 6, 7).finished()),
                    30.0 / std::sqrt(20.0), 1e-6);
}

BOOST_AUTO_TEST_CASE(GetIntersect) {
  using namespace ASV::common::math;

  LineSegment2d ls((Eigen::Vector2d() << 1, 2).finished(),
                   (Eigen::Vector2d() << 5, 4).finished());

  auto [isIntersect1, point1] =
      ls.GetIntersect(LineSegment2d((Eigen::Vector2d() << 1, 3).finished(),
                                    (Eigen::Vector2d() << 5, 5).finished()));
  BOOST_TEST(!isIntersect1);
  BOOST_CHECK_CLOSE(point1[0], 0.0, 1e-6);
  BOOST_CHECK_CLOSE(point1[1], 0.0, 1e-6);

  auto [isIntersect2, point2] =
      ls.GetIntersect(LineSegment2d((Eigen::Vector2d() << 2, 2).finished(),
                                    (Eigen::Vector2d() << 6, 4).finished()));
  BOOST_TEST(!isIntersect2);
  BOOST_CHECK_CLOSE(point2[0], 0.0, 1e-6);
  BOOST_CHECK_CLOSE(point2[1], 0.0, 1e-6);

  auto [isIntersect3, point3] =
      ls.GetIntersect(LineSegment2d((Eigen::Vector2d() << 1, 2).finished(),
                                    (Eigen::Vector2d() << -3, 0).finished()));
  BOOST_TEST(isIntersect3);
  BOOST_CHECK_CLOSE(point3[0], 1.0, 1e-6);
  BOOST_CHECK_CLOSE(point3[1], 2.0, 1e-6);

  auto [isIntersect4, point4] =
      ls.GetIntersect(LineSegment2d((Eigen::Vector2d() << 5, 4).finished(),
                                    (Eigen::Vector2d() << 9, 6).finished()));
  BOOST_TEST(isIntersect4);
  BOOST_CHECK_CLOSE(point4[0], 5.0, 1e-6);
  BOOST_CHECK_CLOSE(point4[1], 4.0, 1e-6);

  auto [isIntersect5, point5] =
      ls.GetIntersect(LineSegment2d((Eigen::Vector2d() << 3, 0).finished(),
                                    (Eigen::Vector2d() << 3, 10).finished()));
  BOOST_TEST(isIntersect5);
  BOOST_CHECK_CLOSE(point5[0], 3.0, 1e-6);
  BOOST_CHECK_CLOSE(point5[1], 3.0, 1e-6);

  auto [isIntersect6, point6] =
      ls.GetIntersect(LineSegment2d((Eigen::Vector2d() << 3, 10).finished(),
                                    (Eigen::Vector2d() << 3, 0).finished()));
  BOOST_TEST(isIntersect6);
  BOOST_CHECK_CLOSE(point6[0], 3.0, 1e-6);
  BOOST_CHECK_CLOSE(point6[1], 3.0, 1e-6);

  auto [isIntersect7, point7] =
      ls.GetIntersect(LineSegment2d((Eigen::Vector2d() << 3, 3).finished(),
                                    (Eigen::Vector2d() << 3, 3).finished()));
  BOOST_TEST(isIntersect7);
  BOOST_CHECK_CLOSE(point7[0], 3.0, 1e-6);
  BOOST_CHECK_CLOSE(point7[1], 3.0, 1e-6);

  auto [isIntersect8, point8] =
      ls.GetIntersect(LineSegment2d((Eigen::Vector2d() << 3, 5).finished(),
                                    (Eigen::Vector2d() << 3, 10).finished()));
  BOOST_TEST(!isIntersect8);

  auto [isIntersect9, point9] =
      ls.GetIntersect(LineSegment2d((Eigen::Vector2d() << 3, 2).finished(),
                                    (Eigen::Vector2d() << 3, 0).finished()));
  BOOST_TEST(!isIntersect9);

  auto [isIntersect10, point10] =
      ls.GetIntersect(LineSegment2d((Eigen::Vector2d() << 4, 4).finished(),
                                    (Eigen::Vector2d() << 4, 4).finished()));
  BOOST_TEST(!isIntersect10);
}

BOOST_AUTO_TEST_CASE(IsPointIn) {
  using namespace ASV::common::math;
  LineSegment2d ls((Eigen::Vector2d() << 1, 2).finished(),
                   (Eigen::Vector2d() << 5, 4).finished());

  BOOST_TEST(ls.IsPointIn((Eigen::Vector2d() << 1, 2).finished()));
  BOOST_TEST(ls.IsPointIn((Eigen::Vector2d() << 5, 4).finished()));
  BOOST_TEST(ls.IsPointIn((Eigen::Vector2d() << 3, 3).finished()));
  BOOST_TEST(!ls.IsPointIn((Eigen::Vector2d() << -1, 1).finished()));
  BOOST_TEST(!ls.IsPointIn((Eigen::Vector2d() << 7, 5).finished()));
  BOOST_TEST(!ls.IsPointIn((Eigen::Vector2d() << 0, 0).finished()));
  BOOST_TEST(!ls.IsPointIn((Eigen::Vector2d() << 6, 6).finished()));
}
/*
***********************************************************************
* box2d_test.cc: (undirected) axes-aligned bounding boxes in 2-D.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/box2d.h"
#include <boost/test/included/unit_test.hpp>

using namespace ASV::common::math;

bool CheckBoxOverlapSlow(const Box2d &box1, const Box2d &box2,
                         bool *const ambiguous) {
  double headings[4] = {box1.heading(), box1.heading() + M_PI / 2,
                        box2.heading(), box2.heading() + M_PI / 2};
  *ambiguous = false;
  for (int k = 0; k < 4; ++k) {
    const double heading = headings[k];
    const double cos_heading = std::cos(heading);
    const double sin_heading = std::sin(heading);
    auto c1 = box1.GetAllCorners();
    auto c2 = box2.GetAllCorners();
    double s1 = std::numeric_limits<double>::infinity();
    double t1 = -std::numeric_limits<double>::infinity();
    double s2 = std::numeric_limits<double>::infinity();
    double t2 = -std::numeric_limits<double>::infinity();

    for (int i = 0; i != 4; ++i) {
      const double proj = c1(0, i) * cos_heading + c1(1, i) * sin_heading;
      if (proj < s1) {
        s1 = proj;
      }
      if (proj > t1) {
        t1 = proj;
      }
    }

    for (int i = 0; i != 4; ++i) {
      const double proj = c2(0, i) * cos_heading + c2(1, i) * sin_heading;
      if (proj < s2) {
        s2 = proj;
      }
      if (proj > t2) {
        t2 = proj;
      }
    }

    if (std::abs(s1 - t2) <= 1e-5 || std::abs(s2 - t1) <= 1e-5) {
      *ambiguous = true;
    }
    if (s1 > t2 || s2 > t1) {
      return false;
    }
  }
  return true;
}

Box2d box1(Eigen::Vector2d::Zero(), 0, 4, 2);
Box2d box2((Eigen::Vector2d() << 5, 2).finished(), 0, 4, 2);
Box2d box3(LineSegment2d((Eigen::Vector2d() << 2, 3).finished(),
                         (Eigen::Vector2d() << 6, 3).finished()),
           2);
Box2d box4((Eigen::Vector2d() << 7, 8).finished(), M_PI / 4.0, 5.0, 3.0);
Box2d box5((Eigen::Vector2d() << -2, -3).finished(), -M_PI, 0.0, 0.0);
Box2d box6(LineSegment2d((Eigen::Vector2d() << 2, 3).finished(),
                         (Eigen::Vector2d() << 6, 3).finished()),
           0.0);
Box2d box7(AABox2d((Eigen::Vector2d() << 4, 5).finished(), 0, 0));

BOOST_AUTO_TEST_CASE(GetAllCorners) {
  auto corners1 = box1.GetAllCorners();

  BOOST_CHECK_CLOSE(corners1(0, 0), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(1, 0), -1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(0, 1), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(1, 1), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(0, 2), -2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(1, 2), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(0, 3), -2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(1, 3), -1.0, 1e-6);

  auto corners3 = box3.GetAllCorners();

  BOOST_CHECK_CLOSE(corners3(0, 0), 6.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3(1, 0), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3(0, 1), 6.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3(1, 1), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3(0, 2), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3(1, 2), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3(0, 3), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3(1, 3), 2.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(CenterAndLength) {
  BOOST_CHECK_CLOSE(box3.center()[0], 4.0, 1e-6);
  BOOST_CHECK_CLOSE(box3.center()[1], 3, 1e-6);
  BOOST_CHECK_CLOSE(box3.length(), 4, 1e-6);
  BOOST_CHECK_CLOSE(box3.width(), 2, 1e-6);
  BOOST_CHECK_CLOSE(box3.half_length(), 2, 1e-6);
  BOOST_CHECK_CLOSE(box3.half_width(), 1, 1e-6);
  BOOST_CHECK_CLOSE(box3.diagonal(), std::hypot(4.0, 2.0), 1e-6);
}

BOOST_AUTO_TEST_CASE(HasOverlap) {
  BOOST_TEST(!box1.HasOverlap(box2));
  BOOST_TEST(!box1.HasOverlap(box3));
  BOOST_TEST(!box1.HasOverlap(box4));
  BOOST_TEST(!box2.HasOverlap(box4));
  BOOST_TEST(!box3.HasOverlap(box4));
  BOOST_TEST(box4.HasOverlap(box4));

  BOOST_TEST(
      box1.HasOverlap(LineSegment2d((Eigen::Vector2d() << 0, 0).finished(),
                                    (Eigen::Vector2d() << 1, 1).finished())));
  BOOST_TEST(
      box1.HasOverlap(LineSegment2d((Eigen::Vector2d() << 0, 0).finished(),
                                    (Eigen::Vector2d() << 3, 3).finished())));
  BOOST_TEST(
      box1.HasOverlap(LineSegment2d((Eigen::Vector2d() << 0, -3).finished(),
                                    (Eigen::Vector2d() << 0, 3).finished())));
  BOOST_TEST(
      box1.HasOverlap(LineSegment2d((Eigen::Vector2d() << 4, 0).finished(),
                                    (Eigen::Vector2d() << -4, 0).finished())));
  BOOST_TEST(
      box1.HasOverlap(LineSegment2d((Eigen::Vector2d() << -4, -4).finished(),
                                    (Eigen::Vector2d() << 4, 4).finished())));
  BOOST_TEST(
      box1.HasOverlap(LineSegment2d((Eigen::Vector2d() << 4, -4).finished(),
                                    (Eigen::Vector2d() << -4, 4).finished())));
  BOOST_TEST(
      !box1.HasOverlap(LineSegment2d((Eigen::Vector2d() << -4, -4).finished(),
                                     (Eigen::Vector2d() << 4, -4).finished())));
  BOOST_TEST(
      !box1.HasOverlap(LineSegment2d((Eigen::Vector2d() << 4, -4).finished(),
                                     (Eigen::Vector2d() << 4, 4).finished())));
}

BOOST_AUTO_TEST_CASE(GetAABox) {
  AABox2d aabox1 = box1.GetSmallestAABox();
  AABox2d aabox2 = box2.GetSmallestAABox();
  AABox2d aabox3 = box3.GetSmallestAABox();
  AABox2d aabox4 = box4.GetSmallestAABox();

  BOOST_CHECK_CLOSE(aabox1.center_x(), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox1.center_y(), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox1.length(), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox1.width(), 2.0, 1e-6);

  BOOST_CHECK_CLOSE(aabox2.center_x(), 5.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox2.center_y(), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox2.length(), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox2.width(), 2.0, 1e-6);

  BOOST_CHECK_CLOSE(aabox3.center_x(), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox3.center_y(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox3.length(), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox3.width(), 2.0, 1e-6);

  BOOST_CHECK_CLOSE(aabox4.center_x(), 7.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox4.center_y(), 8.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox4.length(), 4.0 * std::sqrt(2.0), 1e-6);
  BOOST_CHECK_CLOSE(aabox4.width(), 4.0 * std::sqrt(2.0), 1e-6);
}

BOOST_AUTO_TEST_CASE(DistanceTo) {
  BOOST_CHECK_CLOSE(box1.DistanceTo((Eigen::Vector2d() << 3, 0).finished()),
                    1.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo((Eigen::Vector2d() << -3, 0).finished()),
                    1.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo((Eigen::Vector2d() << 0, 2).finished()),
                    1.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo((Eigen::Vector2d() << 0, -2).finished()),
                    1.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo((Eigen::Vector2d() << 0, 0).finished()),
                    0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo((Eigen::Vector2d() << 0, 1).finished()),
                    0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo((Eigen::Vector2d() << 1, 0).finished()),
                    0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo((Eigen::Vector2d() << 0, -1).finished()),
                    0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo((Eigen::Vector2d() << -1, 0).finished()),
                    0.0, 1e-6);
  BOOST_CHECK_CLOSE(
      box1.DistanceTo(LineSegment2d((Eigen::Vector2d() << -4, -4).finished(),
                                    (Eigen::Vector2d() << 4, 4).finished())),
      0.0, 1e-6);
  BOOST_CHECK_CLOSE(
      box1.DistanceTo(LineSegment2d((Eigen::Vector2d() << 4, -4).finished(),
                                    (Eigen::Vector2d() << -4, 4).finished())),
      0.0, 1e-6);

  BOOST_CHECK_CLOSE(
      box1.DistanceTo(LineSegment2d((Eigen::Vector2d() << 0, 2).finished(),
                                    (Eigen::Vector2d() << 4, 4).finished())),
      1.0, 1e-6);
  BOOST_CHECK_CLOSE(
      box1.DistanceTo(LineSegment2d((Eigen::Vector2d() << 2, 2).finished(),
                                    (Eigen::Vector2d() << 3, 1).finished())),
      std::sqrt(2.0) / 2.0, 1e-6);

  BOOST_CHECK_CLOSE(box1.DistanceTo(box2), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(box3), 1.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(IsPointIn) {
  BOOST_TEST(box1.IsPointIn((Eigen::Vector2d() << 0, 0).finished()));
  BOOST_TEST(box1.IsPointIn((Eigen::Vector2d() << 1, 0.5).finished()));
  BOOST_TEST(box1.IsPointIn((Eigen::Vector2d() << -0.5, -1).finished()));
  BOOST_TEST(box1.IsPointIn((Eigen::Vector2d() << 2, 1).finished()));
  BOOST_TEST(!box1.IsPointIn((Eigen::Vector2d() << -3, 0).finished()));
  BOOST_TEST(!box1.IsPointIn((Eigen::Vector2d() << 0, 2).finished()));
  BOOST_TEST(!box1.IsPointIn((Eigen::Vector2d() << -4, -2).finished()));
}

BOOST_AUTO_TEST_CASE(IsPointOnBoundary) {
  BOOST_TEST(!box1.IsPointOnBoundary((Eigen::Vector2d() << 0, 0).finished()));
  BOOST_TEST(!box1.IsPointOnBoundary((Eigen::Vector2d() << 1, 0.5).finished()));
  BOOST_TEST(
      box1.IsPointOnBoundary((Eigen::Vector2d() << -0.5, -1).finished()));
  BOOST_TEST(box1.IsPointOnBoundary((Eigen::Vector2d() << 2, 0.5).finished()));
  BOOST_TEST(box1.IsPointOnBoundary((Eigen::Vector2d() << -2, 1).finished()));
  BOOST_TEST(!box1.IsPointOnBoundary((Eigen::Vector2d() << -3, 0).finished()));
  BOOST_TEST(!box1.IsPointOnBoundary((Eigen::Vector2d() << 0, 2).finished()));
  BOOST_TEST(!box1.IsPointOnBoundary((Eigen::Vector2d() << -4, -2).finished()));
}

BOOST_AUTO_TEST_CASE(RotateFromCenterAndShift) {
  Box2d box((Eigen::Vector2d() << 0, 0).finished(), 0, 4, 2);
  BOOST_CHECK_CLOSE(box.heading(), 0.0, 1e-6);
  box.RotateFromCenter(M_PI / 2);
  BOOST_CHECK_CLOSE(box.heading(), M_PI / 2, 1e-6);
  auto corners = box.GetAllCorners();

  BOOST_CHECK_CLOSE(corners(0, 0), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(1, 0), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(0, 1), -1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(1, 1), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(0, 2), -1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(1, 2), -2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(0, 3), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(1, 3), -2.0, 1e-6);

  box.Shift({30, 40});
  corners = box.GetAllCorners();

  BOOST_CHECK_CLOSE(corners(0, 0), 31.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(1, 0), 42.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(0, 1), 29.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(1, 1), 42.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(0, 2), 29.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(1, 2), 38.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(0, 3), 31.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(1, 3), 38.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(TESTRANDOM) {
  bool ambiguous = false;

  for (int iter = 0; iter < 10000; ++iter) {
    const double x1 = RandomDouble(-10, 10);
    const double y1 = RandomDouble(-10, 10);
    const double x2 = RandomDouble(-10, 10);
    const double y2 = RandomDouble(-10, 10);
    const double heading1 = RandomDouble(0, M_PI * 2.0);
    const double heading2 = RandomDouble(0, M_PI * 2.0);
    const double l1 = RandomDouble(1, 5);
    const double l2 = RandomDouble(1, 5);
    const double w1 = RandomDouble(1, 5);
    const double w2 = RandomDouble(1, 5);
    const Box2d box1((Eigen::Vector2d() << x1, y1).finished(), heading1, l1,
                     w1);
    const Box2d box2((Eigen::Vector2d() << x2, y2).finished(), heading2, l2,
                     w2);
    bool overlap = CheckBoxOverlapSlow(box1, box2, &ambiguous);
    if (!ambiguous) {
      BOOST_TEST(overlap == box1.HasOverlap(box2));
    }
  }
}

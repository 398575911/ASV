/*
***********************************************************************
* aabox2d_test.cc: (undirected) axes-aligned bounding boxes in 2-D.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/aabox2d.h"
#include <boost/test/included/unit_test.hpp>

using namespace ASV::common::math;

BOOST_AUTO_TEST_CASE(GetAllCorners) {
  AABox2d box1(Eigen::Vector2d::Zero(), 4, 2);
  auto corners1 = box1.GetAllCorners();

  BOOST_CHECK_CLOSE(corners1(0, 0), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(0, 1), -1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(1, 0), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(1, 1), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(2, 0), -2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(2, 1), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(3, 0), -2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1(3, 1), -1.0, 1e-6);

  AABox2d box2((Eigen::Vector2d() << 3, 1).finished(),
               (Eigen::Vector2d() << 7, 3).finished());
  auto corners2 = box2.GetAllCorners();
  BOOST_CHECK_CLOSE(corners2(0, 0), 7.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2(0, 1), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2(1, 0), 7.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2(1, 1), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2(2, 0), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2(2, 1), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2(3, 0), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2(3, 1), 1.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(CenterAndLengths) {
  AABox2d box1(Eigen::Vector2d::Zero(), 10, 6);
  BOOST_CHECK_CLOSE(box1.center()[0], 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.center()[1], 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.length(), 10.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.width(), 6.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.half_length(), 5.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.half_width(), 3.0, 1e-6);

  Eigen::VectorXd point_x(4);
  Eigen::VectorXd point_y(4);
  point_x << 0, 0, 3, 1;
  point_y << 2, -6, 0, 0;

  AABox2d box2(point_x, point_y);
  BOOST_CHECK_CLOSE(box2.center()[0], 1.5, 1e-6);
  BOOST_CHECK_CLOSE(box2.center()[1], -2.0, 1e-6);
  BOOST_CHECK_CLOSE(box2.length(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(box2.width(), 8.0, 1e-6);
  BOOST_CHECK_CLOSE(box2.half_length(), 1.5, 1e-6);
  BOOST_CHECK_CLOSE(box2.half_width(), 4.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(HasOverlap) {
  AABox2d box1((Eigen::Vector2d() << 0, 0).finished(), 4, 2);
  AABox2d box2((Eigen::Vector2d() << 3, 1).finished(),
               (Eigen::Vector2d() << 7, 3).finished());
  AABox2d box3((Eigen::Vector2d() << 0, 0).finished(), 10, 10);

  BOOST_TEST(!box1.HasOverlap(box2));
  BOOST_TEST(box1.HasOverlap(box3));
  BOOST_TEST(box2.HasOverlap(box3));
}

BOOST_AUTO_TEST_CASE(DistanceTo) {
  AABox2d box((Eigen::Vector2d() << 0, 0).finished(), 4, 2);
  BOOST_CHECK_CLOSE(box.DistanceTo((Eigen::Vector2d() << 3, 0).finished()), 1.0,
                    1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo((Eigen::Vector2d() << -3, 0).finished()),
                    1.0, 1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo((Eigen::Vector2d() << 0, 2).finished()), 1.0,
                    1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo((Eigen::Vector2d() << 0, -2).finished()),
                    1.0, 1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo((Eigen::Vector2d() << 0, 0).finished()), 0.0,
                    1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo((Eigen::Vector2d() << 0, 1).finished()), 0.0,
                    1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo((Eigen::Vector2d() << 1, 0).finished()), 0.0,
                    1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo((Eigen::Vector2d() << 0, -1).finished()),
                    0.0, 1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo((Eigen::Vector2d() << -1, 0).finished()),
                    0.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(IsPointIn) {
  AABox2d box({0, 0}, 4, 2);
  BOOST_TEST(box.IsPointIn((Eigen::Vector2d() << 0, 0).finished()));
  BOOST_TEST(box.IsPointIn((Eigen::Vector2d() << 1, 0.5).finished()));
  BOOST_TEST(box.IsPointIn((Eigen::Vector2d() << -0.5, -1).finished()));
  BOOST_TEST(box.IsPointIn((Eigen::Vector2d() << 2, 1).finished()));
  BOOST_TEST(!box.IsPointIn((Eigen::Vector2d() << -3, 0).finished()));
  BOOST_TEST(!box.IsPointIn((Eigen::Vector2d() << 0, 2).finished()));
  BOOST_TEST(!box.IsPointIn((Eigen::Vector2d() << -4, -2).finished()));
}

BOOST_AUTO_TEST_CASE(IsPointOnBoundary) {
  AABox2d box({0, 0}, 4, 2);
  BOOST_TEST(!box.IsPointOnBoundary((Eigen::Vector2d() << 0, 0).finished()));
  BOOST_TEST(!box.IsPointOnBoundary((Eigen::Vector2d() << 1, 0.5).finished()));
  BOOST_TEST(box.IsPointOnBoundary((Eigen::Vector2d() << -0.5, -1).finished()));
  BOOST_TEST(box.IsPointOnBoundary((Eigen::Vector2d() << 2, 0.5).finished()));
  BOOST_TEST(box.IsPointOnBoundary((Eigen::Vector2d() << -2, 1).finished()));
  BOOST_TEST(!box.IsPointOnBoundary((Eigen::Vector2d() << -3, 0).finished()));
  BOOST_TEST(!box.IsPointOnBoundary((Eigen::Vector2d() << 0, 2).finished()));
  BOOST_TEST(!box.IsPointOnBoundary((Eigen::Vector2d() << -4, -2).finished()));
}

BOOST_AUTO_TEST_CASE(MinMax) {
  AABox2d box({0, 0}, 4, 2);
  BOOST_CHECK_CLOSE(box.min_x(), -2, 1e-6);
  BOOST_CHECK_CLOSE(box.max_x(), 2, 1e-6);
  BOOST_CHECK_CLOSE(box.min_y(), -1, 1e-6);
  BOOST_CHECK_CLOSE(box.max_y(), 1, 1e-6);

  AABox2d box2((Eigen::Vector2d() << 3, 1).finished(),
               (Eigen::Vector2d() << 7, 3).finished());
  BOOST_CHECK_CLOSE(box2.min_x(), 3, 1e-6);
  BOOST_CHECK_CLOSE(box2.max_x(), 7, 1e-6);
  BOOST_CHECK_CLOSE(box2.min_y(), 1, 1e-6);
  BOOST_CHECK_CLOSE(box2.max_y(), 3, 1e-6);
}

BOOST_AUTO_TEST_CASE(Shift) {
  AABox2d box({0, 0}, 4, 2);
  box.Shift((Eigen::Vector2d() << 30, 40).finished());
  auto corners = box.GetAllCorners();
  BOOST_CHECK_CLOSE(corners(0, 0), 32.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(0, 1), 39.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(1, 0), 32.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(1, 1), 41.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(2, 0), 28.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(2, 1), 41.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(3, 0), 28.0, 1e-6);
  BOOST_CHECK_CLOSE(corners(3, 1), 39.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(MergeFrom) {
  AABox2d box((Eigen::Vector2d() << 3, 1).finished(),
              (Eigen::Vector2d() << 7, 3).finished());
  box.MergeFrom(AABox2d((Eigen::Vector2d() << 5, -1).finished(),
                        (Eigen::Vector2d() << 10, 7).finished()));
  BOOST_CHECK_CLOSE(box.center()[0], 6.5, 1e-6);
  BOOST_CHECK_CLOSE(box.center()[1], 3.0, 1e-6);
  BOOST_CHECK_CLOSE(box.length(), 7.0, 1e-6);
  BOOST_CHECK_CLOSE(box.width(), 8.0, 1e-6);
  BOOST_CHECK_CLOSE(box.half_length(), 3.5, 1e-6);
  BOOST_CHECK_CLOSE(box.half_width(), 4.0, 1e-6);

  box.MergeFrom((Eigen::Vector2d() << 6, 6).finished());
  BOOST_CHECK_CLOSE(box.center()[0], 6.5, 1e-6);
  BOOST_CHECK_CLOSE(box.center()[1], 3.0, 1e-6);
  BOOST_CHECK_CLOSE(box.length(), 7.0, 1e-6);
  BOOST_CHECK_CLOSE(box.width(), 8.0, 1e-6);
  BOOST_CHECK_CLOSE(box.half_length(), 3.5, 1e-6);
  BOOST_CHECK_CLOSE(box.half_width(), 4.0, 1e-6);

  box.MergeFrom((Eigen::Vector2d() << -5, 20).finished());
  BOOST_CHECK_CLOSE(box.center()[0], 2.5, 1e-6);
  BOOST_CHECK_CLOSE(box.center()[1], 9.5, 1e-6);
  BOOST_CHECK_CLOSE(box.length(), 15, 1e-6);
  BOOST_CHECK_CLOSE(box.width(), 21, 1e-6);
  BOOST_CHECK_CLOSE(box.half_length(), 7.5, 1e-6);
  BOOST_CHECK_CLOSE(box.half_width(), 10.5, 1e-6);
}
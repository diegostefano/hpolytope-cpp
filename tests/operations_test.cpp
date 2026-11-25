// Copyright (C) 2026 Diego Stéfano Fonseca Ferreira
// SPDX-License-Identifier: GPL-3.0-or-later

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "hpolytope-cpp/hpolytope.hpp"
#include "test_helpers.hpp"
#include "test_polytopes.hpp"

namespace {

TEST(HPolytopeIntersection, MatchesMpt3) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const hpolytope::HPolytope s = hpolytope::test::MakeS();
  const hpolytope::HPolytope actual = p & s;

  Eigen::MatrixXd A(4, 2);
  A << -1, 0, 1, 0, 0, -1, 0, 1;

  Eigen::VectorXd b(4);
  b << 0, 2, 2, 1;

  const hpolytope::HPolytope expected(A, b);

  EXPECT_TRUE(actual == expected);
  EXPECT_TRUE(p.contains(actual));
  EXPECT_TRUE(s.contains(actual));
}

TEST(HPolytopeIntersection, RejectsDifferentDimensions) {
  const hpolytope::HPolytope p_2d = hpolytope::test::MakeP();
  const hpolytope::HPolytope interval = hpolytope::test::MakeInterval();

  EXPECT_THROW(p_2d & interval, std::invalid_argument);
}

TEST(HPolytopeMinkowskiSum, MatchesMpt3) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const hpolytope::HPolytope q = hpolytope::test::MakeQ();
  const hpolytope::HPolytope actual = p.minkowskiSum(q);

  Eigen::MatrixXd A(8, 2);
  A << -1, 0, -1, -1, -1, 1, 0, -1, 0, 1, 1, 1, 1, -1, 1, 0;

  Eigen::VectorXd b(8);
  b << 2, 4, 5, 3, 4, 6, 5, 3;

  const hpolytope::HPolytope expected(A, b);

  EXPECT_TRUE(actual == expected);
}

TEST(HPolytopeMinkowskiSum, SupportIsSumOfOperandSupports) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const hpolytope::HPolytope q = hpolytope::test::MakeQ();
  const Eigen::Vector2d d(1.0, 2.0);

  const hpolytope::HPolytope sum = p + q; // Test the operator spelling here.

  const double actual = sum.supportFunction(d);
  const double expected = p.supportFunction(d) + q.supportFunction(d);

  EXPECT_NEAR(actual, expected, hpolytope::test::kOracleTolerance);
}

TEST(HPolytopeMinkowskiSum, RejectsDifferentDimensions) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const hpolytope::HPolytope interval = hpolytope::test::MakeInterval();
  EXPECT_THROW(p + interval, std::invalid_argument);
}

TEST(HPolytopeMinkowskiSum, RightEmptyOperandProducesEmpty) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const hpolytope::HPolytope empty(2);
  EXPECT_TRUE((p + empty).isEmpty());
}

TEST(HPolytopeMinkowskiSum, LeftEmptyOperandProducesEmpty) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const hpolytope::HPolytope empty(2);
  EXPECT_TRUE((empty + p).isEmpty());
}

TEST(HPolytopePontryaginDifference, MatchesMpt3) {
  const hpolytope::HPolytope t = hpolytope::test::MakeT();
  const hpolytope::HPolytope b = hpolytope::test::MakeB();

  const hpolytope::HPolytope actual = t.pontryaginDiff(b);

  Eigen::MatrixXd A(3, 2);
  A << -1, 0, 0, -1, 1, 1;

  Eigen::VectorXd rhs(3);
  rhs << -0.5, -0.25, 3.25;

  const hpolytope::HPolytope expected(A, rhs);

  EXPECT_TRUE(actual == expected);
}

TEST(HPolytopePontryaginDifference, RejectsDifferentDimensions) {
  const hpolytope::HPolytope t = hpolytope::test::MakeT();
  const hpolytope::HPolytope interval = hpolytope::test::MakeInterval();
  EXPECT_THROW(t - interval, std::invalid_argument);
}

} // namespace

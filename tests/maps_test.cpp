// Copyright (C) 2026 Diego Stéfano Fonseca Ferreira
// SPDX-License-Identifier: GPL-3.0-or-later

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "hpolytope-cpp/hpolytope.hpp"
#include "test_helpers.hpp"
#include "test_polytopes.hpp"

namespace {

TEST(HPolytopeLinearMap, MatchesMpt3) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const Eigen::Matrix2d M = hpolytope::test::MakeM();

  const hpolytope::HPolytope actual = p.linearMap(M);

  Eigen::MatrixXd A(4, 2);
  A << -1, 1, -1, -2, 1, 2, 1, -1;

  Eigen::VectorXd b(4);
  b << 3, 6, 9, 6;

  const hpolytope::HPolytope expected(A, b);

  EXPECT_TRUE(actual == expected);
}

TEST(HPolytopeLinearMap, OperatorMatchesNamedMethod) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const Eigen::Matrix2d M = hpolytope::test::MakeM();

  EXPECT_TRUE((M * p) == p.linearMap(M));
}

TEST(HPolytopeLinearMap, RejectsSingularMapOnHRepresentationPath) {
  const hpolytope::HPolytope p =
      hpolytope::test::MakeP(); // Vertices not cached.

  Eigen::Matrix2d singular;
  singular << 1, 0, 0, 0;

  EXPECT_THROW(p.linearMap(singular), std::invalid_argument);
}

TEST(HPolytopePreImage, MatchesMpt3) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const Eigen::Matrix2d M = hpolytope::test::MakeM();

  const hpolytope::HPolytope actual = p.preImage(M);

  Eigen::MatrixXd A(4, 2);
  A << 2, 1, -2, -1, -1, 1, 1, -1;

  Eigen::VectorXd b(4);
  b << 2, 1, 3, 2;

  const hpolytope::HPolytope expected(A, b);

  EXPECT_TRUE(actual == expected);
}

TEST(HPolytopePreImage, RejectsIncompatibleOutputDimension) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const Eigen::MatrixXd wrong = Eigen::MatrixXd::Zero(3, 2);

  EXPECT_THROW(p.preImage(wrong), std::invalid_argument);
}

TEST(HPolytopeOperators, PointContainmentOperatorMatchesMethod) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const Eigen::Vector2d point(0.0, 0.0);

  EXPECT_EQ(p >= point, p.contains(point));
}

} // namespace
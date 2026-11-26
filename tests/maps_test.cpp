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

TEST(HPolytopeLinearMap, ProjectsRectangleOntoXAxis) {
  const auto p = hpolytope::test::MakeP();

  Eigen::MatrixXd M(1, 2);
  M << 1, 0;

  const auto actual = p.linearMap(M);

  Eigen::MatrixXd A(2, 1);
  A << 1, -1;

  Eigen::VectorXd b(2);
  b << 2, 1;

  const hpolytope::HPolytope expected(A, b);

  EXPECT_EQ(actual.dimension(), 1);
  EXPECT_TRUE(actual == expected);
}

TEST(HPolytopeLinearMap, MapsRectangleToLineSegment) {
  const auto p = hpolytope::test::MakeP();

  Eigen::Matrix2d M;
  M << 1, 0, 0, 0;

  const auto actual = p.linearMap(M);

  Eigen::MatrixXd A(4, 2);
  A << 1, 0, -1, 0, 0, 1, 0, -1;

  Eigen::VectorXd b(4);
  b << 2, 1, 0, 0;

  const hpolytope::HPolytope expected(A, b);

  EXPECT_EQ(actual.dimension(), 2);
  EXPECT_FALSE(actual.isEmpty());
  EXPECT_TRUE(actual == expected);
}

TEST(HPolytopeLinearMap, EmbedsRectangleInThreeDimensions) {
  const auto p = hpolytope::test::MakeP();

  Eigen::MatrixXd M(3, 2);
  M << 1, 0, 0, 1, 0, 0;

  const auto actual = p.linearMap(M);

  Eigen::MatrixXd A(6, 3);
  A << 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1;

  Eigen::VectorXd b(6);
  b << 2, 1, 3, 2, 0, 0;

  const hpolytope::HPolytope expected(A, b);

  EXPECT_EQ(actual.dimension(), 3);
  EXPECT_FALSE(actual.isEmpty());
  EXPECT_TRUE(actual == expected);
}

TEST(HPolytopeLinearMap, ZeroMapProducesSingleton) {
  const auto p = hpolytope::test::MakeP();
  const Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

  const auto actual = p.linearMap(M);

  Eigen::MatrixXd A(4, 2);
  A << 1, 0, -1, 0, 0, 1, 0, -1;

  const Eigen::VectorXd b = Eigen::VectorXd::Zero(4);
  const hpolytope::HPolytope expected(A, b);

  EXPECT_EQ(actual.dimension(), 2);
  EXPECT_FALSE(actual.isEmpty());
  EXPECT_TRUE(actual == expected);
  EXPECT_TRUE(actual.contains(Eigen::Vector2d(0, 0)));
  EXPECT_FALSE(actual.contains(Eigen::Vector2d(1, 0)));
}

TEST(HPolytopeLinearMap, MapsEmptySetToEmptySetInOutputDimension) {
  const hpolytope::HPolytope p(2);

  Eigen::MatrixXd M(3, 2);
  M << 1, 0, 0, 1, 0, 0;

  const auto actual = p.linearMap(M);

  const hpolytope::HPolytope expected(3);

  EXPECT_EQ(actual.dimension(), 3);
  EXPECT_TRUE(actual.isEmpty());
  EXPECT_TRUE(actual == expected);
}

TEST(HPolytopeLinearMap, RejectsIncompatibleInputDimension) {
  const auto p = hpolytope::test::MakeP();

  const Eigen::MatrixXd M = Eigen::MatrixXd::Zero(2, 3);

  EXPECT_THROW(p.linearMap(M), std::invalid_argument);
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
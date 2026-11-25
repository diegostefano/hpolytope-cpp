// Copyright (C) 2026 Diego Stéfano Fonseca Ferreira
// SPDX-License-Identifier: GPL-3.0-or-later

#include <gtest/gtest.h>

#include <stdexcept>

#include <Eigen/Dense>

#include "hpolytope-cpp/hpolytope.hpp"
#include "test_polytopes.hpp"

namespace {

using hpolytope::HPolytope;

TEST(HPolytopeConstruction, CreatesOneDimensionalInterval) {
  HPolytope interval = hpolytope::test::MakeInterval();

  EXPECT_EQ(interval.dimension(), 1.0);
  EXPECT_FALSE(interval.isEmpty());
}

TEST(HPolytopeConstruction, RejectsMismatchedConstraintDimensions) {
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);

  EXPECT_THROW(HPolytope(A, b), std::invalid_argument);
}

TEST(HPolytopeEmptiness, RectangleIsNotEmpty) {
  const auto p = hpolytope::test::MakeP();

  EXPECT_FALSE(p.isEmpty());
}

TEST(HPolytopeEmptiness, ContradictoryInequalitiesGiveEmptySet) {
  const auto empty = hpolytope::test::MakeEmpty2D();

  EXPECT_TRUE(empty.isEmpty());
  EXPECT_EQ(empty.dimension(), 2);
}

TEST(HPolytopeEmptiness, ExplicitEmptyConstructorPreservesDimension) {
  const HPolytope empty(3);

  EXPECT_TRUE(empty.isEmpty());
  EXPECT_EQ(empty.dimension(), 3);
  EXPECT_TRUE(empty.generators().empty());
}

} // namespace

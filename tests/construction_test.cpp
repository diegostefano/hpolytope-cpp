#include <gtest/gtest.h>

#include <stdexcept>

#include <Eigen/Dense>

#include "hpolytope-cpp/hpolytope.hpp"

namespace {

using hpolytope::HPolytope;

TEST(HPolytopeConstruction, CreatesOneDimensionalInterval) {
  Eigen::MatrixXd A(2, 1);
  A << -1.0, 1.0;

  Eigen::VectorXd b(2);
  b << 2.0, 3.0;

  HPolytope interval(A, b);

  EXPECT_EQ(interval.dimension(), 1.0);
  EXPECT_FALSE(interval.isEmpty());
}

TEST(HPolytopeConstruction, RejectsMismatchedConstraintDimensions) {
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);

  EXPECT_THROW(HPolytope(A, b), std::invalid_argument);
}

} // namespace

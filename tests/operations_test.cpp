#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <hpolytope-cpp/hpolytope.hpp>

namespace {

using hpolytope::HPolytope;

constexpr double kTolerance = 1e-8;

TEST(HPolytopeOperations, MatchSupportFunction) {
  Eigen::MatrixXd A(2, 1);
  A << -1.0, 1.0;

  Eigen::VectorXd b(2);
  b << 2.0, 3.0;

  HPolytope interval(A, b);

  Eigen::VectorXd direction(1);
  direction << 1.0;

  EXPECT_NEAR(interval.supportFunction(direction), 3.0, kTolerance);
  EXPECT_NEAR(interval.supportFunction(-direction), 2.0, kTolerance);
}

} // namespace

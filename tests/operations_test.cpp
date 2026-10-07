#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "hpolytope-cpp/hpolytope.hpp"
#include "test_polytopes.hpp"

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

struct SupportCase {
  Eigen::Vector2d direction;
  double expected;
};

class PSupportTest : public ::testing::TestWithParam<SupportCase> {};

TEST_P(PSupportTest, MatchesMpt3) {
  hpolytope::HPolytope p{hpolytope::test::MakeP()};
  const auto &test_case = GetParam();

  EXPECT_NEAR(p.supportFunction(test_case.direction), test_case.expected, 1e-8);
}

INSTANTIATE_TEST_SUITE_P(
    Mpt3Reference, PSupportTest,
    ::testing::Values(SupportCase{Eigen::Vector2d{1.0, 0.0}, 2.0},
                      SupportCase{Eigen::Vector2d{-1.0, 0.0}, 1.0},
                      SupportCase{Eigen::Vector2d{0.0, 1.0}, 3.0},
                      SupportCase{Eigen::Vector2d{0.0, -1.0}, 2.0},
                      SupportCase{Eigen::Vector2d{1.0, 2.0}, 8.0},
                      SupportCase{Eigen::Vector2d{-2.0, 1.0}, 5.0}));

class QSupportTest : public ::testing::TestWithParam<SupportCase> {};

TEST_P(QSupportTest, MatchesMpt3) {
  hpolytope::HPolytope q{hpolytope::test::MakeQ()};
  const auto &test_case = GetParam();

  EXPECT_NEAR(q.supportFunction(test_case.direction), test_case.expected, 1e-8);
}

INSTANTIATE_TEST_SUITE_P(
    Mpt3Reference, QSupportTest,
    ::testing::Values(SupportCase{Eigen::Vector2d{1.0, 1.0}, 1.0},
                      SupportCase{Eigen::Vector2d{2.0, -3.0}, 3.0},
                      SupportCase{Eigen::Vector2d{-4.0, 0.5}, 4.0}));

TEST(HPolytopeSupport, RejectsDirectionWithWrongDimension) {
  const auto p = hpolytope::test::MakeP();
  const Eigen::Vector3d direction(1.0, 0.0, 0.0);

  EXPECT_THROW(p.supportFunction(direction), std::invalid_argument);
}

} // namespace

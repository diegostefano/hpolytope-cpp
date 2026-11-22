#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "hpolytope-cpp/hpolytope.hpp"
#include "test_helpers.hpp"
#include "test_polytopes.hpp"

namespace {

TEST(HPolytopeSupport, IntervalPositiveDirectionMatchesMpt3) {
  const hpolytope::HPolytope interval = hpolytope::test::MakeInterval();
  const Eigen::VectorXd direction = Eigen::VectorXd::Constant(1, 1.0);

  const double actual_right = interval.supportFunction(direction);

  EXPECT_NEAR(actual_right, 3.0, hpolytope::test::kOracleTolerance);
}

TEST(HPolytopeSupport, IntervalNegativeDirectionMatchesMpt3) {
  const hpolytope::HPolytope interval = hpolytope::test::MakeInterval();
  const Eigen::VectorXd direction = Eigen::VectorXd::Constant(1, -1.0);

  const double actual_left = interval.supportFunction(direction);

  EXPECT_NEAR(actual_left, 2.0, hpolytope::test::kOracleTolerance);
}

struct SupportCase2D {
  Eigen::Vector2d direction;
  double expected;
};

class RectangleSupportTest : public ::testing::TestWithParam<SupportCase2D> {};

TEST_P(RectangleSupportTest, MatchesMpt3) {
  const auto p = hpolytope::test::MakeP();
  const auto &sample = GetParam();

  const double actual = p.supportFunction(sample.direction);
  EXPECT_NEAR(actual, sample.expected, hpolytope::test::kOracleTolerance);
}

INSTANTIATE_TEST_SUITE_P(
    Mpt3, RectangleSupportTest,
    ::testing::Values(SupportCase2D{Eigen::Vector2d{1.0, 0.0}, 2.0},
                      SupportCase2D{Eigen::Vector2d{-1.0, 0.0}, 1.0},
                      SupportCase2D{Eigen::Vector2d{0.0, 1.0}, 3.0},
                      SupportCase2D{Eigen::Vector2d{0.0, -1.0}, 2.0},
                      SupportCase2D{Eigen::Vector2d{1.0, 2.0}, 8.0},
                      SupportCase2D{Eigen::Vector2d{-2.0, 1.0}, 5.0}));

class DiamondSupportTest : public ::testing::TestWithParam<SupportCase2D> {};

TEST_P(DiamondSupportTest, MatchesMpt3) {
  const hpolytope::HPolytope diamond = hpolytope::test::MakeQ();
  const SupportCase2D &sample = GetParam();

  const double actual = diamond.supportFunction(sample.direction);
  EXPECT_NEAR(actual, sample.expected, hpolytope::test::kOracleTolerance);
}

INSTANTIATE_TEST_SUITE_P(
    Mpt3, DiamondSupportTest,
    ::testing::Values(SupportCase2D{Eigen::Vector2d{1, 1}, 1},
                      SupportCase2D{Eigen::Vector2d{2, -3}, 3},
                      SupportCase2D{Eigen::Vector2d{-4, 0.5}, 4}));

TEST(HPolytopeSupport, RejectsDirectionWithWrongDimension) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();
  const Eigen::Vector3d wrong_direction(1.0, 0.0, 0.0);

  EXPECT_THROW(p.supportFunction(wrong_direction), std::invalid_argument);
}

TEST(HPolytopeSupport, CachedAndUncachedPathsAgree) {
  hpolytope::HPolytope p = hpolytope::test::MakeP();
  const Eigen::Vector2d direction(1.0, 2.0);

  const double before = p.supportFunction(direction);
  (void)p.generators();
  const double after = p.supportFunction(direction);

  EXPECT_NEAR(before, 8.0, hpolytope::test::kOracleTolerance);
  EXPECT_NEAR(after, 8.0, hpolytope::test::kOracleTolerance);
}
}; // namespace

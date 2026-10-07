#include <gtest/gtest.h>

#include "hpolytope-cpp/hpolytope.hpp"
#include "test_polytopes.hpp"

namespace {

class Core2DTest : public ::testing::Test {
protected:
  hpolytope::HPolytope p_{hpolytope::test::MakeP()};
  hpolytope::HPolytope q_{hpolytope::test::MakeQ()};
};

TEST_F(Core2DTest, ContainsInteriorPoint) {
  Eigen::Vector2d point(0.0, 0.0);

  EXPECT_TRUE(p_.contains(point));
}

TEST_F(Core2DTest, ContainsBoundaryPoint) {
  Eigen::Vector2d point(2.0, 3.0);

  EXPECT_TRUE(p_.contains(point));
}

TEST_F(Core2DTest, RejectsOutsidePoint) {
  Eigen::Vector2d point(2.0001, 3.0);

  EXPECT_FALSE(p_.contains(point));
}

TEST_F(Core2DTest, ContainsDiamond) { EXPECT_TRUE(p_.contains(q_)); }

TEST_F(Core2DTest, DiamondDoesNotContainRectangle) {
  EXPECT_FALSE(q_.contains(p_));
}

TEST_F(Core2DTest, RejectsPointWithWrongDimension) {
  Eigen::Vector3d point(0.0, 0.0, 0.0);

  EXPECT_THROW(p_.contains(point), std::invalid_argument);
}

TEST(HPolytopeEmptiness, DetectsFeasiblePolytope) {
  hpolytope::HPolytope p{hpolytope::test::MakeP()};

  EXPECT_FALSE(p.isEmpty());
}

TEST(HPolytopeEmptiness, DetectsInfeasiblePolytope) {
  hpolytope::HPolytope empty{hpolytope::test::MakeEmpty2D()};

  EXPECT_TRUE(empty.isEmpty());
  EXPECT_EQ(empty.dimension(), 2);
}

TEST(HPolytopeEmptiness, ExplicitEmptyConstructorPreservesDimension) {
  hpolytope::HPolytope empty(3);

  EXPECT_TRUE(empty.isEmpty());
  EXPECT_EQ(empty.dimension(), 3);
  EXPECT_TRUE(empty.generators().empty());
}

} // namespace

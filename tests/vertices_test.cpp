#include "test_helpers.hpp"
#include "test_polytopes.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

TEST(HPolytopeVertices, RectangleMatchesMpt3) {
  const hpolytope::HPolytope p = hpolytope::test::MakeP();

  const std::vector<Eigen::VectorXd> expected{
      Eigen::Vector2d{-1.0, -2.0},
      Eigen::Vector2d{-1.0, 3.0},
      Eigen::Vector2d{2.0, -2.0},
      Eigen::Vector2d{2.0, 3.0},
  };

  const std::vector<Eigen::VectorXd> actual = p.generators();

  hpolytope::test::ExpectVerticesNearUnordered(actual, expected);
}

TEST(HPolytopeVertices, IntervalMatchesMpt3) {
  const hpolytope::HPolytope interval = hpolytope::test::MakeInterval();

  const std::vector<Eigen::VectorXd> expected{
      Eigen::VectorXd::Constant(1, -2.0), Eigen::VectorXd::Constant(1, 3.0)};

  const std::vector<Eigen::VectorXd> actual = interval.generators();

  hpolytope::test::ExpectVerticesNearUnordered(actual, expected);
}

TEST(HPolytopeVertices, Box3DMatchesMpt3) {
  const auto box = hpolytope::test::MakeB3();

  const std::vector<Eigen::VectorXd> expected{
      Eigen::Vector3d{-1.0, 0.0, -2.0}, Eigen::Vector3d{-1.0, 0.0, 3.0},
      Eigen::Vector3d{-1.0, 2.0, -2.0}, Eigen::Vector3d{-1.0, 2.0, 3.0},
      Eigen::Vector3d{1.0, 0.0, -2.0},  Eigen::Vector3d{1.0, 0.0, 3.0},
      Eigen::Vector3d{1.0, 2.0, -2.0},  Eigen::Vector3d{1.0, 2.0, 3.0},
  };

  const std::vector<Eigen::VectorXd> actual = box.generators();

  hpolytope::test::ExpectVerticesNearUnordered(actual, expected);
}

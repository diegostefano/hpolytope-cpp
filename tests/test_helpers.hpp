// Copyright (C) 2026 Diego Stéfano Fonseca Ferreira
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HPOLYTOPE__TESTS__TEST_HELPERS_HPP
#define HPOLYTOPE__TESTS__TEST_HELPERS_HPP

#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace hpolytope::test {

inline constexpr double kOracleTolerance = 1e-8;

inline bool VectorsNear(const Eigen::VectorXd &a, const Eigen::VectorXd &b,
                        double tolerance = kOracleTolerance) {
  if (a.size() != b.size())
    return false;

  return (a - b).lpNorm<Eigen::Infinity>() <= tolerance;
}

inline void
ExpectVerticesNearUnordered(const std::vector<Eigen::VectorXd> &actual,
                            const std::vector<Eigen::VectorXd> &expected,
                            double tolerance = kOracleTolerance) {
  ASSERT_EQ(actual.size(), expected.size()) << "Unexpected number of vertices";

  std::vector<bool> used(expected.size(), false);

  for (std::size_t i = 0; i < actual.size(); ++i) {
    bool found = false;
    for (std::size_t j = 0; j < expected.size(); ++j) {
      if (!used[j] && VectorsNear(actual[i], expected[j], tolerance)) {
        used[j] = true;
        found = true;

        break;
      }
    }

    EXPECT_TRUE(found) << "Unexpected/unmatched vertex " << i << ": "
                       << actual[i].transpose();
  }
}

} // namespace hpolytope::test

#endif

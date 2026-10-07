#pragma once

#include <Eigen/Dense>

#include "hpolytope-cpp/hpolytope.hpp"

namespace hpolytope::test {

inline HPolytope MakeP() {
  Eigen::MatrixXd A(4, 2);
  A << 1, 0, -1, 0, 0, 1, 0, -1;

  Eigen::VectorXd b(4);
  b << 2, 1, 3, 2;

  return HPolytope(A, b);
}

inline HPolytope MakeQ() {
  Eigen::MatrixXd A(4, 2);
  A << 1, 1, 1, -1, -1, 1, -1, -1;

  Eigen::VectorXd b = Eigen::VectorXd::Ones(4);

  return HPolytope(A, b);
}

inline HPolytope MakeS() {
  // S = [0,4] x [-3,1]
  Eigen::MatrixXd A(4, 2);
  A << -1, 0, 1, 0, 0, -1, 0, 1;

  Eigen::VectorXd b(4);
  b << 0, 4, 3, 1;

  return HPolytope(A, b);
}

inline HPolytope MakeT() {
  Eigen::MatrixXd A(3, 2);
  A << -1, 0, 0, -1, 1, 1;

  Eigen::VectorXd b(3);
  b << 0, 0, 4;

  return HPolytope(A, b);
}

inline HPolytope MakeB() {
  Eigen::MatrixXd A(4, 2);
  A << -1, 0, 1, 0, 0, -1, 0, 1;

  Eigen::VectorXd b(4);
  b << 0.5, 0.5, 0.25, 0.25;

  return HPolytope(A, b);
}

inline HPolytope MakeEmpty2D() {
  Eigen::MatrixXd A(2, 2);
  A << 1, 0, -1, 0;

  Eigen::VectorXd b(2);
  b << 0, -1;

  return HPolytope(A, b);
}

inline HPolytope MakeRedundantP() {
  Eigen::MatrixXd A(9, 2);
  A << 0, -1, 2, 0, 0, 1, -1, 0, 1, 0, 0, 3, 1, 0, -1, 0, 0, -1;

  Eigen::VectorXd b(9);
  b << 2, 4, 3, 1, 2, 9, 10, 5, 20;

  return HPolytope(A, b);
}

inline HPolytope MakeB3() {
  Eigen::MatrixXd A(6, 3);
  A << -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1;

  Eigen::VectorXd b(6);
  b << 1, 1, 0, 2, 2, 3;

  return HPolytope(A, b);
}

} // namespace hpolytope::test

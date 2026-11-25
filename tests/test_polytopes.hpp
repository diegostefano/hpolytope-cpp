// Copyright (C) 2026 Diego Stéfano Fonseca Ferreira
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HPOLYTOPES__TESTS__TEST_POLYTOPES_HPP
#define HPOLYTOPES__TESTS__TEST_POLYTOPES_HPP

#include <Eigen/Dense>

#include "hpolytope-cpp/hpolytope.hpp"

namespace hpolytope::test {

// I = [-2,3]
inline HPolytope MakeInterval() {
  Eigen::MatrixXd A(2, 1);
  A << -1, 1;

  Eigen::VectorXd b(2);
  b << 2, 3;

  return HPolytope(A, b);
}

// P = [-1,2] x [-2,3]
inline HPolytope MakeP() {
  Eigen::MatrixXd A(4, 2);
  A << 1, 0, -1, 0, 0, 1, 0, -1;

  Eigen::VectorXd b(4);
  b << 2, 1, 3, 2;

  return HPolytope(A, b);
}

// Q = { (x,y) : |x| + |y| <= 1 }
inline HPolytope MakeQ() {
  Eigen::MatrixXd A(4, 2);
  A << 1, 1, 1, -1, -1, 1, -1, -1;

  Eigen::VectorXd b = Eigen::VectorXd::Ones(4);

  return HPolytope(A, b);
}

// S = [0,4] x [-3,1]
inline HPolytope MakeS() {
  Eigen::MatrixXd A(4, 2);
  A << -1, 0, 1, 0, 0, -1, 0, 1;

  Eigen::VectorXd b(4);
  b << 0, 4, 3, 1;

  return HPolytope(A, b);
}

// T = {(x,y): x>=0, y>=0, x+y<=4}
inline HPolytope MakeT() {
  Eigen::MatrixXd A(3, 2);
  A << -1, 0, 0, -1, 1, 1;

  Eigen::VectorXd b(3);
  b << 0, 0, 4;

  return HPolytope(A, b);
}

// B = [-0.5,0.5] x [-0.25,0.25]
inline HPolytope MakeB() {
  Eigen::MatrixXd A(4, 2);
  A << -1, 0, 1, 0, 0, -1, 0, 1;

  Eigen::VectorXd b(4);
  b << 0.5, 0.5, 0.25, 0.25;

  return HPolytope(A, b);
}

// E: x<=0 and x>=1; no feasible point exists.
inline HPolytope MakeEmpty2D() {
  Eigen::MatrixXd A(2, 2);
  A << 1, 0, -1, 0;

  Eigen::VectorXd b(2);
  b << 0, -1;

  return HPolytope(A, b);
}

// Same mathematical set as P, but with nine redundant/scaled rows.
inline HPolytope MakeRedundantP() {
  Eigen::MatrixXd A(9, 2);
  A << 0, -1, 2, 0, 0, 1, -1, 0, 1, 0, 0, 3, 1, 0, -1, 0, 0, -1;

  Eigen::VectorXd b(9);
  b << 2, 4, 3, 1, 2, 9, 10, 5, 20;

  return HPolytope(A, b);
}

// B3 = [-1,1] x [0,2] x [-2,3]
inline HPolytope MakeB3() {
  Eigen::MatrixXd A(6, 3);
  A << -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1;

  Eigen::VectorXd b(6);
  b << 1, 1, 0, 2, 2, 3;

  return HPolytope(A, b);
}

// Shared matrix for the linear-map and preimage MPT3 references.
inline Eigen::Matrix2d MakeM() {
  Eigen::Matrix2d M;
  M << 2, 1, -1, 1;
  return M;
}

} // namespace hpolytope::test

#endif
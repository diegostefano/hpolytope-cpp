// Copyright (C) 2026 Diego Stéfano Fonseca Ferreira
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hpolytope-cpp/hpolytope.hpp"

#include "cddlib/adapter.hpp"
#include "cddlib/linear_programming.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <stdexcept>

namespace hpolytope {

HPolytope::HPolytope(size_t dimension) {
  A_ = Eigen::MatrixXd::Zero(1, dimension);
  b_ = Eigen::VectorXd::Constant(1, -1.0);

  cached_emptiness_flag_ = true;
  cached_generators_ = std::vector<Eigen::VectorXd>{};
}

HPolytope::HPolytope(const Eigen::MatrixXd &A, const Eigen::VectorXd &b) {
  if (A.rows() != b.size()) {
    throw std::invalid_argument(
        "Incompatible dimensions: cannot create HPolytope.");
  }

  A_ = A;
  b_ = b;
}

bool HPolytope::isEmpty() const {
  if (cached_emptiness_flag_) {
    return *cached_emptiness_flag_;
  }

  if (cached_generators_) {
    cached_emptiness_flag_ = cached_generators_->empty();
    return *cached_emptiness_flag_;
  }

  cddlib::LinearProgramming lp(dd_LPmin, dimension(), A_.rows());
  lp.setConstraints(A_, b_);
  lp.setCost(Eigen::VectorXd::Zero(dimension()), 0.0);
  lp.solve();

  if (lp.Ok()) {
    cached_emptiness_flag_ = false;
    return false;
  }

  if (lp.Infeasible()) {
    cached_emptiness_flag_ = true;
    return true;
  }

  throw std::runtime_error("Unable to determine polytope emptiness from LP.");
}

HPolytope HPolytope::normalized() const {
  Eigen::VectorXd norms = A_.rowwise().norm();

  Eigen::MatrixXd A_norm = A_.array().colwise() / norms.array();
  Eigen::VectorXd b_norm = b_.array() / norms.array();

  return HPolytope(A_norm, b_norm);
}

bool HPolytope::contains(const Eigen::VectorXd &point) const {
  if (point.size() != dimension()) {
    throw std::invalid_argument("contains: invalid point dimension!");
  }

  const HPolytope norm_poly = normalized();

  return ((norm_poly.A() * point).array() <= norm_poly.b().array() + kTolerance)
      .all();
}

bool HPolytope::contains(const HPolytope &poly) const {
  if (dimension() != poly.dimension()) {
    throw std::invalid_argument("contains: incompatible dimensions.");
  }

  if (poly.isEmpty())
    return true;

  const HPolytope norm_poly = normalized();

  if (poly.hasCachedGenerators()) {
    return std::all_of(poly.cached_generators_->begin(),
                       poly.cached_generators_->end(),
                       [&](const Eigen::VectorXd &vertice) {
                         return ((norm_poly.A() * vertice).array() <=
                                 norm_poly.b().array() + kTolerance)
                             .all();
                       });
  }

  cddlib::LinearProgramming lp(dd_LPmax, poly.dimension(), poly.A_.rows());
  lp.setConstraints(poly.A_, poly.b_);

  for (Eigen::Index i = 0; i < A_.rows(); ++i) {
    lp.setCost(A_.row(i).transpose(), 0.0);
    const double max_value = lp.solve();

    if (lp.Infeasible()) {
      throw std::runtime_error("Unexpected infeasible LP");
    }

    if (lp.Unbounded()) {
      return false;
    }

    if (!lp.Ok()) {
      throw std::runtime_error("LP solver failure");
    }

    if (max_value > b_(i) + kTolerance)
      return false;
  }

  return true;
}

std::vector<Eigen::VectorXd> HPolytope::generators() const {
  if (!cached_generators_.has_value()) {
    cached_generators_ = cddlib::verticesFromInequalities(A_, b_);
  }

  return *cached_generators_;
}

HPolytope HPolytope::minimumRepresentation() const {
  auto [minimal_A, minimal_b] = cddlib::canonicalizeInequalities(A_, b_);

  return HPolytope(minimal_A, minimal_b);
}

double HPolytope::supportFunction(const Eigen::VectorXd &direction) const {
  if (direction.size() != dimension()) {
    throw std::invalid_argument(
        "Invalid direction dimensionality for support function!");
  }

  if (cached_generators_.has_value()) {
    double support = -std::numeric_limits<double>::infinity();

    for (const auto &vertex : *cached_generators_) {
      support = std::max(support, direction.dot(vertex));
    }

    return support;
  }

  cddlib::LinearProgramming lp(dd_LPmax, dimension(), A_.rows());
  lp.setConstraints(A_, b_);
  lp.setCost(direction, 0.0);

  const double result = lp.solve();

  if (lp.Ok())
    return result;

  if (lp.Infeasible())
    return -std::numeric_limits<double>::infinity();

  if (lp.Unbounded())
    return std::numeric_limits<double>::infinity();

  throw std::runtime_error("Unable to evaluate support function.");
}

HPolytope HPolytope::minkowskiSum(const HPolytope &rhs) const {
  if (dimension() != rhs.dimension()) {
    throw std::invalid_argument("Minkowski sum requires both polytopes to "
                                "have the same dimensionality.");
  }

  if (isEmpty() || rhs.isEmpty()) {
    return HPolytope(dimension());
  }

  const auto &left_generators = generators();
  const auto &right_generators = rhs.generators();

  std::vector<Eigen::VectorXd> output;
  output.reserve(left_generators.size() * right_generators.size());

  for (const Eigen::VectorXd &v : left_generators) {
    for (const Eigen::VectorXd &w : right_generators) {
      output.emplace_back(v + w);
    }
  }

  const auto &[A, b] = cddlib::inequalitiesFromVertices(output);
  HPolytope result(A, b);
  result.cached_generators_ = output;

  return result;
}

HPolytope HPolytope::pontryaginDiff(const HPolytope &rhs) const {
  if (dimension() != rhs.dimension()) {
    throw std::invalid_argument("Minkowski sum requires both polytopes to "
                                "have the same dimensionality.");
  }

  Eigen::MatrixXd A;
  A = A_;

  Eigen::VectorXd b;
  b.resizeLike(b_);

  for (Eigen::Index row = 0; row < A_.rows(); ++row) {
    b(row) = b_(row) - rhs.supportFunction(A_.row(row).transpose());
  }

  return HPolytope(A, b);
}

HPolytope HPolytope::linearMap(const Eigen::MatrixXd &M) const {
  if (M.cols() != dimension()) {
    throw std::invalid_argument(
        "Inconsistent dimension for linearMap input matrix.");
  }

  if (isEmpty()) {
    return HPolytope(M.rows());
  }

  if (M.rows() == M.cols()) {
    Eigen::FullPivLU<Eigen::MatrixXd> lu(M.transpose());

    if (lu.isInvertible()) {
      Eigen::MatrixXd A_new = lu.solve(A_.transpose()).transpose();

      return HPolytope(A_new, b_);
    }
  }

  const auto original_generators = generators();

  std::vector<Eigen::VectorXd> new_generators;
  new_generators.reserve(original_generators.size());

  for (const auto &v : original_generators) {
    new_generators.emplace_back(M * v);
  }

  const auto &[A, b] = cddlib::inequalitiesFromVertices(new_generators);

  HPolytope result(A, b);
  result.cached_generators_ = std::move(new_generators);

  return result;
}

HPolytope HPolytope::preImage(const Eigen::MatrixXd &M) const {
  if (M.rows() != dimension()) {
    throw std::invalid_argument(
        "Inconsistent dimension for preImage input matrix.");
  }

  return HPolytope(A_ * M, b_);
}

bool HPolytope::operator==(const HPolytope &rhs) const {
  if (dimension() != rhs.dimension())
    return false;

  if (isEmpty())
    return rhs.isEmpty();

  if (rhs.isEmpty())
    return false;

  return contains(rhs) && rhs.contains(*this);
}

bool HPolytope::operator>=(const Eigen::VectorXd &point) const {
  return this->contains(point);
}

bool HPolytope::operator>=(const HPolytope &poly) const {
  return this->contains(poly);
}

HPolytope HPolytope::operator+(const HPolytope &rhs) const {
  return minkowskiSum(rhs);
}

HPolytope HPolytope::operator-(const HPolytope &rhs) const {
  return pontryaginDiff(rhs);
}

HPolytope HPolytope::operator&(const HPolytope &rhs) const {
  if (dimension() != rhs.dimension()) {
    throw std::invalid_argument("Incompatible dimensions for instersection.");
  }

  Eigen::MatrixXd A_new(A_.rows() + rhs.A_.rows(), dimension());

  Eigen::VectorXd b_new(b_.size() + rhs.b_.size());

  A_new << A_, rhs.A_;
  b_new << b_, rhs.b_;

  return HPolytope(A_new, b_new);
}

HPolytope operator*(const Eigen::MatrixXd &lhs, const HPolytope &rhs) {
  return rhs.linearMap(lhs);
}

} // namespace hpolytope

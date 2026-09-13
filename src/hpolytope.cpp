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

  is_empty_ = true;
  cached_vertices_ = std::vector<Eigen::VectorXd>{};
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
  if (is_empty_.has_value()) {
    return *is_empty_;
  }

  if (cached_vertices_.has_value()) {
    return cached_vertices_->empty();
  }

  cddlib::LinearProgramming lp(dd_LPmin, dimension(), A_.rows());
  lp.setConstraints(A_, b_);
  lp.setCost(Eigen::VectorXd::Zero(dimension()), 0.0);
  lp.solve();

  if (lp.Ok()) {
    is_empty_ = false;
    return false;
  }

  if (lp.Infeasible()) {
    is_empty_ = true;
    return true;
  }

  throw std::runtime_error("Unable to determine polytope emptiness from LP.");
}

bool HPolytope::contains(const Eigen::VectorXd &point) const {
  if (point.size() != dimension()) {
    throw std::invalid_argument("contains: invalid point dimension!");
  }

  return ((A_ * point).array() <= b_.array()).all();
}

bool HPolytope::contains(const HPolytope &poly) const {
  if (poly.isEmpty())
    return true;

  if (poly.hasCachedVertices()) {
    return std::all_of(poly.cached_vertices_->begin(),
                       poly.cached_vertices_->end(),
                       [&](const Eigen::VectorXd &vertice) {
                         return ((A_ * vertice).array() <= b_.array()).all();
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

std::vector<Eigen::VectorXd> HPolytope::vertices() const {
  if (!cached_vertices_.has_value()) {
    cached_vertices_ = cddlib::verticesFromInequalities(A_, b_);
  }

  return *cached_vertices_;
}

HPolytope HPolytope::minimumRepresentation() const {
  if (cached_vertices_.has_value()) {
    auto [minimal_A, minimal_b] =
        cddlib::inequalitiesFromVertices(cached_vertices_.value());
    return HPolytope(minimal_A, minimal_b);
  }

  auto [minimal_A, minimal_b] = cddlib::canonicalizeInequalities(A_, b_);
  return HPolytope(minimal_A, minimal_b);
}

double HPolytope::supportFunction(const Eigen::VectorXd &direction) const {
  if (direction.size() != dimension()) {
    throw std::invalid_argument(
        "Invalid direction dimensionality for support function!");
  }

  if (cached_vertices_.has_value()) {
    double support = -std::numeric_limits<double>::infinity();

    for (const auto &vertex : *cached_vertices_) {
      support = std::max(support, direction.dot(vertex));
    }

    return support;
  }

  cddlib::LinearProgramming lp(dd_LPmax, dimension(), A_.rows());
  lp.setConstraints(A_, b_);
  lp.setCost(direction, 0.0);

  return lp.solve();
}

HPolytope HPolytope::minkowskiSum(const HPolytope &rhs) const {
  if (dimension() != rhs.dimension()) {
    throw std::invalid_argument("Minkowski sum requires both polytopes to "
                                "have the same dimensionality.");
  }

  if (isEmpty() || rhs.isEmpty()) {
    return HPolytope(dimension());
  }

  const auto &left_vertices = vertices();
  const auto &right_vertices = rhs.vertices();

  std::vector<Eigen::VectorXd> output;
  output.reserve(left_vertices.size() * right_vertices.size());

  for (const Eigen::VectorXd &v : left_vertices) {
    for (const Eigen::VectorXd &w : right_vertices) {
      output.emplace_back(v + w);
    }
  }

  const auto &[A, b] = cddlib::inequalitiesFromVertices(output);
  HPolytope result(A, b);
  result.cached_vertices_ = output;

  return result;
}

HPolytope HPolytope::pontryaginDiff(const HPolytope &rhs) const {
  if (dimension() != rhs.dimension()) {
    throw std::invalid_argument("Minkowski sum requires both polytopes to "
                                "have the same dimensionality.");
  }

  if (rhs.isEmpty()) {
    return *this;
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
  if (cached_vertices_.has_value()) {
    if (M.cols() != dimension()) {
      throw std::invalid_argument(
          "Inconsistent dimension for linearMap input matrix.");
    }

    std::vector<Eigen::VectorXd> new_vertices;
    std::transform(cached_vertices_->begin(), cached_vertices_->end(),
                   std::back_inserter(new_vertices),
                   [&](const Eigen::VectorXd &v) { return M * v; });

    const auto &[A, b] = cddlib::inequalitiesFromVertices(new_vertices);
    HPolytope mapped_hpolytope(A, b);
    mapped_hpolytope.cached_vertices_ = new_vertices;

    return mapped_hpolytope;
  }

  if (M.cols() != dimension() || M.rows() != dimension() ||
      std::abs(M.determinant()) < kTolerance) {
    throw std::invalid_argument("linearMap requires map matrix to be square "
                                "and invertible for H-representation.");
  }

  Eigen::MatrixXd A_new = A_ * M.inverse();

  return HPolytope(A_new, b_);
}

HPolytope HPolytope::preImage(const Eigen::MatrixXd &M) const {
  if (M.cols() != dimension()) {
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

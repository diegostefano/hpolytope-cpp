#include "hpolytope-cpp/hpolytope.hpp"

#include "cddlib/linear_programming.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <stdexcept>

namespace hpolytope {

HPolytope::HPolytope(const Eigen::MatrixXd &A, const Eigen::VectorXd &b) {}

bool HPolytope::isEmpty() const {
  if (is_empty_.value_or(false)) {
    return true;
  }

  if (cached_vertices_.has_value()) {
    return cached_vertices_->empty();
  }

  cddlib::LinearProgramming lp(dd_LPmin, A_.cols(), A_.rows());
  lp.setConstraints(A_, b_);
  lp.setCost(Eigen::VectorXd::Zero(A_.cols()), 0.0);
  lp.solve();

  return lp.Ok();
}

bool HPolytope::contains(const Eigen::VectorXd &point) {
  if (point.size() != A_.cols())
    std::invalid_argument("contains: invalid point dimension!");

  return ((A_ * point).array() <= b_.array()).all();
}

bool HPolytope::contains(const HPolytope &poly) {
  if (poly.isEmpty())
    return true;

  if (poly.hasCachedVertices()) {
    return std::all_of(poly.cached_vertices_->begin(),
                       poly.cached_vertices_->end(),
                       [&](const Eigen::VectorXd &vertice) {
                         return ((A_ * vertice).array() <= b_.array()).all();
                       });
  }

  cddlib::LinearProgramming lp(dd_LPmax, A_.cols(), poly.A_.rows());
  lp.setConstraints(poly.A_, poly.b_);

  for (Eigen::Index i = 0; i < A_.rows(); ++i) {
    lp.setCost(A_.row(i).transpose(), 0.0);
    const double max_value = lp.solve();

    if (max_value > b_(i) + kTolerance)
      return false;
  }

  return false;
}

std::vector<Eigen::VectorXd> vertices() {
  // TODO: if there are cached vertices return it;
  // if not, perform vertex enumeration over the V-rep.
  return {};
}

HPolytope HPolytope::minimumRepresentation() {
  // TODO: if there are cached vertices, return hull;
  // if not, remove redundancies using LPs.
  return *this;
}

double HPolytope::supportFunction(const Eigen::VectorXd &direction) {
  // TODO: solve the LP.
  return 0.0;
}

HPolytope HPolytope::minkowskySum(const HPolytope &rhs) {
  // TODO: if there are cached vertices, just use it;
  // if not, generate them and use it.
  return *this;
}

HPolytope HPolytope::pontryaginDiff(const HPolytope &rhs) {
  // TODO: use the support function method.
  return *this;
}

HPolytope HPolytope::linearMap(const Eigen::MatrixXd &A) const {
  // TODO: use H-rep directly.
  return *this;
}

HPolytope HPolytope::preImage(const Eigen::MatrixXd &A) {
  // TODO: use H-rep directly.
  return *this;
}

bool HPolytope::operator>(const Eigen::VectorXd &point) {
  return this->contains(point);
}

bool HPolytope::operator>(const HPolytope &poly) {
  return this->contains(poly);
}

HPolytope HPolytope::operator+(const HPolytope &rhs) {
  return minkowskySum(rhs);
}

HPolytope HPolytope::operator-(const HPolytope &rhs) {
  return pontryaginDiff(rhs);
}

HPolytope HPolytope::operator&(const HPolytope &rhs) {
  // TODO: stack inequalities and calculate min-rep.
  return *this;
}

HPolytope HPolytope::operator|(const HPolytope &rhs) {
  // TODO: calculate directly if V-rep is available;
  // if not, generate it and calculate.
  return *this;
}

HPolytope operator*(const Eigen::MatrixXd &lhs, const HPolytope &rhs) {
  return rhs.linearMap(lhs);
}

} // namespace hpolytope

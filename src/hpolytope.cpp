#include "hpolytope.hpp"

HPolytope::HPolytope(const Eigen::MatrixXd &A, const Eigen::VectorXd &b) {}

bool HPolytope::isEmpty() const {
  // TODO: test emptiness verifying wether a cached
  // emptiness flag already exists. If it doesn't,
  // use V-representation if available; if not,
  // make the LP test with the H-representation.
  return false;
}

bool HPolytope::contains(const Eigen::VectorXd &point) {
  // TODO: use H-rep directly here.
  return false;
}

bool HPolytope::contains(const HPolytope &poly) {
  // TODO: If a cached V-rep exists for the smaller,
  // use the H-rep inequalities to verify containment;
  // if not, solve a bunch of LPs using H-rep from
  // both.
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

#ifndef __HPOLYTOPES__POLYTOPE_HPP__
#define __HPOLYTOPES__POLYTOPE_HPP__

#include <optional>

#include <Eigen/Dense>

namespace hpolytope {

class HPolytope {
public:
  HPolytope() = delete;
  HPolytope(size_t);
  HPolytope(const Eigen::MatrixXd &A, const Eigen::VectorXd &b);

  bool isEmpty() const;
  bool contains(const Eigen::VectorXd &point) const;
  bool contains(const HPolytope &poly) const;

  bool hasCachedGenerators() const { return cached_generators_.has_value(); }
  std::vector<Eigen::VectorXd> generators() const;
  HPolytope minimumRepresentation() const;
  double supportFunction(const Eigen::VectorXd &direction) const;

  HPolytope minkowskiSum(const HPolytope &rhs) const;
  HPolytope pontryaginDiff(const HPolytope &rhs) const;
  HPolytope linearMap(const Eigen::MatrixXd &A) const;
  HPolytope preImage(const Eigen::MatrixXd &A) const;

  bool operator==(const HPolytope &rhs) const;
  bool operator>=(const Eigen::VectorXd &point) const;
  bool operator>=(const HPolytope &poly) const;
  HPolytope operator+(const HPolytope &rhs) const;
  HPolytope operator-(const HPolytope &rhs) const;
  HPolytope operator&(const HPolytope &rhs) const;
  friend HPolytope operator*(const Eigen::MatrixXd &lhs, const HPolytope &rhs);

  Eigen::Index dimension() const { return A_.cols(); }

private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;

  mutable std::optional<std::vector<Eigen::VectorXd>> cached_generators_{
      std::nullopt};
  mutable std::optional<bool> cached_emptiness_flag_{std::nullopt};

  static constexpr double kTolerance = 1e-6;
};

} // namespace hpolytope

#endif // __HPOLYTOPES__POLYTOPE_HPP__

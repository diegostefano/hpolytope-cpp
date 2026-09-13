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
  bool contains(const Eigen::VectorXd &point);
  bool contains(const HPolytope &poly);

  bool hasCachedVertices() const { return cached_vertices_.has_value(); }
  std::vector<Eigen::VectorXd> vertices() const;
  HPolytope minimumRepresentation();
  double supportFunction(const Eigen::VectorXd &direction);

  HPolytope minkowskiSum(const HPolytope &rhs) const;
  HPolytope pontryaginDiff(const HPolytope &rhs);
  HPolytope linearMap(const Eigen::MatrixXd &A) const;
  HPolytope preImage(const Eigen::MatrixXd &A);

  bool operator>(const Eigen::VectorXd &point);
  bool operator>(const HPolytope &poly);
  HPolytope operator+(const HPolytope &rhs) const;
  HPolytope operator-(const HPolytope &rhs);
  HPolytope operator&(const HPolytope &rhs);
  HPolytope operator|(const HPolytope &rhs);
  friend HPolytope operator*(const Eigen::MatrixXd &lhs, const HPolytope &rhs);

private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;

  mutable std::optional<std::vector<Eigen::VectorXd>> cached_vertices_{
      std::nullopt};
  mutable std::optional<bool> is_empty_{std::nullopt};

  static constexpr double kTolerance = 1e-6;
};

} // namespace hpolytope

#endif // __HPOLYTOPES__POLYTOPE_HPP__

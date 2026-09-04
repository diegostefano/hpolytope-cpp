#ifndef __HPOLYTOPES__POLYTOPE_HPP__
#define __HPOLYTOPES__POLYTOPE_HPP__

#include <optional>

#include <Eigen/Dense>

class HPolytope {
public:
  HPolytope() = delete;
  HPolytope(const Eigen::MatrixXd &A, const Eigen::VectorXd &b);

  bool isEmpty() const;
  bool contains(const Eigen::VectorXd &point);
  bool contains(const HPolytope &poly);

  std::vector<Eigen::VectorXd> vertices();
  HPolytope minimumRepresentation();
  double supportFunction(const Eigen::VectorXd &direction);

  HPolytope minkowskySum(const HPolytope &rhs);
  HPolytope pontryaginDiff(const HPolytope &rhs);
  HPolytope linearMap(const Eigen::MatrixXd &A) const;
  HPolytope preImage(const Eigen::MatrixXd &A);

  bool operator>(const Eigen::VectorXd &point);
  bool operator>(const HPolytope &poly);
  HPolytope operator+(const HPolytope &rhs);
  HPolytope operator-(const HPolytope &rhs);
  HPolytope operator&(const HPolytope &rhs);
  HPolytope operator|(const HPolytope &rhs);
  friend HPolytope operator*(const Eigen::MatrixXd &lhs, const HPolytope &rhs);

private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;

  std::optional<std::vector<Eigen::VectorXd>> cached_vertices_{std::nullopt};
  std::optional<bool> is_empty_{std::nullopt};
};

#endif // __HPOLYTOPES__POLYTOPE_HPP__

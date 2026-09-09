#ifndef HPOLYTOPE__CDDLIB__ADAPTER_HPP
#define HPOLYTOPE__CDDLIB__ADAPTER_HPP

#include <memory>
#include <type_traits>

// clang-format off
#include <cddlib/setoper.h>
#include <cddlib/cdd.h>
// clang-format on

#include <Eigen/Dense>

namespace hpolytope::cddlib {

struct CddPolyhedraDeleter {
  void operator()(dd_PolyhedraPtr poly) { dd_FreePolyhedra(poly); }
};

struct CddMatrixDeleter {
  void operator()(dd_MatrixPtr matrix) { dd_FreeMatrix(matrix); }
};

using PolyhedraOwner = std::unique_ptr<std::remove_pointer_t<dd_PolyhedraPtr>,
                                       CddPolyhedraDeleter>;

using MatrixOwner =
    std::unique_ptr<std::remove_pointer_t<dd_MatrixPtr>, CddMatrixDeleter>;

PolyhedraOwner eigenInequalitiesToCddPoly(const Eigen::MatrixXd &,
                                          const Eigen::VectorXd &);

PolyhedraOwner eigenVerticesToCddPoly(const std::vector<Eigen::MatrixXd> &);

std::vector<Eigen::VectorXd> verticesFromInequalities(const Eigen::MatrixXd &,
                                                      const Eigen::VectorXd &);

std::pair<Eigen::MatrixXd, Eigen::VectorXd>
inequalitiesFromVertices(const std::vector<Eigen::VectorXd> &);

} // namespace hpolytope::cddlib

#endif

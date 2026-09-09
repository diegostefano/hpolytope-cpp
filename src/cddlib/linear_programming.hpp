#ifndef HPOLYTOPE__CDDLIB__LINEARPROGRAMMING_HPP
#define HPOLYTOPE__CDDLIB__LINEARPROGRAMMING_HPP

#include <memory>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "runtime.hpp"

// clang-format off
#include <cddlib/setoper.h>
#include <cddlib/cdd.h>
// clang-format on

#include <Eigen/Dense>

namespace hpolytope::cddlib {

class LinearProgramming {
public:
  LinearProgramming(dd_LPObjectiveType type, size_t num_variables,
                    size_t num_constraints);
  ~LinearProgramming() = default;

  void setCost(const Eigen::VectorXd &coefficients, double offset);

  void setConstraints(const Eigen::MatrixXd &coefficients_matrix,
                      const Eigen::VectorXd &upper_bound);

  dd_LPPtr getLp() const { return lp_.get(); }

  double solve();

  bool Ok() const { return lp_->LPS == dd_Optimal; }

private:
  struct LPDataDeleter {
    void operator()(dd_LPPtr lp) { dd_FreeLPData(lp); }
  };

  using LPDataPtrType =
      std::unique_ptr<std::remove_pointer_t<dd_LPPtr>, LPDataDeleter>;

  LPDataPtrType lp_;
};

} // namespace hpolytope::cddlib

#endif

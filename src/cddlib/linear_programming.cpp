// Copyright (C) 2026 Diego Stéfano Fonseca Ferreira
// SPDX-License-Identifier: GPL-3.0-or-later

#include "linear_programming.hpp"

#include <sstream>

namespace hpolytope::cddlib {

LinearProgramming::LinearProgramming(dd_LPObjectiveType type,
                                     size_t num_variables,
                                     size_t num_constraints) {
  ensureCddlibInitialized();

  lp_ = LPDataPtrType(dd_CreateLPData(
      type, dd_Real, static_cast<dd_rowrange>(num_constraints + 1),
      static_cast<dd_colrange>(num_variables + 1)));

  lp_->objrow = static_cast<dd_rowrange>(1);
  lp_->rhscol = static_cast<dd_colrange>(1);
}

void LinearProgramming::setCost(const Eigen::VectorXd &coefficients,
                                double offset) {
  if (coefficients.size() != static_cast<double>(lp_->d) - 1) {
    throw std::invalid_argument(
        "Incompatible dimmension for coefficients vector");
  }

  dd_set_d(lp_->A[0][0], offset);

  for (dd_colrange col = 1; col < lp_->d; ++col) {
    dd_set_d(lp_->A[0][col], coefficients(col - 1));
  }
}

void LinearProgramming::setConstraints(
    const Eigen::MatrixXd &coefficients_matrix,
    const Eigen::VectorXd &upper_bound) {
  const double expected_dimension =
      (static_cast<double>(lp_->d) - 1) * (static_cast<double>(lp_->m) - 1);

  if (coefficients_matrix.size() != expected_dimension) {
    std::stringstream oss;
    oss << "A have size " << coefficients_matrix.size() << " and expected is "
        << expected_dimension << "\n";
    throw std::invalid_argument(
        "Incompatible dimmension for constraint matrix: " + oss.str());
  }

  if (upper_bound.size() != static_cast<double>(lp_->m) - 1) {
    std::stringstream oss;
    oss << "b have size " << upper_bound.size() << " and expected is "
        << static_cast<double>(lp_->m) - 1 << "\n";
    throw std::invalid_argument(
        "Incompatible dimmension for upper bound vector: " + oss.str());
  }

  for (dd_rowrange row = 1; row < lp_->m; ++row) {
    dd_set_d(lp_->A[row][0], upper_bound(row - 1));
  }

  for (dd_colrange col = 1; col < lp_->d; ++col) {
    for (dd_rowrange row = 1; row < lp_->m; ++row) {
      dd_set_d(lp_->A[row][col], -coefficients_matrix(row - 1, col - 1));
    }
  }
}

double LinearProgramming::solve() {
  dd_ErrorType error = dd_NoError;
  dd_boolean lp_ok = false;

  lp_ok = dd_LPSolve(lp_.get(), dd_DualSimplex, &error);

  if (error != dd_NoError) {
    std::stringstream oss;
    oss << "Error occured during LP solving. (Code" << error << " )";
    throw std::runtime_error(oss.str());
  }

  if (lp_ok == dd_FALSE) {
    throw std::runtime_error("dd_LPSolve returned false.");
  }

  return dd_get_d(lp_->optvalue);
}

} // namespace hpolytope::cddlib

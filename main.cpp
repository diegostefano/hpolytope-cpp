#include <cassert>
#include <iostream>

#include "Highs.h"

using std::cout;
using std::endl;

// clang-format off
#include <cddlib/setoper.h>
#include <cddlib/cdd.h>
// clang-format on

#include "src/cddlib/linear_programming.hpp"

#include <Eigen/Dense>

int main() {
  dd_set_global_constants();
  // Create and populate a HighsModel instance for the LP
  //
  // Min    f  =  x_0 +  x_1 + 3
  // s.t.                x_1 <= 7
  //        5 <=  x_0 + 2x_1 <= 15
  //        6 <= 3x_0 + 2x_1
  // 0 <= x_0 <= 4; 1 <= x_1
  //
  // Although the first constraint could be expressed as an upper
  // bound on x_1, it serves to illustrate a non-trivial packed
  // column-wise matrix.
  //
  HighsModel model;

  model.lp_.num_col_ = 2;
  model.lp_.num_row_ = 3;
  model.lp_.sense_ = ObjSense::kMinimize;
  model.lp_.offset_ = 3;
  model.lp_.col_cost_ = {1.0, 1.0};
  model.lp_.col_lower_ = {0.0, 1.0};
  model.lp_.col_upper_ = {4.0, 1.0e30};
  model.lp_.row_lower_ = {-1.0e30, 5.0, 6.0};
  model.lp_.row_upper_ = {7.0, 15.0, 1.0e30};
  //
  // Here the orientation of the matrix is column-wise
  model.lp_.a_matrix_.format_ = MatrixFormat::kColwise;
  // a_start_ has num_col_1 entries, and the last entry is the number
  // of nonzeros in A, allowing the number of nonzeros in the last
  // column to be defined
  model.lp_.a_matrix_.start_ = {0, 2, 5};
  model.lp_.a_matrix_.index_ = {1, 2, 0, 1, 2};
  model.lp_.a_matrix_.value_ = {1.0, 3.0, 1.0, 2.0, 2.0};
  //
  // Create a Highs instance
  Highs highs;
  HighsStatus return_status;
  //
  // Pass the model to HiGHS
  return_status = highs.passModel(model);
  assert(return_status == HighsStatus::kOk);
  // If a user passes a model with entries in
  // model.lp_.a_matrix_.value_ less than (the option)
  // small_matrix_value in magnitude, they will be ignored. A logging
  // message will indicate this, and passModel will return
  // HighsStatus::kWarning
  //
  // Get a const reference to the LP data in HiGHS
  const HighsLp &lp = highs.getLp();
  //
  // Solve the model
  return_status = highs.run();
  assert(return_status == HighsStatus::kOk);
  //
  // Get the model status
  const HighsModelStatus &model_status = highs.getModelStatus();
  assert(model_status == HighsModelStatus::kOptimal);
  cout << "Model status: " << highs.modelStatusToString(model_status) << endl;
  //
  // Get the solution information
  const HighsInfo &info = highs.getInfo();
  cout << "Simplex iteration count: " << info.simplex_iteration_count << endl;
  cout << "Objective function value: " << info.objective_function_value << endl;
  cout << "Primal  solution status: "
       << highs.solutionStatusToString(info.primal_solution_status) << endl;
  cout << "Dual    solution status: "
       << highs.solutionStatusToString(info.dual_solution_status) << endl;
  cout << "Basis: " << highs.basisValidityToString(info.basis_validity) << endl;
  const bool has_values = info.primal_solution_status;
  const bool has_duals = info.dual_solution_status;
  const bool has_basis = info.basis_validity;
  //
  // Get the solution values and basis
  const HighsSolution &solution = highs.getSolution();
  const HighsBasis &basis = highs.getBasis();
  //
  // Report the primal and solution values and basis
  for (int col = 0; col < lp.num_col_; col++) {
    cout << "Column " << col;
    if (has_values)
      cout << "; value = " << solution.col_value[col];
    if (has_duals)
      cout << "; dual = " << solution.col_dual[col];
    if (has_basis)
      cout << "; status: " << highs.basisStatusToString(basis.col_status[col]);
    cout << endl;
  }
  for (int row = 0; row < lp.num_row_; row++) {
    cout << "Row    " << row;
    if (has_values)
      cout << "; value = " << solution.row_value[row];
    if (has_duals)
      cout << "; dual = " << solution.row_dual[row];
    if (has_basis)
      cout << "; status: " << highs.basisStatusToString(basis.row_status[row]);
    cout << endl;
  }

  // Now indicate that all the variables must take integer values
  model.lp_.integrality_.resize(lp.num_col_);
  for (int col = 0; col < lp.num_col_; col++)
    model.lp_.integrality_[col] = HighsVarType::kInteger;

  highs.passModel(model);
  // Solve the model
  return_status = highs.run();
  assert(return_status == HighsStatus::kOk);
  // Report the primal solution values
  for (int col = 0; col < lp.num_col_; col++) {
    cout << "Column " << col;
    if (info.primal_solution_status)
      cout << "; value = " << solution.col_value[col];
    cout << endl;
  }
  for (int row = 0; row < lp.num_row_; row++) {
    cout << "Row    " << row;
    if (info.primal_solution_status)
      cout << "; value = " << solution.row_value[row];
    cout << endl;
  }

  highs.resetGlobalScheduler(true);

  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  A.resize(7, 2);
  A << 0, 1, 1, 2, -1, -2, -3, -2, 1, 0, -1, 0, 0, -1;

  std::cout << "A matrix:\n" << A.size() << '\n';

  b.resize(7);
  b << 7, 15, -5, -6, 4, 0, -1;

  std::cout << "b vector:\n" << b.size() << '\n';

  Eigen::VectorXd cost_coeff;

  cost_coeff.resize(2);
  cost_coeff << 1, 1;

  hpolytope::cddlib::LinearProgramming myLp(dd_LPmin, 2, 7);
  myLp.setConstraints(A, b);
  myLp.setCost(cost_coeff, 3);
  double myOpt = myLp.solve();

  std::cout << "myOptim = " << myOpt << '\n';

  // Create and populate a HighsModel instance for the LP
  //
  // Min    f  =  x_0 +  x_1 + 3
  // s.t.                x_1 <= 7
  //        5 <=  x_0 + 2x_1 <= 15
  //        6 <= 3x_0 + 2x_1
  // 0 <= x_0 <= 4; 1 <= x_1
  //
  // Although the first constraint could be expressed as an upper
  // bound on x_1, it serves to illustrate a non-trivial packed
  // column-wise matrix.
  //
  dd_MatrixPtr cddLpProblem = nullptr;
  dd_LPObjectiveType cddLpObjType = dd_LPmin;
  dd_LPSolverType cddLpSolver = dd_DualSimplex;
  dd_LPSolutionPtr cddLpSolution = nullptr;
  dd_ErrorType cddError = dd_NoError;
  dd_boolean cddLpOk = dd_FALSE;
  dd_LPPtr cddLp = dd_CreateLPData(cddLpObjType, dd_Real, 8, 3);

  cddLp->objrow = static_cast<dd_rowrange>(1);
  cddLp->rhscol = static_cast<dd_colrange>(1);

  // x_0 +  x_1 + 3
  dd_set_d(cddLp->A[0][0], 3);
  dd_set_d(cddLp->A[0][1], 1);
  dd_set_d(cddLp->A[0][2], 1);

  // x_1 <= 7
  dd_set_d(cddLp->A[1][0], 7);
  dd_set_d(cddLp->A[1][1], 0);
  dd_set_d(cddLp->A[1][2], -1);

  // 5 <=  x_0 + 2x_1 <= 15
  dd_set_d(cddLp->A[2][0], 15);
  dd_set_d(cddLp->A[2][1], -1);
  dd_set_d(cddLp->A[2][2], -2);

  dd_set_d(cddLp->A[3][0], -5);
  dd_set_d(cddLp->A[3][1], 1);
  dd_set_d(cddLp->A[3][2], 2);

  // 6 <= 3x_0 + 2x_1
  dd_set_d(cddLp->A[4][0], -6);
  dd_set_d(cddLp->A[4][1], 3);
  dd_set_d(cddLp->A[4][2], 2);

  // 0 <= x_0 <= 4
  dd_set_d(cddLp->A[5][0], 4);
  dd_set_d(cddLp->A[5][1], -1);
  dd_set_d(cddLp->A[5][2], 0);

  dd_set_d(cddLp->A[6][0], 0);
  dd_set_d(cddLp->A[6][1], 1);
  dd_set_d(cddLp->A[6][2], 0);

  // 1 <= x_1
  dd_set_d(cddLp->A[7][0], -1);
  dd_set_d(cddLp->A[7][1], 0);
  dd_set_d(cddLp->A[7][2], 1);

  cddLp->objective = cddLpObjType;

  cddLpOk = dd_LPSolve(cddLp, cddLpSolver, &cddError);

  if (cddError == dd_NoError && cddLpOk) {
    cout << "solution: ";
    for (dd_colrange col = 0; col < cddLp->d; ++col) {
      cout << " " << dd_get_d(cddLp->sol[col]);
    }
    cout << '\n';

    cout << "dual solution: ";
    for (dd_colrange col = 0; col < cddLp->d; ++col) {
      cout << " " << dd_get_d(cddLp->dsol[col]);
    }
    cout << '\n';

    cout << "optimal value: " << dd_get_d(cddLp->optvalue) << '\n';
  } else {
    cout << "Error code: " << cddError << '\n';
  }

  dd_FreeLPData(cddLp);

  dd_free_global_constants();
  return 0;
}

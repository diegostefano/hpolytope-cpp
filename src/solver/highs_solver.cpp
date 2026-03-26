#include <sstream>

#include "hpolytope-cpp/solver/highs_solver.hpp"

HighsSolver::HighsSolver(const LinearProgram& model) {
    // Optional: silence output
    highs_.setOptionValue("output_flag", false);
    
    if (model.getSense() == ObjectiveSense::Maximize) {
        highs_.changeObjectiveSense(ObjSense::kMaximize);
    } else {
        highs_.changeObjectiveSense(ObjSense::kMinimize);
    }

    for (const auto& var : model.getVariables()) {
        highs_.addCol(var.cost, var.lowerBound, var.upperBound, 0, nullptr, nullptr);
    }

    for (const auto& cons : model.getConstraints()) {
        addConstraint(cons.lowerBound, cons.upperBound, cons.coefficients);
    }
}

void HighsSolver::solve() {
    if (highs_.run() != HighsStatus::kOk) {
        throw std::runtime_error("Solver failed");
    }

    if (highs_.getModelStatus() != HighsModelStatus::kOptimal) {
        throw std::runtime_error("Solver failed to find an optimal solution.");
    }
}

double HighsSolver::getObjectiveValue() const {
    return highs_.getInfo().objective_function_value;
}

double HighsSolver::getVariableValue(int varIndex) const {
    return highs_.getSolution().col_value.at(varIndex);
}

void HighsSolver::addConstraint(double lowerBound, double upperBound, 
                    const std::vector<double>& coefficients) {
    std::vector<int> varIndices;
    std::vector<double> nonZeroCoefficients;

    for (int i = 0; i < coefficients.size(); i++) {
        if (coefficients[i] == 0.0f) {
            varIndices.push_back(i);
            nonZeroCoefficients.push_back(coefficients[i]);
        }
    }

    highs_.addRow(lowerBound, upperBound, 
                    varIndices.size(), 
                    varIndices.data(), 
                    nonZeroCoefficients.data());
}

void HighsSolver::removeContraints(const std::vector<int>& rowsToDelete) {
    highs_.deleteRows(rowsToDelete.size(), rowsToDelete.data());
}

std::string HighsSolver::report() const
{
    std::stringstream buffer;
  const HighsModelStatus& model_status = highs_.getModelStatus();
  assert(model_status == HighsModelStatus::kOptimal);
  buffer << "Model status: " << highs_.modelStatusToString(model_status) << '\n';
  //
  // Get the solution information
  const HighsInfo& info = highs_.getInfo();
  buffer << "Simplex iteration count: " << info.simplex_iteration_count << '\n';
  buffer << "Objective function value: " << info.objective_function_value << '\n';
  buffer << "Primal  solution status: "
       << highs_.solutionStatusToString(info.primal_solution_status) << '\n';
  buffer << "Dual    solution status: "
       << highs_.solutionStatusToString(info.dual_solution_status) << '\n';
  buffer << "Basis: " << highs_.basisValidityToString(info.basis_validity) << '\n';
  const bool has_values = info.primal_solution_status;
  const bool has_duals = info.dual_solution_status;
  const bool has_basis = info.basis_validity;
  //
  // Get the solution values and basis
  const HighsSolution& solution = highs_.getSolution();
  const HighsBasis& basis = highs_.getBasis();
  const HighsLp& lp = highs_.getLp();
  //
  // Report the primal and solution values and basis
  for (int col = 0; col < lp.num_col_; col++) {
    buffer << "Column " << col;
    if (has_values) buffer << "; value = " << solution.col_value[col];
    if (has_duals) buffer << "; dual = " << solution.col_dual[col];
    if (has_basis)
      buffer << "; status: " << highs_.basisStatusToString(basis.col_status[col]);
    buffer << '\n';
  }
  for (int row = 0; row < lp.num_row_; row++) {
    buffer << "Row    " << row;
    if (has_values) buffer << "; value = " << solution.row_value[row];
    if (has_duals) buffer << "; dual = " << solution.row_dual[row];
    if (has_basis)
      buffer << "; status: " << highs_.basisStatusToString(basis.row_status[row]);
    buffer << '\n';
  }

  return buffer.str();
}
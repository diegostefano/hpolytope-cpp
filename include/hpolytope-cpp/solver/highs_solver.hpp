// highs_solver.hpp
#ifndef POLYTOPES_CPP__HIGHS_SOLVER_HPP
#define POLYTOPES_CPP__HIGHS_SOLVER_HPP

#include "i_solver.hpp"
#include "linear_program.hpp"
#include "Highs.h"
#include <stdexcept>

class HighsSolver : public ISolver {
public:
    explicit HighsSolver(const LinearProgram& model);

    void solve() override;
    double getObjectiveValue() const override;
    double getVariableValue(int varIndex) const override;

    void addConstraint(double lowerBound, double upperBound, 
                       const std::vector<double>& coefficients) override;
    void removeContraints(const std::vector<int>& rowsToDelete) override;

    std::string report() const;

private:
    Highs highs_;
};

#endif
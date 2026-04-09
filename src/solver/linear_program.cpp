#include "hpolytope-cpp/solver/linear_program.hpp"

LinearProgram::Builder& LinearProgram::Builder::setSense(ObjectiveSense sense) { 
    data_.sense = sense; 
    return *this; 
}

LinearProgram::Builder& LinearProgram::Builder::setOffset(double offset) {
  data_.offset = offset;
  return *this;
}

LinearProgram::Builder& LinearProgram::Builder::addVariable(double cost, double lb, double ub) {
    data_.variables.push_back({cost, lb, ub});
    return *this;
}

LinearProgram::Builder& LinearProgram::Builder::addConstraint(
        double lb, double ub,
        const std::vector<double>& weights) {
    data_.constraints.push_back({lb, ub, weights});
    return *this;
}

LinearProgram LinearProgram::Builder::build() {
    if (data_.variables.empty()) {
        throw std::logic_error("Cannot build an LP model without variables.");
    }
    return LinearProgram(std::move(data_));
}

#ifndef POLYTOPES_CPP__LINEAR_PROGRAM_HPP
#define POLYTOPES_CPP__LINEAR_PROGRAM_HPP

#include <vector>
#include <stdexcept>

enum class ObjectiveSense { Minimize, Maximize };

struct Variable {
    double cost;
    double lowerBound;
    double upperBound;
};

struct Constraint {
    double lowerBound;
    double upperBound;
    std::vector<double> coefficients;
};

struct InternalData {
    ObjectiveSense sense = ObjectiveSense::Minimize;
    std::vector<Variable> variables;
    std::vector<Constraint> constraints;
    double offset = 0.0;
};

class LinearProgram {
public:
    LinearProgram() = delete;
    
    class Builder {
    public:
        Builder& setSense(ObjectiveSense sense);

        Builder& setOffset(double offset);

        Builder& addVariable(double cost, double lb, double ub);

        Builder& addConstraint(
            double lb, double ub,
            const std::vector<double>& coefficients);

        LinearProgram build();

    protected:
        InternalData data_;
    };

    ObjectiveSense getSense() const { return sense_; }
    double getOffset() const { return offset_; }
    const std::vector<Variable>& getVariables() const { return variables_; }
    const std::vector<Constraint>& getConstraints() const { return constraints_; }

private:
    explicit LinearProgram(InternalData data) 
        : sense_(data.sense)
        , offset_(data.offset)
        , variables_(std::move(data.variables))
        , constraints_(std::move(data.constraints)) {}

    ObjectiveSense sense_;
    std::vector<Variable> variables_;
    std::vector<Constraint> constraints_;
    double offset_;
};

#endif
#ifndef POLYTOPES_CPP__I_SOLVER_HPP
#define POLYTOPES_CPP__I_SOLVER_HPP

#include <vector>

class ISolver {
public:
    virtual ~ISolver() = default;

    virtual void solve() = 0;

    virtual double getObjectiveValue() const = 0;
    virtual double getVariableValue(int varIndex) const = 0;
};

#endif
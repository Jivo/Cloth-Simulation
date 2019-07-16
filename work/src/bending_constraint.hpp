#pragma once

#include "constraint.hpp"
#include "particle.hpp"

class BendingConstraint : public Constraint {
public:

    BendingConstraint(Particle &a, Particle &b, Particle &c, Particle &d, float target, float stiffness) :
    a(a), b(b), c(c), d(d), target(target) {
        this->type = ConstraintType::equality;
        this->stiffness = stiffness;
    }

    float target;

    virtual float constraint();
    virtual void solve(int iterationSteps);

private:
    Particle &a;
    Particle &b;
    Particle &c;
    Particle &d;
};
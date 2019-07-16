#pragma once

#include "constraint.hpp"
#include "particle.hpp"

class DistanceConstraint : public Constraint {
public:

	DistanceConstraint(Particle &a, Particle &b, float distance, float stiffness) :
	a(a), b(b), distance(distance) {
		this->type = ConstraintType::equality;
        this->stiffness = stiffness;
	}

	float distance;

	virtual float constraint();
	virtual void solve(int iterationSteps);

private:
	Particle &a;
	Particle &b;
};
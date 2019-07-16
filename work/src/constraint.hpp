#pragma once

#include <vector>

#include "particle.hpp"

class Constraint {
public:
	enum ConstraintType {inequality, equality};
	ConstraintType type;
	std::vector<Particle *> particles;
	float stiffness;


	virtual float constraint() = 0;
	virtual void solve(int iterationSteps) = 0;
};
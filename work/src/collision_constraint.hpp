#pragma once
#pragma once

#include "constraint.hpp"
#include "particle.hpp"

class CollisionConstraint : public Constraint {
public:

	CollisionConstraint(Particle &a, glm::vec3 collisionPosition, glm::vec3 normal, float stiffness) :
		a(a), collisionPosition(collisionPosition), normal(normal) {
		this->type = ConstraintType::inequality;
		this->stiffness = stiffness;
	}

	glm::vec3 collisionPosition;
	glm::vec3 normal;

	virtual float constraint();
	virtual void solve(int iterationSteps);

private:
	Particle &a;
};
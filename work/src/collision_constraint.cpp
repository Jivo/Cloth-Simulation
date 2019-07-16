#include "collision_constraint.hpp"
#include <iostream>

float CollisionConstraint::constraint() {
	return glm::dot(a.position - collisionPosition, normal);
}


void CollisionConstraint::solve(int iterationSteps) {
	if (a.invMass == 0) {
		return;
	}

	float c = constraint();

	glm::vec3 dCA = normal;

	float k = 1 - glm::pow(1 - stiffness, 1.0 / iterationSteps);
	a.position -= k * c * dCA;
}
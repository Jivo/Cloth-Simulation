#include "distance_constraint.hpp"
#include <iostream>

float DistanceConstraint::constraint() {
	return glm::distance(a.position, b.position) - distance;
}


void DistanceConstraint::solve(int iterationSteps) {
	if (a.invMass == 0 && b.invMass == 0) {
		return;
	}

	float c = constraint();
	glm::vec3 n = glm::normalize(a.position - b.position);

	// Derivative of C with respect to particle A's position etc.
	glm::vec3 dCA = n;
	glm::vec3 dCB = -n;

	float scale = 1 / (a.invMass + b.invMass);

	float k = 1 - glm::pow(1 - stiffness, 1.0 / iterationSteps); 
	a.position -= k * a.invMass * scale * c * dCA;
	b.position -= k * b.invMass * scale * c * dCB;
}
#include "bending_constraint.hpp"
#include <iostream>
#include <limits>

float BendingConstraint::constraint() {
	glm::vec3 normA = glm::cross(c.position - a.position, d.position - a.position);
	glm::vec3 normB = glm::cross(d.position - b.position, c.position - b.position);


    return glm::acos(glm::dot(normA, normB)) - target;
 }



void BendingConstraint::solve(int iterationSteps) {
	if (a.invMass == 0.0 && b.invMass == 0.0) {
		return;
	}

	glm::vec3 diagonal = d.position - c.position;
	float diagonalLength = glm::length(diagonal);
	if (diagonalLength < std::numeric_limits<float>::epsilon()) {
		return;
	}

	float invDiagonal = 1.0 / diagonalLength;

	glm::vec3 normA = glm::cross(c.position - a.position, d.position - a.position);
	normA /= glm::dot(normA, normA);
	glm::vec3 normB = glm::cross(d.position - b.position, c.position - b.position);
	normB /= glm::dot(normB, normB);

	// These partial derivatives can be found here https://www.cs.ubc.ca/~rbridson/docs/cloth2003.pdf
	// Page 4

	glm::vec3 dCA = diagonalLength * normA;
	glm::vec3 dCB = diagonalLength * normB;
	glm::vec3 dCC = glm::dot(a.position - d.position, diagonal) * normA + glm::dot(b.position - d.position, diagonal) * normB;
	glm::vec3 dCD = glm::dot(c.position - a.position, diagonal) * normA + glm::dot(c.position - b.position, diagonal) * normB;

	dCC *= invDiagonal;
	dCD *= invDiagonal;

	normA = glm::normalize(normA);
	normB = glm::normalize(normB);
	float dot = glm::clamp(glm::dot(normA, normB), -1.0f, 1.0f);

	float currentAngle = acos(dot);

	float denominator = a.invMass * glm::dot(dCA, dCA) + b.invMass * glm::dot(dCB, dCB) + c.invMass * glm::dot(dCC, dCC) + d.invMass * glm::dot(dCD, dCD);

	if (denominator < std::numeric_limits<float>::epsilon()) {
		return;
	}

	float C = currentAngle - target;
	float scale = 1 / denominator;

	if (glm::dot(glm::cross(normA, normB), diagonal) > 0.0) {
		scale = -scale;
	}

	float k = 1 - glm::pow(1 - stiffness, 1.0 / iterationSteps);
	a.position -= k * a.invMass * scale * C * dCA;
	b.position -= k * b.invMass * scale * C * dCB;
	c.position -= k * c.invMass * scale * C * dCC;
	d.position -= k * d.invMass * scale * C * dCD;
}
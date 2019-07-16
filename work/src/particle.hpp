#pragma once

#include "glm/glm.hpp"

class Particle {
public:
	Particle(glm::vec3 position, float mass) :
		oldPos(position), position(position), velocity(0), invMass(1 / mass) {}

	glm::vec3 oldPos;
	glm::vec3 position;
	glm::vec3 velocity;
	float invMass;
};
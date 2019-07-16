#pragma once

#include "particle.hpp"
#include "constraint.hpp"

#include <vector>
#include <memory>

class Entity {
public:
	std::vector<Particle> particles;
	std::vector<std::shared_ptr<Constraint>> constraints;
	std::vector<std::shared_ptr<Constraint>> tempConstraints;

	virtual void draw() = 0;
	virtual void handleCollision(std::shared_ptr<Entity> &e) = 0;
};
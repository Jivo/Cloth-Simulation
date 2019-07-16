#pragma once

#include <vector>
#include "cgra/mesh.hpp"
#include "cgra/shader.hpp"

#include "constraint.hpp"
#include "particle.hpp"
#include "entity.hpp"
#include "glm/glm.hpp"

class Sphere : public Entity {
public:
	Sphere(glm::vec3 position, float radius, cgra::Program &shader);

	virtual void draw();
	virtual void handleCollision(std::shared_ptr<Entity> &e);

private:
	glm::vec3 rayIntersection(glm::vec3 &a, glm::vec3 &b);
	cgra::Program &shader;
	float radius;
	cgra::Mesh mesh;
};
#pragma once

#include <vector>
#include "cgra/mesh.hpp"

#include "constraint.hpp"
#include "particle.hpp"
#include "entity.hpp"

class Cloth : public Entity {
public:
	Cloth(glm::vec3 position, glm::vec3 xDir, glm::vec3 yDir, int xParticles, int yParticles, float stiffness) :
		width(width), height(height), xParticles(xParticles), yParticles(yParticles), stiffness(stiffness) {
		particleGrid(position, xDir, yDir, xParticles, yParticles, stiffness);
	}

	Cloth(std::vector<glm::vec3> points, glm::vec3 xDir, glm::vec3 yDir, int xParticles, int yParticles, float stiffness) :
		width(width), height(height), xParticles(xParticles), yParticles(yParticles), stiffness(stiffness) {
		particleGridReshaped(points, xDir, yDir, xParticles, yParticles, stiffness);
	}

	void setColor(glm::vec3 col);
	void setDrawWireFrame(bool useWireFrame);
	void updateStiffness(float s);
	virtual void draw();

	virtual void handleCollision(std::shared_ptr<Entity> &e) {}
	cgra::Mesh mesh;

	void particleGridReshaped(std::vector<glm::vec3> points, glm::vec3 xDir, glm::vec3 yDir, int xParticles, int yParticles, float stiffness);
	glm::vec3 color = glm::vec3(182 / 255.0f, 63 / 255.0f, 2 / 255.0f);


private:
	float width;
	float height;

	bool useWireFrame;

	int xParticles;
	int yParticles;

	float stiffness;



	std::vector<std::shared_ptr<Constraint>> changeableConstraints;
	void particleGrid(glm::vec3 position, glm::vec3 xDir, glm::vec3 yDir, int xParticles, int yParticles, float stiffness);

};
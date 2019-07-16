#include "cloth.hpp"

#include "distance_constraint.hpp"
#include "bending_constraint.hpp"
#include "cgra/matrix.hpp"

#include <iostream>

void Cloth::particleGrid(glm::vec3 position, glm::vec3 xDir, glm::vec3 yDir, int xParticles, int yParticles, float stiffness) {
	// Creating particles
	particles.clear();
	for (int y = 0; y < yParticles; ++y) {
		float xPos = 0;
		for (int x = 0; x < xParticles; ++x) {
			glm::vec3 offset = x / (xParticles - 1.0f) * xDir + y / (yParticles - 1.0f) * yDir;
			particles.push_back(Particle(position + offset, 1));
		}
	}

	cgra::Matrix<double> vertices(xParticles * yParticles * 2, 3);
	cgra::Matrix<unsigned int> triangles((xParticles - 1) * (yParticles - 1) * 4, 3);
	// Setting up distance constraints and mesh

	float clothWidth = -0.01f;
	glm::vec3 zOffset = clothWidth * glm::normalize(glm::cross(xDir, yDir));
	int backIndexOffset = xParticles * yParticles;
	std::cout << "zOffset" << zOffset.x << ", " << zOffset.y << ", " << zOffset.z << std::endl;
	int nTri = 0;
	float xDist = glm::length(xDir / (xParticles - 1.0f));
	float yDist = glm::length(yDir / (yParticles - 1.0f));

	for (int y = 0; y < yParticles - 1; ++y) {
		for (int x = 0; x < xParticles - 1; ++x) {
			unsigned int currentIndex = y * xParticles + x;
			unsigned int rightIndex = currentIndex + 1;
			unsigned int topIndex = currentIndex + xParticles;
			unsigned int topRightIndex = topIndex + 1;

			Particle &current = particles[currentIndex];
			Particle &right = particles[rightIndex];
			Particle &up = particles[topIndex];
			Particle &rightup = particles[topRightIndex];


			constraints.push_back(std::make_shared<DistanceConstraint>(current, right, glm::distance(right.position, current.position), stiffness));
			constraints.push_back(std::make_shared<DistanceConstraint>(current, up, glm::distance(up.position, current.position), stiffness));


			changeableConstraints.push_back(constraints[constraints.size() - 1]);
			changeableConstraints.push_back(constraints[constraints.size() - 2]);

			constraints.push_back(std::make_shared<BendingConstraint>(current, rightup, right, up, 0, 0.1));
			constraints.push_back(std::make_shared<BendingConstraint>(right, up, rightup, current, 0, 0.1));

			vertices.setRow(currentIndex, { current.position.x, current.position.y, current.position.z });
			vertices.setRow(rightIndex, { right.position.x, right.position.y, right.position.z });
			vertices.setRow(topIndex, { up.position.x, up.position.y, up.position.z });
			vertices.setRow(topRightIndex, { rightup.position.x, rightup.position.y, rightup.position.z });

			triangles.setRow(nTri++, { currentIndex, rightIndex, topIndex });
			triangles.setRow(nTri++, { topRightIndex, topIndex, rightIndex });


			// back facing triangles;
			vertices.setRow(backIndexOffset + currentIndex, { current.position.x + zOffset.x, current.position.y + zOffset.y, current.position.z + zOffset.z});
			vertices.setRow(backIndexOffset + rightIndex, { right.position.x + zOffset.x, right.position.y + zOffset.y, right.position.z + zOffset.z });
			vertices.setRow(backIndexOffset + topIndex, { up.position.x + zOffset.x, up.position.y + zOffset.y, up.position.z + zOffset.z });
			vertices.setRow(backIndexOffset + topRightIndex, { rightup.position.x + zOffset.x, rightup.position.y + zOffset.y, rightup.position.z + zOffset.z });

			triangles.setRow(nTri++, { backIndexOffset + rightIndex, backIndexOffset + currentIndex, backIndexOffset + topIndex });
			triangles.setRow(nTri++, { backIndexOffset + topIndex, backIndexOffset + topRightIndex, backIndexOffset + rightIndex });
		}
	}

	// top row
	for (int x = 0; x < xParticles - 1; ++x) {
		Particle &current = particles[(yParticles - 1) * xParticles + x];
		Particle &right = particles[(yParticles - 1) * xParticles + x + 1];
		constraints.push_back(std::make_shared<DistanceConstraint>(current, right, glm::distance(right.position, current.position), stiffness));
	}

	// right column
	for (int y = 0; y < yParticles - 1; ++y) {
		Particle &current = particles[y * xParticles + xParticles - 1];
		Particle &up = particles[(y + 1) * xParticles + xParticles - 1];
		constraints.push_back(std::make_shared<DistanceConstraint>(current, up, glm::distance(up.position, current.position), stiffness));
	}

	// fixing points
	for (int x = 0; x < xParticles; ++x) {
		Particle &current = particles[(yParticles - 1) * xParticles + x];
		current.invMass = 0;
	}


	mesh.setData(vertices, triangles);
	mesh.setColor(color);
}

void Cloth::draw() {
	glm::vec3 xDir(particles[1].position - particles[0].position);
	glm::vec3 yDir(particles[xParticles].position - particles[0].position);
	float clothWidth = -0.01f;
	glm::vec3 zOffset = clothWidth * glm::normalize(glm::cross(xDir, yDir));
	//std::cout << zOffset.x << ", " << zOffset.y << ", " << zOffset.z << std::endl;
	// todo: fix this ineffiency
	std::vector<glm::vec3> vertices;
	for (Particle p : particles) {
		vertices.push_back(p.position);
	}
	for (Particle p : particles) {
		vertices.push_back(p.position + zOffset);
	}
	mesh.updateVertices(vertices);

	mesh.setDrawWireframe(useWireFrame);
	mesh.draw();
}

void Cloth::updateStiffness(float s) {
	for (std::shared_ptr<Constraint> c : changeableConstraints) {
		c->stiffness = s;
	}
}


void Cloth::setDrawWireFrame(bool useWireFrame) {
	this->useWireFrame = useWireFrame;
}


void Cloth::setColor(glm::vec3 col) {
	color = col;
	mesh.setColor(color);
}

/*********
TODO: Clean this up if it becomes a problem :P
**********/
void Cloth::particleGridReshaped(std::vector<glm::vec3> points, glm::vec3 xDir, glm::vec3 yDir, int xParticles, int yParticles, float stiffness) {
	particles.clear();
	
	for (glm::vec3 &position : points) {
		particles.push_back(Particle(position, 1));
	}

	cgra::Matrix<double> vertices(xParticles * yParticles * 2, 3);
	cgra::Matrix<unsigned int> triangles((xParticles - 1) * (yParticles - 1) * 4, 3);
	// Setting up distance constraints and mesh

	float clothWidth = -0.01f;
	glm::vec3 zOffset = clothWidth * glm::normalize(glm::cross(xDir, yDir));
	int backIndexOffset = xParticles * yParticles;
	// std::cout << "zOffset" << zOffset.x << ", " << zOffset.y << ", " << zOffset.z << std::endl;
	int nTri = 0;
	float xDist = glm::length(xDir / (xParticles - 1.0f));
	float yDist = glm::length(yDir / (yParticles - 1.0f));

	for (int y = 0; y < yParticles - 1; ++y) {
		for (int x = 0; x < xParticles - 1; ++x) {
			unsigned int currentIndex = y * xParticles + x;
			unsigned int rightIndex = currentIndex + 1;
			unsigned int topIndex = currentIndex + xParticles;
			unsigned int topRightIndex = topIndex + 1;

			Particle &current = particles[currentIndex];
			Particle &right = particles[rightIndex];
			Particle &up = particles[topIndex];
			Particle &rightup = particles[topRightIndex];


			constraints.push_back(std::make_shared<DistanceConstraint>(current, right, glm::distance(right.position, current.position), stiffness));
			constraints.push_back(std::make_shared<DistanceConstraint>(current, up, glm::distance(up.position, current.position), stiffness));


			changeableConstraints.push_back(constraints[constraints.size() - 1]);
			changeableConstraints.push_back(constraints[constraints.size() - 2]);

			constraints.push_back(std::make_shared<BendingConstraint>(current, rightup, right, up, 0, 0.1));
			constraints.push_back(std::make_shared<BendingConstraint>(right, up, rightup, current, 0, 0.1));

			vertices.setRow(currentIndex, { current.position.x, current.position.y, current.position.z });
			vertices.setRow(rightIndex, { right.position.x, right.position.y, right.position.z });
			vertices.setRow(topIndex, { up.position.x, up.position.y, up.position.z });
			vertices.setRow(topRightIndex, { rightup.position.x, rightup.position.y, rightup.position.z });

			triangles.setRow(nTri++, { currentIndex, rightIndex, topIndex });
			triangles.setRow(nTri++, { topRightIndex, topIndex, rightIndex });


			// back facing triangles;
			vertices.setRow(backIndexOffset + currentIndex, { current.position.x + zOffset.x, current.position.y + zOffset.y, current.position.z + zOffset.z });
			vertices.setRow(backIndexOffset + rightIndex, { right.position.x + zOffset.x, right.position.y + zOffset.y, right.position.z + zOffset.z });
			vertices.setRow(backIndexOffset + topIndex, { up.position.x + zOffset.x, up.position.y + zOffset.y, up.position.z + zOffset.z });
			vertices.setRow(backIndexOffset + topRightIndex, { rightup.position.x + zOffset.x, rightup.position.y + zOffset.y, rightup.position.z + zOffset.z });

			triangles.setRow(nTri++, { backIndexOffset + rightIndex, backIndexOffset + currentIndex, backIndexOffset + topIndex });
			triangles.setRow(nTri++, { backIndexOffset + topIndex, backIndexOffset + topRightIndex, backIndexOffset + rightIndex });
		}
	}

	// top row
	for (int x = 0; x < xParticles - 1; ++x) {
		Particle &current = particles[(yParticles - 1) * xParticles + x];
		Particle &right = particles[(yParticles - 1) * xParticles + x + 1];
		constraints.push_back(std::make_shared<DistanceConstraint>(current, right, glm::distance(right.position, current.position), stiffness));
	}

	// right column
	for (int y = 0; y < yParticles - 1; ++y) {
		Particle &current = particles[y * xParticles + xParticles - 1];
		Particle &up = particles[(y + 1) * xParticles + xParticles - 1];
		constraints.push_back(std::make_shared<DistanceConstraint>(current, up, glm::distance(up.position, current.position), stiffness));
	}

	// fixing points
	for (int x = 0; x < xParticles; ++x) {
		Particle &current = particles[(yParticles - 1) * xParticles + x];
		current.invMass = 0;
	}


	mesh.setData(vertices, triangles);
	mesh.setColor(color);
}
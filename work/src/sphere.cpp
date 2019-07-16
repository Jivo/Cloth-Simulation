#include "sphere.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "collision_constraint.hpp"

#define _USE_MATH_DEFINES
#include <math.h> 
#include <memory>


cgra::Mesh sphereMesh(int xSteps, int ySteps) {
	cgra::Matrix<double> vertices(xSteps * ySteps, 3);
	cgra::Matrix<unsigned int> triangles(xSteps * ySteps * 2, 3);

	float elevationStep = M_PI / (ySteps - 1);
	float azimuthStep = 2 * M_PI / (xSteps);

	float elevation = -M_PI / 2;

	for (int y = 0; y < ySteps; ++y) {
		float azimuth = 0;
		for (int x = 0; x < xSteps; ++x) {
			float cElevation = glm::cos(elevation);
			vertices.setRow(y * xSteps + x, { glm::cos(azimuth) * cElevation, glm::sin(azimuth) * cElevation, glm::sin(elevation) });
			azimuth += azimuthStep;
		}

		elevation += elevationStep;
	}

	int t = 0;
	for (int y = 0; y < ySteps - 1; ++y) {
		for (int x = 0; x < xSteps; ++x) {
			unsigned int a = y * xSteps + x % (xSteps);
			unsigned int b = y * xSteps + (x + 1) % (xSteps);
			unsigned int c = (y + 1) * xSteps + x % (xSteps);
			triangles.setRow(t++, { a, b, c });

			a = (y + 1) * xSteps + (x + 1) % (xSteps);
			triangles.setRow(t++, { c, b, a });

		}
	}

	cgra::Mesh mesh;
	if (t > 0) {
		mesh.setData(vertices, triangles);
	}

	return mesh;
}

Sphere::Sphere(glm::vec3 position, float radius, cgra::Program &shader) : radius(radius), shader(shader) {
	float mass = 10;
	particles.push_back(Particle(position, mass));
	mesh = sphereMesh(50, 50);
}


void Sphere::draw() {
	glm::mat4 modelTransform = glm::translate(glm::mat4(1.0f), particles[0].position) * glm::scale(glm::mat4(1.0f), glm::vec3(radius * 0.95));
	mesh.setColor(glm::vec3(1, 1, 1));

	shader.setModelMatrix(modelTransform);
	mesh.draw();
}


void Sphere::handleCollision(std::shared_ptr<Entity> &e) {
	glm::vec3 spherePos = particles[0].position;
	for (Particle &p : e->particles) {
		bool wasInside = glm::distance(p.oldPos, spherePos) < radius;
		bool isInside = glm::distance(p.position, spherePos) < radius;

		if (!wasInside && isInside) {
			glm::vec3 position(rayIntersection(p.oldPos, p.position));
			glm::vec3 normal(glm::normalize(position - spherePos));
			//e->tempConstraints.push_back(std::make_shared<CollisionConstraint>(p, position, normal, 1));
		}

		if (wasInside && isInside) {
			glm::vec3 normal(glm::normalize(p.position - spherePos));
			glm::vec3 nearestPoint(spherePos + normal * radius);

			e->tempConstraints.push_back(std::make_shared<CollisionConstraint>(p, nearestPoint, normal, 1));
		}
	}
}

glm::vec3 Sphere::rayIntersection(glm::vec3 &from, glm::vec3 &to) {
	glm::vec3 spherePos = particles[0].position;
	glm::vec3 rayDirection = to - from;
	glm::vec3 sphereOriginToRayStart = from - spherePos;;
	float a = glm::dot(rayDirection, rayDirection);
	float b = 2 * glm::dot(rayDirection, sphereOriginToRayStart);
	float c = glm::dot(sphereOriginToRayStart, sphereOriginToRayStart) - radius * radius;
	float discr = b * b - 4 * a * c;

	if (discr < 0) {
		return glm::vec3();
	}
	
	float t = (-b - glm::sqrt(discr)) / (2 * a);
	//glm::vec3 temp(from + t * rayDirection);

	//glm::mat4 modelTransform = glm::translate(glm::mat4(1.0f), temp) * glm::scale(glm::mat4(1.0f), glm::vec3(0.1));
	//shader.setModelMatrix(modelTransform);
	//mesh.setColor(glm::vec3(0, 0, 1));
	//mesh.draw();
	return from + t * rayDirection;
}
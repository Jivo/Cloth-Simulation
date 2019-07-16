#include "pbd.hpp"

#include <iostream>

#include <algorithm>
#include <iterator>


template <class Container, class RemoveTargetContainer>
typename Container::iterator remove_for(Container& container, const RemoveTargetContainer& targets)
{
	auto container_first = std::begin(container);
	auto container_last = std::end(container);

	std::for_each(std::begin(targets), std::end(targets),
		[&](typename RemoveTargetContainer::const_reference target)
	{
		container_last = std::remove(container_first, container_last, target);
	}
	);

	return container_last;
}

// consider enttiy array/vector instead
void PBD::step(std::vector<std::shared_ptr<Entity>> &es, float dt) {
	for (std::shared_ptr<Entity> &e : es) {
		for (Particle &p : e->particles) {
			glm::vec3 externalForces(0, -5, 0);
			p.velocity += dt * p.invMass * externalForces;
		}
	}

	dampenVelocities(es, dt);

	for (std::shared_ptr<Entity> &e : es) {
		for (Particle &p : e->particles) {
			p.oldPos = p.position;
			p.position += dt * p.velocity;
		}
	}

	// generateConsraints()

	for (int i = 0; i < es.size(); ++i) {
		for (int j = 0; j < es.size(); ++j) {
			if (i == j) {
				continue;
			}

			es[i]->handleCollision(es[j]);
		}
	}



	for (std::shared_ptr<Entity> &e : es) {
		//std::vector<std::shared_ptr<Constraint>> constraintsToRemove;
		//std::vector<Particle> particlesToRemove;
		int k = 5;
		for (int i = 0; i < k; ++i) {
			for (std::shared_ptr<Constraint> &c : e->constraints) {
				c->solve(k);

				//if (glm::abs(c->constraint()) > 10) {
				//	constraintsToRemove.push_back(c);
				//	particlesToRemove.insert(particlesToRemove.end(), c->particles.begin(), c->particles.end());
				//}
			}
		}

		for (int i = 0; i < k; ++i) {
			for (std::shared_ptr<Constraint> &c : e->tempConstraints) {
				c->solve(k);
			}
		}

		e->tempConstraints.clear();


		for (Particle &p : e->particles) {
			p.velocity = (p.position - p.oldPos) / dt;
		}


		//e->constraints.erase(remove_for(e->constraints, constraintsToRemove), std::end(e->constraints));
		//e->particles.erase(remove_for(e->particles, particlesToRemove), std::end(e->particles));
	}


}


void PBD::dampenVelocities(std::vector<std::shared_ptr<Entity>> &es, float dt) {
	for (std::shared_ptr<Entity> &e : es) {
		for (Particle &p : e->particles) {
			p.velocity -= 0.1f * p.velocity * p.invMass * dt;
		}
	}
}
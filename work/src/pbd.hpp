#pragma once

#include "entity.hpp"
#include <memory>


class PBD {
public:
    void step(std::vector<std::shared_ptr<Entity>> &es, float dt);
    void dampenVelocities(std::vector<std::shared_ptr<Entity>> &es, float dt);
};
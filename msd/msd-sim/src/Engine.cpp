#include "msd-sim/src/Engine.hpp"
#include <iostream>

namespace msd_sim
{

Engine::Engine() : worldModel_{}
{
}

void Engine::update(std::chrono::milliseconds simTime)
{
  worldModel_.update(simTime);
}
}  // namespace msd_sim
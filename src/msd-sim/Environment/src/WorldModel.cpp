#include <iostream>

#include "Environment/src/WorldModel.hpp"

namespace msd_sim
{

void WorldModel::update(std::chrono::milliseconds simTime)
{
  time_ = simTime;
  std::cout << "Simulation Time: " << time_.count() << " ms" << std::endl;
}

} // namespace msd_sim
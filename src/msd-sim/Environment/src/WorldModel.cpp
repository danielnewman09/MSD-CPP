#include "Environment/src/WorldModel.hpp"

namespace msd_sim
{

void WorldModel::update(std::chrono::milliseconds simTime)
{
  time_ = simTime;
}

} // namespace msd_sim
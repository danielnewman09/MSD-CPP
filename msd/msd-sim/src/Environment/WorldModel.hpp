#ifndef WORLD_MODEL_HPP
#define WORLD_MODEL_HPP

#include <chrono>
#include <vector>

#include "msd-sim/src/Environment/Platform.hpp"

namespace msd_sim
{

class WorldModel
{
public:
  WorldModel() = default;
  ~WorldModel() = default;

  void update(std::chrono::milliseconds simTime); // Update the world model

  void addPlatform(Platform &&platform);

private:
  //! Platforms in the world
  std::vector<Platform> platforms_;

  //! Current time in the world
  std::chrono::milliseconds time_;

};

} // namespace msd_sim

#endif // WORLD_MODEL_HPP
#ifndef BASEAGENT_HPP
#define BASEAGENT_HPP

#include "msd-sim/src/Environment/PlatformState.hpp"

namespace msd_sim
{
class BaseAgent
{
public:
  virtual ~BaseAgent() = default;

  virtual PlatformState updateState(const PlatformState& currentState) = 0;
};
}  // namespace msd_sim

#endif  // BASEAGENT_HPP
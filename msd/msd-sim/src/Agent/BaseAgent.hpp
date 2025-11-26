#ifndef BASEAGENT_HPP
#define BASEAGENT_HPP

#include "msd-sim/src/Environment/InertialState.hpp"

namespace msd_sim
{
class BaseAgent
{
public:
  virtual ~BaseAgent() = default;

  virtual InertialState updateState(const InertialState& currentState) = 0;
};
}  // namespace msd_sim

#endif  // BASEAGENT_HPP
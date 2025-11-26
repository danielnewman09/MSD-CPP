#ifndef INERTIAL_STATE_HPP
#define INERTIAL_STATE_HPP

#include <cstdint>

#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/EulerAngles.hpp"

namespace msd_sim
{

struct InertialState
{
  Coordinate position;
  Coordinate velocity;
  Coordinate acceleration;

  EulerAngles angularPosition;
  EulerAngles angularVelocity;
  EulerAngles angularAcceleration;
};

}  // namespace msd_sim

#endif
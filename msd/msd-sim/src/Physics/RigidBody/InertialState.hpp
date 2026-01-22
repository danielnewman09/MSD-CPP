// Ticket: 0024_angular_coordinate
// Design: docs/designs/0024_angular_coordinate/design.md

#ifndef INERTIAL_STATE_HPP
#define INERTIAL_STATE_HPP

#include <cstdint>

#include "msd-sim/src/Environment/AngularCoordinate.hpp"
#include "msd-sim/src/Environment/AngularRate.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_sim
{

/**
 * @brief Complete kinematic state representation with 6 degrees of freedom.
 *
 * Contains position, velocity, and acceleration for both linear and angular motion.
 * Type-safe separation of orientation (AngularCoordinate with normalization) and
 * angular rates (AngularRate without normalization).
 *
 * @see docs/designs/0024_angular_coordinate/0024_angular_coordinate.puml
 * @ticket 0024_angular_coordinate
 */
struct InertialState
{
  // Linear components
  Coordinate position;
  Coordinate velocity;
  Coordinate acceleration;

  // Angular components (type-safe separation)
  AngularCoordinate orientation;       // Was: EulerAngles (auto-normalizing)
  AngularRate angularVelocity;         // Was: Coordinate (no normalization)
  AngularRate angularAcceleration;     // Was: Coordinate (no normalization)
};

}  // namespace msd_sim

#endif
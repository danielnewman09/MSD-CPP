// Ticket: 0023a_force_application_scaffolding
// Design: docs/designs/0023a_force_application_scaffolding/design.md

#ifndef INERTIAL_STATE_HPP
#define INERTIAL_STATE_HPP

#include <cstdint>

#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/EulerAngles.hpp"

namespace msd_sim
{

/**
 * @brief Complete kinematic state representation with 6 degrees of freedom.
 *
 * Contains position, velocity, and acceleration for both linear and angular motion.
 * Angular velocity and acceleration are represented as Coordinate (vector form) rather
 * than EulerAngles to align with physics equations (τ = I·α).
 *
 * @see docs/designs/0023a_force_application_scaffolding/0023a_force_application_scaffolding.puml
 * @ticket 0023a_force_application_scaffolding
 */
struct InertialState
{
  // Linear components
  Coordinate position;
  Coordinate velocity;
  Coordinate acceleration;

  // Angular components
  EulerAngles orientation;          // Renamed from angularPosition
  Coordinate angularVelocity;       // Changed from EulerAngles to Coordinate
  Coordinate angularAcceleration;   // Changed from EulerAngles to Coordinate
};

}  // namespace msd_sim

#endif
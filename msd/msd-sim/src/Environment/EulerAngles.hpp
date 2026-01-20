// Ticket: 0023a_force_application_scaffolding
// Design: docs/designs/0023a_force_application_scaffolding/design.md

#ifndef EULER_ANGLES_HPP
#define EULER_ANGLES_HPP

#include <Eigen/Dense>
#include "msd-sim/src/Environment/Angle.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_sim
{

/**
 * @brief Represents 3D orientation using a triplet of Angle objects.
 *
 * Uses ZYX intrinsic rotation convention (yaw → pitch → roll).
 *
 * @see docs/designs/0023a_force_application_scaffolding/0023a_force_application_scaffolding.puml
 * @ticket 0023a_force_application_scaffolding
 */
struct EulerAngles
{
  Angle pitch;  // Rotation around Y-axis
  Angle roll;   // Rotation around X-axis
  Angle yaw;    // Rotation around Z-axis

  /**
   * @brief Convert EulerAngles to Coordinate vector representation.
   *
   * Represents the Euler angles as a 3D coordinate vector [pitch, roll, yaw].
   * Useful for debugging and testing angular velocity representations.
   *
   * @return Coordinate vector with components [pitch, roll, yaw] in radians
   */
  Coordinate toCoordinate() const;

  /**
   * @brief Construct EulerAngles from a Coordinate vector.
   *
   * Interprets the coordinate components as [pitch, roll, yaw] angles.
   * Inverse of toCoordinate().
   *
   * @param coord Coordinate vector with components [pitch, roll, yaw] in radians
   * @return EulerAngles constructed from coordinate components
   */
  static EulerAngles fromCoordinate(const Coordinate& coord);
};


}  // namespace msd_sim

#endif  // EULER_ANGLES_HPP

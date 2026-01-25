// Ticket: 0027a_expanding_polytope_algorithm
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_RESULT_HPP
#define MSD_SIM_PHYSICS_COLLISION_RESULT_HPP

#include <limits>
#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_sim
{

/**
 * @brief Complete collision information for physics response.
 *
 * This struct is returned by EPA when a collision is detected.
 * It does NOT contain an 'intersecting' boolean because:
 * - CollisionHandler returns std::optional<CollisionResult>
 * - std::nullopt indicates no collision
 * - Presence of CollisionResult implies collision exists
 *
 * All coordinates are in world space.
 * Contact normal points from object A toward object B.
 *
 * Breaking change (Ticket 0028_epa_witness_points):
 * - Replaced single contactPoint with contactPointA and contactPointB
 * - Witness points enable accurate torque calculation (τ = r × F)
 *
 * @see docs/designs/0027a_expanding_polytope_algorithm/0027a_expanding_polytope_algorithm.puml
 * @see docs/designs/0028_epa_witness_points/0028_epa_witness_points.puml
 * @ticket 0027a_expanding_polytope_algorithm
 * @ticket 0028_epa_witness_points
 */
struct CollisionResult
{
  Coordinate normal;  // Contact normal (world space, A→B, unit length)
  double penetrationDepth{
      std::numeric_limits<double>::quiet_NaN()};  // Overlap distance [m]
  Coordinate contactPointA;  // Contact point on A's surface (world space) [m]
  Coordinate contactPointB;  // Contact point on B's surface (world space) [m]

  CollisionResult() = default;

  CollisionResult(const Coordinate& n,
                  double depth,
                  const Coordinate& pointA,
                  const Coordinate& pointB)
    : normal{n}, penetrationDepth{depth}, contactPointA{pointA}, contactPointB{pointB}
  {
  }
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_RESULT_HPP

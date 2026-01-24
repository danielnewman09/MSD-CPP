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
 * @see docs/designs/0027a_expanding_polytope_algorithm/0027a_expanding_polytope_algorithm.puml
 * @ticket 0027a_expanding_polytope_algorithm
 */
struct CollisionResult
{
  Coordinate normal;  // Contact normal (world space, A→B, unit length)
  double penetrationDepth{
      std::numeric_limits<double>::quiet_NaN()};  // Overlap distance [m]
  Coordinate contactPoint;  // Contact location (world space) [m]

  CollisionResult() = default;

  CollisionResult(const Coordinate& n, double depth, const Coordinate& point)
    : normal{n}, penetrationDepth{depth}, contactPoint{point}
  {
  }
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_RESULT_HPP

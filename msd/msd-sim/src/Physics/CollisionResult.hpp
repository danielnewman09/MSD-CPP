// Ticket: 0027a_expanding_polytope_algorithm
// Ticket: 0029_contact_manifold_generation
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md
// Design: docs/designs/0029_contact_manifold_generation/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_RESULT_HPP
#define MSD_SIM_PHYSICS_COLLISION_RESULT_HPP

#include <array>
#include <limits>
#include <stdexcept>
#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_sim
{

/**
 * @brief A single contact pair within a collision manifold.
 *
 * Stores contact locations on both object surfaces in world space.
 * Simple POD struct with value semantics.
 *
 * @see docs/designs/0029_contact_manifold_generation/0029_contact_manifold_generation.puml
 * @ticket 0029_contact_manifold_generation
 */
struct ContactPoint
{
  Coordinate pointA;  // Contact point on object A's surface (world space) [m]
  Coordinate pointB;  // Contact point on object B's surface (world space) [m]

  ContactPoint() = default;

  ContactPoint(const Coordinate& pA, const Coordinate& pB)
    : pointA{pA}, pointB{pB}
  {
  }

  ContactPoint(const ContactPoint&) = default;
  ContactPoint(ContactPoint&&) noexcept = default;
  ContactPoint& operator=(const ContactPoint&) = default;
  ContactPoint& operator=(ContactPoint&&) noexcept = default;
  ~ContactPoint() = default;
};

/**
 * @brief Complete collision information for physics response with contact manifold.
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
 * Breaking change (Ticket 0029_contact_manifold_generation):
 * - Replaced contactPointA/contactPointB with contacts array
 * - Added contactCount field for number of valid contacts [1, 4]
 * - Enables multi-point collision response for improved stability
 *
 * @see docs/designs/0027a_expanding_polytope_algorithm/0027a_expanding_polytope_algorithm.puml
 * @see docs/designs/0028_epa_witness_points/0028_epa_witness_points.puml
 * @see docs/designs/0029_contact_manifold_generation/0029_contact_manifold_generation.puml
 * @ticket 0027a_expanding_polytope_algorithm
 * @ticket 0028_epa_witness_points
 * @ticket 0029_contact_manifold_generation
 */
struct CollisionResult
{
  Coordinate normal;  // Contact normal (world space, A→B, unit length)
  double penetrationDepth{
      std::numeric_limits<double>::quiet_NaN()};  // Overlap distance [m]

  // Contact manifold storage (replaces legacy contactPointA/contactPointB)
  std::array<ContactPoint, 4> contacts;  // Up to 4 contact points
  size_t contactCount{0};                 // Number of valid contacts [1, 4]

  // Default constructor
  CollisionResult() = default;

  // Manifold constructor
  CollisionResult(const Coordinate& n,
                  double depth,
                  const std::array<ContactPoint, 4>& contactsArray,
                  size_t count)
    : normal{n}, penetrationDepth{depth}, contacts{contactsArray}, contactCount{count}
  {
    if (count < 1 || count > 4)
    {
      throw std::invalid_argument("contactCount must be in [1, 4]");
    }
  }

  // Single-contact convenience constructor
  CollisionResult(const Coordinate& n,
                  double depth,
                  const Coordinate& pointA,
                  const Coordinate& pointB)
    : normal{n}, penetrationDepth{depth}, contacts{ContactPoint{pointA, pointB}}, contactCount{1}
  {
  }

  CollisionResult(const CollisionResult&) = default;
  CollisionResult(CollisionResult&&) noexcept = default;
  CollisionResult& operator=(const CollisionResult&) = default;
  CollisionResult& operator=(CollisionResult&&) noexcept = default;
  ~CollisionResult() = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_RESULT_HPP

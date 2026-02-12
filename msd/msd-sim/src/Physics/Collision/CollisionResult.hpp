// Ticket: 0027a_expanding_polytope_algorithm
// Ticket: 0029_contact_manifold_generation
// Ticket: 0040a_per_contact_penetration_depth
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md
// Design: docs/designs/0029_contact_manifold_generation/design.md
// Design: docs/designs/0040a-per-contact-penetration-depth/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_RESULT_HPP
#define MSD_SIM_PHYSICS_COLLISION_RESULT_HPP

#include <array>
#include <limits>
#include <stdexcept>
#include <utility>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-transfer/src/CollisionResultRecord.hpp"

namespace msd_sim
{

/**
 * @brief A single contact pair within a collision manifold.
 *
 * Stores contact locations on both object surfaces in world space,
 * along with the penetration depth at this specific contact point.
 * Simple POD struct with value semantics.
 *
 * @see
 * docs/designs/0029_contact_manifold_generation/0029_contact_manifold_generation.puml
 * @see
 * docs/designs/0040a-per-contact-penetration-depth/0040a-per-contact-penetration-depth.puml
 * @ticket 0029_contact_manifold_generation
 * @ticket 0040a_per_contact_penetration_depth
 */
struct ContactPoint
{
  Coordinate pointA;  // Contact point on object A's surface (world space) [m]
  Coordinate pointB;  // Contact point on object B's surface (world space) [m]
  double depth{0.0};  // Penetration depth at this contact point [m]

  ContactPoint() = default;

  ContactPoint(Coordinate pA, Coordinate pB, double d = 0.0)
    : pointA{std::move(pA)}, pointB{std::move(pB)}, depth{d}
  {
  }

  ContactPoint(const ContactPoint&) = default;
  ContactPoint(ContactPoint&&) noexcept = default;
  ContactPoint& operator=(const ContactPoint&) = default;
  ContactPoint& operator=(ContactPoint&&) noexcept = default;
  ~ContactPoint() = default;

  // Transfer methods
  static ContactPoint fromRecord(
    const msd_transfer::ContactPointRecord& record)
  {
    return ContactPoint{Coordinate::fromRecord(record.pointA),
                        Coordinate::fromRecord(record.pointB),
                        record.depth};
  }

  [[nodiscard]] msd_transfer::ContactPointRecord toRecord() const
  {
    msd_transfer::ContactPointRecord record;
    record.pointA = pointA.toRecord();
    record.pointB = pointB.toRecord();
    record.depth = depth;
    return record;
  }
};

/**
 * @brief Complete collision information for physics response with contact
 * manifold.
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
 * @see
 * docs/designs/0027a_expanding_polytope_algorithm/0027a_expanding_polytope_algorithm.puml
 * @see docs/designs/0028_epa_witness_points/0028_epa_witness_points.puml
 * @see
 * docs/designs/0029_contact_manifold_generation/0029_contact_manifold_generation.puml
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
  size_t contactCount{0};                // Number of valid contacts [1, 4]

  // Default constructor
  CollisionResult() = default;

  // Manifold constructor
  CollisionResult(Coordinate n,
                  double depth,
                  const std::array<ContactPoint, 4>& contactsArray,
                  size_t count)
    : normal{std::move(n)},
      penetrationDepth{depth},
      contacts{contactsArray},
      contactCount{count}
  {
    if (count < 1 || count > 4)
    {
      throw std::invalid_argument("contactCount must be in [1, 4]");
    }
  }

  // Single-contact convenience constructor
  CollisionResult(Coordinate n,
                  double depth,
                  const Coordinate& pointA,
                  const Coordinate& pointB)
    : normal{std::move(n)},
      penetrationDepth{depth},
      contacts{ContactPoint{pointA, pointB}},
      contactCount{1}
  {
  }

  // Transfer methods
  static CollisionResult fromRecord(
    const msd_transfer::CollisionResultRecord& record)
  {
    CollisionResult result;
    result.normal =
      Coordinate{record.normal.x, record.normal.y, record.normal.z};
    result.penetrationDepth = record.penetrationDepth;
    result.contactCount = record.contacts.data.size();
    for (size_t i = 0; i < result.contactCount && i < 4; ++i)
    {
      result.contacts[i] =
        ContactPoint::fromRecord(record.contacts.data[i]);
    }
    return result;
  }

  [[nodiscard]] msd_transfer::CollisionResultRecord toRecord(
    uint32_t bodyAId,
    uint32_t bodyBId) const
  {
    msd_transfer::CollisionResultRecord record;
    record.body_a_id = bodyAId;
    record.body_b_id = bodyBId;
    record.normal = Vector3D{normal.x(), normal.y(), normal.z()}.toRecord();
    record.penetrationDepth = penetrationDepth;
    for (size_t i = 0; i < contactCount; ++i)
    {
      record.contacts.data.push_back(contacts[i].toRecord());
    }
    return record;
  }
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_RESULT_HPP

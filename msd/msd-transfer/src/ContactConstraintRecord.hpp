#pragma once

#include "Vector3DRecord.hpp"
#include "SimulationFrameRecord.hpp"
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>
#include <boost/describe.hpp>
#include <limits>

namespace msd_transfer {

/**
 * @brief Transfer record for contact constraint state
 *
 * Records the geometry and parameters of a contact constraint after constraint
 * creation. Used for debugging constraint solver behavior and validating
 * contact detection.
 *
 * Note: All vector fields (normal, lever arms) are Vector3DRecord (directions),
 * not CoordinateRecord (points), to avoid globalToLocal overload issues.
 */
struct ContactConstraintRecord : public cpp_sqlite::BaseTransferObject {
  /// ID of body A in the contact pair
  uint32_t body_a_id{0};

  /// ID of body B in the contact pair
  uint32_t body_b_id{0};

  /// Contact normal in world space (unit vector pointing from B to A)
  Vector3DRecord normal;

  /// Lever arm from body A's center of mass to contact point
  Vector3DRecord lever_arm_a;

  /// Lever arm from body B's center of mass to contact point
  Vector3DRecord lever_arm_b;

  /// Penetration depth at constraint creation (meters)
  double penetration_depth{std::numeric_limits<double>::quiet_NaN()};

  /// Coefficient of restitution for this contact
  double restitution{std::numeric_limits<double>::quiet_NaN()};

  /// Pre-impact relative velocity projected onto normal (m/s)
  double pre_impact_rel_vel_normal{std::numeric_limits<double>::quiet_NaN()};

  /// Foreign key to simulation frame
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(ContactConstraintRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_a_id,
                       body_b_id,
                       normal,
                       lever_arm_a,
                       lever_arm_b,
                       penetration_depth,
                       restitution,
                       pre_impact_rel_vel_normal,
                       frame));

}  // namespace msd_transfer

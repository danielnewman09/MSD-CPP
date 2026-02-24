#pragma once

// Ticket: 0075a_unified_constraint_data_structure
// Design: docs/designs/0075_unified_contact_constraint/design.md (Phase 1)

#include "Vector3DRecord.hpp"
#include "SimulationFrameRecord.hpp"
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>
#include <boost/describe.hpp>
#include <limits>

namespace msd_transfer {

/**
 * @brief Unified transfer record combining contact geometry and friction fields
 *
 * Supersedes the separate ContactConstraintRecord and FrictionConstraintRecord.
 * Stores all per-contact-point data needed by the Block PGS solver: contact
 * geometry (normal, lever arms, penetration depth, restitution), tangent basis,
 * friction coefficient, and all three solved impulse components (normal +
 * two tangent directions).
 *
 * When friction is absent (frictionless contact), tangent1 and tangent2 are
 * zero vectors, friction_coefficient is 0, and tangent1_lambda /
 * tangent2_lambda are 0.
 *
 * Note: All vector fields (normal, tangents, lever arms) are Vector3DRecord
 * (directions), not CoordinateRecord (points), to avoid globalToLocal overload
 * issues when transforming between reference frames.
 *
 * @ticket 0075a_unified_constraint_data_structure
 */
struct UnifiedContactConstraintRecord : public cpp_sqlite::BaseTransferObject {
  // --- Contact geometry (from ContactConstraintRecord) ---

  /// ID of body A in the contact pair
  uint32_t body_a_id{0};

  /// ID of body B in the contact pair
  uint32_t body_b_id{0};

  /// Contact normal in world space (unit vector, A → B)
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

  // --- Friction fields (from FrictionConstraintRecord) ---

  /// First tangent direction in world space (zero vector if frictionless)
  Vector3DRecord tangent1;

  /// Second tangent direction in world space (zero vector if frictionless)
  Vector3DRecord tangent2;

  /// Coulomb friction coefficient (0.0 for frictionless contacts)
  double friction_coefficient{std::numeric_limits<double>::quiet_NaN()};

  /// Normal impulse magnitude (lambda from constraint solver)
  double normal_lambda{std::numeric_limits<double>::quiet_NaN()};

  /// Tangent1 impulse magnitude (lambda from Block PGS solver, Newtons)
  double tangent1_lambda{std::numeric_limits<double>::quiet_NaN()};

  /// Tangent2 impulse magnitude (lambda from Block PGS solver, Newtons)
  double tangent2_lambda{std::numeric_limits<double>::quiet_NaN()};

  /// Foreign key to simulation frame
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(UnifiedContactConstraintRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_a_id,
                       body_b_id,
                       normal,
                       lever_arm_a,
                       lever_arm_b,
                       penetration_depth,
                       restitution,
                       pre_impact_rel_vel_normal,
                       tangent1,
                       tangent2,
                       friction_coefficient,
                       normal_lambda,
                       tangent1_lambda,
                       tangent2_lambda,
                       frame));

}  // namespace msd_transfer

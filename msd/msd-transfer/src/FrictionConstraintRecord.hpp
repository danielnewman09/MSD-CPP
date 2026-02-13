#pragma once

#include "Vector3DRecord.hpp"
#include "SimulationFrameRecord.hpp"
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>
#include <boost/describe.hpp>
#include <limits>

namespace msd_transfer {

/**
 * @brief Transfer record for friction constraint state
 *
 * Records the tangent basis and parameters of a friction constraint after
 * constraint creation. Enables visualization of contact frames (normal + two
 * orthogonal tangents) and debugging of friction force magnitudes.
 *
 * Note: All vector fields (normal, tangents, lever arms) are Vector3DRecord
 * (directions), not CoordinateRecord (points), to avoid globalToLocal overload
 * issues when transforming between reference frames.
 */
struct FrictionConstraintRecord : public cpp_sqlite::BaseTransferObject {
  /// ID of body A in the contact pair
  uint32_t body_a_id{0};

  /// ID of body B in the contact pair
  uint32_t body_b_id{0};

  /// Contact normal in world space (unit vector pointing from B to A)
  Vector3DRecord normal;

  /// First tangent direction in world space (unit vector orthogonal to normal)
  Vector3DRecord tangent1;

  /// Second tangent direction in world space (unit vector orthogonal to normal and tangent1)
  Vector3DRecord tangent2;

  /// Lever arm from body A's center of mass to contact point
  Vector3DRecord lever_arm_a;

  /// Lever arm from body B's center of mass to contact point
  Vector3DRecord lever_arm_b;

  /// Coefficient of friction (Coulomb friction model)
  double friction_coefficient{std::numeric_limits<double>::quiet_NaN()};

  /// Normal force magnitude (lambda from constraint solver)
  double normal_lambda{std::numeric_limits<double>::quiet_NaN()};

  /// Foreign key to simulation frame
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(FrictionConstraintRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (body_a_id,
                       body_b_id,
                       normal,
                       tangent1,
                       tangent2,
                       lever_arm_a,
                       lever_arm_b,
                       friction_coefficient,
                       normal_lambda,
                       frame));

}  // namespace msd_transfer

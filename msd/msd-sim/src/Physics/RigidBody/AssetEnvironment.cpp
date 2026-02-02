// Ticket: 0032_contact_constraint_refactor
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include <stdexcept>

namespace msd_sim
{

// Static member definition
const Eigen::Matrix3d AssetEnvironment::kZeroInertia = Eigen::Matrix3d::Zero();

AssetEnvironment::AssetEnvironment(uint32_t assetId,
                                   uint32_t instanceId,
                                   ConvexHull& hull,
                                   const ReferenceFrame& frame)
  : AssetPhysical{assetId, instanceId, hull, frame},
    coefficient_of_restitution_{0.5}
{
  // Initialize static state with position from frame, zero velocities
  static_state_.position = frame.getOrigin();
  static_state_.orientation = frame.getQuaternion();
  // Velocities already default to zero in InertialState
}

AssetEnvironment::AssetEnvironment(uint32_t assetId,
                                   uint32_t instanceId,
                                   ConvexHull& hull,
                                   const ReferenceFrame& frame,
                                   double coefficientOfRestitution)
  : AssetPhysical{assetId, instanceId, hull, frame},
    coefficient_of_restitution_{coefficientOfRestitution},
    friction_coefficient_{0.5}  // Default: moderate friction
{
  // Validate restitution
  if (coefficientOfRestitution < 0.0 || coefficientOfRestitution > 1.0) {
    throw std::invalid_argument(
        "AssetEnvironment: coefficient of restitution must be in [0, 1] (e = " +
        std::to_string(coefficientOfRestitution) + ")");
  }

  // Initialize static state with position from frame, zero velocities
  static_state_.position = frame.getOrigin();
  static_state_.orientation = frame.getQuaternion();
  // Velocities already default to zero in InertialState
}

AssetEnvironment::AssetEnvironment(uint32_t assetId,
                                   uint32_t instanceId,
                                   ConvexHull& hull,
                                   const ReferenceFrame& frame,
                                   double coefficientOfRestitution,
                                   double frictionCoefficient)
  : AssetPhysical{assetId, instanceId, hull, frame},
    coefficient_of_restitution_{coefficientOfRestitution},
    friction_coefficient_{frictionCoefficient}
{
  // Validate restitution
  if (coefficientOfRestitution < 0.0 || coefficientOfRestitution > 1.0) {
    throw std::invalid_argument(
        "AssetEnvironment: coefficient of restitution must be in [0, 1] (e = " +
        std::to_string(coefficientOfRestitution) + ")");
  }

  // Validate friction coefficient
  if (frictionCoefficient < 0.0) {
    throw std::invalid_argument(
        "AssetEnvironment: friction coefficient must be non-negative (mu = " +
        std::to_string(frictionCoefficient) + ")");
  }

  // Initialize static state with position from frame, zero velocities
  static_state_.position = frame.getOrigin();
  static_state_.orientation = frame.getQuaternion();
  // Velocities already default to zero in InertialState
}

void AssetEnvironment::setCoefficientOfRestitution(double e)
{
  if (e < 0.0 || e > 1.0) {
    throw std::invalid_argument(
        "AssetEnvironment: coefficient of restitution must be in [0, 1] (e = " +
        std::to_string(e) + ")");
  }
  coefficient_of_restitution_ = e;
}

}  // namespace msd_sim

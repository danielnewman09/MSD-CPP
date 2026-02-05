// Ticket: 0032_contact_constraint_refactor
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include <stdexcept>

namespace msd_sim
{

AssetEnvironment::AssetEnvironment(uint32_t assetId,
                                   uint32_t instanceId,
                                   ConvexHull& hull,
                                   const ReferenceFrame& frame)
  : AssetPhysical{assetId, instanceId, hull, frame}

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
    coefficient_of_restitution_{coefficientOfRestitution}
{
  // Validate restitution
  if (coefficientOfRestitution < 0.0 || coefficientOfRestitution > 1.0)
  {
    throw std::invalid_argument(
      "AssetEnvironment: coefficient of restitution must be in [0, 1] (e = " +
      std::to_string(coefficientOfRestitution) + ")");
  }

  // Initialize static state with position from frame, zero velocities
  static_state_.position = frame.getOrigin();
  static_state_.orientation = frame.getQuaternion();
  // Velocities already default to zero in InertialState
}

void AssetEnvironment::setCoefficientOfRestitution(double e)
{
  if (e < 0.0 || e > 1.0)
  {
    throw std::invalid_argument(
      "AssetEnvironment: coefficient of restitution must be in [0, 1] (e = " +
      std::to_string(e) + ")");
  }
  coefficient_of_restitution_ = e;
}

}  // namespace msd_sim

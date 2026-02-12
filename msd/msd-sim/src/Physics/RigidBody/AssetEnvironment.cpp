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
  : AssetPhysical{assetId, instanceId, hull, frame}
{
  // Validate and set restitution via MaterialProperties
  material_.setCoefficientOfRestitution(coefficientOfRestitution);

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
  : AssetPhysical{assetId, instanceId, hull, frame}
{
  // Validate and set material properties via MaterialProperties
  material_.setCoefficientOfRestitution(coefficientOfRestitution);
  material_.setFrictionCoefficient(frictionCoefficient);

  static_state_.position = frame.getOrigin();
  static_state_.orientation = frame.getQuaternion();
}

void AssetEnvironment::setFrictionCoefficient(double mu)
{
  material_.setFrictionCoefficient(mu);
}

void AssetEnvironment::setCoefficientOfRestitution(double e)
{
  material_.setCoefficientOfRestitution(e);
}

}  // namespace msd_sim

#include <stdexcept>

#include "msd-assets/src/Geometry.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialCalculations.hpp"

namespace msd_sim
{

AssetInertial::AssetInertial(uint32_t assetId,
                             uint32_t instanceId,
                             ConvexHull& hull,
                             double mass,
                             const ReferenceFrame& frame)
  : AssetPhysical{assetId, instanceId, hull, frame},
    mass_{mass},
    inertiaTensor_{Eigen::Matrix3d::Zero()},
    inverseInertiaTensor_{Eigen::Matrix3d::Zero()},
    centerOfMass_{Coordinate{0, 0, 0}},
    dynamicState_{}
{
  if (mass <= 0.0)
  {
    throw std::invalid_argument("Mass must be positive, got: " +
                                std::to_string(mass));
  }

  // Compute inertia tensor about the centroid
  inertiaTensor_ = InertialCalculations::computeInertiaTensorAboutCentroid(
    getCollisionHull(), mass);

  // Compute inverse inertia tensor for physics calculations
  inverseInertiaTensor_ = inertiaTensor_.inverse();
}

double AssetInertial::getMass() const
{
  return mass_;
}

const Eigen::Matrix3d& AssetInertial::getInertiaTensor() const
{
  return inertiaTensor_;
}


const InertialState& AssetInertial::getInertialState() const
{
  return dynamicState_;
}

InertialState& AssetInertial::getInertialState()
{
  return dynamicState_;
}


const Eigen::Matrix3d& AssetInertial::getInverseInertiaTensor() const
{
  return inverseInertiaTensor_;
}

// ========== Force Application API (ticket 0023a) ==========

void AssetInertial::applyForce(const Coordinate& force)
{
  accumulatedForce_ += force;
  // Ticket: 0023a_force_application_scaffolding
}

void AssetInertial::applyForceAtPoint(const Coordinate& force,
                                       [[maybe_unused]] const Coordinate& worldPoint)
{
  accumulatedForce_ += force;

  // TODO (ticket 0023): Compute torque from r × F
  // Coordinate r = worldPoint - getReferenceFrame().getOrigin();
  // Coordinate torqueVec = r.cross(force);
  // accumulatedTorque_ += torqueVec;

  // Ticket: 0023a_force_application_scaffolding
}

void AssetInertial::applyTorque(const Coordinate& torque)
{
  accumulatedTorque_ += torque;
  // Ticket: 0023a_force_application_scaffolding
}

void AssetInertial::clearForces()
{
  accumulatedForce_ = Coordinate{0.0, 0.0, 0.0};
  accumulatedTorque_ = Coordinate{0.0, 0.0, 0.0};
  // Ticket: 0023a_force_application_scaffolding
}

const Coordinate& AssetInertial::getAccumulatedForce() const
{
  return accumulatedForce_;
}

const Coordinate& AssetInertial::getAccumulatedTorque() const
{
  return accumulatedTorque_;
}

}  // namespace msd_sim

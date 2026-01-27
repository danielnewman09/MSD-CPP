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
    dynamicState_{},
    coefficientOfRestitution_{0.5}  // Default: moderate elasticity
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

  // Initialize InertialState position and orientation from ReferenceFrame
  dynamicState_.position = frame.getOrigin();
  dynamicState_.orientation = frame.getAngularCoordinate();
  // Ticket: 0023_force_application_system
}

AssetInertial::AssetInertial(uint32_t assetId,
                             uint32_t instanceId,
                             ConvexHull& hull,
                             double mass,
                             const ReferenceFrame& frame,
                             double coefficientOfRestitution)
  : AssetPhysical{assetId, instanceId, hull, frame},
    mass_{mass},
    inertiaTensor_{Eigen::Matrix3d::Zero()},
    inverseInertiaTensor_{Eigen::Matrix3d::Zero()},
    centerOfMass_{Coordinate{0, 0, 0}},
    dynamicState_{},
    coefficientOfRestitution_{coefficientOfRestitution}
{
  if (mass <= 0.0)
  {
    throw std::invalid_argument("Mass must be positive, got: " +
                                std::to_string(mass));
  }

  if (coefficientOfRestitution < 0.0 || coefficientOfRestitution > 1.0)
  {
    throw std::invalid_argument(
      "Coefficient of restitution must be in [0, 1], got: " +
      std::to_string(coefficientOfRestitution));
  }

  // Compute inertia tensor about the centroid
  inertiaTensor_ = InertialCalculations::computeInertiaTensorAboutCentroid(
    getCollisionHull(), mass);

  // Compute inverse inertia tensor for physics calculations
  inverseInertiaTensor_ = inertiaTensor_.inverse();

  // Initialize InertialState position and orientation from ReferenceFrame
  dynamicState_.position = frame.getOrigin();
  dynamicState_.orientation = frame.getAngularCoordinate();
  // Ticket: 0027_collision_response_system
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

void AssetInertial::applyForce(const CoordinateRate& force)
{
  accumulatedForce_ += force;
  // Ticket: 0023a_force_application_scaffolding
}

void AssetInertial::applyForceAtPoint(const CoordinateRate& force,
                                      const Coordinate& worldPoint)
{
  // Accumulate linear force
  accumulatedForce_ += force;

  // Compute torque: τ = r × F
  // r is the vector from center of mass to application point
  Coordinate r = worldPoint - getReferenceFrame().getOrigin();
  Coordinate torque = r.cross(force);

  // Accumulate torque
  accumulatedTorque_ += torque;

  // Ticket: 0023_force_application_system
}

void AssetInertial::applyTorque(const CoordinateRate& torque)
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

const CoordinateRate& AssetInertial::getAccumulatedForce() const
{
  return accumulatedForce_;
}

const CoordinateRate& AssetInertial::getAccumulatedTorque() const
{
  return accumulatedTorque_;
}

// ========== Coefficient of Restitution (ticket 0027) ==========

double AssetInertial::getCoefficientOfRestitution() const
{
  return coefficientOfRestitution_;
}

void AssetInertial::setCoefficientOfRestitution(double e)
{
  if (e < 0.0 || e > 1.0)
  {
    throw std::invalid_argument(
      "Coefficient of restitution must be in [0, 1], got: " +
      std::to_string(e));
  }
  coefficientOfRestitution_ = e;
}

// ========== Impulse Application API (ticket 0027) ==========

void AssetInertial::applyImpulse(const Coordinate& impulse)
{
  // Impulse directly modifies velocity: Δv = J / m
  // Unlike forces, this is timestep-independent
  dynamicState_.velocity += impulse / mass_;
  // Ticket: 0027_collision_response_system
}

void AssetInertial::applyAngularImpulse(const AngularRate& angularImpulse)
{
  // Angular impulse directly modifies angular velocity: Δω = I⁻¹ * L
  // Unlike torques, this is timestep-independent
  dynamicState_.angularVelocity += angularImpulse;
  // Ticket: 0027_collision_response_system
}

}  // namespace msd_sim

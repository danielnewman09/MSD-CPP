#include "msd-sim/src/Physics/RigidBody/PhysicsComponent.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialCalculations.hpp"
#include <stdexcept>

namespace msd_sim
{

PhysicsComponent::PhysicsComponent(const ConvexHull& hull, double mass)
  : mass_{mass},
    inertiaTensor_{Eigen::Matrix3d::Zero()},
    inverseInertiaTensor_{Eigen::Matrix3d::Zero()},
    centerOfMass_{hull.getCentroid()},
    dynamicState_{}
{
  if (mass <= 0.0)
  {
    throw std::invalid_argument("Mass must be positive, got: " +
                                std::to_string(mass));
  }

  if (!hull.isValid())
  {
    throw std::invalid_argument(
      "Cannot create PhysicsComponent from invalid convex hull");
  }

  // Compute inertia tensor about the centroid using InertialCalculations
  inertiaTensor_ =
    InertialCalculations::computeInertiaTensorAboutCentroid(hull, mass);

  // Compute inverse inertia tensor for physics calculations
  inverseInertiaTensor_ = inertiaTensor_.inverse();
}

double PhysicsComponent::getKineticEnergy() const
{
  return dynamicState_.getTotalKineticEnergy(mass_, inertiaTensor_);
}

void PhysicsComponent::applyForce(const Coordinate& force)
{
  // F = ma => a = F/m
  Coordinate acceleration = force / mass_;

  // Add to current acceleration (allows multiple forces per frame)
  Coordinate currentAccel = dynamicState_.getLinearAcceleration();
  dynamicState_.setLinearAcceleration(currentAccel + acceleration);
}

void PhysicsComponent::applyForceAtPoint(const Coordinate& force,
                                         const Coordinate& localOffset)
{
  // Apply linear component
  applyForce(force);

  // Compute torque: τ = r × F
  Eigen::Vector3d r(localOffset.x(), localOffset.y(), localOffset.z());
  Eigen::Vector3d f(force.x(), force.y(), force.z());
  Eigen::Vector3d torque = r.cross(f);

  // Apply angular component
  applyTorque(torque);
}

void PhysicsComponent::applyTorque(const Eigen::Vector3d& torque)
{
  // τ = Iα => α = I⁻¹τ
  Eigen::Vector3d angularAccel = inverseInertiaTensor_ * torque;

  // Add to current angular acceleration
  Eigen::Vector3d currentAngularAccel =
    dynamicState_.getAngularAcceleration();
  dynamicState_.setAngularAcceleration(currentAngularAccel + angularAccel);
}

void PhysicsComponent::clearForces()
{
  dynamicState_.setLinearAcceleration(Coordinate(0, 0, 0));
  dynamicState_.setAngularAcceleration(Eigen::Vector3d::Zero());
}

}  // namespace msd_sim

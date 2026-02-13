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
    staticState_{mass, {}},
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
  inertiaTensor_ = inertial_calculations::computeInertiaTensorAboutCentroid(
    getCollisionHull(), mass);

  // Compute inverse inertia tensor for physics calculations
  inverseInertiaTensor_ = inertiaTensor_.inverse();

  // Initialize InertialState position and orientation from ReferenceFrame
  dynamicState_.inertialState.position = frame.getOrigin();
  dynamicState_.inertialState.orientation = frame.getQuaternion();
  dynamicState_.inertialState.quaternionRate =
    Eigen::Vector4d{0.0, 0.0, 0.0, 0.0};
  // Ticket: 0030_lagrangian_quaternion_physics

  // Ticket: 0045_constraint_solver_unification
  // Removed default UnitQuaternionConstraint - quaternion normalization
  // now handled by integrator via state.orientation.normalize()
}

AssetInertial::AssetInertial(uint32_t assetId,
                             uint32_t instanceId,
                             ConvexHull& hull,
                             double mass,
                             const ReferenceFrame& frame,
                             double coefficientOfRestitution)
  : AssetPhysical{assetId, instanceId, hull, frame},
    staticState_{mass, {}},
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

  // Validate and set restitution via MaterialProperties
  staticState_.material.setCoefficientOfRestitution(coefficientOfRestitution);

  // Compute inertia tensor about the centroid
  inertiaTensor_ = inertial_calculations::computeInertiaTensorAboutCentroid(
    getCollisionHull(), mass);

  // Compute inverse inertia tensor for physics calculations
  inverseInertiaTensor_ = inertiaTensor_.inverse();

  // Initialize InertialState position and orientation from ReferenceFrame
  dynamicState_.inertialState.position = frame.getOrigin();
  dynamicState_.inertialState.orientation = frame.getQuaternion();
  dynamicState_.inertialState.quaternionRate =
    Eigen::Vector4d{0.0, 0.0, 0.0, 0.0};
  // Ticket: 0030_lagrangian_quaternion_physics

  // Ticket: 0045_constraint_solver_unification
  // Removed default UnitQuaternionConstraint - quaternion normalization
  // now handled by integrator via state.orientation.normalize()
}

AssetInertial::AssetInertial(uint32_t assetId,
                             uint32_t instanceId,
                             ConvexHull& hull,
                             double mass,
                             const ReferenceFrame& frame,
                             double coefficientOfRestitution,
                             double frictionCoefficient)
  : AssetPhysical{assetId, instanceId, hull, frame},
    staticState_{mass, {}},
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

  // Validate and set material properties via MaterialProperties
  staticState_.material.setCoefficientOfRestitution(coefficientOfRestitution);
  staticState_.material.setFrictionCoefficient(frictionCoefficient);

  inertiaTensor_ = inertial_calculations::computeInertiaTensorAboutCentroid(
    getCollisionHull(), mass);
  inverseInertiaTensor_ = inertiaTensor_.inverse();

  dynamicState_.inertialState.position = frame.getOrigin();
  dynamicState_.inertialState.orientation = frame.getQuaternion();
  dynamicState_.inertialState.quaternionRate =
    Eigen::Vector4d{0.0, 0.0, 0.0, 0.0};
}

double AssetInertial::getMass() const
{
  return staticState_.mass;
}

double AssetInertial::getInverseMass() const
{
  return 1.0 / staticState_.mass;
}

const Eigen::Matrix3d& AssetInertial::getInertiaTensor() const
{
  return inertiaTensor_;
}


AssetDynamicState& AssetInertial::getDynamicState()
{
  return dynamicState_;
}

const AssetDynamicState& AssetInertial::getDynamicState() const
{
  return dynamicState_;
}

const InertialState& AssetInertial::getInertialState() const
{
  return dynamicState_.inertialState;
}

InertialState& AssetInertial::getInertialState()
{
  return dynamicState_.inertialState;
}


const Eigen::Matrix3d& AssetInertial::getInverseInertiaTensor() const
{
  return inverseInertiaTensor_;
}

Eigen::Matrix3d AssetInertial::getInverseInertiaTensorWorld() const
{
  Eigen::Matrix3d r =
    dynamicState_.inertialState.orientation.toRotationMatrix();
  return r * inverseInertiaTensor_ * r.transpose();
}

// ========== Force Application API (ticket 0023a) ==========

void AssetInertial::applyForce(const ForceVector& force)
{
  dynamicState_.externalForces.emplace_back(
    std::move(force), TorqueVector{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 0.0});
}

void AssetInertial::applyForceAtPoint(const ForceVector& force,
                                      const Coordinate& worldPoint)
{
  // Compute torque: τ = r × F
  // r is the vector from center of mass to application point
  Coordinate const r = worldPoint - getReferenceFrame().getOrigin();
  Coordinate const torque = r.cross(force);

  dynamicState_.externalForces.push_back(
    ExternalForce{ForceVector{force}, TorqueVector{torque}, worldPoint});
}

void AssetInertial::applyTorque(const TorqueVector& torque)
{
  dynamicState_.externalForces.emplace_back(
    ForceVector{0.0, 0.0, 0.0}, std::move(torque), Coordinate{0.0, 0.0, 0.0});
}

void AssetInertial::clearForces()
{
  dynamicState_.externalForces.clear();
}

ForceVector AssetInertial::getAccumulatedForce() const
{
  return ExternalForce::accumulate(dynamicState_.externalForces).force;
}

TorqueVector AssetInertial::getAccumulatedTorque() const
{
  return ExternalForce::accumulate(dynamicState_.externalForces).torque;
}

// ========== Coefficient of Restitution (ticket 0027) ==========

double AssetInertial::getCoefficientOfRestitution() const
{
  return staticState_.material.coefficientOfRestitution;
}

void AssetInertial::setCoefficientOfRestitution(double e)
{
  staticState_.material.setCoefficientOfRestitution(e);
}

// ========== Friction Coefficient (ticket 0052d) ==========

double AssetInertial::getFrictionCoefficient() const
{
  return staticState_.material.frictionCoefficient;
}

void AssetInertial::setFrictionCoefficient(double mu)
{
  staticState_.material.setFrictionCoefficient(mu);
}

// ========== Impulse Application API (ticket 0027) ==========

void AssetInertial::applyImpulse(const Vector3D& impulse)
{
  // Impulse directly modifies velocity: Δv = J / m
  // Unlike forces, this is timestep-independent
  dynamicState_.inertialState.velocity += impulse / staticState_.mass;
  // Ticket: 0027_collision_response_system
}

void AssetInertial::applyAngularImpulse(const AngularVelocity& angularImpulse)
{
  // Angular impulse directly modifies angular velocity: Δω = I⁻¹ * L
  // Unlike torques, this is timestep-independent
  AngularVelocity omega = dynamicState_.inertialState.getAngularVelocity();
  omega += angularImpulse;
  dynamicState_.inertialState.setAngularVelocity(omega);
  // Ticket: 0027_collision_response_system (updated for quaternions, ticket
  // 0030)
}

// Constraint management removed (ticket 0058_constraint_ownership_cleanup)

// ========== Transfer Object Support ==========

msd_transfer::AssetDynamicStateRecord AssetInertial::toDynamicStateRecord()
  const
{
  return dynamicState_.toRecord(instanceId_);
}

msd_transfer::AssetInertialStaticRecord AssetInertial::toStaticRecord() const
{
  return staticState_.toRecord(instanceId_);
}

}  // namespace msd_sim

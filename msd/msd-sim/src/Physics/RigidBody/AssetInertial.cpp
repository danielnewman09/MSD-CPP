#include <stdexcept>

#include "msd-assets/src/Geometry.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
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
  inertiaTensor_ = inertial_calculations::computeInertiaTensorAboutCentroid(
    getCollisionHull(), mass);

  // Compute inverse inertia tensor for physics calculations
  inverseInertiaTensor_ = inertiaTensor_.inverse();

  // Initialize InertialState position and orientation from ReferenceFrame
  dynamicState_.position = frame.getOrigin();
  dynamicState_.orientation = frame.getQuaternion();
  dynamicState_.quaternionRate = Eigen::Vector4d{0.0, 0.0, 0.0, 0.0};
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
  inertiaTensor_ = inertial_calculations::computeInertiaTensorAboutCentroid(
    getCollisionHull(), mass);

  // Compute inverse inertia tensor for physics calculations
  inverseInertiaTensor_ = inertiaTensor_.inverse();

  // Initialize InertialState position and orientation from ReferenceFrame
  dynamicState_.position = frame.getOrigin();
  dynamicState_.orientation = frame.getQuaternion();
  dynamicState_.quaternionRate = Eigen::Vector4d{0.0, 0.0, 0.0, 0.0};
  // Ticket: 0030_lagrangian_quaternion_physics

  // Ticket: 0045_constraint_solver_unification
  // Removed default UnitQuaternionConstraint - quaternion normalization
  // now handled by integrator via state.orientation.normalize()
}

double AssetInertial::getMass() const
{
  return mass_;
}

double AssetInertial::getInverseMass() const
{
  return 1.0 / mass_;
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

Eigen::Matrix3d AssetInertial::getInverseInertiaTensorWorld() const
{
  Eigen::Matrix3d r = dynamicState_.orientation.toRotationMatrix();
  return r * inverseInertiaTensor_ * r.transpose();
}

// ========== Force Application API (ticket 0023a) ==========

void AssetInertial::applyForce(const msd_sim::Vector3D& force)
{
  accumulatedForce_ += force;
  // Ticket: 0023a_force_application_scaffolding
}

void AssetInertial::applyForceAtPoint(const msd_sim::Vector3D& force,
                                      const Coordinate& worldPoint)
{
  // Accumulate linear force
  accumulatedForce_ += force;

  // Compute torque: τ = r × F
  // r is the vector from center of mass to application point
  Coordinate const r = worldPoint - getReferenceFrame().getOrigin();
  Coordinate const torque = r.cross(force);

  // Accumulate torque
  accumulatedTorque_ += torque;

  // Ticket: 0023_force_application_system
}

void AssetInertial::applyTorque(const msd_sim::Vector3D& torque)
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

const msd_sim::Vector3D& AssetInertial::getAccumulatedForce() const
{
  return accumulatedForce_;
}

const msd_sim::Vector3D& AssetInertial::getAccumulatedTorque() const
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
  AngularRate omega = dynamicState_.getAngularVelocity();
  omega += angularImpulse;
  dynamicState_.setAngularVelocity(omega);
  // Ticket: 0027_collision_response_system (updated for quaternions, ticket
  // 0030)
}

// ========== Constraint Management (ticket 0031) ==========

void AssetInertial::addConstraint(std::unique_ptr<Constraint> constraint)
{
  constraints_.push_back(std::move(constraint));
}

void AssetInertial::removeConstraint(size_t index)
{
  if (index >= constraints_.size())
  {
    throw std::out_of_range(
      "Constraint index out of range: " + std::to_string(index) +
      " >= " + std::to_string(constraints_.size()));
  }
  constraints_.erase(constraints_.begin() + static_cast<std::ptrdiff_t>(index));
}

std::vector<Constraint*> AssetInertial::getConstraints()
{
  std::vector<Constraint*> result;
  result.reserve(constraints_.size());
  for (auto& constraint : constraints_)
  {
    result.push_back(constraint.get());
  }
  return result;
}

std::vector<const Constraint*> AssetInertial::getConstraints() const
{
  std::vector<const Constraint*> result;
  result.reserve(constraints_.size());
  for (const auto& constraint : constraints_)
  {
    result.push_back(constraint.get());
  }
  return result;
}

void AssetInertial::clearConstraints()
{
  constraints_.clear();
}

size_t AssetInertial::getConstraintCount() const
{
  return constraints_.size();
}

}  // namespace msd_sim

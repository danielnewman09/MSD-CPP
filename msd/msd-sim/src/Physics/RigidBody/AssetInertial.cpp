#include <stdexcept>

#include "msd-assets/src/Geometry.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialCalculations.hpp"

namespace msd_sim
{

AssetInertial::AssetInertial(
  std::shared_ptr<msd_assets::CollisionGeometry> geometry,
  double mass,
  const ReferenceFrame& frame)
  : AssetPhysical{geometry, frame},
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

const DynamicState& AssetInertial::getDynamicState() const
{
  return dynamicState_;
}

DynamicState& AssetInertial::getDynamicState()
{
  return dynamicState_;
}

double AssetInertial::getMass() const
{
  return mass_;
}

double AssetInertial::getKineticEnergy() const
{
  return dynamicState_.getTotalKineticEnergy(mass_, inertiaTensor_);
}

const Eigen::Matrix3d& AssetInertial::getInertiaTensor() const
{
  return inertiaTensor_;
}

const Eigen::Matrix3d& AssetInertial::getInverseInertiaTensor() const
{
  return inverseInertiaTensor_;
}

}  // namespace msd_sim

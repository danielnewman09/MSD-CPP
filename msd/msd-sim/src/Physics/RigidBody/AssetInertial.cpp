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

}  // namespace msd_sim

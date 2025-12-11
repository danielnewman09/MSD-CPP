#include "PhysicalAsset.hpp"
#include "msd-assets/src/Geometry.hpp"
#include <stdexcept>
#include <sstream>

namespace msd_sim {

std::shared_ptr<PhysicalAsset> PhysicalAsset::create(
  std::shared_ptr<msd_assets::Geometry> geometry,
  float mass,
  bool computeHull)
{
  if (!geometry) {
    throw std::invalid_argument("Cannot create PhysicalAsset with null geometry");
  }

  if (geometry->getVertexCount() == 0) {
    throw std::invalid_argument("Cannot create PhysicalAsset with empty geometry");
  }

  if (mass <= 0.0f) {
    throw std::invalid_argument("Mass must be positive");
  }

  return std::shared_ptr<PhysicalAsset>(
    new PhysicalAsset(geometry, nullptr, mass, computeHull));
}

std::shared_ptr<PhysicalAsset> PhysicalAsset::createWithCustomHull(
  std::shared_ptr<msd_assets::Geometry> geometry,
  std::shared_ptr<ConvexHull> collisionHull,
  float mass)
{
  if (!geometry) {
    throw std::invalid_argument("Cannot create PhysicalAsset with null geometry");
  }

  if (!collisionHull) {
    throw std::invalid_argument("Cannot create PhysicalAsset with null collision hull");
  }

  if (mass <= 0.0f) {
    throw std::invalid_argument("Mass must be positive");
  }

  return std::shared_ptr<PhysicalAsset>(
    new PhysicalAsset(geometry, collisionHull, mass, true));
}

PhysicalAsset::PhysicalAsset(std::shared_ptr<msd_assets::Geometry> geometry,
                             std::shared_ptr<ConvexHull> customHull,
                             float mass,
                             bool computeHullNow)
  : visualGeometry_(geometry)
  , collisionHull_(customHull)
  , hullComputed_(customHull != nullptr)
{
  if (computeHullNow && !hullComputed_) {
    computeConvexHull();
  }

  // Compute physics properties if we have a hull, otherwise defer
  if (hullComputed_) {
    computePhysicsProperties(mass);
  } else {
    // Create temporary properties with given mass
    // These will be recomputed when hull is computed
    physicsProps_ = RigidBodyProperties(
      mass,
      Eigen::Matrix3f::Identity(),
      Coordinate(0, 0, 0));
  }
}

const msd_assets::Geometry& PhysicalAsset::getVisualGeometry() const
{
  return *visualGeometry_;
}

const ConvexHull& PhysicalAsset::getCollisionHull() const
{
  if (!hullComputed_) {
    computeConvexHull();
    // Update physics properties now that we have the hull
    const_cast<PhysicalAsset*>(this)->computePhysicsProperties(physicsProps_.getMass());
  }
  return *collisionHull_;
}

const RigidBodyProperties& PhysicalAsset::getPhysicsProperties() const
{
  // Ensure hull is computed (which ensures properties are up-to-date)
  if (!hullComputed_) {
    getCollisionHull();
  }
  return physicsProps_;
}

void PhysicalAsset::setMass(float mass)
{
  if (mass <= 0.0f) {
    throw std::invalid_argument("Mass must be positive");
  }

  // Ensure hull is computed before updating mass
  ensureHullComputed();

  computePhysicsProperties(mass);
}

float PhysicalAsset::getMass() const
{
  return physicsProps_.getMass();
}

bool PhysicalAsset::isHullComputed() const
{
  return hullComputed_;
}

void PhysicalAsset::ensureHullComputed() const
{
  if (!hullComputed_) {
    computeConvexHull();
    const_cast<PhysicalAsset*>(this)->computePhysicsProperties(physicsProps_.getMass());
  }
}

size_t PhysicalAsset::estimateMemoryUsage() const
{
  size_t total = 0;

  // Visual geometry memory
  total += visualGeometry_->getVertexCount() * sizeof(Coordinate);

  // Collision hull memory (if computed)
  if (hullComputed_) {
    total += collisionHull_->getVertexCount() * sizeof(Coordinate);
    total += collisionHull_->getFacetCount() * sizeof(ConvexHull::Facet);
  }

  // Physics properties (fixed size)
  total += sizeof(RigidBodyProperties);

  return total;
}

void PhysicalAsset::computeConvexHull() const
{
  try {
    collisionHull_ = std::make_shared<ConvexHull>(
      ConvexHull::fromGeometry(*visualGeometry_));
    hullComputed_ = true;
  } catch (const std::exception& e) {
    std::ostringstream oss;
    oss << "Failed to compute convex hull for PhysicalAsset: " << e.what();
    throw std::runtime_error(oss.str());
  }
}

void PhysicalAsset::computePhysicsProperties(float mass)
{
  if (!hullComputed_) {
    throw std::logic_error(
      "Cannot compute physics properties before convex hull is computed");
  }

  // Compute inertia tensor from convex hull
  physicsProps_ = RigidBodyProperties::fromConvexHull(*collisionHull_, mass);
}

}  // namespace msd_sim

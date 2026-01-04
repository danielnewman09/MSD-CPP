#include "msd-sim/src/Environment/Object.hpp"
#include <stdexcept>

namespace msd_sim
{

// ========== Private Constructor ==========

Object::Object(
  Type type,
  std::optional<std::reference_wrapper<const msd_assets::Asset>> asset,
  const ReferenceFrame& frame,
  std::optional<PhysicsComponent> physics,
  std::optional<ConvexHull> collisionHull,
  float r,
  float g,
  float b)
  : type_{type},
    asset_{asset},
    transform_{frame},
    physics_{std::move(physics)},
    collisionHull_{std::move(collisionHull)},
    color_{r, g, b}
{
}

// ========== Factory Methods ==========

Object Object::createGraphical(const msd_assets::Asset& asset,
                               const ReferenceFrame& frame,
                               float r,
                               float g,
                               float b)
{
  // Verify asset has collision geometry
  if (!asset.hasVisualGeometry())
  {
    throw std::invalid_argument(
      "Cannot create Visual object: asset lacks visual geometry");
  }
  return Object(Type::Graphical,
                std::cref(asset),
                frame,
                std::nullopt,  // No physics
                std::nullopt,  // No collision
                r,
                g,
                b);
}

Object Object::createInertial(const msd_assets::Asset& asset,
                              const ReferenceFrame& frame,
                              double mass,
                              float r,
                              float g,
                              float b)
{
  // Verify asset has collision geometry
  if (!asset.hasCollisionGeometry())
  {
    throw std::invalid_argument(
      "Cannot create Inertial object: asset lacks collision geometry");
  }

  // Get collision geometry and create convex hull
  const auto& collisionGeom = asset.getCollisionGeometry().value().get();
  ConvexHull hull(collisionGeom);

  // Create physics component from hull and mass
  PhysicsComponent physics(hull, mass);

  return Object(Type::Inertial,
                std::cref(asset),
                frame,
                std::move(physics),
                std::move(hull),
                r,
                g,
                b);
}

Object Object::createEnvironmental(const msd_assets::Asset& asset,
                                   const ReferenceFrame& frame,
                                   float r,
                                   float g,
                                   float b)
{
  // Verify asset has collision geometry
  if (!asset.hasCollisionGeometry())
  {
    throw std::invalid_argument(
      "Cannot create Environmental object: asset lacks collision geometry");
  }

  // Get collision geometry and create convex hull
  const auto& collisionGeom = asset.getCollisionGeometry().value().get();
  ConvexHull hull(collisionGeom);

  return Object(Type::Environmental,
                std::cref(asset),
                frame,
                std::nullopt,  // No physics
                std::move(hull),
                r,
                g,
                b);
}

Object Object::createBoundary(const ConvexHull& collisionHull,
                              const ReferenceFrame& frame)
{
  // Create a copy of the hull
  ConvexHull hull = collisionHull;

  return Object(Type::Boundary,
                std::nullopt,  // No asset
                frame,
                std::nullopt,  // No physics
                std::move(hull),
                0.0f,
                0.0f,
                0.0f);  // Color doesn't matter for invisible boundary
}

// ========== Component Access ==========

bool Object::hasVisualGeometry() const
{
  if (!asset_.has_value())
  {
    return false;
  }
  return asset_.value().get().hasVisualGeometry();
}

std::optional<std::reference_wrapper<const msd_assets::Asset>>
Object::getAsset() const
{
  return asset_;
}

const msd_assets::VisualGeometry* Object::getVisualGeometry() const
{
  if (!asset_.has_value())
  {
    return nullptr;
  }

  auto visualGeom = asset_.value().get().getVisualGeometry();
  if (visualGeom.has_value())
  {
    return &visualGeom.value().get();
  }
  return nullptr;
}

const ConvexHull& Object::getCollisionHull() const
{
  if (!collisionHull_.has_value())
  {
    throw std::runtime_error("Object does not have a collision hull (type: " +
                             std::to_string(static_cast<int>(type_)) + ")");
  }
  return collisionHull_.value();
}

ConvexHull& Object::getCollisionHull()
{
  if (!collisionHull_.has_value())
  {
    throw std::runtime_error("Object does not have a collision hull (type: " +
                             std::to_string(static_cast<int>(type_)) + ")");
  }
  return collisionHull_.value();
}

const PhysicsComponent& Object::getPhysics() const
{
  if (!physics_.has_value())
  {
    throw std::runtime_error(
      "Object does not have a physics component (type: " +
      std::to_string(static_cast<int>(type_)) + ")");
  }
  return physics_.value();
}

PhysicsComponent& Object::getPhysics()
{
  if (!physics_.has_value())
  {
    throw std::runtime_error(
      "Object does not have a physics component (type: " +
      std::to_string(static_cast<int>(type_)) + ")");
  }
  return physics_.value();
}

// ========== Transform and Appearance ==========

void Object::setColor(float r, float g, float b)
{
  color_[0] = r;
  color_[1] = g;
  color_[2] = b;
}

void Object::getColor(float& r, float& g, float& b) const
{
  r = color_[0];
  g = color_[1];
  b = color_[2];
}

}  // namespace msd_sim

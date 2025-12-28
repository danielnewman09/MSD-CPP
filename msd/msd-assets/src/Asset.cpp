#include "msd-assets/src/Asset.hpp"
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include "msd-assets/src/Geometry.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

namespace msd_assets
{

// Private constructor
Asset::Asset(uint32_t id,
             std::string name,
             std::string category,
             std::optional<VisualGeometry> visualGeom,
             std::optional<CollisionGeometry> collisionGeom)
  : id_{id},
    name_{std::move(name)},
    category_{std::move(category)},
    visualGeometry_{std::move(visualGeom)},
    collisionGeometry_{std::move(collisionGeom)}
{
}

// Static factory method
Asset Asset::fromObjectRecord(msd_transfer::ObjectRecord& record,
                              cpp_sqlite::Database& db)
{
  std::optional<VisualGeometry> visualGeom = std::nullopt;
  std::optional<CollisionGeometry> collisionGeom = std::nullopt;

  // Load visual geometry if foreign key is set
  if (record.meshRecord.isSet())
  {
    visualGeom =
      VisualGeometry::fromMeshRecord(record.meshRecord.resolve(db)->get());
  }

  // Load collision geometry if foreign key is set
  if (record.collisionMeshRecord.isSet())
  {
    collisionGeom = CollisionGeometry::fromMeshRecord(
      record.collisionMeshRecord.resolve(db)->get());
  }

  return Asset{
    record.id, record.name, record.category, visualGeom, collisionGeom};
}

uint32_t Asset::getId() const
{
  return id_;
}

const std::string& Asset::getName() const
{
  return name_;
}

const std::string& Asset::getCategory() const
{
  return category_;
}

std::optional<std::reference_wrapper<const VisualGeometry>>
Asset::getVisualGeometry() const
{
  if (visualGeometry_.has_value())
  {
    return std::cref(visualGeometry_.value());
  }
  return std::nullopt;
}

std::optional<std::reference_wrapper<const CollisionGeometry>>
Asset::getCollisionGeometry() const
{
  if (collisionGeometry_.has_value())
  {
    return std::cref(collisionGeometry_.value());
  }
  return std::nullopt;
}

bool Asset::hasVisualGeometry() const
{
  return visualGeometry_.has_value();
}

bool Asset::hasCollisionGeometry() const
{
  return collisionGeometry_.has_value();
}

}  // namespace msd_assets

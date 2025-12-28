#include "AssetRegistry.hpp"
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <stdexcept>
#include "Geometry.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

namespace msd_assets
{

AssetRegistry& AssetRegistry::getInstance()
{
  static AssetRegistry instance;
  return instance;
}

void AssetRegistry::loadFromDatabase(const std::string& dbPath)
{
  std::lock_guard<std::mutex> lock(cacheMutex_);

  // Get logger instance
  auto& logger = cpp_sqlite::Logger::getInstance();

  // Create database connection (read-only mode)
  database_ =
    std::make_unique<cpp_sqlite::Database>(dbPath, false, logger.getLogger());

  // Load all object records and build name->ID maps
  auto& objectDAO = database_->getDAO<msd_transfer::ObjectRecord>();
  auto objectRecords = objectDAO.selectAll();

  for (const auto& objRecord : objectRecords)
  {
    // Map visual mesh: object name -> mesh record ID
    if (objRecord.meshRecord.id != 0)
    {
      visualIdMap_[objRecord.name] = objRecord.meshRecord.id;
    }

    // Map collision mesh: object name -> collision mesh record ID
    if (objRecord.collisionMeshRecord.id != 0)
    {
      collisionIdMap_[objRecord.name] = objRecord.collisionMeshRecord.id;
    }
  }
}

std::optional<std::reference_wrapper<const VisualGeometry>>
AssetRegistry::loadVisualGeometry(const std::string& objectName)
{
  std::lock_guard<std::mutex> lock(cacheMutex_);

  // Check if already loaded in geometry map
  auto geomIt = visualGeometryMap_.find(objectName);
  if (geomIt != visualGeometryMap_.end())
  {
    return std::cref(geomIt->second);
  }

  // Not loaded yet - check if object exists in ID map
  auto idIt = visualIdMap_.find(objectName);
  if (idIt == visualIdMap_.end())
  {
    return std::nullopt;  // Object not found or has no visual mesh
  }

  // Load from database
  if (!database_)
  {
    return std::nullopt;  // Database not initialized
  }

  // Load MeshRecord using the mapped ID
  auto& meshDAO = database_->getDAO<msd_transfer::MeshRecord>();
  auto meshRecordOpt = meshDAO.selectById(idIt->second);

  if (!meshRecordOpt.has_value())
  {
    return std::nullopt;  // Record not found in database
  }

  // Deserialize to VisualGeometry
  auto geometry = VisualGeometry::fromMeshRecord(meshRecordOpt.value());

  // Insert into geometry map and return reference
  auto result = visualGeometryMap_.emplace(objectName, std::move(geometry));
  return std::cref(result.first->second);
}

std::optional<std::reference_wrapper<const CollisionGeometry>>
AssetRegistry::loadCollisionGeometry(const std::string& objectName)
{
  std::lock_guard<std::mutex> lock(cacheMutex_);

  // Check if already loaded in geometry map
  auto geomIt = collisionGeometryMap_.find(objectName);
  if (geomIt != collisionGeometryMap_.end())
  {
    return std::cref(geomIt->second);
  }

  // Not loaded yet - check if object exists in ID map
  auto idIt = collisionIdMap_.find(objectName);
  if (idIt == collisionIdMap_.end())
  {
    return std::nullopt;  // Object not found or has no collision mesh
  }

  // Load from database
  if (!database_)
  {
    return std::nullopt;  // Database not initialized
  }

  // Load CollisionMeshRecord using the mapped ID
  auto& collisionDAO = database_->getDAO<msd_transfer::CollisionMeshRecord>();
  auto collisionRecordOpt = collisionDAO.selectById(idIt->second);

  if (!collisionRecordOpt.has_value())
  {
    return std::nullopt;  // Record not found in database
  }

  // Deserialize to CollisionGeometry
  auto geometry =
    CollisionGeometry::fromMeshRecord(collisionRecordOpt.value());

  // Insert into geometry map and return reference
  auto result = collisionGeometryMap_.emplace(objectName, std::move(geometry));
  return std::cref(result.first->second);
}

size_t AssetRegistry::getCacheMemoryUsage() const
{
  std::lock_guard<std::mutex> lock(cacheMutex_);

  size_t totalBytes = 0;

  // Calculate visual geometry cache size
  for (const auto& [name, geometry] : visualGeometryMap_)
  {
    // Visual vertices: 36 bytes each (Vertex struct)
    totalBytes += geometry.getVertexCount() * 36;
    // Add string overhead
    totalBytes += name.size();
  }

  // Calculate collision geometry cache size
  for (const auto& [name, geometry] : collisionGeometryMap_)
  {
    // Hull vertices: 24 bytes each (3 doubles)
    totalBytes += geometry.getVertexCount() * 24;
    // BoundingBox: 7 floats = 28 bytes
    totalBytes += 28;
    // Add string overhead
    totalBytes += name.size();
  }

  return totalBytes;
}


}  // namespace msd_assets

#include "msd-assets/src/AssetRegistry.hpp"
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <stdexcept>
#include "msd-assets/src/Geometry.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

namespace msd_assets
{


AssetRegistry::AssetRegistry(const std::string& dbPath)
  : database_{std::make_unique<cpp_sqlite::Database>(
      dbPath,
      false,
      cpp_sqlite::Logger::getInstance().getLogger())}

{
  loadFromDatabase();
}

void AssetRegistry::loadFromDatabase()
{
  std::lock_guard<std::mutex> lock(cacheMutex_);

  // Load all object records and create Asset instances
  auto& objectDAO = database_->getDAO<msd_transfer::ObjectRecord>();
  auto objectRecords = objectDAO.selectAll();

  for (auto objRecord : objectRecords)
  {
    // Create Asset from ObjectRecord using factory method
    Asset asset = Asset::fromObjectRecord(objRecord, *database_);

    // Cache the asset by name
    assetCache_.emplace(objRecord.name, std::move(asset));
  }
}

std::optional<std::reference_wrapper<const Asset>>
AssetRegistry::getAsset(const std::string& objectName)
{
  std::lock_guard<std::mutex> lock(cacheMutex_);

  // Check if asset exists in cache
  auto assetIt = assetCache_.find(objectName);
  if (assetIt != assetCache_.end())
  {
    return std::cref(assetIt->second);
  }

  return std::nullopt;  // Asset not found
}

std::optional<std::reference_wrapper<const VisualGeometry>>
AssetRegistry::loadVisualGeometry(const std::string& objectName)
{
  std::lock_guard<std::mutex> lock(cacheMutex_);

  // Find asset in cache
  auto assetIt = assetCache_.find(objectName);
  if (assetIt == assetCache_.end())
  {
    return std::nullopt;  // Asset not found
  }

  // Return visual geometry from the asset
  return assetIt->second.getVisualGeometry();
}

std::optional<std::reference_wrapper<const CollisionGeometry>>
AssetRegistry::loadCollisionGeometry(const std::string& objectName)
{
  std::lock_guard<std::mutex> lock(cacheMutex_);

  // Find asset in cache
  auto assetIt = assetCache_.find(objectName);
  if (assetIt == assetCache_.end())
  {
    return std::nullopt;  // Asset not found
  }

  // Return collision geometry from the asset
  return assetIt->second.getCollisionGeometry();
}

size_t AssetRegistry::getCacheMemoryUsage() const
{
  std::lock_guard<std::mutex> lock(cacheMutex_);

  size_t totalBytes = 0;

  // Calculate memory usage for all cached assets
  for (const auto& [name, asset] : assetCache_)
  {
    // Add string overhead for name
    totalBytes += name.size();

    // Add visual geometry if present
    if (asset.hasVisualGeometry())
    {
      auto visualGeom = asset.getVisualGeometry();
      if (visualGeom.has_value())
      {
        // Visual vertices: 36 bytes each (Vertex struct)
        totalBytes += visualGeom.value().get().getVertexCount() * 36;
      }
    }

    // Add collision geometry if present
    if (asset.hasCollisionGeometry())
    {
      auto collisionGeom = asset.getCollisionGeometry();
      if (collisionGeom.has_value())
      {
        // Collision vertices: 24 bytes each (3 doubles)
        totalBytes += collisionGeom.value().get().getVertexCount() * 24;
      }
    }

    // Add Asset object overhead (id, name, category strings)
    totalBytes += asset.getName().size() + asset.getCategory().size() +
                  sizeof(uint32_t);
  }

  return totalBytes;
}


}  // namespace msd_assets

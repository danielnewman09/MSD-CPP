
#include <algorithm>
#include <ranges>
#include <stdexcept>

#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>

#include "msd-assets/src/AssetRegistry.hpp"
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
  const std::scoped_lock<std::mutex> lock(cacheMutex_);

  // Load all object records and create Asset instances
  auto& objectDAO = database_->getDAO<msd_transfer::ObjectRecord>();
  auto objectRecords = objectDAO.selectAll();

  for (auto objRecord : objectRecords)
  {
    // Create Asset from ObjectRecord using factory method
    Asset asset = Asset::fromObjectRecord(objRecord, *database_);

    // Cache the asset
    assetCache_.push_back(std::move(asset));
  }
}

std::optional<std::reference_wrapper<const Asset>> AssetRegistry::getAsset(
  uint32_t assetId)
{
  const std::scoped_lock<std::mutex> lock(cacheMutex_);

  auto assetIt = findAssetById(assetId);
  if (assetIt != assetCache_.end())
  {
    return std::cref(*assetIt);
  }

  return std::nullopt;
}


std::optional<std::reference_wrapper<const Asset>> AssetRegistry::getAsset(
  const std::string& assetName)
{
  const std::scoped_lock<std::mutex> lock(cacheMutex_);

  auto assetIt = findAssetByName(assetName);
  if (assetIt != assetCache_.end())
  {
    return std::cref(*assetIt);
  }

  return std::nullopt;
}


std::optional<std::reference_wrapper<const VisualGeometry>>
AssetRegistry::loadVisualGeometry(uint32_t assetId)
{
  const std::scoped_lock<std::mutex> lock(cacheMutex_);

  auto assetIt = findAssetById(assetId);
  if (assetIt == assetCache_.end())
  {
    return std::nullopt;
  }

  return assetIt->getVisualGeometry();
}

std::optional<std::reference_wrapper<const CollisionGeometry>>
AssetRegistry::loadCollisionGeometry(uint32_t assetId)
{
  const std::scoped_lock<std::mutex> lock(cacheMutex_);

  auto assetIt = findAssetById(assetId);
  if (assetIt == assetCache_.end())
  {
    return std::nullopt;
  }

  return assetIt->getCollisionGeometry();
}

size_t AssetRegistry::getCacheMemoryUsage() const
{
  const std::scoped_lock<std::mutex> lock(cacheMutex_);

  size_t totalBytes = 0;

  for (const auto& asset : assetCache_)
  {
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
    totalBytes +=
      asset.getName().size() + asset.getCategory().size() + sizeof(uint32_t);
  }

  return totalBytes;
}

const std::vector<Asset>& AssetRegistry::getAssetCache() const
{
  return assetCache_;
}

std::vector<Asset>::iterator AssetRegistry::findAssetById(uint32_t assetId)
{
  return std::ranges::find_if(assetCache_,
                              [assetId](const Asset& asset)
                              { return asset.getId() == assetId; });
}

std::vector<Asset>::const_iterator AssetRegistry::findAssetById(
  uint32_t assetId) const
{
  return std::ranges::find_if(assetCache_,
                              [assetId](const Asset& asset)
                              { return asset.getId() == assetId; });
}

std::vector<Asset>::iterator AssetRegistry::findAssetByName(
  const std::string& assetName)
{
  return std::ranges::find_if(assetCache_,
                              [&assetName](const Asset& asset)
                              { return asset.getName() == assetName; });
}

std::vector<Asset>::const_iterator AssetRegistry::findAssetByName(
  const std::string& assetName) const
{
  return std::ranges::find_if(assetCache_,
                              [&assetName](const Asset& asset)
                              { return asset.getName() == assetName; });
}

}  // namespace msd_assets

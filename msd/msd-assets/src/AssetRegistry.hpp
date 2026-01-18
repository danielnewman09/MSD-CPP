#ifndef MSD_ASSETS_ASSET_REGISTRY_HPP
#define MSD_ASSETS_ASSET_REGISTRY_HPP

#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include "msd-assets/src/Asset.hpp"

namespace msd_assets
{

/**
 * @brief Singleton registry for managing in-memory geometry asset cache
 *
 * AssetRegistry provides lazy-loading and caching of geometry assets from the
 * database. Visual meshes are loaded on-demand when first accessed and cached
 * for subsequent use.
 *
 * Thread Safety:
 *   All public methods are thread-safe via internal mutex.
 */
class AssetRegistry
{
public:
  // Singleton pattern: private constructor
  AssetRegistry(const std::string& dbPath);

  /**
   * @brief Load asset from database (lazy-loaded and cached)
   * @param assetId ID of the asset to load
   * @return Optional reference to cached asset, std::nullopt if asset not
   * found
   */
  std::optional<std::reference_wrapper<const Asset>> getAsset(uint32_t assetId);

  /**
   * @brief Load asset from database (lazy-loaded and cached)
   * @param assetId ID of the asset to load
   * @return Optional reference to cached asset, std::nullopt if asset not
   * found
   */
  std::optional<std::reference_wrapper<const Asset>> getAsset(
    std::string assetName);


  /**
   * @brief Load visual geometry from database (lazy-loaded and cached)
   * @param assetId ID of the asset to load visual mesh for
   * @return Optional reference to cached visual geometry, std::nullopt if
   * asset not found or has no visual mesh
   */
  std::optional<std::reference_wrapper<const VisualGeometry>>
  loadVisualGeometry(uint32_t assetId);

  /**
   * @brief Load collision geometry from database (lazy-loaded and cached)
   * @param assetId ID of the asset to load collision mesh for
   * @return Optional reference to cached collision geometry, std::nullopt if
   * asset not found or has no collision mesh
   */
  std::optional<std::reference_wrapper<const CollisionGeometry>>
  loadCollisionGeometry(uint32_t assetId);

  /**
   * @brief Estimate current cache memory usage
   * @return Approximate memory usage in bytes
   */
  size_t getCacheMemoryUsage() const;

  const std::vector<Asset>& getAssetCache() const;

private:
  /**
   * @brief Initialize registry with database connection
   * @param dbPath Path to SQLite database file
   */
  void loadFromDatabase();


  // Delete copy constructor and assignment operator
  AssetRegistry(const AssetRegistry&) = delete;
  AssetRegistry& operator=(const AssetRegistry&) = delete;

  // Database connection
  std::unique_ptr<cpp_sqlite::Database> database_;

  // Cached complete assets loaded from database
  // Key: asset ID, Value: complete Asset with both visual and collision
  // geometry
  std::vector<Asset> assetCache_;

  // Thread safety for multi-threaded access
  mutable std::mutex cacheMutex_;

  /**
   * @brief Find asset in cache by ID
   * @param assetId ID of the asset to find
   * @return Iterator to the asset, or assetCache_.end() if not found
   */
  std::vector<Asset>::iterator findAssetById(uint32_t assetId);
  std::vector<Asset>::const_iterator findAssetById(uint32_t assetId) const;

  /**
   * @brief Find asset in cache by name
   * @param assetName Name of the asset to find
   * @return Iterator to the asset, or assetCache_.end() if not found
   */
  std::vector<Asset>::iterator findAssetByName(const std::string& assetName);
  std::vector<Asset>::const_iterator findAssetByName(
    const std::string& assetName) const;
};

}  // namespace msd_assets

#endif  // MSD_ASSETS_ASSET_REGISTRY_HPP

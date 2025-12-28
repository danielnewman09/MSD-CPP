#ifndef MSD_ASSETS_ASSET_REGISTRY_HPP
#define MSD_ASSETS_ASSET_REGISTRY_HPP

#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
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
  /**
   * @brief Get singleton instance
   */
  static AssetRegistry& getInstance();

  /**
   * @brief Initialize registry with database connection
   * @param dbPath Path to SQLite database file
   */
  void loadFromDatabase(const std::string& dbPath);

  /**
   * @brief Load visual geometry from database (lazy-loaded and cached)
   * @param objectName Name of the object to load visual mesh for
   * @return Optional reference to cached visual geometry, std::nullopt if
   * object not found or has no visual mesh
   */
  std::optional<std::reference_wrapper<const VisualGeometry>>
  loadVisualGeometry(const std::string& objectName);

  /**
   * @brief Load collision geometry from database (lazy-loaded and cached)
   * @param objectName Name of the object to load collision mesh for
   * @return Optional reference to cached collision geometry, std::nullopt if
   * object not found or has no collision mesh
   */
  std::optional<std::reference_wrapper<const CollisionGeometry>>
  loadCollisionGeometry(const std::string& objectName);

  /**
   * @brief Estimate current cache memory usage
   * @return Approximate memory usage in bytes
   */
  size_t getCacheMemoryUsage() const;


private:
  // Singleton pattern: private constructor
  AssetRegistry() = default;

  // Delete copy constructor and assignment operator
  AssetRegistry(const AssetRegistry&) = delete;
  AssetRegistry& operator=(const AssetRegistry&) = delete;

  // Database connection
  std::unique_ptr<cpp_sqlite::Database> database_;

  // Cached associations between the string name and record id
  // in the database. Loaded on construction.
  std::unordered_map<std::string, uint32_t> visualIdMap_;
  std::unordered_map<std::string, uint32_t> collisionIdMap_;


  std::unordered_map<std::string, VisualGeometry> visualGeometryMap_;
  std::unordered_map<std::string, CollisionGeometry> collisionGeometryMap_;


  // Thread safety for multi-threaded access
  mutable std::mutex cacheMutex_;
};

}  // namespace msd_assets

#endif  // MSD_ASSETS_ASSET_REGISTRY_HPP

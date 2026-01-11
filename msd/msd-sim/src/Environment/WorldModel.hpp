// Ticket: 0021_worldmodel_asset_refactor
// Design: docs/designs/worldmodel-asset-refactor/design.md

#ifndef WORLD_MODEL_HPP
#define WORLD_MODEL_HPP

#include <chrono>
#include <optional>
#include <vector>
#include "msd-sim/src/Environment/Object.hpp"
#include "msd-sim/src/Environment/Platform.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

namespace msd_sim
{

/**
 * @brief Container and manager for all simulation objects
 * @ticket 0021_worldmodel_asset_refactor
 *
 * WorldModel uses typed storage for efficient physics updates and collision detection.
 * The refactored design separates static environment assets, dynamic physics assets,
 * and simulation boundaries into dedicated storage vectors.
 *
 * The WorldModel maintains:
 * - Typed vectors for environment assets (static collision objects)
 * - Typed vectors for inertial assets (dynamic physics objects)
 * - Optional simulation boundary (single ConvexHull)
 * - Deprecated: Unified Object storage (legacy, will be removed)
 *
 * Efficient iteration patterns:
 * @code
 * // Physics update - direct iteration over dynamic objects
 * for (auto& asset : worldModel.getInertialAssets()) {
 *   asset.getDynamicState().applyForce(...);
 * }
 *
 * // Collision detection - iterate moving objects vs static environment
 * for (auto& movingAsset : worldModel.getInertialAssets()) {
 *   for (const auto& staticAsset : worldModel.getEnvironmentAssets()) {
 *     // Check collision...
 *   }
 *   if (worldModel.hasBoundary()) {
 *     // Check collision with boundary...
 *   }
 * }
 * @endcode
 */
class WorldModel
{
public:
  WorldModel() = default;
  ~WorldModel() = default;

  // ========== Environment Asset Management ==========

  /**
   * @brief Add an environment asset (static collision object)
   * @param asset Environment asset to add (uses move semantics)
   * @return Index of added asset
   */
  size_t addEnvironmentAsset(AssetEnvironment&& asset);

  /**
   * @brief Get environment asset by index (const)
   * @throws std::out_of_range if index is invalid
   */
  const AssetEnvironment& getEnvironmentAsset(size_t index) const;

  /**
   * @brief Get environment asset by index (mutable)
   * @throws std::out_of_range if index is invalid
   */
  AssetEnvironment& getEnvironmentAsset(size_t index);

  /**
   * @brief Get all environment assets (const)
   */
  const std::vector<AssetEnvironment>& getEnvironmentAssets() const
  {
    return environmentAssets_;
  }

  /**
   * @brief Get all environment assets (mutable)
   */
  std::vector<AssetEnvironment>& getEnvironmentAssets()
  {
    return environmentAssets_;
  }

  /**
   * @brief Remove environment asset by index
   * @param index Index of asset to remove
   * @throws std::out_of_range if index is invalid
   */
  void removeEnvironmentAsset(size_t index);

  /**
   * @brief Get environment asset count
   */
  size_t getEnvironmentAssetCount() const { return environmentAssets_.size(); }

  // ========== Inertial Asset Management ==========

  /**
   * @brief Add an inertial asset (dynamic physics object)
   * @param asset Inertial asset to add (uses move semantics)
   * @return Index of added asset
   */
  size_t addInertialAsset(AssetInertial&& asset);

  /**
   * @brief Get inertial asset by index (const)
   * @throws std::out_of_range if index is invalid
   */
  const AssetInertial& getInertialAsset(size_t index) const;

  /**
   * @brief Get inertial asset by index (mutable)
   * @throws std::out_of_range if index is invalid
   */
  AssetInertial& getInertialAsset(size_t index);

  /**
   * @brief Get all inertial assets (const)
   */
  const std::vector<AssetInertial>& getInertialAssets() const
  {
    return inertialAssets_;
  }

  /**
   * @brief Get all inertial assets (mutable)
   */
  std::vector<AssetInertial>& getInertialAssets() { return inertialAssets_; }

  /**
   * @brief Remove inertial asset by index
   * @param index Index of asset to remove
   * @throws std::out_of_range if index is invalid
   */
  void removeInertialAsset(size_t index);

  /**
   * @brief Get inertial asset count
   */
  size_t getInertialAssetCount() const { return inertialAssets_.size(); }

  // ========== Boundary Management ==========

  /**
   * @brief Set simulation boundary
   * @param boundary Convex hull defining simulation boundary
   */
  void setBoundary(ConvexHull boundary);

  /**
   * @brief Get simulation boundary (const)
   * @return Optional reference to boundary hull
   */
  const std::optional<ConvexHull>& getBoundary() const { return simulationBoundary_; }

  /**
   * @brief Get simulation boundary (mutable)
   * @return Optional reference to boundary hull
   */
  std::optional<ConvexHull>& getBoundary() { return simulationBoundary_; }

  /**
   * @brief Check if simulation has a boundary
   */
  bool hasBoundary() const { return simulationBoundary_.has_value(); }

  /**
   * @brief Clear simulation boundary
   */
  void clearBoundary() { simulationBoundary_.reset(); }

  // ========== Object Management (DEPRECATED) ==========

  /**
   * @brief Spawn a new object in the world
   * @deprecated Use addEnvironmentAsset() or addInertialAsset() instead
   *
   * Adds the object to the world and updates internal index caches.
   *
   * @param object Object to add (uses move semantics)
   * @return Index of added object
   */
  [[deprecated("Use addEnvironmentAsset() or addInertialAsset() instead")]]
  size_t spawnObject(Object&& object);

  /**
   * @brief Get object by index (const)
   * @deprecated Use getEnvironmentAsset() or getInertialAsset() instead
   * @throws std::out_of_range if index is invalid
   */
  [[deprecated("Use getEnvironmentAsset() or getInertialAsset() instead")]]
  const Object& getObject(size_t index) const;

  /**
   * @brief Get object by index (mutable)
   * @deprecated Use getEnvironmentAsset() or getInertialAsset() instead
   * @throws std::out_of_range if index is invalid
   */
  [[deprecated("Use getEnvironmentAsset() or getInertialAsset() instead")]]
  Object& getObject(size_t index);

  /**
   * @brief Get all objects (const)
   * @deprecated Use getEnvironmentAssets() or getInertialAssets() instead
   */
  [[deprecated("Use getEnvironmentAssets() or getInertialAssets() instead")]]
  const std::vector<Object>& getObjects() const { return objects_; }

  /**
   * @brief Get all objects (mutable)
   * @deprecated Use getEnvironmentAssets() or getInertialAssets() instead
   */
  [[deprecated("Use getEnvironmentAssets() or getInertialAssets() instead")]]
  std::vector<Object>& getObjects() { return objects_; }

  /**
   * @brief Remove object by index
   * @deprecated Use removeEnvironmentAsset() or removeInertialAsset() instead
   *
   * Note: This invalidates indices > removed index.
   * Consider marking objects for removal and cleaning up between frames.
   *
   * @param index Index of object to remove
   * @throws std::out_of_range if index is invalid
   */
  [[deprecated("Use removeEnvironmentAsset() or removeInertialAsset() instead")]]
  void removeObject(size_t index);

  /**
   * @brief Get object count
   * @deprecated Use getEnvironmentAssetCount() or getInertialAssetCount() instead
   */
  [[deprecated("Use getEnvironmentAssetCount() or getInertialAssetCount() instead")]]
  size_t getObjectCount() const { return objects_.size(); }

  /**
   * @brief Clear all objects from the world
   * @deprecated Manually clear typed vectors instead
   */
  [[deprecated("Manually clear typed asset vectors instead")]]
  void clearObjects();

  // ========== Efficient Iteration Helpers (DEPRECATED) ==========

  /**
   * @brief Get indices of all objects with physics components
   * @deprecated Use getInertialAssets() for direct iteration instead
   *
   * Use this for physics integration loops.
   */
  [[deprecated("Use getInertialAssets() for direct iteration instead")]]
  const std::vector<size_t>& getPhysicsObjectIndices() const
  {
    return physicsObjectIndices_;
  }

  /**
   * @brief Get indices of all objects with collision hulls
   * @deprecated Use getInertialAssets() and getEnvironmentAssets() for direct iteration instead
   *
   * Use this for collision detection loops.
   */
  [[deprecated("Use getInertialAssets() and getEnvironmentAssets() for direct iteration instead")]]
  const std::vector<size_t>& getCollisionObjectIndices() const
  {
    return collisionObjectIndices_;
  }

  /**
   * @brief Get indices of all objects with visual geometry
   * @deprecated Rendering is handled by msd-gpu layer
   *
   * Use this for rendering loops.
   */
  [[deprecated("Rendering is handled by msd-gpu layer")]]
  const std::vector<size_t>& getRenderObjectIndices() const
  {
    return renderObjectIndices_;
  }

  // ========== Simulation Update ==========

  /**
   * @brief Update world simulation
   *
   * Performs:
   * 1. Physics integration for objects with PhysicsComponent
   * 2. Collision detection for objects with collision hulls
   * 3. Collision resolution
   * 4. Updates simulation time
   *
   * @param deltaTime Time step for integration
   */
  void update(std::chrono::milliseconds deltaTime);

  /**
   * @brief Get current simulation time
   */
  std::chrono::milliseconds getTime() const { return time_; }

  // ========== Platform Management (Legacy) ==========

  /**
   * @brief Add a platform to the world
   * @param platform Platform to add
   */
  void addPlatform(Platform&& platform);

  /**
   * @brief Get all platforms (const)
   */
  const std::vector<Platform>& getPlatforms() const { return platforms_; }

  /**
   * @brief Get all platforms (mutable)
   *
   * Allows modification of platforms for input command updates.
   */
  std::vector<Platform>& getPlatforms() { return platforms_; }

  /**
   * @brief Get next platform ID
   * @return Next available platform ID
   */
  uint32_t getNextPlatformId() const { return nextPlatformId_++; }

private:
  // ========== Internal Update Methods ==========

  /**
   * @brief Update physics for all dynamic objects
   * @param dt Time step in seconds
   */
  void updatePhysics(double dt);

  /**
   * @brief Detect and resolve collisions
   */
  void updateCollisions();

  /**
   * @brief Rebuild index caches for efficient iteration
   * @deprecated No longer needed with typed storage
   *
   * Call this after adding or removing objects.
   */
  [[deprecated("No longer needed with typed storage")]]
  void rebuildIndexCaches();

  // ========== Data ==========

  // Typed asset storage
  std::vector<AssetEnvironment> environmentAssets_;
  std::vector<AssetInertial> inertialAssets_;
  std::optional<ConvexHull> simulationBoundary_;

  // Deprecated: Unified object storage (will be removed in future)
  [[deprecated("Use environmentAssets_/inertialAssets_ instead")]]
  std::vector<Object> objects_;

  // Deprecated: Cached indices for efficient iteration (no longer needed)
  [[deprecated("No longer needed with typed storage")]]
  std::vector<size_t> physicsObjectIndices_;   // Objects with PhysicsComponent

  [[deprecated("No longer needed with typed storage")]]
  std::vector<size_t> collisionObjectIndices_; // Objects with ConvexHull

  [[deprecated("No longer needed with typed storage")]]
  std::vector<size_t> renderObjectIndices_;    // Objects with visual geometry

  // Legacy platform support
  std::vector<Platform> platforms_;
  mutable uint32_t nextPlatformId_{0};  // Mutable for getNextPlatformId() const

  // Current simulation time
  std::chrono::milliseconds time_{0};
};

}  // namespace msd_sim

#endif  // WORLD_MODEL_HPP

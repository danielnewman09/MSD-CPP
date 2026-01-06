#ifndef WORLD_MODEL_HPP
#define WORLD_MODEL_HPP

#include <chrono>
#include <vector>
#include "msd-sim/src/Environment/Object.hpp"
#include "msd-sim/src/Environment/Platform.hpp"

namespace msd_sim
{

/**
 * @brief Container and manager for all simulation objects
 *
 * WorldModel uses a unified storage approach with index-based iteration
 * for efficient physics updates and collision detection.
 *
 * The WorldModel maintains:
 * - A single vector of all objects (graphical, inertial, environmental,
 * boundary)
 * - Cached index lists for efficient iteration over specific object types
 * - Current simulation time
 *
 * Efficient iteration patterns:
 * @code
 * // Physics update - only iterate objects with physics
 * for (size_t idx : worldModel.getPhysicsObjectIndices()) {
 *   Object& obj = worldModel.getObject(idx);
 *   // Update physics...
 * }
 *
 * // Collision detection - only iterate objects with collision hulls
 * for (size_t idx : worldModel.getCollisionObjectIndices()) {
 *   // Check collisions...
 * }
 *
 * // Rendering - only iterate objects with visual geometry
 * for (size_t idx : worldModel.getRenderObjectIndices()) {
 *   // Render...
 * }
 * @endcode
 */
class WorldModel
{
public:
  WorldModel() = default;
  ~WorldModel() = default;

  // ========== Object Management ==========

  /**
   * @brief Spawn a new object in the world
   *
   * Adds the object to the world and updates internal index caches.
   *
   * @param object Object to add (uses move semantics)
   * @return Index of added object
   */
  size_t spawnObject(Object&& object);

  /**
   * @brief Get object by index (const)
   * @throws std::out_of_range if index is invalid
   */
  const Object& getObject(size_t index) const;

  /**
   * @brief Get object by index (mutable)
   * @throws std::out_of_range if index is invalid
   */
  Object& getObject(size_t index);

  /**
   * @brief Get all objects (const)
   */
  const std::vector<Object>& getObjects() const { return objects_; }

  /**
   * @brief Get all objects (mutable)
   */
  std::vector<Object>& getObjects() { return objects_; }

  /**
   * @brief Remove object by index
   *
   * Note: This invalidates indices > removed index.
   * Consider marking objects for removal and cleaning up between frames.
   *
   * @param index Index of object to remove
   * @throws std::out_of_range if index is invalid
   */
  void removeObject(size_t index);

  /**
   * @brief Get object count
   */
  size_t getObjectCount() const { return objects_.size(); }

  /**
   * @brief Clear all objects from the world
   */
  void clearObjects();

  // ========== Efficient Iteration Helpers ==========

  /**
   * @brief Get indices of all objects with physics components
   *
   * Use this for physics integration loops.
   */
  const std::vector<size_t>& getPhysicsObjectIndices() const
  {
    return physicsObjectIndices_;
  }

  /**
   * @brief Get indices of all objects with collision hulls
   *
   * Use this for collision detection loops.
   */
  const std::vector<size_t>& getCollisionObjectIndices() const
  {
    return collisionObjectIndices_;
  }

  /**
   * @brief Get indices of all objects with visual geometry
   *
   * Use this for rendering loops.
   */
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
   *
   * Call this after adding or removing objects.
   */
  void rebuildIndexCaches();

  // ========== Data ==========

  std::vector<Object> objects_;

  // Cached indices for efficient iteration
  std::vector<size_t> physicsObjectIndices_;   // Objects with PhysicsComponent
  std::vector<size_t> collisionObjectIndices_; // Objects with ConvexHull
  std::vector<size_t> renderObjectIndices_;    // Objects with visual geometry

  // Legacy platform support
  std::vector<Platform> platforms_;
  mutable uint32_t nextPlatformId_{0};  // Mutable for getNextPlatformId() const

  // Current simulation time
  std::chrono::milliseconds time_{0};
};

}  // namespace msd_sim

#endif  // WORLD_MODEL_HPP

#ifndef MSD_SIM_PHYSICS_PHYSICAL_ASSET_HPP
#define MSD_SIM_PHYSICS_PHYSICAL_ASSET_HPP

#include "msd-sim/src/Physics/ConvexHull.hpp"
#include "msd-sim/src/Physics/RigidBodyProperties.hpp"
#include <memory>

namespace msd_assets {
class Geometry;
}

namespace msd_sim {

/**
 * @brief Combines visual geometry with physics representations for efficient simulation.
 *
 * PhysicalAsset pairs a visual mesh (msd_assets::Geometry) with physics-optimized
 * representations (ConvexHull and RigidBodyProperties). This allows the same visual
 * asset to be shared across multiple entities while maintaining separate physics state.
 *
 * Key features:
 * - Lazy hull computation: Convex hull is computed once on first access
 * - Shared ownership: Multiple platforms can share the same PhysicalAsset
 * - Automatic mass property computation from convex hull geometry
 * - Clean separation between rendering (Geometry) and physics (ConvexHull)
 *
 * Usage pattern:
 * @code
 * auto geometry = std::make_shared<msd_assets::Geometry>(...);
 * auto asset = PhysicalAsset::create(geometry, 100.0f);  // 100 kg mass
 *
 * // Use for rendering
 * renderer.draw(asset->getVisualGeometry());
 *
 * // Use for physics
 * bool collision = asset->getCollisionHull().contains(point);
 * applyForce(asset->getPhysicsProperties());
 * @endcode
 */
class PhysicalAsset
{
public:
  /**
   * @brief Create a PhysicalAsset from visual geometry with automatic hull computation.
   *
   * This factory method is the preferred way to create PhysicalAssets. It computes
   * the convex hull and physics properties automatically from the visual geometry.
   *
   * @param geometry Shared pointer to visual geometry (must not be null)
   * @param mass Total mass in kilograms [kg]
   * @param computeHull If true, compute convex hull immediately; if false, defer until first access
   * @return Shared pointer to the created PhysicalAsset
   * @throws std::invalid_argument if geometry is null or empty
   */
  static std::shared_ptr<PhysicalAsset> create(
    std::shared_ptr<msd_assets::Geometry> geometry,
    float mass,
    bool computeHull = false);

  /**
   * @brief Create a PhysicalAsset with a custom collision hull.
   *
   * Use this when you want a simplified collision hull that differs from the
   * visual geometry's convex hull (e.g., a box collider for a complex mesh).
   *
   * @param geometry Shared pointer to visual geometry
   * @param collisionHull Custom convex hull for collision detection
   * @param mass Total mass in kilograms [kg]
   * @return Shared pointer to the created PhysicalAsset
   */
  static std::shared_ptr<PhysicalAsset> createWithCustomHull(
    std::shared_ptr<msd_assets::Geometry> geometry,
    std::shared_ptr<ConvexHull> collisionHull,
    float mass);

  /**
   * @brief Get the visual geometry for rendering.
   * @return Reference to the visual geometry
   */
  const msd_assets::Geometry& getVisualGeometry() const;

  /**
   * @brief Get the collision hull for physics simulation.
   *
   * If the hull hasn't been computed yet (deferred computation), this will
   * compute it on first access.
   *
   * @return Reference to the collision convex hull
   * @throws std::runtime_error if hull computation fails
   */
  const ConvexHull& getCollisionHull() const;

  /**
   * @brief Get the rigid body physics properties.
   * @return Reference to the physics properties (mass, inertia, etc.)
   */
  const RigidBodyProperties& getPhysicsProperties() const;

  /**
   * @brief Update the mass and recompute physics properties.
   *
   * This updates the mass and recomputes the inertia tensor based on the
   * current convex hull geometry, maintaining the same shape but scaling
   * mass-dependent properties.
   *
   * @param mass New mass in kilograms [kg]
   */
  void setMass(float mass);

  /**
   * @brief Get the current mass.
   * @return Mass in kilograms [kg]
   */
  float getMass() const;

  /**
   * @brief Check if the convex hull has been computed.
   * @return true if hull is available, false if it will be computed on next access
   */
  bool isHullComputed() const;

  /**
   * @brief Force computation of the convex hull if not already computed.
   *
   * Useful for preloading physics assets to avoid hitches during gameplay.
   */
  void ensureHullComputed() const;

  /**
   * @brief Get memory usage estimate for this asset.
   *
   * Returns approximate memory in bytes used by visual geometry, hull, and properties.
   *
   * @return Estimated memory usage in bytes
   */
  size_t estimateMemoryUsage() const;

private:
  /**
   * @brief Private constructor - use factory methods instead.
   */
  PhysicalAsset(std::shared_ptr<msd_assets::Geometry> geometry,
                std::shared_ptr<ConvexHull> customHull,
                float mass,
                bool computeHullNow);

  /**
   * @brief Compute the convex hull from visual geometry (called lazily).
   */
  void computeConvexHull() const;

  /**
   * @brief Compute physics properties from the convex hull.
   * @param mass Mass to use for physics properties
   */
  void computePhysicsProperties(float mass);

  // Visual representation (shared across multiple instances)
  std::shared_ptr<msd_assets::Geometry> visualGeometry_;

  // Physics collision hull (computed lazily, then cached)
  mutable std::shared_ptr<ConvexHull> collisionHull_;

  // Rigid body physics properties (mass, inertia tensor, center of mass)
  RigidBodyProperties physicsProps_;

  // Flag for lazy computation
  mutable bool hullComputed_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_PHYSICAL_ASSET_HPP

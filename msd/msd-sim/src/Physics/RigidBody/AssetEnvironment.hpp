#ifndef MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP
#define MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP

#include <memory>
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"

namespace msd_sim
{

/**
 * @brief Stationary geometric element with no rigid body dynamics.
 *
 * AssetEnvironment represents static, immovable objects in the simulation
 * environment. These assets inherit visual geometry, convex hull, and
 * reference frame from AssetPhysical, but have no rigid body properties
 * and cannot be moved by forces or interactions.
 *
 * Key features:
 * - Visual geometry for rendering (inherited)
 * - Convex hull for collision detection (inherited)
 * - Reference frame defining position and orientation (inherited)
 * - Always stationary (no dynamics)
 * - Lightweight compared to InertialAsset (no mass properties)
 *
 * Typical use cases:
 * - Terrain and ground surfaces
 * - Buildings and structures
 * - Static obstacles
 * - Environmental boundaries
 * - Walls and barriers
 *
 * Usage pattern:
 * @code
 * auto geometry = std::make_shared<msd_assets::Geometry>(...);
 * ReferenceFrame frame(Coordinate(10, 0, 0));  // Position at x=10
 * auto asset = AssetEnvironment::create(geometry, frame);
 *
 * // Use for rendering
 * renderer.draw(asset->getVisualGeometry(), asset->getReferenceFrame());
 *
 * // Use for collision detection
 * if (asset->getCollisionHull().contains(point)) {
 *   // Handle collision with static environment
 * }
 * @endcode
 */
class AssetEnvironment : public AssetPhysical
{
public:
  /**
   * @brief Create an AssetEnvironment with automatic hull computation.
   *
   * This factory method creates a stationary AssetEnvironment at the
   * specified reference frame. The convex hull is computed from the visual
   * geometry.
   *
   * @param geometry Shared pointer to visual geometry (must not be null)
   * @param frame Reference frame defining position and orientation in world
   * @param computeHull If true, compute convex hull immediately; if false,
   *                    defer until first access
   * @return Shared pointer to the created AssetEnvironment
   * @throws std::invalid_argument if geometry is null or empty
   */
  static std::shared_ptr<AssetEnvironment> create(
    std::shared_ptr<msd_assets::Geometry> geometry,
    const ReferenceFrame& frame = ReferenceFrame(),
    bool computeHull = false);

  /**
   * @brief Create an AssetEnvironment with a custom collision hull.
   *
   * Use this when you want a simplified collision hull that differs from
   * the visual geometry's convex hull (e.g., a box collider for complex
   * terrain).
   *
   * @param geometry Shared pointer to visual geometry
   * @param collisionHull Custom convex hull for collision detection
   * @param frame Reference frame defining position and orientation in world
   * @return Shared pointer to the created AssetEnvironment
   */
  static std::shared_ptr<AssetEnvironment> createWithCustomHull(
    std::shared_ptr<msd_assets::Geometry> geometry,
    std::shared_ptr<ConvexHull> collisionHull,
    const ReferenceFrame& frame = ReferenceFrame());

private:
  /**
   * @brief Private constructor - use factory methods instead.
   */
  AssetEnvironment(std::shared_ptr<msd_assets::Geometry> geometry,
                   std::shared_ptr<ConvexHull> customHull,
                   const ReferenceFrame& frame,
                   bool computeHullNow);
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP

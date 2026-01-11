// Ticket: 0021_worldmodel_asset_refactor
// Design: docs/designs/worldmodel-asset-refactor/design.md

#ifndef MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP
#define MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP

#include <memory>
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"

namespace msd_sim
{

/**
 * @brief Stationary geometric element with no rigid body dynamics.
 * @ticket 0021_worldmodel_asset_refactor
 *
 * AssetEnvironment represents static, immovable objects in the simulation
 * environment. These assets inherit visual geometry, convex hull, and
 * reference frame from AssetPhysical, but have no rigid body properties
 * and cannot be moved by forces or interactions.
 *
 * This class uses value semantics for WorldModel storage.
 *
 * Key features:
 * - Visual geometry stored by value (inherited)
 * - Convex hull for collision detection (inherited)
 * - Reference frame defining position and orientation (inherited)
 * - Always stationary (no dynamics)
 * - Lightweight compared to AssetInertial (no mass properties)
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
 * // Create from collision geometry
 * msd_assets::CollisionGeometry geom{vertices};
 * ReferenceFrame frame{Coordinate{10, 0, 0}};  // Position at x=10
 * AssetEnvironment asset{std::move(geom), frame};
 *
 * // Add to WorldModel
 * worldModel.addEnvironmentAsset(std::move(asset));
 *
 * // Use for collision detection
 * if (asset.getCollisionHull().contains(point)) {
 *   // Handle collision with static environment
 * }
 * @endcode
 */
class AssetEnvironment : public AssetPhysical
{
public:
  /**
   * @brief Constructor for value-based WorldModel storage.
   *
   * Creates a stationary AssetEnvironment at the specified reference frame.
   * The convex hull is computed from the collision geometry.
   *
   * @param geometry Collision geometry (moved, value semantics)
   * @param frame Reference frame defining position and orientation in world
   * @throws std::invalid_argument if geometry is empty
   */
  explicit AssetEnvironment(msd_assets::CollisionGeometry&& geometry,
                            const ReferenceFrame& frame = ReferenceFrame());

  /**
   * @brief Create an AssetEnvironment with automatic hull computation.
   * @deprecated Use public constructor instead: AssetEnvironment{std::move(geometry), frame}
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
  [[deprecated("Use public constructor instead: AssetEnvironment{std::move(geometry), frame}")]]
  static std::shared_ptr<AssetEnvironment> create(
    std::shared_ptr<msd_assets::CollisionGeometry> geometry,
    const ReferenceFrame& frame = ReferenceFrame(),
    bool computeHull = false);

  /**
   * @brief Copy constructor (default)
   */
  AssetEnvironment(const AssetEnvironment&) = default;

  /**
   * @brief Copy assignment (default)
   */
  AssetEnvironment& operator=(const AssetEnvironment&) = default;

  /**
   * @brief Move constructor (default)
   */
  AssetEnvironment(AssetEnvironment&&) noexcept = default;

  /**
   * @brief Move assignment (default)
   */
  AssetEnvironment& operator=(AssetEnvironment&&) noexcept = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP

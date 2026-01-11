// Ticket: 0021_worldmodel_asset_refactor
// Design: docs/designs/worldmodel-asset-refactor/design.md

#ifndef MSD_SIM_PHYSICS_PHYSICAL_ASSET_HPP
#define MSD_SIM_PHYSICS_PHYSICAL_ASSET_HPP

#include "msd-assets/src/Geometry.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

namespace msd_sim
{

/**
 * @brief Base class for geometric elements in the simulation.
 * @ticket 0021_worldmodel_asset_refactor
 *
 * AssetPhysical is the base class for all geometric objects in the simulation,
 * providing visual geometry, collision detection via convex hull, and a
 * reference frame defining position and orientation in world space.
 *
 * This class uses value semantics for geometry storage, enabling efficient
 * storage in typed vectors without pointer indirection.
 *
 * This class serves as the foundation for:
 * - AssetEnvironment: Stationary objects with no dynamics
 * - AssetInertial: Dynamic objects with rigid body properties
 *
 * Key features:
 * - Visual geometry stored by value
 * - Convex hull for collision detection
 * - Reference frame defining position and orientation in world space
 * - Value semantics for WorldModel vector storage
 *
 * Usage pattern:
 * @code
 * // Create from CollisionGeometry
 * msd_assets::CollisionGeometry geom{vertices};
 * ReferenceFrame frame{Coordinate{10, 0, 0}};
 *
 * // Use via derived classes
 * AssetEnvironment envAsset{std::move(geom), frame};
 * AssetInertial inertialAsset{std::move(geom), 10.0, frame};
 *
 * // Access base class functionality
 * const auto& hull = envAsset.getCollisionHull();
 * const auto& frame = envAsset.getReferenceFrame();
 * @endcode
 */
class AssetPhysical
{
public:
  /**
   * @brief Constructor with collision geometry and reference frame.
   *
   * @param geometry Collision geometry (moved, value semantics)
   * @param frame Reference frame defining position and orientation
   */
  explicit AssetPhysical(msd_assets::CollisionGeometry&& geometry,
                         const ReferenceFrame& frame = ReferenceFrame());

  /**
   * @brief Virtual destructor for proper cleanup of derived classes.
   */
  virtual ~AssetPhysical() = default;

  /**
   * @brief Copy constructor (default)
   */
  AssetPhysical(const AssetPhysical&) = default;

  /**
   * @brief Copy assignment (default)
   */
  AssetPhysical& operator=(const AssetPhysical&) = default;

  /**
   * @brief Move constructor (default)
   */
  AssetPhysical(AssetPhysical&&) noexcept = default;

  /**
   * @brief Move assignment (default)
   */
  AssetPhysical& operator=(AssetPhysical&&) noexcept = default;

  /**
   * @brief Get the collision hull for collision detection.
   * @return Reference to the collision convex hull
   */
  const ConvexHull& getCollisionHull() const;

  /**
   * @brief Get the reference frame defining position and orientation.
   * @return Reference to the reference frame
   */
  const ReferenceFrame& getReferenceFrame() const;

  /**
   * @brief Get the reference frame (mutable version).
   *
   * Use this to modify the asset's position and orientation.
   *
   * @return Mutable reference to the reference frame
   */
  ReferenceFrame& getReferenceFrame();

protected:
  // Visual geometry stored by value
  msd_assets::CollisionGeometry visualGeometry_;

  // Collision hull (computed from geometry)
  ConvexHull collisionHull_;

  // Reference frame defining position and orientation in world space
  ReferenceFrame referenceFrame_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_PHYSICAL_ASSET_HPP

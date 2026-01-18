#ifndef MSD_SIM_PHYSICS_PHYSICAL_ASSET_HPP
#define MSD_SIM_PHYSICS_PHYSICAL_ASSET_HPP

#include <memory>
#include "msd-assets/src/Geometry.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

namespace msd_sim
{

/**
 * @brief Base class for geometric elements in the simulation.
 *
 * AssetPhysical is the base class for all geometric objects in the simulation,
 * providing visual geometry, collision detection via convex hull, and a
 * reference frame defining position and orientation in world space.
 *
 * This class serves as the foundation for:
 * - EnvironmentAsset: Stationary objects with no dynamics
 * - InertialAsset: Dynamic objects with rigid body properties
 *
 * Key features:
 * - Visual geometry for rendering
 * - Convex hull for collision detection (computed lazily)
 * - Reference frame defining position and orientation in world space
 * - Shared ownership model for efficient memory usage
 *
 * Usage pattern:
 * @code
 * // Typically used via derived classes EnvironmentAsset or InertialAsset
 * auto geometry = std::make_shared<msd_assets::Geometry>(...);
 * ReferenceFrame frame(Coordinate(10, 0, 0));
 * auto asset = EnvironmentAsset::create(geometry, frame);
 *
 * // Access base class functionality
 * const auto& hull = asset->getCollisionHull();
 * const auto& frame = asset->getReferenceFrame();
 * @endcode
 */
class AssetPhysical
{
public:
  /**
   * @brief Protected constructor - use derived class factory methods.
   *
   * @param geometry Shared pointer to visual geometry
   * @param customHull Optional custom collision hull (if null, computed from
   * geometry)
   */
  AssetPhysical(uint32_t assetId,
                uint32_t instanceId,
                ConvexHull& hull,
                const ReferenceFrame& frame);

  /**
   * @brief Virtual destructor for proper cleanup of derived classes.
   */
  virtual ~AssetPhysical() = default;

  /**
   * @brief Get the collision hull for collision detection.
   *
   * If the hull hasn't been computed yet (deferred computation), this will
   * compute it on first access.
   *
   * @return Reference to the collision convex hull
   * @throws std::runtime_error if hull computation fails
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

  uint32_t getAssetId() const;

  uint32_t getInstanceId() const;

protected:
  // The identifier for the asset from which this object was built
  uint32_t referenceAssetId_;
  
  uint32_t instanceId_;
  // Collision hull (computed lazily, then cached)
  const ConvexHull& collisionHull_;

  // Reference frame defining position and orientation in world space
  ReferenceFrame referenceFrame_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_PHYSICAL_ASSET_HPP

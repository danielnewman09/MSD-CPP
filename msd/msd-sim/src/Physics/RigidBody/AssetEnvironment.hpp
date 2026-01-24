#ifndef MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP
#define MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP

#include <memory>
#include "msd-assets/src/Geometry.hpp"
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
   * @brief Protected constructor - use derived class factory methods.
   *
   * @param geometry Shared pointer to visual geometry
   * @param customHull Optional custom collision hull (if null, computed from
   * geometry)
   */
  AssetEnvironment(uint32_t assetId,
                   uint32_t instanceId,
                   ConvexHull& hull,
                   const ReferenceFrame& frame);
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP

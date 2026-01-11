// Ticket: 0021_worldmodel_asset_refactor
// Design: docs/designs/worldmodel-asset-refactor/design.md

#ifndef MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP
#define MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP

#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
#include "msd-sim/src/Physics/RigidBody/DynamicState.hpp"

namespace msd_sim
{

/**
 * @brief Dynamic geometric element with full rigid body physics.
 * @ticket 0021_worldmodel_asset_refactor
 *
 * AssetInertial extends AssetPhysical by adding rigid body dynamics
 * properties (mass, inertia tensor, center of mass) and dynamic state
 * (velocities and accelerations). These assets can be acted upon by forces,
 * collide with other objects, and move according to the laws of physics.
 *
 * This class uses value semantics for WorldModel storage.
 *
 * Key features:
 * - Visual geometry stored by value (inherited)
 * - Convex hull for collision detection (inherited)
 * - Reference frame defining position and orientation (inherited)
 * - Rigid body properties (mass, inertia tensor, center of mass)
 * - Dynamic state (linear/angular velocity and acceleration)
 * - Can participate in physics simulations
 * - Automatic mass property computation from geometry
 *
 * Typical use cases:
 * - Movable objects (boxes, spheres, etc.)
 * - Vehicles and characters
 * - Projectiles
 * - Debris and destructible objects
 * - Any object that responds to forces and collisions
 *
 * Usage pattern:
 * @code
 * // Create from collision geometry
 * msd_assets::CollisionGeometry geom{vertices};
 * ReferenceFrame frame{Coordinate{5, 0, 10}};  // Initial position
 * AssetInertial asset{std::move(geom), 50.0, frame};  // 50 kg mass
 *
 * // Add to WorldModel
 * worldModel.addInertialAsset(std::move(asset));
 *
 * // Use for physics
 * asset.getDynamicState().setLinearVelocity(Coordinate{1, 0, 0});
 *
 * // Update position and orientation after physics step
 * asset.getReferenceFrame().setOrigin(newPosition);
 * @endcode
 */
class AssetInertial : public AssetPhysical
{
public:
  /**
   * @brief Constructor for value-based WorldModel storage.
   *
   * Creates a dynamic physics asset with the specified mass.
   * The inertia tensor is automatically computed from the collision geometry.
   *
   * @param geometry Collision geometry (moved, value semantics)
   * @param mass Mass in kilograms [kg] (must be positive)
   * @param frame Initial reference frame defining position and orientation
   * @throws std::invalid_argument if mass <= 0 or geometry is empty
   */
  explicit AssetInertial(msd_assets::CollisionGeometry&& geometry,
                         double mass,
                         const ReferenceFrame& frame = ReferenceFrame());

  /**
   * @brief Get the dynamic state (velocities and accelerations).
   * @return Reference to the dynamic state
   */
  const DynamicState& getDynamicState() const;

  /**
   * @brief Get the dynamic state (mutable version).
   *
   * Use this to modify velocities and accelerations during physics
   * integration.
   *
   * @return Mutable reference to the dynamic state
   */
  DynamicState& getDynamicState();

  /**
   * @brief Get the current mass.
   * @return Mass in kilograms [kg]
   */
  double getMass() const;

  /**
   * @brief Get the total kinetic energy of this asset.
   *
   * Computes total KE = linear KE + rotational KE using mass, inertia,
   * and current velocities.
   *
   * @return Total kinetic energy [J]
   */
  double getKineticEnergy() const;

  /**
   * @brief Get the inertia tensor about the centroid.
   * @return 3x3 inertia tensor [kg⋅m²]
   */
  const Eigen::Matrix3d& getInertiaTensor() const;

  /**
   * @brief Get the inverse inertia tensor about the centroid.
   * @return 3x3 inverse inertia tensor [1/(kg⋅m²)]
   */
  const Eigen::Matrix3d& getInverseInertiaTensor() const;

  /**
   * @brief Copy constructor (default)
   */
  AssetInertial(const AssetInertial&) = default;

  /**
   * @brief Copy assignment (default)
   */
  AssetInertial& operator=(const AssetInertial&) = default;

  /**
   * @brief Move constructor (default)
   */
  AssetInertial(AssetInertial&&) noexcept = default;

  /**
   * @brief Move assignment (default)
   */
  AssetInertial& operator=(AssetInertial&&) noexcept = default;

private:
  // Rigid body physics properties
  double mass_;                    // Mass in kg
  Eigen::Matrix3d inertiaTensor_;  // Inertia tensor about centroid [kg⋅m²]
  Eigen::Matrix3d inverseInertiaTensor_;  // Inverse inertia tensor [1/(kg⋅m²)]
  Coordinate centerOfMass_;               // Center of mass in local coordinates

  // Dynamic state (linear/angular velocity and acceleration)
  DynamicState dynamicState_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP

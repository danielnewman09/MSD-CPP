#ifndef MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP
#define MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP

#include <memory>
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"


namespace msd_sim
{

/**
 * @brief Dynamic geometric element with full rigid body physics.
 *
 * AssetInertial extends AssertPhysical by adding rigid body dynamics
 * properties (mass, inertia tensor, center of mass) and dynamic state
 * (velocities and accelerations). These assets can be acted upon by forces,
 * collide with other objects, and move according to the laws of physics.
 *
 * Key features:
 * - Visual geometry for rendering (inherited)
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
 * auto geometry = std::make_shared<msd_assets::Geometry>(...);
 * ReferenceFrame frame(Coordinate(5, 0, 10));  // Initial position
 * auto asset = AssetInertial::create(geometry, 50.0, frame);  // 50 kg mass
 *
 * // Use for rendering
 * renderer.draw(asset->getVisualGeometry(), asset->getReferenceFrame());
 *
 * // Use for physics
 * const auto& props = asset->getPhysicsProperties();
 * asset->getDynamicState().setLinearVelocity(Coordinate(1, 0, 0));
 * applyForce(force, asset->getReferenceFrame());
 *
 * // Update position and orientation after physics step
 * asset->getReferenceFrame().setOrigin(newPosition);
 * @endcode
 */
class AssetInertial : public AssetPhysical
{
public:
  AssetInertial(uint32_t assetId,
                uint32_t instanceId,
                ConvexHull& hull,
                double mass,
                const ReferenceFrame& frame);

  /**
   * @brief Get the dynamic state (mutable version).
   *
   * Use this to modify velocities and accelerations during physics
   * integration.
   *
   * @return Mutable reference to the dynamic state
   */
  InertialState& getInertialState();

  const InertialState& getInertialState() const;

  /**
   * @brief Get the current mass.
   * @return Mass in kilograms [kg]
   */
  double getMass() const;

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

private:
  // Rigid body physics properties
  double mass_;                    // Mass in kg
  Eigen::Matrix3d inertiaTensor_;  // Inertia tensor about centroid [kg⋅m²]
  Eigen::Matrix3d inverseInertiaTensor_;  // Inverse inertia tensor [1/(kg⋅m²)]
  Coordinate centerOfMass_;               // Center of mass in local coordinates

  // Dynamic state (linear/angular velocity and acceleration)
  InertialState dynamicState_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP

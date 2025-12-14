#ifndef MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP
#define MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP

#include <memory>
#include "msd-sim/src/Physics/DynamicState.hpp"
#include "msd-sim/src/Physics/PhysicalAsset.hpp"
#include "msd-sim/src/Physics/RigidBodyProperties.hpp"

namespace msd_assets
{
class Geometry;
}

namespace msd_sim
{

/**
 * @brief Dynamic geometric element with full rigid body physics.
 *
 * InertialAsset extends PhysicalAsset by adding rigid body dynamics
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
 * auto asset = InertialAsset::create(geometry, 50.0, frame);  // 50 kg mass
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
class InertialAsset : public PhysicalAsset
{
public:
  /**
   * @brief Create an InertialAsset with automatic hull and physics
   * computation.
   *
   * This factory method creates an InertialAsset with the specified mass
   * at the given reference frame. The convex hull and rigid body properties
   * are computed from the visual geometry.
   *
   * @param geometry Shared pointer to visual geometry (must not be null)
   * @param mass Total mass in kilograms [kg]
   * @param frame Reference frame defining initial position and orientation
   * @param computeHull If true, compute convex hull immediately; if false,
   *                    defer until first access
   * @return Shared pointer to the created InertialAsset
   * @throws std::invalid_argument if geometry is null or empty, or mass <= 0
   */
  static std::shared_ptr<InertialAsset> create(
    std::shared_ptr<msd_assets::Geometry> geometry,
    double mass,
    const ReferenceFrame& frame = ReferenceFrame(),
    bool computeHull = false);

  /**
   * @brief Create an InertialAsset with a custom collision hull.
   *
   * Use this when you want a simplified collision hull that differs from
   * the visual geometry's convex hull (e.g., a sphere collider for a complex
   * mesh).
   *
   * @param geometry Shared pointer to visual geometry
   * @param collisionHull Custom convex hull for collision detection
   * @param mass Total mass in kilograms [kg]
   * @param frame Reference frame defining initial position and orientation
   * @return Shared pointer to the created InertialAsset
   * @throws std::invalid_argument if mass <= 0
   */
  static std::shared_ptr<InertialAsset> createWithCustomHull(
    std::shared_ptr<msd_assets::Geometry> geometry,
    std::shared_ptr<ConvexHull> collisionHull,
    double mass,
    const ReferenceFrame& frame = ReferenceFrame());

  /**
   * @brief Get the rigid body physics properties.
   * @return Reference to the physics properties (mass, inertia, etc.)
   */
  const RigidBodyProperties& getPhysicsProperties() const;

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
   * @brief Set the dynamic state.
   *
   * Updates all velocities and accelerations.
   *
   * @param state New dynamic state
   */
  void setDynamicState(const DynamicState& state);

  /**
   * @brief Update the mass and recompute physics properties.
   *
   * This updates the mass and recomputes the inertia tensor based on the
   * current convex hull geometry, maintaining the same shape but scaling
   * mass-dependent properties.
   *
   * @param mass New mass in kilograms [kg]
   * @throws std::invalid_argument if mass <= 0
   */
  void setMass(double mass);

  /**
   * @brief Get the current mass.
   * @return Mass in kilograms [kg]
   */
  double getMass() const;

  /**
   * @brief Check if the asset is currently at rest.
   *
   * Delegates to DynamicState::isAtRest() to check if velocities are
   * below thresholds.
   *
   * @param linearThreshold Linear velocity threshold [m/s] (default: 1e-6)
   * @param angularThreshold Angular velocity threshold [rad/s] (default: 1e-6)
   * @return true if asset is at rest, false otherwise
   */
  bool isAtRest(double linearThreshold = 1e-6,
                double angularThreshold = 1e-6) const;

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
   * @brief Get memory usage estimate for this asset.
   *
   * Returns approximate memory in bytes used by visual geometry, hull,
   * physics properties, dynamic state, and reference frame.
   *
   * @return Estimated memory usage in bytes
   */
  size_t estimateMemoryUsage() const override;

private:
  /**
   * @brief Private constructor - use factory methods instead.
   */
  InertialAsset(std::shared_ptr<msd_assets::Geometry> geometry,
                std::shared_ptr<ConvexHull> customHull,
                double mass,
                const ReferenceFrame& frame,
                bool computeHullNow);

  /**
   * @brief Compute physics properties from the convex hull.
   * @param mass Mass to use for physics properties
   */
  void computePhysicsProperties(double mass);

  // Rigid body physics properties (mass, inertia tensor, center of mass)
  RigidBodyProperties physicsProps_;

  // Dynamic state (linear/angular velocity and acceleration)
  DynamicState dynamicState_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP

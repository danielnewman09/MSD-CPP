#ifndef MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP
#define MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP

#include <memory>
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
#include "msd-sim/src/Physics/Constraints/QuaternionConstraint.hpp"


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
   * @brief Extended constructor with coefficient of restitution.
   *
   * @param assetId Asset type identifier
   * @param instanceId Unique instance identifier
   * @param hull Reference to collision hull
   * @param mass Mass in kilograms [kg]
   * @param frame Initial reference frame (position and orientation)
   * @param coefficientOfRestitution Elasticity [0, 1] (0=inelastic, 1=elastic)
   *
   * @throws std::invalid_argument if mass <= 0
   * @throws std::invalid_argument if coefficientOfRestitution ∉ [0, 1]
   *
   * @ticket 0027_collision_response_system
   */
  AssetInertial(uint32_t assetId,
                uint32_t instanceId,
                ConvexHull& hull,
                double mass,
                const ReferenceFrame& frame,
                double coefficientOfRestitution);

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

  // ========== NEW: Force Application API (ticket 0023a) ==========

  /**
   * @brief Apply a force at the center of mass.
   *
   * Placeholder implementation for ticket 0023a - only accumulates force.
   * Actual integration will be implemented in ticket 0023.
   *
   * @param force Force vector in world coordinates [N]
   */
  void applyForce(const CoordinateRate& force);

  /**
   * @brief Apply a force at a specific world-space point.
   *
   * Generates torque via τ = r × F, where r is the vector from the center
   * of mass to the application point. Forces at the center of mass generate
   * zero torque.
   *
   * @param force Force vector in world coordinates [N]
   * @param worldPoint Application point in world coordinates [m]
   */
  void applyForceAtPoint(const CoordinateRate& force,
                         const Coordinate& worldPoint);

  /**
   * @brief Apply a torque about the center of mass.
   *
   * Placeholder implementation for ticket 0023a - only accumulates torque.
   *
   * @param torque Torque vector in world coordinates [N·m]
   */
  void applyTorque(const CoordinateRate& torque);

  /**
   * @brief Clear all accumulated forces and torques.
   *
   * Call this at the end of each physics step after integration.
   */
  void clearForces();

  /**
   * @brief Get the accumulated force for this frame.
   * @return Accumulated force vector [N]
   */
  const CoordinateRate& getAccumulatedForce() const;

  /**
   * @brief Get the accumulated torque for this frame.
   * @return Accumulated torque vector [N·m]
   */
  const CoordinateRate& getAccumulatedTorque() const;

  // ========== NEW: Coefficient of Restitution (ticket 0027) ==========

  /**
   * @brief Get the coefficient of restitution.
   *
   * Determines elasticity of collisions:
   * - 0.0: Fully inelastic (objects stick together)
   * - 0.5: Moderate elasticity (default)
   * - 1.0: Fully elastic (perfect bounce)
   *
   * @return Coefficient of restitution [0, 1]
   */
  double getCoefficientOfRestitution() const;

  /**
   * @brief Set the coefficient of restitution.
   *
   * @param e Coefficient of restitution [0, 1]
   * @throws std::invalid_argument if e < 0.0 or e > 1.0
   *
   * @ticket 0027_collision_response_system
   */
  void setCoefficientOfRestitution(double e);

  // ========== NEW: Impulse Application API (ticket 0027) ==========

  /**
   * @brief Apply an instantaneous linear impulse at the center of mass.
   *
   * Unlike forces which are accumulated and integrated over time,
   * impulses directly modify velocity: Δv = J / m
   *
   * This is used for collision response where velocity changes
   * must be timestep-independent.
   *
   * @param impulse Impulse vector in world coordinates [N·s]
   *
   * @ticket 0027_collision_response_system
   */
  void applyImpulse(const Coordinate& impulse);

  /**
   * @brief Apply an instantaneous angular impulse.
   *
   * Unlike torques which are accumulated and integrated over time,
   * angular impulses directly modify angular velocity: Δω = I⁻¹ * L
   *
   * This is used for collision response where angular velocity changes
   * must be timestep-independent.
   *
   * @param angularImpulse Angular impulse vector in world coordinates [N·m·s]
   *
   * @ticket 0027_collision_response_system
   */
  void applyAngularImpulse(const AngularRate& angularImpulse);

  // ========== NEW: Quaternion Constraint (ticket 0030) ==========

  /**
   * @brief Get the quaternion constraint (mutable).
   *
   * Each asset owns its own constraint for better encapsulation.
   * Used by Integrator to enforce |Q|=1 during physics integration.
   *
   * @return Mutable reference to quaternion constraint
   *
   * @ticket 0030_lagrangian_quaternion_physics
   */
  QuaternionConstraint& getQuaternionConstraint();

  /**
   * @brief Get the quaternion constraint (const).
   * @return Const reference to quaternion constraint
   *
   * @ticket 0030_lagrangian_quaternion_physics
   */
  const QuaternionConstraint& getQuaternionConstraint() const;

private:
  // Rigid body physics properties
  double mass_;                    // Mass in kg
  Eigen::Matrix3d inertiaTensor_;  // Inertia tensor about centroid [kg⋅m²]
  Eigen::Matrix3d inverseInertiaTensor_;  // Inverse inertia tensor [1/(kg⋅m²)]
  Coordinate centerOfMass_;               // Center of mass in local coordinates

  // Dynamic state (linear/angular velocity and acceleration)
  InertialState dynamicState_;

  // NEW: Force accumulation (ticket 0023a)
  CoordinateRate accumulatedForce_{0.0, 0.0, 0.0};
  CoordinateRate accumulatedTorque_{0.0, 0.0, 0.0};

  // NEW: Coefficient of restitution (ticket 0027)
  double coefficientOfRestitution_{0.5};  // Default: moderate elasticity

  // NEW: Quaternion constraint (ticket 0030)
  QuaternionConstraint quaternionConstraint_{10.0, 10.0};  // Default Baumgarte parameters
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP

#ifndef MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP
#define MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP

#include <memory>
#include <vector>
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetDynamicState.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetStaticState.hpp"

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
   * @brief Extended constructor with coefficient of restitution and friction.
   *
   * @param assetId Asset type identifier
   * @param instanceId Unique instance identifier
   * @param hull Reference to collision hull
   * @param mass Mass in kilograms [kg]
   * @param frame Initial reference frame (position and orientation)
   * @param coefficientOfRestitution Elasticity [0, 1] (0=inelastic, 1=elastic)
   * @param frictionCoefficient Friction coefficient [0, inf) (default: 0.5)
   *
   * @throws std::invalid_argument if mass <= 0
   * @throws std::invalid_argument if coefficientOfRestitution not in [0, 1]
   * @throws std::invalid_argument if frictionCoefficient < 0
   *
   * @ticket 0052d_solver_integration_ecos_removal
   */
  AssetInertial(uint32_t assetId,
                uint32_t instanceId,
                ConvexHull& hull,
                double mass,
                const ReferenceFrame& frame,
                double coefficientOfRestitution,
                double frictionCoefficient);

  // Rule of Five: Move-only type due to std::unique_ptr<Constraint> ownership
  // Note: Move assignment deleted because base class has reference member
  // Ticket: 0031_generalized_lagrange_constraints
  ~AssetInertial() override = default;
  AssetInertial(const AssetInertial&) = delete;
  AssetInertial& operator=(const AssetInertial&) = delete;
  AssetInertial(AssetInertial&&) noexcept = default;
  AssetInertial& operator=(AssetInertial&&) noexcept = delete;

  /**
   * @brief Get the full dynamic state (mutable version).
   *
   * Provides access to the grouped per-frame state: kinematic state,
   * accumulated force, and accumulated torque.
   *
   * @return Mutable reference to the dynamic state
   * @ticket 0056h_asset_dynamic_state_struct
   */
  AssetDynamicState& getDynamicState();
  const AssetDynamicState& getDynamicState() const;

  /**
   * @brief Get the kinematic state (mutable version).
   *
   * Use this to modify velocities and accelerations during physics
   * integration.
   *
   * @return Mutable reference to the inertial state
   */
  InertialState& getInertialState();

  const InertialState& getInertialState() const;

  /**
   * @brief Get the current mass.
   * @return Mass in kilograms [kg]
   */
  double getMass() const;

  /**
   * @brief Get the inverse mass (convenience method).
   * @return Inverse mass [1/kg]
   * @ticket 0032_contact_constraint_refactor
   */
  double getInverseMass() const;

  /**
   * @brief Get the inertia tensor about the centroid.
   * @return 3x3 inertia tensor [kg⋅m²]
   */
  const Eigen::Matrix3d& getInertiaTensor() const;

  /**
   * @brief Get the inverse inertia tensor about the centroid (body frame).
   * @return 3x3 inverse inertia tensor [1/(kg⋅m²)]
   */
  const Eigen::Matrix3d& getInverseInertiaTensor() const;

  /**
   * @brief Get the inverse inertia tensor in world frame.
   *
   * Computes I⁻¹_world = R · I⁻¹_body · Rᵀ using the current orientation
   * quaternion. Use this when multiplying with world-frame quantities
   * (torques, angular velocities in world coordinates).
   *
   * @return 3x3 inverse inertia tensor in world frame [1/(kg⋅m²)]
   *
   * @ticket 0032_contact_constraint_refactor
   */
  Eigen::Matrix3d getInverseInertiaTensorWorld() const;

  // ========== NEW: Force Application API (ticket 0023a) ==========

  /**
   * @brief Apply a force at the center of mass.
   *
   * Placeholder implementation for ticket 0023a - only accumulates force.
   * Actual integration will be implemented in ticket 0023.
   *
   * @param force Force vector in world coordinates [N]
   */
  void applyForce(const ForceVector& force);

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
  void applyForceAtPoint(const ForceVector& force,
                         const Coordinate& worldPoint);

  /**
   * @brief Apply a torque about the center of mass.
   *
   * Placeholder implementation for ticket 0023a - only accumulates torque.
   *
   * @param torque Torque vector in world coordinates [N·m]
   */
  void applyTorque(const TorqueVector& torque);

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
  ForceVector getAccumulatedForce() const;

  /**
   * @brief Get the accumulated torque for this frame.
   * @return Accumulated torque vector [N·m]
   */
  TorqueVector getAccumulatedTorque() const;

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

  // ========== Friction Coefficient (ticket 0052d) ==========

  /**
   * @brief Get the friction coefficient.
   *
   * Determines tangential friction force magnitude:
   * - 0.0: Frictionless surface (ice)
   * - 0.5: Moderate friction (default)
   * - 1.0+: High friction (rubber)
   *
   * @return Friction coefficient [0, inf)
   * @ticket 0052d_solver_integration_ecos_removal
   */
  double getFrictionCoefficient() const;

  /**
   * @brief Set the friction coefficient.
   *
   * @param mu Friction coefficient [0, inf)
   * @throws std::invalid_argument if mu < 0
   *
   * @ticket 0052d_solver_integration_ecos_removal
   */
  void setFrictionCoefficient(double mu);

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
  void applyImpulse(const Vector3D& impulse);

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
  void applyAngularImpulse(const AngularVelocity& angularImpulse);

  // ========== NEW: Constraint Management (ticket 0031) ==========

  /**
   * @brief Add a constraint to this object
   *
   * Transfers ownership of the constraint to this asset. The constraint will
   * be enforced during physics integration.
   *
   * @param constraint Constraint to add (ownership transferred)
   *
   * @ticket 0031_generalized_lagrange_constraints
   */
  void addConstraint(std::unique_ptr<Constraint> constraint);

  /**
   * @brief Remove constraint at specified index
   *
   * @param index Index of constraint to remove [0, getConstraintCount())
   * @throws std::out_of_range if index >= getConstraintCount()
   *
   * @ticket 0031_generalized_lagrange_constraints
   */
  void removeConstraint(size_t index);

  /**
   * @brief Get all constraints (mutable)
   *
   * Returns raw pointers for integration (non-owning access).
   * Used by Integrator to enforce constraints during physics integration.
   *
   * @return Vector of non-owning constraint pointers
   *
   * @ticket 0031_generalized_lagrange_constraints
   */
  std::vector<Constraint*> getConstraints();

  /**
   * @brief Get all constraints (const)
   *
   * Returns raw pointers for query (non-owning access).
   *
   * @return Vector of non-owning const constraint pointers
   *
   * @ticket 0031_generalized_lagrange_constraints
   */
  std::vector<const Constraint*> getConstraints() const;

  /**
   * @brief Clear all constraints
   *
   * Removes all constraints from this object. Quaternion normalization
   * constraint will need to be manually re-added if desired.
   *
   * @ticket 0031_generalized_lagrange_constraints
   */
  void clearConstraints();

  /**
   * @brief Get number of constraints
   *
   * @return Number of constraints attached to this object
   *
   * @ticket 0031_generalized_lagrange_constraints
   */
  size_t getConstraintCount() const;

  // ========== Transfer Object Support ==========

  /**
   * @brief Convert dynamic state to a database record.
   *
   * Captures the complete per-frame dynamic state: kinematic state
   * (position, velocity, acceleration, orientation, quaternion rate,
   * angular acceleration) plus accumulated force and torque.
   *
   * @return Transfer record containing all dynamic state data
   */
  [[nodiscard]] msd_transfer::AssetDynamicStateRecord toDynamicStateRecord()
    const;

  /**
   * @brief Convert physics properties to a spawn-time static record.
   *
   * Captures mass, restitution, and friction for this inertial asset.
   *
   * @return Transfer record with body_id, mass, restitution, friction
   * @ticket 0056a_collision_force_transfer_records
   */
  [[nodiscard]] msd_transfer::AssetInertialStaticRecord toStaticRecord() const;

private:
  // Spawn-time static properties (mass + material)
  // Ticket: 0056a_collision_force_transfer_records
  AssetStaticState staticState_;

  // Rigid body inertia properties (computed from hull + mass)
  Eigen::Matrix3d inertiaTensor_;  // Inertia tensor about centroid [kg⋅m²]
  Eigen::Matrix3d inverseInertiaTensor_;  // Inverse inertia tensor [1/(kg⋅m²)]
  Coordinate centerOfMass_;               // Center of mass in local coordinates

  // Per-frame mutable state (kinematic + force/torque accumulators)
  // Ticket: 0056h_asset_dynamic_state_struct
  AssetDynamicState dynamicState_;

  // Constraint management (ticket 0031)
  std::vector<std::unique_ptr<Constraint>> constraints_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_INERTIAL_ASSET_HPP

#ifndef MSD_SIM_PHYSICS_COMPONENT_HPP
#define MSD_SIM_PHYSICS_COMPONENT_HPP

#include <Eigen/Dense>
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Physics/RigidBody/DynamicState.hpp"

namespace msd_sim
{

/**
 * @brief Rigid body physics properties for dynamic objects
 *
 * Encapsulates mass properties and dynamic state. Separated from
 * Object to allow efficient storage and iteration during physics updates.
 *
 * This component is only present in Inertial objects that participate
 * in dynamic physics simulation. Static objects (Environmental, Boundary)
 * and visual-only objects (Graphical) do not have this component.
 *
 * The inertia tensor is computed automatically from the collision hull
 * using InertialCalculations, assuming uniform density.
 *
 * Usage:
 * @code
 * ConvexHull hull(collisionGeometry);
 * PhysicsComponent physics(hull, 50.0);  // 50 kg mass
 *
 * // Apply forces
 * physics.applyForce(Coordinate(0, 0, -9.81 * 50.0));  // Gravity
 * physics.applyForceAtPoint(windForce, sailPosition);
 *
 * // Access state
 * physics.getDynamicState().setLinearVelocity(Coordinate(1, 0, 0));
 * double ke = physics.getKineticEnergy();
 * @endcode
 */
class PhysicsComponent
{
public:
  /**
   * @brief Construct physics component from collision hull and mass
   *
   * The inertia tensor and center of mass are automatically computed
   * from the hull geometry using InertialCalculations, assuming uniform
   * density throughout the volume.
   *
   * @param hull Convex hull for mass property calculation
   * @param mass Mass in kg (must be > 0)
   * @throws std::invalid_argument if mass <= 0 or hull is invalid
   */
  PhysicsComponent(const ConvexHull& hull, double mass);

  // ========== Mass Properties ==========

  /**
   * @brief Get mass of the object
   * @return Mass [kg]
   */
  double getMass() const { return mass_; }

  /**
   * @brief Get inertia tensor about center of mass
   * @return 3x3 inertia tensor [kg⋅m²]
   */
  const Eigen::Matrix3d& getInertiaTensor() const { return inertiaTensor_; }

  /**
   * @brief Get inverse inertia tensor about center of mass
   * @return 3x3 inverse inertia tensor [1/(kg⋅m²)]
   */
  const Eigen::Matrix3d& getInverseInertiaTensor() const
  {
    return inverseInertiaTensor_;
  }

  /**
   * @brief Get center of mass in local coordinates
   * @return Center of mass position
   */
  const Coordinate& getCenterOfMass() const { return centerOfMass_; }

  // ========== Dynamic State ==========

  /**
   * @brief Get dynamic state (const)
   * @return Const reference to velocities and accelerations
   */
  const DynamicState& getDynamicState() const { return dynamicState_; }

  /**
   * @brief Get dynamic state (mutable)
   * @return Mutable reference to velocities and accelerations
   */
  DynamicState& getDynamicState() { return dynamicState_; }

  /**
   * @brief Compute total kinetic energy
   *
   * KE = 0.5 * m * v² + 0.5 * ω^T * I * ω
   *
   * @return Total KE (linear + rotational) [J]
   */
  double getKineticEnergy() const;

  // ========== Force and Torque Application ==========

  /**
   * @brief Apply force at center of mass (no torque)
   *
   * Updates linear acceleration: a = F/m
   * Does not affect angular acceleration.
   *
   * @param force Force vector in world frame [N]
   */
  void applyForce(const Coordinate& force);

  /**
   * @brief Apply force at a point offset from center of mass
   *
   * Updates both linear and angular acceleration.
   * Linear: a = F/m
   * Angular: α = I⁻¹ (r × F), where r is the offset from CoM
   *
   * @param force Force vector in world frame [N]
   * @param localOffset Point of application in local frame [m]
   */
  void applyForceAtPoint(const Coordinate& force,
                         const Coordinate& localOffset);

  /**
   * @brief Apply pure torque (no linear force)
   *
   * Updates angular acceleration: α = I⁻¹ τ
   * Does not affect linear acceleration.
   *
   * @param torque Torque vector in world frame [N⋅m]
   */
  void applyTorque(const Eigen::Vector3d& torque);

  /**
   * @brief Clear all accumulated forces and torques
   *
   * Resets linear and angular accelerations to zero.
   * Call this after integrating forces in the physics update loop.
   */
  void clearForces();

private:
  double mass_;                          // Mass [kg]
  Eigen::Matrix3d inertiaTensor_;        // Inertia tensor [kg⋅m²]
  Eigen::Matrix3d inverseInertiaTensor_; // I⁻¹ for fast computation
  Coordinate centerOfMass_;              // CoM in local coordinates

  DynamicState dynamicState_; // Velocities and accelerations
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COMPONENT_HPP

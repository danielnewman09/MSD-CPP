#ifndef MSD_SIM_PHYSICS_DYNAMIC_STATE_HPP
#define MSD_SIM_PHYSICS_DYNAMIC_STATE_HPP

#include <Eigen/Dense>
#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_sim
{

/**
 * @brief Complete dynamic state for rigid body motion.
 *
 * DynamicState encapsulates all time-varying kinematic quantities needed to
 * describe rigid body motion: linear and angular velocities and accelerations.
 * This class is used in conjunction with RigidBodyProperties to fully specify
 * the state of a dynamic object in a physics simulation.
 *
 * Convention:
 * - Linear quantities are in world/inertial frame
 * - Angular quantities are in body frame (or world frame, depending on usage)
 * - Velocities: first time derivative of position/orientation
 * - Accelerations: second time derivative of position/orientation
 *
 * Key features:
 * - Stores linear velocity and acceleration
 * - Stores angular velocity and acceleration
 * - Provides convenience methods for common operations
 * - Integrates cleanly with physics integrators
 *
 * Usage pattern:
 * @code
 * DynamicState state;
 * state.setLinearVelocity(Coordinate(1.0, 0.0, 0.0));  // Moving in +x
 * state.setAngularVelocity(Eigen::Vector3d(0, 0, 1.0));  // Rotating around z
 *
 * // Apply forces and compute accelerations
 * state.setLinearAcceleration(force / mass);
 * state.setAngularAcceleration(inverseInertia * torque);
 *
 * // Integrate state forward in time
 * integrator.step(state, dt);
 * @endcode
 */
class DynamicState
{
public:
  /**
   * @brief Default constructor - creates state at rest.
   *
   * Initializes all velocities and accelerations to zero.
   */
  DynamicState();

  /**
   * @brief Constructor with specified velocities.
   *
   * @param linearVelocity Linear velocity vector [m/s]
   * @param angularVelocity Angular velocity vector [rad/s]
   */
  DynamicState(const Coordinate& linearVelocity,
               const Eigen::Vector3d& angularVelocity);

  /**
   * @brief Get the linear velocity.
   * @return Linear velocity in world frame [m/s]
   */
  const Coordinate& getLinearVelocity() const;

  /**
   * @brief Get the angular velocity.
   * @return Angular velocity vector [rad/s]
   */
  const Eigen::Vector3d& getAngularVelocity() const;

  /**
   * @brief Get the linear acceleration.
   * @return Linear acceleration in world frame [m/s²]
   */
  const Coordinate& getLinearAcceleration() const;

  /**
   * @brief Get the angular acceleration.
   * @return Angular acceleration vector [rad/s²]
   */
  const Eigen::Vector3d& getAngularAcceleration() const;

  /**
   * @brief Set the linear velocity.
   * @param velocity New linear velocity [m/s]
   */
  void setLinearVelocity(const Coordinate& velocity);

  /**
   * @brief Set the angular velocity.
   * @param velocity New angular velocity [rad/s]
   */
  void setAngularVelocity(const Eigen::Vector3d& velocity);

  /**
   * @brief Set the linear acceleration.
   * @param acceleration New linear acceleration [m/s²]
   */
  void setLinearAcceleration(const Coordinate& acceleration);

  /**
   * @brief Set the angular acceleration.
   * @param acceleration New angular acceleration [rad/s²]
   */
  void setAngularAcceleration(const Eigen::Vector3d& acceleration);

  /**
   * @brief Get the kinetic energy of linear motion.
   *
   * KE_linear = 0.5 * mass * v²
   *
   * @param mass Mass of the body [kg]
   * @return Linear kinetic energy [J]
   */
  double getLinearKineticEnergy(double mass) const;

  /**
   * @brief Get the kinetic energy of rotational motion.
   *
   * KE_rotational = 0.5 * ω^T * I * ω
   *
   * @param inertiaTensor Inertia tensor in body frame [kg⋅m²]
   * @return Rotational kinetic energy [J]
   */
  double getRotationalKineticEnergy(const Eigen::Matrix3d& inertiaTensor) const;

  /**
   * @brief Get the total kinetic energy.
   *
   * Total KE = Linear KE + Rotational KE
   *
   * @param mass Mass of the body [kg]
   * @param inertiaTensor Inertia tensor in body frame [kg⋅m²]
   * @return Total kinetic energy [J]
   */
  double getTotalKineticEnergy(double mass,
                               const Eigen::Matrix3d& inertiaTensor) const;

  /**
   * @brief Get the magnitude of linear velocity.
   * @return Speed [m/s]
   */
  double getSpeed() const;

  /**
   * @brief Get the magnitude of angular velocity.
   * @return Angular speed [rad/s]
   */
  double getAngularSpeed() const;

  /**
   * @brief Check if the body is at rest.
   *
   * A body is considered at rest if both linear and angular velocities
   * are below the specified thresholds.
   *
   * @param linearThreshold Linear velocity threshold [m/s] (default: 1e-6)
   * @param angularThreshold Angular velocity threshold [rad/s] (default: 1e-6)
   * @return true if body is at rest, false otherwise
   */
  bool isAtRest(double linearThreshold = 1e-6,
                double angularThreshold = 1e-6) const;

  /**
   * @brief Reset all velocities and accelerations to zero.
   */
  void reset();

  /**
   * @brief Apply a linear impulse to change velocity.
   *
   * Δv = impulse / mass
   *
   * @param impulse Linear impulse vector [N⋅s]
   * @param mass Mass of the body [kg]
   */
  void applyLinearImpulse(const Coordinate& impulse, double mass);

  /**
   * @brief Apply an angular impulse to change angular velocity.
   *
   * Δω = I^(-1) * impulse
   *
   * @param impulse Angular impulse vector [N⋅m⋅s]
   * @param inverseInertia Inverse inertia tensor [1/(kg⋅m²)]
   */
  void applyAngularImpulse(const Eigen::Vector3d& impulse,
                          const Eigen::Matrix3d& inverseInertia);

  /**
   * @brief Create a state at rest (zero velocities and accelerations).
   * @return DynamicState with all quantities zero
   */
  static DynamicState createAtRest();

private:
  Coordinate linearVelocity_;         // Linear velocity [m/s]
  Eigen::Vector3d angularVelocity_;   // Angular velocity [rad/s]
  Coordinate linearAcceleration_;     // Linear acceleration [m/s²]
  Eigen::Vector3d angularAcceleration_;  // Angular acceleration [rad/s²]
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_DYNAMIC_STATE_HPP

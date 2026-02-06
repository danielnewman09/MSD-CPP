// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md

#ifndef INERTIAL_STATE_HPP
#define INERTIAL_STATE_HPP

#include <Eigen/Geometry>
#include <cstdint>

#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"
#include "msd-sim/src/DataTypes/AngularRate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector4D.hpp"
#include "msd-transfer/src/InertialStateRecord.hpp"

namespace msd_sim
{

/**
 * @brief Complete kinematic state representation with 7-state quaternion
 * formulation.
 *
 * State vector: (X, Q, Ẋ, Q̇) with 14 components total.
 *
 * Contains position, velocity, and acceleration for both linear and angular
 * motion. Uses quaternion representation for orientation to eliminate gimbal
 * lock singularities. Quaternion rate Q̇ is stored directly for efficiency
 * (avoids ω→Q̇ conversion every step).
 *
 * Conversion formulas:
 * - Q̇ → ω: ω = 2 * Q̄ ⊗ Q̇  (where Q̄ is conjugate)
 * - ω → Q̇: Q̇ = ½ * Q ⊗ [0, ω]
 *
 * @see
 * docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml
 * @ticket 0030_lagrangian_quaternion_physics
 */
struct InertialState
{
  // Linear components
  Coordinate position;
  Coordinate velocity;
  Coordinate acceleration;

  // Angular components (quaternion representation)
  Eigen::Quaterniond orientation{1.0,
                                 0.0,
                                 0.0,
                                 0.0};  // Identity quaternion (w, x, y, z)
  Eigen::Vector4d quaternionRate{0.0, 0.0, 0.0, 0.0};  // Q̇ (w, x, y, z)
  AngularRate angularAcceleration;                     // α = I⁻¹ * τ [rad/s²]

  /**
   * @brief Convert quaternion rate to angular velocity
   * @return Angular velocity ω in world frame [rad/s]
   *
   * Formula: ω = 2 * Q̄ ⊗ Q̇
   * where Q̄ is the conjugate quaternion (w, -x, -y, -z)
   */
  [[nodiscard]] AngularRate getAngularVelocity() const;

  /**
   * @brief Set angular velocity (converts to quaternion rate)
   * @param omega Angular velocity in world frame [rad/s]
   *
   * Formula: Q̇ = ½ * Q ⊗ [0, ω]
   */
  void setAngularVelocity(const AngularRate& omega);

  /**
   * @brief Convert angular velocity to quaternion rate
   * @param omega Angular velocity in world frame [rad/s]
   * @param Q Current quaternion orientation
   * @return Quaternion rate Q̇
   *
   * Formula: Q̇ = ½ * Q ⊗ [0, ω]
   */
  static Vector4D omegaToQuaternionRate(const AngularRate& omega,
                                        const Eigen::Quaterniond& Q);

  /**
   * @brief Convert quaternion rate to angular velocity
   * @param Qdot Quaternion rate
   * @param Q Current quaternion orientation
   * @return Angular velocity ω in world frame [rad/s]
   *
   * Formula: ω = 2 * Q̄ ⊗ Q̇
   */
  static AngularRate quaternionRateToOmega(const Eigen::Vector4d& Qdot,
                                           const Eigen::Quaterniond& Q);

  /**
   * @brief Get Euler angles from quaternion (deprecated, for backward
   * compatibility)
   * @return Euler angles extracted from current quaternion
   * @deprecated Use quaternion representation directly. Euler angles have
   * gimbal lock.
   *
   * Extracts ZYX Euler angles from quaternion using Eigen's conversion.
   * Warning: This conversion is ambiguous near gimbal lock (pitch ≈ ±90°).
   */
  [[nodiscard]] [[deprecated(
    "Use quaternion representation directly. Euler angles have gimbal lock.")]]
  AngularCoordinate getEulerAngles() const;
};

}  // namespace msd_sim

#endif
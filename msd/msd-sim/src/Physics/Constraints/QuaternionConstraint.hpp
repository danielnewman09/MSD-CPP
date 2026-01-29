// Ticket: 0030_lagrangian_quaternion_physics
// Design: docs/designs/0030_lagrangian_quaternion_physics/design.md

#ifndef MSD_SIM_PHYSICS_QUATERNION_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_QUATERNION_CONSTRAINT_HPP

#include <Eigen/Geometry>

namespace msd_sim
{

/**
 * @brief Enforces unit quaternion constraint via Lagrange multipliers
 *
 * Maintains the constraint g(Q) = QᵀQ - 1 = 0 using Baumgarte stabilization
 * to correct drift accumulated during numerical integration.
 *
 * Constraint equations:
 * - Position: g(Q) = QᵀQ - 1 = 0
 * - Velocity: ġ = 2QᵀQ̇ = 0  (Q̇ ⊥ Q)
 *
 * Baumgarte stabilization computes Lagrange multiplier as:
 *   λ = -α * g - β * ġ
 * where α, β > 0 are tuning parameters.
 *
 * Ownership: Each AssetInertial owns its own QuaternionConstraint instance,
 * since the constraint operates on per-object quaternion state.
 *
 * Thread safety: Not thread-safe (modifies quaternion state)
 *
 * @see docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml
 * @ticket 0030_lagrangian_quaternion_physics
 */
class QuaternionConstraint
{
public:
  /**
   * @brief Construct constraint with Baumgarte parameters
   * @param alpha Position error gain (default: 10.0)
   * @param beta Velocity error gain (default: 10.0)
   */
  explicit QuaternionConstraint(double alpha = 10.0, double beta = 10.0);

  ~QuaternionConstraint() = default;

  /**
   * @brief Enforce unit quaternion constraint with Baumgarte stabilization
   *
   * Modifies Q and Qdot to satisfy:
   * 1. Normalize Q to unit length
   * 2. Project Qdot onto tangent space (perpendicular to Q)
   * 3. Apply Baumgarte correction to reduce drift
   *
   * @param Q Quaternion to constrain (modified in place)
   * @param Qdot Quaternion rate (modified in place)
   */
  void enforceConstraint(Eigen::Quaterniond& Q, Eigen::Vector4d& Qdot);

  /**
   * @brief Compute constraint force for dynamics integration
   * @param Q Current quaternion
   * @param Qdot Current quaternion rate
   * @return Constraint force F_c = G^T * λ where G = ∂g/∂Q
   */
  Eigen::Vector4d computeConstraintForce(const Eigen::Quaterniond& Q,
                                         const Eigen::Vector4d& Qdot) const;

  // Parameter configuration
  void setAlpha(double alpha);
  void setBeta(double beta);
  double getAlpha() const;
  double getBeta() const;

  // Constraint violation queries (for diagnostics)
  double positionViolation(const Eigen::Quaterniond& Q) const;
  double velocityViolation(const Eigen::Quaterniond& Q,
                           const Eigen::Vector4d& Qdot) const;

  // Rule of Five
  QuaternionConstraint(const QuaternionConstraint&) = default;
  QuaternionConstraint& operator=(const QuaternionConstraint&) = default;
  QuaternionConstraint(QuaternionConstraint&&) noexcept = default;
  QuaternionConstraint& operator=(QuaternionConstraint&&) noexcept = default;

private:
  double alpha_{10.0};  // Position error gain
  double beta_{10.0};   // Velocity error gain
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_QUATERNION_CONSTRAINT_HPP

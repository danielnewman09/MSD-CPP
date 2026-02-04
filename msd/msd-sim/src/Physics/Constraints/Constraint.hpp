// Ticket: 0031_generalized_lagrange_constraints
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md

#ifndef MSD_SIM_PHYSICS_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_CONSTRAINT_HPP

#include <Eigen/Dense>
#include <string>

#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/**
 * @brief Abstract base class for constraint definitions in Lagrangian mechanics
 *
 * Defines the mathematical interface for arbitrary constraints C(q, t) that can
 * be enforced via Lagrange multipliers. Each constraint implementation
 * provides:
 * - Constraint function evaluation C(q, t)
 * - Constraint Jacobian J = ∂C/∂q
 * - Optional time derivative ∂C/∂t
 * - Baumgarte stabilization parameters (α, β)
 *
 * Lagrange multiplier formulation:
 *   J·M⁻¹·Jᵀ·λ = -J·M⁻¹·F_ext - J̇·q̇ - α·C - β·Ċ
 *   F_constraint = Jᵀ·λ
 *
 * Thread safety: Implementations must be thread-safe for concurrent evaluation
 * (read-only after construction).
 *
 * Error handling: Implementations may throw std::invalid_argument for invalid
 * state or parameters.
 *
 * @see
 * docs/designs/0031_generalized_lagrange_constraints/0031_generalized_lagrange_constraints.puml
 * @ticket 0031_generalized_lagrange_constraints
 */
class Constraint
{
public:
  virtual ~Constraint() = default;

  /**
   * @brief Number of scalar constraint equations
   *
   * @return Dimension of constraint vector C(q, t)
   */
  [[nodiscard]] virtual int dimension() const = 0;

  /**
   * @brief Evaluate constraint function C(q, t)
   *
   * Returns vector of constraint violations. For bilateral constraints,
   * C = 0 is the target. For unilateral constraints, C ≥ 0 is the target.
   *
   * @param state Current inertial state (position, orientation, velocities)
   * @param time Simulation time [s]
   * @return Constraint violation vector (dimension × 1)
   *
   * @throws std::invalid_argument if state is invalid
   */
  [[nodiscard]] virtual Eigen::VectorXd evaluate(const InertialState& state,
                                   double time) const = 0;

  /**
   * @brief Compute constraint Jacobian J = ∂C/∂q
   *
   * Returns partial derivatives of constraint function w.r.t. position-level
   * DOFs. For the current 14-component state vector (X, Q, Ẋ, Q̇), the Jacobian
   * operates on the 7 position-level DOFs (X: 3, Q: 4).
   *
   * @param state Current inertial state
   * @param time Simulation time [s]
   * @return Jacobian matrix (dimension × 7)
   *
   * @throws std::invalid_argument if state is invalid
   */
  [[nodiscard]] virtual Eigen::MatrixXd jacobian(const InertialState& state,
                                   double time) const = 0;

  /**
   * @brief Compute constraint time derivative ∂C/∂t (optional)
   *
   * Returns partial derivative of constraint w.r.t. explicit time dependence.
   * For time-independent constraints, this returns a zero vector.
   *
   * Default implementation: zero vector (no explicit time dependence)
   *
   * @param state Current inertial state
   * @param time Simulation time [s]
   * @return Time derivative vector (dimension × 1)
   */
  [[nodiscard]] virtual Eigen::VectorXd partialTimeDerivative(const InertialState& state,
                                                double time) const;

  /**
   * @brief Baumgarte stabilization position error gain
   *
   * Used in stabilization term: -α·C
   * Higher values increase correction strength but may cause instability.
   *
   * Default: 10.0 (literature standard for dt ~ 0.016s)
   *
   * @return Position error gain α [1/s²]
   */
  [[nodiscard]] virtual double alpha() const
  {
    return 10.0;
  }

  /**
   * @brief Baumgarte stabilization velocity error gain
   *
   * Used in stabilization term: -β·Ċ
   * Higher values increase damping but may cause stiffness.
   *
   * Default: 10.0 (literature standard for dt ~ 0.016s)
   *
   * @return Velocity error gain β [1/s]
   */
  [[nodiscard]] virtual double beta() const
  {
    return 10.0;
  }

  /**
   * @brief Constraint type identifier for debugging/logging
   *
   * @return Human-readable constraint type name
   */
  [[nodiscard]] virtual std::string typeName() const = 0;

protected:
  // Protected Rule of Five: prevent direct instantiation of abstract base
  Constraint() = default;
  Constraint(const Constraint&) = default;
  Constraint& operator=(const Constraint&) = default;
  Constraint(Constraint&&) noexcept = default;
  Constraint& operator=(Constraint&&) noexcept = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINT_HPP

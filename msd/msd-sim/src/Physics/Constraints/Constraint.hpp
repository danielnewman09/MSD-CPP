// Ticket: 0043_constraint_hierarchy_refactor
// Design: docs/designs/0043_constraint_hierarchy_refactor/design.md

#ifndef MSD_SIM_PHYSICS_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_CONSTRAINT_HPP

#include <Eigen/Dense>
#include <string>

#include "msd-sim/src/Physics/Constraints/LambdaBounds.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

// Forward declaration for visitor pattern
namespace msd_transfer {
class ConstraintRecordVisitor;
}

namespace msd_sim
{

/**
 * @brief Abstract base class for constraint definitions in Lagrangian mechanics
 *
 * Defines the mathematical interface for arbitrary constraints C(q, t) that can
 * be enforced via Lagrange multipliers. Each constraint implementation
 * provides:
 * - Constraint function evaluation C(qA, qB, t) for two bodies
 * - Constraint Jacobian J = ∂C/∂q
 * - Optional time derivative ∂C/∂t
 * - Baumgarte stabilization parameters (α, β)
 * - Lagrange multiplier bounds (bilateral/unilateral/box-constrained)
 * - Body count and indices for solver dispatch
 *
 * Lagrange multiplier formulation:
 *   J·M⁻¹·Jᵀ·λ = -J·M⁻¹·F_ext - J̇·q̇ - α·C - β·Ċ
 *   F_constraint = Jᵀ·λ
 *
 * Unified Evaluation Signature:
 *   All constraints use a two-body signature evaluate(stateA, stateB, time).
 *   Single-body constraints ignore stateB. This eliminates LSP violations
 *   and enables polymorphic treatment of all constraint types.
 *
 * Convenience Overloads:
 *   Non-virtual convenience methods evaluate(state, time) and jacobian(state, time)
 *   delegate to the two-body virtual methods, passing state as both stateA and stateB.
 *
 * Jacobian Column Count:
 *   - Single-body constraints (bodyCount() == 1): dimension × 7 (position-level DOFs)
 *   - Two-body constraints (bodyCount() == 2): dimension × 12 (velocity-level DOFs)
 *
 * Thread safety: Implementations must be thread-safe for concurrent evaluation
 * (read-only after construction).
 *
 * Error handling: Implementations may throw std::invalid_argument for invalid
 * state or parameters.
 *
 * @see docs/designs/0043_constraint_hierarchy_refactor/0043_constraint_hierarchy_refactor.puml
 * @ticket 0043_constraint_hierarchy_refactor
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
   * @brief Evaluate constraint function C(qA, qB, t) for two bodies
   *
   * Returns vector of constraint violations. For bilateral constraints,
   * C = 0 is the target. For unilateral constraints, C ≥ 0 is the target.
   *
   * Single-body constraints ignore stateB.
   *
   * @param stateA Current inertial state of body A (or single body)
   * @param stateB Current inertial state of body B (ignored for single-body)
   * @param time Simulation time [s]
   * @return Constraint violation vector (dimension × 1)
   *
   * @throws std::invalid_argument if state is invalid
   */
  [[nodiscard]] virtual Eigen::VectorXd evaluate(const InertialState& stateA,
                                                  const InertialState& stateB,
                                                  double time) const = 0;

  /**
   * @brief Convenience overload for single-body constraints
   *
   * Delegates to evaluate(state, state, time). Non-virtual.
   *
   * @param state Current inertial state
   * @param time Simulation time [s]
   * @return Constraint violation vector (dimension × 1)
   */
  [[nodiscard]] Eigen::VectorXd evaluate(const InertialState& state,
                                          double time) const
  {
    return evaluate(state, state, time);
  }

  /**
   * @brief Compute constraint Jacobian J = ∂C/∂q for two bodies
   *
   * Returns partial derivatives of constraint function w.r.t. position-level
   * or velocity-level DOFs (depending on bodyCount):
   * - Single-body (bodyCount() == 1): dimension × 7 (X: 3, Q: 4)
   * - Two-body (bodyCount() == 2): dimension × 12 (v_A: 3, ω_A: 3, v_B: 3, ω_B: 3)
   *
   * Single-body constraints ignore stateB.
   *
   * @param stateA Current inertial state of body A (or single body)
   * @param stateB Current inertial state of body B (ignored for single-body)
   * @param time Simulation time [s]
   * @return Jacobian matrix (dimension × 7 or dimension × 12)
   *
   * @throws std::invalid_argument if state is invalid
   */
  [[nodiscard]] virtual Eigen::MatrixXd jacobian(const InertialState& stateA,
                                                  const InertialState& stateB,
                                                  double time) const = 0;

  /**
   * @brief Convenience overload for single-body constraints
   *
   * Delegates to jacobian(state, state, time). Non-virtual.
   *
   * @param state Current inertial state
   * @param time Simulation time [s]
   * @return Jacobian matrix (dimension × 7)
   */
  [[nodiscard]] Eigen::MatrixXd jacobian(const InertialState& state,
                                          double time) const
  {
    return jacobian(state, state, time);
  }

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
   * @return Position error gain α [1/s²]
   */
  [[nodiscard]] double alpha() const
  {
    return alpha_;
  }

  /**
   * @brief Baumgarte stabilization velocity error gain
   *
   * Used in stabilization term: -β·Ċ
   * Higher values increase damping but may cause stiffness.
   *
   * @return Velocity error gain β [1/s]
   */
  [[nodiscard]] double beta() const
  {
    return beta_;
  }

  /**
   * @brief Set Baumgarte position error gain
   *
   * @param alpha New position error gain α [1/s²]
   */
  void setAlpha(double alpha)
  {
    alpha_ = alpha;
  }

  /**
   * @brief Set Baumgarte velocity error gain
   *
   * @param beta New velocity error gain β [1/s]
   */
  void setBeta(double beta)
  {
    beta_ = beta;
  }

  /**
   * @brief Get Lagrange multiplier bounds
   *
   * Specifies the lower and upper bounds on the constraint's Lagrange multiplier:
   * - Bilateral constraints: (-∞, +∞)
   * - Unilateral constraints: (0, +∞)
   * - Box-constrained: (lo, hi)
   *
   * @return LambdaBounds struct with lower and upper fields
   */
  [[nodiscard]] virtual LambdaBounds lambdaBounds() const = 0;

  /**
   * @brief Check if constraint is active
   *
   * Determines whether the constraint should be enforced at the given state.
   * Default implementation returns true (always active).
   *
   * @param stateA Current inertial state of body A (or single body)
   * @param stateB Current inertial state of body B (ignored for single-body)
   * @param time Simulation time [s]
   * @return true if constraint should be enforced
   */
  [[nodiscard]] virtual bool isActive(const InertialState& stateA,
                                       const InertialState& stateB,
                                       double time) const
  {
    (void)stateA;
    (void)stateB;
    (void)time;
    return true;
  }

  /**
   * @brief Number of bodies involved in this constraint
   *
   * Single-body constraints (UnitQuaternionConstraint, DistanceConstraint) return 1.
   * Two-body constraints (ContactConstraint, FrictionConstraint) return 2.
   *
   * @return 1 for single-body, 2 for two-body
   */
  [[nodiscard]] virtual int bodyCount() const
  {
    return 1;
  }

  /**
   * @brief Get body A index in the solver's body list
   *
   * Single-body constraints use this to identify their target body.
   * Two-body constraints use this for the first body.
   *
   * @return Index of body A (0-based)
   */
  [[nodiscard]] size_t bodyAIndex() const
  {
    return body_a_index_;
  }

  /**
   * @brief Get body B index in the solver's body list
   *
   * Unused for single-body constraints (conventionally set to 0).
   * Two-body constraints use this for the second body.
   *
   * @return Index of body B (0-based)
   */
  [[nodiscard]] size_t bodyBIndex() const
  {
    return body_b_index_;
  }

  /**
   * @brief Constraint type identifier for debugging/logging
   *
   * @return Human-readable constraint type name
   */
  [[nodiscard]] virtual std::string typeName() const = 0;

  /**
   * @brief Record constraint state via visitor pattern
   *
   * Builds a typed constraint record (ContactConstraintRecord, FrictionConstraintRecord,
   * etc.) and dispatches to the visitor via overload resolution. The visitor decides
   * what to do with the record (e.g., buffer to DataRecorder DAO).
   *
   * Enables compile-time type-safe constraint recording without dynamic_cast or std::any_cast.
   * New constraint types extend ConstraintRecordVisitor with a new visit() overload.
   *
   * @param visitor Visitor that processes the typed constraint record
   * @param bodyAId Persistent ID of body A for database FK reference
   * @param bodyBId Persistent ID of body B for database FK reference (ignored for single-body)
   *
   * @see msd_transfer::ConstraintRecordVisitor
   * @see msd_sim::DataRecorderVisitor
   */
  virtual void recordState(msd_transfer::ConstraintRecordVisitor& visitor,
                           uint32_t bodyAId,
                           uint32_t bodyBId) const = 0;

protected:
  /**
   * @brief Protected constructor for subclasses
   *
   * @param bodyAIndex Index of body A in solver body list (default 0)
   * @param bodyBIndex Index of body B in solver body list (default 0)
   * @param alpha Baumgarte position error gain (default 10.0)
   * @param beta Baumgarte velocity error gain (default 10.0)
   */
  Constraint(size_t bodyAIndex = 0,
             size_t bodyBIndex = 0,
             double alpha = 10.0,
             double beta = 10.0)
    : body_a_index_{bodyAIndex},
      body_b_index_{bodyBIndex},
      alpha_{alpha},
      beta_{beta}
  {
  }

  // Protected Rule of Five: prevent direct instantiation of abstract base
  Constraint(const Constraint&) = default;
  Constraint& operator=(const Constraint&) = default;
  Constraint(Constraint&&) noexcept = default;
  Constraint& operator=(Constraint&&) noexcept = default;

  size_t body_a_index_;
  size_t body_b_index_;
  double alpha_;
  double beta_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINT_HPP

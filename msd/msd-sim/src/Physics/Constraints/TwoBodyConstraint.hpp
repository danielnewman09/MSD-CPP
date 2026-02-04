// Ticket: 0032_contact_constraint_refactor
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#ifndef MSD_SIM_PHYSICS_TWO_BODY_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_TWO_BODY_CONSTRAINT_HPP

#include "msd-sim/src/Physics/Constraints/UnilateralConstraint.hpp"

namespace msd_sim
{

/**
 * @brief Abstract interface for constraints operating on two rigid bodies
 *
 * TwoBodyConstraint extends UnilateralConstraint to support constraints that
 * couple two rigid bodies (e.g., contacts, joints, springs). The constraint
 * function C(qA, qB, t) and Jacobian J depend on both bodies' states.
 *
 * The Jacobian uses a 6-DOF per body formulation [v_A, omega_A, v_B, omega_B]
 * (12 columns total), matching the velocity-level formulation used for
 * collision response. The solver handles quaternion conversion internally.
 *
 * Design rationale: Introducing TwoBodyConstraint as a subclass avoids
 * modifying the existing Constraint interface, which would risk breaking all
 * single-body constraint implementations (UnitQuaternionConstraint,
 * DistanceConstraint). The solver uses dynamic_cast<TwoBodyConstraint*> to
 * dispatch between single-body and two-body evaluation.
 *
 * Thread safety: Read-only operations thread-safe after construction
 * Error handling: Single-body methods throw std::logic_error (misuse)
 *
 * @see
 * docs/designs/0032_contact_constraint_refactor/0032_contact_constraint_refactor.puml
 * @ticket 0032_contact_constraint_refactor
 */
class TwoBodyConstraint : public UnilateralConstraint
{
public:
  ~TwoBodyConstraint() override = default;

  // ===== Two-Body Interface (subclass must implement) =====

  /**
   * @brief Evaluate constraint function C(qA, qB, t) for two bodies
   *
   * @param stateA Inertial state of body A
   * @param stateB Inertial state of body B
   * @param time Simulation time [s]
   * @return Constraint violation vector (dimension x 1)
   */
  [[nodiscard]] virtual Eigen::VectorXd
  evaluateTwoBody(const InertialState& stateA,
                  const InertialState& stateB,
                  double time) const = 0;

  /**
   * @brief Compute two-body Jacobian [J_A | J_B]
   *
   * The Jacobian uses the 6-DOF per body formulation from math-formulation
   * Section 3.6: [v_A, omega_A, v_B, omega_B] (12 columns). Each row
   * represents one scalar constraint equation.
   *
   * @param stateA Inertial state of body A
   * @param stateB Inertial state of body B
   * @param time Simulation time [s]
   * @return Jacobian matrix (dimension x 12)
   */
  [[nodiscard]] virtual Eigen::MatrixXd
  jacobianTwoBody(const InertialState& stateA,
                  const InertialState& stateB,
                  double time) const = 0;

  /**
   * @brief Check if constraint is active for two-body state
   *
   * @param stateA Inertial state of body A
   * @param stateB Inertial state of body B
   * @param time Simulation time [s]
   * @return true if constraint is active (should be enforced)
   */
  [[nodiscard]] virtual bool isActiveTwoBody(
    const InertialState& stateA,
    const InertialState& stateB,
    double time) const = 0;

  /**
   * @brief Get body A index in the solver's body list
   * @return Index of body A (0-based)
   */
  [[nodiscard]] size_t getBodyAIndex() const
  {
    return body_a_index_;
  }

  /**
   * @brief Get body B index in the solver's body list
   * @return Index of body B (0-based)
   */
  [[nodiscard]] size_t getBodyBIndex() const
  {
    return body_b_index_;
  }

  // ===== Single-Body Overrides (throw std::logic_error) =====

  /**
   * @brief Single-body evaluate (NOT SUPPORTED for two-body constraints)
   * @throws std::logic_error Two-body constraints must use evaluateTwoBody()
   */
  [[nodiscard]] Eigen::VectorXd evaluate(
    const InertialState& state,
    double time) const override;

  /**
   * @brief Single-body Jacobian (NOT SUPPORTED for two-body constraints)
   * @throws std::logic_error Two-body constraints must use jacobianTwoBody()
   */
  [[nodiscard]] Eigen::MatrixXd jacobian(
    const InertialState& state,
    double time) const override;

  /**
   * @brief Single-body isActive (NOT SUPPORTED for two-body constraints)
   * @throws std::logic_error Two-body constraints must use isActiveTwoBody()
   */
  [[nodiscard]] bool isActive(
    const InertialState& state,
    double time) const override;

  // Rule of Five
  TwoBodyConstraint(const TwoBodyConstraint&) = default;
  TwoBodyConstraint& operator=(const TwoBodyConstraint&) = default;
  TwoBodyConstraint(TwoBodyConstraint&&) noexcept = default;
  TwoBodyConstraint& operator=(TwoBodyConstraint&&) noexcept = default;

protected:
  /**
   * @brief Construct two-body constraint with body indices
   *
   * @param bodyAIndex Index of body A in solver body list
   * @param bodyBIndex Index of body B in solver body list
   */
  TwoBodyConstraint(size_t bodyAIndex, size_t bodyBIndex)
    : body_a_index_{bodyAIndex}, body_b_index_{bodyBIndex}
  {
  }

private:
  size_t body_a_index_;
  size_t body_b_index_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_TWO_BODY_CONSTRAINT_HPP

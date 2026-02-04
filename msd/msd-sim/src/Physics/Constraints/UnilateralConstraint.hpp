// Ticket: 0031_generalized_lagrange_constraints
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md

#ifndef MSD_SIM_PHYSICS_UNILATERAL_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_UNILATERAL_CONSTRAINT_HPP

#include "msd-sim/src/Physics/Constraints/Constraint.hpp"

namespace msd_sim
{

/**
 * @brief Specialization for inequality constraints C(q, t) ≥ 0
 *
 * Unilateral constraints enforce one-sided inequalities with complementarity
 * conditions:
 * - C ≥ 0 (constraint satisfied when non-negative)
 * - λ ≥ 0 (force only pushes, never pulls)
 * - λ·C = 0 (force is zero when constraint is inactive)
 *
 * Examples:
 * - Contact non-penetration: n·(p₁ - p₂) - d ≥ 0
 * - Joint angle limits: θ_max - θ ≥ 0
 *
 * Unilateral constraints are conditionally active based on state. Inactive
 * constraints are excluded from the constraint solver to avoid redundant
 * computation.
 *
 * NOTE: This ticket implements the interface but defers complementarity solver
 * to future ticket 0033. Current ConstraintSolver only handles bilateral
 * constraints.
 *
 * Thread safety: Same as Constraint base class
 * Error handling: Same as Constraint base class
 *
 * @see
 * docs/designs/0031_generalized_lagrange_constraints/0031_generalized_lagrange_constraints.puml
 * @ticket 0031_generalized_lagrange_constraints
 */
class UnilateralConstraint : public Constraint
{
public:
  ~UnilateralConstraint() override = default;

  /**
   * @brief Check if constraint is active (C ≈ 0)
   *
   * Inactive constraints (C > threshold) are not included in solver.
   * This prevents wasted computation on constraints far from violation.
   *
   * Typical threshold: 0.01 to 0.1 depending on application
   *
   * @param state Current inertial state
   * @param time Simulation time [s]
   * @return true if constraint is active (should be enforced)
   */
  [[nodiscard]] virtual bool isActive(
    const InertialState& state,
    double time) const = 0;

protected:
  // Protected Rule of Five: prevent direct instantiation of abstract base
  UnilateralConstraint() = default;
  UnilateralConstraint(const UnilateralConstraint&) = default;
  UnilateralConstraint& operator=(const UnilateralConstraint&) = default;
  UnilateralConstraint(UnilateralConstraint&&) noexcept = default;
  UnilateralConstraint& operator=(UnilateralConstraint&&) noexcept = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_UNILATERAL_CONSTRAINT_HPP

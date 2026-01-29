// Ticket: 0031_generalized_lagrange_constraints
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md

#ifndef MSD_SIM_PHYSICS_BILATERAL_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_BILATERAL_CONSTRAINT_HPP

#include "msd-sim/src/Physics/Constraints/Constraint.hpp"

namespace msd_sim
{

/**
 * @brief Specialization for equality constraints C(q, t) = 0
 *
 * Bilateral constraints enforce exact equalities using unrestricted Lagrange
 * multipliers (λ can be positive or negative). Examples:
 * - Unit quaternion: |Q|² - 1 = 0
 * - Fixed distance: |p₁ - p₂|² - d² = 0
 * - Ball-socket joint: p₁ + R₁·r₁ - p₂ - R₂·r₂ = 0
 *
 * This class is a semantic marker for bilateral (equality) constraints,
 * enabling future solver optimizations that distinguish between equality
 * and inequality constraints.
 *
 * Thread safety: Same as Constraint base class
 * Error handling: Same as Constraint base class
 *
 * @see docs/designs/0031_generalized_lagrange_constraints/0031_generalized_lagrange_constraints.puml
 * @ticket 0031_generalized_lagrange_constraints
 */
class BilateralConstraint : public Constraint
{
public:
  virtual ~BilateralConstraint() = default;

  // No additional interface beyond Constraint
  // Semantic marker for bilateral (equality) constraints

protected:
  // Protected Rule of Five: prevent direct instantiation of abstract base
  BilateralConstraint() = default;
  BilateralConstraint(const BilateralConstraint&) = default;
  BilateralConstraint& operator=(const BilateralConstraint&) = default;
  BilateralConstraint(BilateralConstraint&&) noexcept = default;
  BilateralConstraint& operator=(BilateralConstraint&&) noexcept = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_BILATERAL_CONSTRAINT_HPP

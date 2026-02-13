// Ticket: 0043_constraint_hierarchy_refactor
// Design: docs/designs/0043_constraint_hierarchy_refactor/design.md

#ifndef MSD_SIM_PHYSICS_UNIT_QUATERNION_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_UNIT_QUATERNION_CONSTRAINT_HPP

#include "msd-sim/src/Physics/Constraints/Constraint.hpp"

namespace msd_sim
{

/**
 * @brief Unit quaternion normalization constraint using Lagrange multipliers
 *
 * Maintains the constraint g(Q) = Q^T·Q - 1 = 0 to ensure quaternion remains
 * normalized during numerical integration. This is a drop-in replacement for
 * the deprecated QuaternionConstraint, reimplemented using the generalized
 * constraint framework.
 *
 * Mathematical formulation:
 * - Constraint: C(Q) = Q^T·Q - 1 = 0 (scalar constraint)
 * - Jacobian: J = 2·Q^T (1×4 row vector w.r.t. quaternion components)
 * - Time derivative: ∂C/∂t = 0 (time-independent)
 *
 * Baumgarte stabilization:
 * - Position term: α·(Q^T·Q - 1)
 * - Velocity term: β·(2·Q^T·Q̇)
 * - Default parameters: α = 10.0, β = 10.0 (validated for dt ~ 0.016s)
 *
 * Thread safety: Thread-safe for concurrent evaluation after construction
 * Error handling: No exceptions (all quaternion operations numerically stable)
 *
 * @see docs/designs/0043_constraint_hierarchy_refactor/0043_constraint_hierarchy_refactor.puml
 * @ticket 0043_constraint_hierarchy_refactor
 */
class UnitQuaternionConstraint : public Constraint
{
public:
  /**
   * @brief Construct constraint with body index and Baumgarte parameters
   * @param bodyAIndex Index of body in solver body list (default: 0)
   * @param alpha Position error gain [1/s²] (default: 10.0)
   * @param beta Velocity error gain [1/s] (default: 10.0)
   */
  explicit UnitQuaternionConstraint(size_t bodyAIndex = 0,
                                     double alpha = 10.0,
                                     double beta = 10.0);

  ~UnitQuaternionConstraint() override = default;

  // Constraint interface implementation
  [[nodiscard]] int dimension() const override;

  using Constraint::evaluate;  // Bring base class convenience overload into scope
  [[nodiscard]] Eigen::VectorXd evaluate(const InertialState& stateA,
                                          const InertialState& stateB,
                                          double time) const override;

  using Constraint::jacobian;  // Bring base class convenience overload into scope
  [[nodiscard]] Eigen::MatrixXd jacobian(const InertialState& stateA,
                                          const InertialState& stateB,
                                          double time) const override;
  [[nodiscard]] Eigen::VectorXd partialTimeDerivative(const InertialState& state,
                                        double time) const override;
  [[nodiscard]] LambdaBounds lambdaBounds() const override;
  [[nodiscard]] std::string typeName() const override;

  void recordState(msd_transfer::ConstraintRecordVisitor& visitor,
                   uint32_t bodyAId,
                   uint32_t bodyBId) const override;

  // Rule of Five
  UnitQuaternionConstraint(const UnitQuaternionConstraint&) = default;
  UnitQuaternionConstraint& operator=(const UnitQuaternionConstraint&) = default;
  UnitQuaternionConstraint(UnitQuaternionConstraint&&) noexcept = default;
  UnitQuaternionConstraint& operator=(UnitQuaternionConstraint&&) noexcept = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_UNIT_QUATERNION_CONSTRAINT_HPP

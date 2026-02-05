// Ticket: 0031_generalized_lagrange_constraints
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md

#ifndef MSD_SIM_PHYSICS_UNIT_QUATERNION_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_UNIT_QUATERNION_CONSTRAINT_HPP

#include "msd-sim/src/Physics/Constraints/BilateralConstraint.hpp"

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
 * @see docs/designs/0031_generalized_lagrange_constraints/0031_generalized_lagrange_constraints.puml
 * @ticket 0031_generalized_lagrange_constraints
 */
class UnitQuaternionConstraint : public BilateralConstraint
{
public:
  /**
   * @brief Construct constraint with Baumgarte stabilization parameters
   * @param alpha Position error gain [1/s²] (default: 10.0)
   * @param beta Velocity error gain [1/s] (default: 10.0)
   */
  explicit UnitQuaternionConstraint(double alpha = 10.0, double beta = 10.0);

  ~UnitQuaternionConstraint() override = default;

  // Constraint interface implementation
  [[nodiscard]] int dimension() const override;
  [[nodiscard]] Eigen::VectorXd evaluate(const InertialState& state, double time) const override;
  [[nodiscard]] Eigen::MatrixXd jacobian(const InertialState& state, double time) const override;
  [[nodiscard]] Eigen::VectorXd partialTimeDerivative(const InertialState& state,
                                        double time) const override;
  [[nodiscard]] std::string typeName() const override;

  // Baumgarte parameters
  [[nodiscard]] double alpha() const override;
  [[nodiscard]] double beta() const override;

  /**
   * @brief Set position error gain
   * @param alpha Position error gain [1/s²]
   */
  void setAlpha(double alpha);

  /**
   * @brief Set velocity error gain
   * @param beta Velocity error gain [1/s]
   */
  void setBeta(double beta);

  // Rule of Five
  UnitQuaternionConstraint(const UnitQuaternionConstraint&) = default;
  UnitQuaternionConstraint& operator=(const UnitQuaternionConstraint&) = default;
  UnitQuaternionConstraint(UnitQuaternionConstraint&&) noexcept = default;
  UnitQuaternionConstraint& operator=(UnitQuaternionConstraint&&) noexcept = default;

private:
  double alpha_{10.0};  // Position error gain [1/s²]
  double beta_{10.0};   // Velocity error gain [1/s]
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_UNIT_QUATERNION_CONSTRAINT_HPP

// Ticket: 0043_constraint_hierarchy_refactor
// Design: docs/designs/0043_constraint_hierarchy_refactor/design.md

#ifndef MSD_SIM_PHYSICS_DISTANCE_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_DISTANCE_CONSTRAINT_HPP

#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include <limits>

namespace msd_sim
{

/**
 * @brief Fixed distance constraint from origin (single-object example)
 *
 * Maintains the constraint C(X) = |X|² - d² = 0 to keep an object at a fixed
 * distance d from the origin. This is a simplified single-object constraint
 * demonstrating the constraint framework. Multi-object distance constraints
 * (between two objects) are deferred to future ticket 0032.
 *
 * Mathematical formulation:
 * - Constraint: C(X) = |X|² - d² = 0 (scalar constraint)
 * - Jacobian: J = 2·X^T (1×3 row vector w.r.t. position components)
 * - Time derivative: ∂C/∂t = 0 (time-independent)
 *
 * Use cases:
 * - Orbital mechanics: fixed orbital radius
 * - Pendulum: fixed rod length
 * - Tethered objects: cable length constraint
 *
 * Thread safety: Thread-safe for concurrent evaluation after construction
 * Error handling: Throws std::invalid_argument if targetDistance <= 0
 *
 * @see docs/designs/0043_constraint_hierarchy_refactor/0043_constraint_hierarchy_refactor.puml
 * @ticket 0043_constraint_hierarchy_refactor
 */
class DistanceConstraint : public Constraint
{
public:
  /**
   * @brief Construct constraint with target distance from origin
   * @param targetDistance Desired distance from origin [m]
   * @param bodyAIndex Index of body in solver body list (default: 0)
   * @param alpha Position error gain [1/s²] (default: 10.0)
   * @param beta Velocity error gain [1/s] (default: 10.0)
   * @throws std::invalid_argument if targetDistance <= 0
   */
  explicit DistanceConstraint(double targetDistance,
                              size_t bodyAIndex = 0,
                              double alpha = 10.0,
                              double beta = 10.0);

  ~DistanceConstraint() override = default;

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

  /**
   * @brief Get target distance from origin
   * @return Target distance [m]
   */
  [[nodiscard]] double getTargetDistance() const;

  // Rule of Five
  DistanceConstraint(const DistanceConstraint&) = default;
  DistanceConstraint& operator=(const DistanceConstraint&) = default;
  DistanceConstraint(DistanceConstraint&&) noexcept = default;
  DistanceConstraint& operator=(DistanceConstraint&&) noexcept = default;

private:
  double targetDistance_{std::numeric_limits<double>::quiet_NaN()};  // Target distance [m]
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_DISTANCE_CONSTRAINT_HPP

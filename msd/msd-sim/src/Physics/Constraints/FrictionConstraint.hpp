// Ticket: 0035a_tangent_basis_and_friction_constraint
// Design: docs/designs/0035a_tangent_basis_and_friction_constraint/design.md

#ifndef MSD_SIM_PHYSICS_FRICTION_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_FRICTION_CONSTRAINT_HPP

#include <Eigen/Dense>
#include <utility>
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Collision/TangentBasis.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/LambdaBounds.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/**
 * @brief Friction constraint for two-body contact (two tangential constraint
 * rows)
 *
 * Implements tangential friction constraints for a contact point using two
 * orthogonal tangent directions. Each FrictionConstraint has dimension() = 2
 * (one row per tangent).
 *
 * Constraint formulation:
 * - C_t1(q) = relative tangential velocity in t1 direction
 * - C_t2(q) = relative tangential velocity in t2 direction
 * - Jacobian J_ti = [ti^T, (rA×ti)^T, -ti^T, -(rB×ti)^T] ∈ R^(1×12)
 *
 * Box constraints (Coulomb cone projection):
 * - -μ/√2 · λn ≤ λt1 ≤ μ/√2 · λn
 * - -μ/√2 · λn ≤ λt2 ≤ μ/√2 · λn
 * - √2 factor distributes circular cone into two orthogonal box constraints
 *
 * Design notes:
 * - Tangent basis computed once at construction via
 * TangentBasis::computeTangentBasis()
 * - Friction coefficient μ stored (combined from materials, computed
 * externally)
 * - Normal force λn updated each solver iteration via setNormalLambda()
 * - Follows same 12-DOF Jacobian structure as ContactConstraint (direct angular
 * velocity)
 *
 * Mathematical formulation:
 * docs/designs/0035_friction_constraints/M2-friction-jacobian.md Thread safety:
 * Immutable after construction except setNormalLambda() (not thread-safe) Error
 * handling: Constructor validates normal unit length, mu in [0, ∞)
 *
 * @see Constraint
 * @see ContactConstraint
 * @see
 * docs/designs/0035a_tangent_basis_and_friction_constraint/0035a_tangent_basis_and_friction_constraint.puml
 * @ticket 0035a_tangent_basis_and_friction_constraint
 */
class FrictionConstraint : public Constraint
{
public:
  /**
   * @brief Construct friction constraint for a contact point
   *
   * @param bodyAIndex Index of body A in solver body list
   * @param bodyBIndex Index of body B in solver body list
   * @param normal Contact normal (A → B, unit length)
   * @param contactPointA Contact point on A's surface (world space) [m]
   * @param contactPointB Contact point on B's surface (world space) [m]
   * @param comA Center of mass of body A (world space) [m]
   * @param comB Center of mass of body B (world space) [m]
   * @param frictionCoefficient Combined friction coefficient μ [0, ∞)
   *
   * @throws std::invalid_argument if normal is not unit length (within 1e-6)
   * @throws std::invalid_argument if frictionCoefficient < 0
   *
   * Thread safety: Not thread-safe during construction
   * Complexity: O(1) - computes tangent basis and lever arms once
   */
  FrictionConstraint(size_t bodyAIndex,
                     size_t bodyBIndex,
                     const Coordinate& normal,
                     const Coordinate& contactPointA,
                     const Coordinate& contactPointB,
                     const Coordinate& comA,
                     const Coordinate& comB,
                     double frictionCoefficient);

  ~FrictionConstraint() override = default;

  // ===== Constraint interface =====

  /**
   * @brief Number of scalar constraint equations (always 2 for friction)
   * @return 2 (one row for t1 direction, one row for t2 direction)
   */
  [[nodiscard]] int dimension() const override
  {
    return 2;
  }

  /**
   * @brief Evaluate tangential relative velocities at contact point
   *
   * Computes:
   * - C(0) = v_rel · t1 (relative velocity in first tangent direction)
   * - C(1) = v_rel · t2 (relative velocity in second tangent direction)
   *
   * where v_rel = (vA + ωA × rA) - (vB + ωB × rB)
   *
   * @param stateA Inertial state of body A
   * @param stateB Inertial state of body B
   * @param time Simulation time [s] (unused for friction)
   * @return 2×1 vector of tangential relative velocities [m/s]
   */
  [[nodiscard]] Eigen::VectorXd evaluate(const InertialState& stateA,
                                  const InertialState& stateB,
                                  double time) const override;

  /**
   * @brief Compute two-body Jacobian for friction constraints
   *
   * Returns 2×12 matrix:
   * - Row 0: J_t1 = [t1^T, (rA×t1)^T, -t1^T, -(rB×t1)^T]
   * - Row 1: J_t2 = [t2^T, (rA×t2)^T, -t2^T, -(rB×t2)^T]
   *
   * Each row has structure: [v_A (3), ω_A (3), v_B (3), ω_B (3)]
   *
   * @param stateA Inertial state of body A (unused, pre-computed geometry)
   * @param stateB Inertial state of body B (unused, pre-computed geometry)
   * @param time Simulation time [s] (unused for friction)
   * @return 2×12 Jacobian matrix
   */
  [[nodiscard]] Eigen::MatrixXd jacobian(const InertialState& stateA,
                                  const InertialState& stateB,
                                  double time) const override;

  /**
   * @brief Check if friction constraint is active
   *
   * Friction is active when:
   * - Normal contact force λn > 0 (contact is active)
   * - Friction coefficient μ > 0 (non-zero friction)
   *
   * @param stateA Inertial state of body A (unused for activation check)
   * @param stateB Inertial state of body B (unused for activation check)
   * @param time Simulation time [s] (unused for activation check)
   * @return true if friction should be enforced
   */
  [[nodiscard]] bool isActive(const InertialState& stateA,
                       const InertialState& stateB,
                       double time) const override;

  [[nodiscard]] LambdaBounds lambdaBounds() const override
  {
    auto [lower, upper] = getFrictionBounds();
    return LambdaBounds::boxConstrained(lower, upper);
  }

  [[nodiscard]] int bodyCount() const override
  {
    return 2;
  }

  [[nodiscard]] std::string typeName() const override
  {
    return "FrictionConstraint";
  }

  void recordState(msd_transfer::ConstraintRecordVisitor& visitor,
                   uint32_t bodyAId,
                   uint32_t bodyBId) const override;

  // ===== Friction-specific interface =====

  /**
   * @brief Update normal contact force for friction bound computation
   *
   * Called by solver each iteration to update friction bounds based on current
   * normal force. Must be called before getFrictionBounds().
   *
   * @param normalLambda Normal contact force λn from ContactConstraint [N]
   *
   * Thread safety: Not thread-safe (mutates internal state)
   */
  void setNormalLambda(double normalLambda);

  /**
   * @brief Set solved tangent force magnitudes for recording.
   *
   * Called after solver completes to store the actual tangent friction
   * forces (in Newtons) for database recording and visualization.
   *
   * @param t1Lambda Solved tangent1 force [N]
   * @param t2Lambda Solved tangent2 force [N]
   */
  void setTangentLambdas(double t1Lambda, double t2Lambda);

  /**
   * @brief Get friction force bounds for box-constrained LCP
   *
   * Returns bounds for friction forces in both tangent directions:
   * - Lower bound: -μ/√2 · λn
   * - Upper bound: +μ/√2 · λn
   *
   * The √2 factor distributes the circular Coulomb cone into two orthogonal
   * box constraints (inscribed square approximation).
   *
   * @return {lower_bound, upper_bound} pair [N]
   *
   * Thread safety: Not thread-safe if setNormalLambda() called concurrently
   * Precondition: setNormalLambda() must have been called with valid λn
   */
  [[nodiscard]] std::pair<double, double> getFrictionBounds() const;

  /**
   * @brief Get first tangent direction (world space)
   * @return Unit tangent vector t1
   */
  [[nodiscard]] const Coordinate& getTangent1() const
  {
    return tangent1_;
  }

  /**
   * @brief Get second tangent direction (world space)
   * @return Unit tangent vector t2
   */
  [[nodiscard]] const Coordinate& getTangent2() const
  {
    return tangent2_;
  }

  /**
   * @brief Get friction coefficient
   * @return Combined friction coefficient μ
   */
  [[nodiscard]] double getFrictionCoefficient() const
  {
    return friction_coefficient_;
  }

  // Rule of Five
  FrictionConstraint(const FrictionConstraint&) = default;
  FrictionConstraint& operator=(const FrictionConstraint&) = default;
  FrictionConstraint(FrictionConstraint&&) noexcept = default;
  FrictionConstraint& operator=(FrictionConstraint&&) noexcept = default;

private:
  Coordinate contact_normal_;    // A → B, unit length (stored for reference)
  Coordinate tangent1_;          // First tangent direction (unit length)
  Coordinate tangent2_;          // Second tangent direction (unit length)
  Coordinate lever_arm_a_;       // contactPointA - comA (world frame) [m]
  Coordinate lever_arm_b_;       // contactPointB - comB (world frame) [m]
  double friction_coefficient_;  // Combined friction coefficient μ [0, ∞)
  double normal_lambda_{
    0.0};  // Normal contact force λn [N] (updated each iteration)
  double tangent1_lambda_{0.0};  // Solved tangent1 force [N]
  double tangent2_lambda_{0.0};  // Solved tangent2 force [N]
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_FRICTION_CONSTRAINT_HPP

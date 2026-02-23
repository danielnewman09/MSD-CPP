// Ticket: 0032_contact_constraint_refactor
// Ticket: 0075a_unified_constraint_data_structure
// Design: docs/designs/0032_contact_constraint_refactor/design.md
// Design: docs/designs/0075_unified_contact_constraint/design.md (Phase 1)

#ifndef MSD_SIM_PHYSICS_CONTACT_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_CONTACT_CONSTRAINT_HPP

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/LambdaBounds.hpp"

namespace msd_sim
{

/**
 * @brief Unified non-penetration and friction constraint for a single contact
 * point
 *
 * In Phase 1 (ticket 0075a), ContactConstraint is extended to absorb the
 * friction fields from FrictionConstraint. When frictionCoefficient > 0:
 * - dimension() returns 3 (normal + 2 tangent rows)
 * - jacobian() returns a 3x12 matrix ([J_n; J_t1; J_t2])
 * - hasFriction() returns true
 *
 * When frictionCoefficient == 0 (default):
 * - dimension() returns 1 (normal only, backward-compatible)
 * - jacobian() returns a 1x12 matrix
 * - hasFriction() returns false
 *
 * Backward compatibility: The 10-argument constructor remains valid via the
 * default frictionCoefficient = 0.0 argument. Existing callsites that do not
 * pass friction are unaffected.
 *
 * Design decisions:
 * - Pre-computed lever arms: r_A and r_B computed once at construction
 * - Stores pre-impact velocity: Relative normal velocity at impact time
 *   captured for restitution RHS term
 * - Baumgarte parameters: Uses Error Reduction Parameter (ERP) formulation
 *   standard in physics engines. Default: ERP = 0.2 (equivalent to α_accel ≈
 *   781 [1/s²] at 60 FPS). Validated by Prototype P1 parameter sweep.
 * - Tangent basis: Computed once at construction from normal via Duff et al.
 *   (2017). Only populated when frictionCoefficient > 0.
 * - setSlidingMode(): Overrides tangent basis to align t1 with the sliding
 *   direction (opposing motion). Preserved from FrictionConstraint for safety
 *   net during Block PGS validation.
 *
 * CRITICAL: Restitution formula uses v_target = -e · v_pre (not -(1+e)·v_pre).
 * The constraint RHS for PGS is b = -(1+e) · J·q̇⁻, but the target velocity is
 * -e·v_pre. See P2 Debug Findings for details.
 *
 * Thread safety: Immutable after construction except setSlidingMode() and
 * setTangentLambdas() (not thread-safe)
 * Error handling: Constructor validates normal is unit length, penetration >=
 * 0, restitution in [0, 1], frictionCoefficient >= 0
 *
 * @see docs/designs/0075_unified_contact_constraint/design.md
 * @see docs/designs/0032_contact_constraint_refactor/0032_contact_constraint_refactor.puml
 * @see prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/Debug_Findings.md
 * @ticket 0032_contact_constraint_refactor
 * @ticket 0075a_unified_constraint_data_structure
 */
class ContactConstraint : public Constraint
{
public:
  /**
   * @brief Construct a unified contact constraint for a single contact point
   *
   * @param bodyAIndex Index of body A in the solver body list
   * @param bodyBIndex Index of body B in the solver body list
   * @param normal Contact normal (A → B, unit length)
   * @param contactPointA Contact point on A's surface (world space) [m]
   * @param contactPointB Contact point on B's surface (world space) [m]
   * @param penetrationDepth Overlap distance [m] (positive when penetrating)
   * @param comA Center of mass of body A (world space) [m]
   * @param comB Center of mass of body B (world space) [m]
   * @param restitution Coefficient of restitution [0, 1]
   * @param preImpactRelVelNormal Pre-impact relative normal velocity [m/s]
   * @param frictionCoefficient Combined friction coefficient μ [0, ∞)
   *        Default 0.0 → frictionless (dimension=1, backward-compatible)
   *
   * @throws std::invalid_argument if normal is not unit length (within 1e-6)
   * @throws std::invalid_argument if penetration < 0
   * @throws std::invalid_argument if restitution not in [0, 1]
   * @throws std::invalid_argument if frictionCoefficient < 0
   */
  ContactConstraint(size_t bodyAIndex,
                    size_t bodyBIndex,
                    const Coordinate& normal,
                    const Coordinate& contactPointA,
                    const Coordinate& contactPointB,
                    double penetrationDepth,
                    const Coordinate& comA,
                    const Coordinate& comB,
                    double restitution,
                    double preImpactRelVelNormal,
                    double frictionCoefficient = 0.0);

  ~ContactConstraint() override = default;

  // ===== Constraint interface =====

  /**
   * @brief Number of scalar constraint equations
   *
   * Returns 1 when frictionless (mu == 0), 3 when friction is active (mu > 0).
   */
  [[nodiscard]] int dimension() const override
  {
    return hasFriction() ? 3 : 1;
  }

  [[nodiscard]] Eigen::VectorXd evaluate(
    const InertialState& stateA,
    const InertialState& stateB,
    double time) const override;

  /**
   * @brief Compute constraint Jacobian
   *
   * Returns 1x12 when frictionless, 3x12 when friction active.
   * Structure (3x12 case):
   *   Row 0: J_n  = [-n^T,  -(rA×n)^T,   n^T,  (rB×n)^T ]
   *   Row 1: J_t1 = [ t1^T, (rA×t1)^T, -t1^T, -(rB×t1)^T]
   *   Row 2: J_t2 = [ t2^T, (rA×t2)^T, -t2^T, -(rB×t2)^T]
   */
  [[nodiscard]] Eigen::MatrixXd jacobian(
    const InertialState& stateA,
    const InertialState& stateB,
    double time) const override;

  [[nodiscard]] bool isActive(
    const InertialState& stateA,
    const InertialState& stateB,
    double time) const override;

  [[nodiscard]] LambdaBounds lambdaBounds() const override
  {
    return LambdaBounds::unilateral();
  }

  [[nodiscard]] int bodyCount() const override
  {
    return 2;
  }

  [[nodiscard]] std::string typeName()
    const override
  {
    return "ContactConstraint";
  }

  void recordState(msd_transfer::ConstraintRecordVisitor& visitor,
                   uint32_t bodyAId,
                   uint32_t bodyBId) const override;

  // ===== Contact accessors =====

  [[nodiscard]] const Coordinate& getContactNormal()
    const
  {
    return contact_normal_;
  }
  [[nodiscard]] double getPenetrationDepth() const
  {
    return penetration_depth_;
  }
  [[nodiscard]] double getRestitution() const
  {
    return restitution_;
  }
  [[nodiscard]] double getPreImpactRelVelNormal()
    const
  {
    return pre_impact_rel_vel_normal_;
  }
  [[nodiscard]] const Coordinate& getLeverArmA()
    const
  {
    return lever_arm_a_;
  }
  [[nodiscard]] const Coordinate& getLeverArmB()
    const
  {
    return lever_arm_b_;
  }

  // ===== Friction accessors =====

  /**
   * @brief Whether this contact has friction (mu > 0)
   */
  [[nodiscard]] bool hasFriction() const
  {
    return friction_coefficient_ > 0.0;
  }

  /**
   * @brief Get the friction coefficient
   */
  [[nodiscard]] double getFrictionCoefficient() const
  {
    return friction_coefficient_;
  }

  /**
   * @brief Get first tangent direction (world space)
   * @return Unit tangent vector t1 (zero vector if frictionless)
   */
  [[nodiscard]] const Coordinate& getTangent1() const
  {
    return tangent1_;
  }

  /**
   * @brief Get second tangent direction (world space)
   * @return Unit tangent vector t2 (zero vector if frictionless)
   */
  [[nodiscard]] const Coordinate& getTangent2() const
  {
    return tangent2_;
  }

  /**
   * @brief Check if constraint is in sliding mode
   * @return true if sliding mode is active
   * @ticket 0069_friction_velocity_reversal
   */
  [[nodiscard]] bool isSlidingMode() const
  {
    return is_sliding_mode_;
  }

  /**
   * @brief Enable sliding friction mode with aligned tangent basis
   *
   * Overrides the tangent basis to align t1 with -slidingDirection (opposing
   * motion), with t2 perpendicular to both t1 and the contact normal. This
   * ensures lambda_t1 directly corresponds to deceleration along the sliding
   * direction. No-op when hasFriction() is false.
   *
   * @param slidingDirection Unit sliding direction (world frame)
   *
   * Thread safety: Not thread-safe (mutates tangent basis)
   * @ticket 0069_friction_velocity_reversal
   */
  void setSlidingMode(const Vector3D& slidingDirection);

  /**
   * @brief Set solved tangent force magnitudes for recording.
   *
   * Called after Block PGS solver completes to store the actual tangent
   * friction forces (in Newtons) for database recording and visualization.
   *
   * @param t1Lambda Solved tangent1 force [N]
   * @param t2Lambda Solved tangent2 force [N]
   */
  void setTangentLambdas(double t1Lambda, double t2Lambda);

  /**
   * @brief Set solved normal force magnitude for recording.
   *
   * Called after solver completes to store the normal force value.
   *
   * @param normalLambda Solved normal force [N]
   */
  void setNormalLambda(double normalLambda);

  // Rule of Five
  ContactConstraint(const ContactConstraint&) = default;
  ContactConstraint& operator=(const ContactConstraint&) = default;
  ContactConstraint(ContactConstraint&&) noexcept = default;
  ContactConstraint& operator=(ContactConstraint&&) noexcept = default;

private:
  // Contact geometry
  Coordinate contact_normal_;         // A → B, unit length
  Coordinate lever_arm_a_;            // contactPointA - comA (world frame) [m]
  Coordinate lever_arm_b_;            // contactPointB - comB (world frame) [m]
  double penetration_depth_;          // Overlap distance [m]
  double restitution_;                // Coefficient of restitution [0, 1]
  double pre_impact_rel_vel_normal_;  // For restitution RHS [m/s]

  // Friction fields (0075a: merged from FrictionConstraint)
  double friction_coefficient_{0.0};  // Combined friction coefficient μ [0, ∞)
  Coordinate tangent1_;               // First tangent direction (zero if frictionless)
  Coordinate tangent2_;               // Second tangent direction (zero if frictionless)
  double normal_lambda_{0.0};         // Solved normal force λn [N]
  double tangent1_lambda_{0.0};       // Solved tangent1 force [N]
  double tangent2_lambda_{0.0};       // Solved tangent2 force [N]
  bool is_sliding_mode_{false};       // Sliding mode active flag (ticket 0069)
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONTACT_CONSTRAINT_HPP

// Ticket: 0032_contact_constraint_refactor
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#ifndef MSD_SIM_PHYSICS_CONTACT_CONSTRAINT_HPP
#define MSD_SIM_PHYSICS_CONTACT_CONSTRAINT_HPP

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/LambdaBounds.hpp"

namespace msd_sim
{

/**
 * @brief Non-penetration unilateral constraint for a single contact point
 *
 * Implements the contact constraint C(q) = (x_B - x_A) · n ≥ 0 where:
 * - x_A, x_B are contact points on bodies A and B (world space)
 * - n is the contact normal (A → B, unit length)
 *
 * Each ContactConstraint has dimension() = 1 (one scalar constraint per contact
 * point). A CollisionResult with contactCount = 4 produces 4 ContactConstraint
 * instances. This follows the math formulation Section 2.5 and provides better
 * stability for resting contacts than a centroid approximation.
 *
 * Design decisions:
 * - Pre-computed lever arms: r_A and r_B computed once at construction
 * - Stores pre-impact velocity: Relative normal velocity at impact time
 * captured for restitution RHS term
 * - Baumgarte parameters: Uses Error Reduction Parameter (ERP) formulation
 *   standard in physics engines. Default: ERP = 0.2 (equivalent to α_accel ≈
 * 781 [1/s²] at 60 FPS). Validated by Prototype P1 parameter sweep.
 *
 * CRITICAL: Restitution formula uses v_target = -e · v_pre (not -(1+e)·v_pre).
 * The constraint RHS for PGS is b = -(1+e) · J·q̇⁻, but the target velocity is
 * -e·v_pre. See P2 Debug Findings for details.
 *
 * Thread safety: Immutable after construction; thread-safe for concurrent reads
 * Error handling: Constructor validates normal is unit length, penetration >=
 * 0, restitution in [0, 1]
 *
 * @see
 * docs/designs/0032_contact_constraint_refactor/0032_contact_constraint_refactor.puml
 * @see
 * prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/Debug_Findings.md
 * @see
 * prototypes/0032_contact_constraint_refactor/p2_energy_conservation/Debug_Findings.md
 * @ticket 0032_contact_constraint_refactor
 */
class ContactConstraint : public Constraint
{
public:
  /**
   * @brief Construct a contact constraint for a single contact point
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
   *
   * @throws std::invalid_argument if normal is not unit length (within 1e-6)
   * @throws std::invalid_argument if penetration < 0
   * @throws std::invalid_argument if restitution not in [0, 1]
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
                    double preImpactRelVelNormal);

  ~ContactConstraint() override = default;

  // ===== Constraint interface =====

  [[nodiscard]] int dimension() const override
  {
    return 1;
  }

  [[nodiscard]] Eigen::VectorXd evaluate(
    const InertialState& stateA,
    const InertialState& stateB,
    double time) const override;

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

  // ===== Accessors =====

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

  // Rule of Five
  ContactConstraint(const ContactConstraint&) = default;
  ContactConstraint& operator=(const ContactConstraint&) = default;
  ContactConstraint(ContactConstraint&&) noexcept = default;
  ContactConstraint& operator=(ContactConstraint&&) noexcept = default;

private:
  Coordinate contact_normal_;         // A → B, unit length
  Coordinate lever_arm_a_;            // contactPointA - comA (world frame) [m]
  Coordinate lever_arm_b_;            // contactPointB - comB (world frame) [m]
  double penetration_depth_;          // Overlap distance [m]
  double restitution_;                // Coefficient of restitution [0, 1]
  double pre_impact_rel_vel_normal_;  // For restitution RHS [m/s]
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONTACT_CONSTRAINT_HPP

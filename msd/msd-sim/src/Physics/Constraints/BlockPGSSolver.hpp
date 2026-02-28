// Ticket: 0075b_block_pgs_solver
// Design: docs/designs/0075_unified_contact_constraint/design.md (Phase 2)

#ifndef MSD_SIM_PHYSICS_CONSTRAINTS_BLOCK_PGS_SOLVER_HPP
#define MSD_SIM_PHYSICS_CONSTRAINTS_BLOCK_PGS_SOLVER_HPP

#include <Eigen/Dense>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

#include "msd-sim/src/DataTypes/ForceVector.hpp"
#include "msd-sim/src/DataTypes/TorqueVector.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/**
 * @brief Two-phase Block Projected Gauss-Seidel contact solver.
 *
 * Replaces the decoupled NLopt friction solver path in ConstraintSolver for
 * contacts with friction. The two-phase structure separates restitution energy
 * injection (Phase A) from the purely dissipative friction solve (Phase B),
 * preventing the energy injection mechanism identified in DD-0070-H2.
 *
 * ## Algorithm Overview
 *
 * **Phase A — Restitution Pre-Solve** (sequential, once per solve):
 *   For each bouncing contact (e > 0 and Jv_n < 0):
 *   1. Compute bounce impulse: lambda_bounce = (1+e) * (-Jv_n) / K_nn
 *   2. Update velocity residual (normal direction only)
 *   3. Store bounce impulse for final lambda accumulation
 *
 *   Phase A is a no-op for resting contacts (e=0), which is the common case
 *   for persistent contact.
 *
 * **Phase B — Dissipative Block PGS Sweeps** (iterative):
 *   For each sweep, for each contact c (3x3 block):
 *   1. v_err = J_block * (v_pre + vRes_[bodyA, bodyB])
 *   2. delta_lambda = K^{-1} * (-v_err)  [no restitution term]
 *   3. lambda_acc += delta_lambda, then project onto Coulomb cone
 *   4. vRes_ += M^{-1} * J_block^T * (lambda_proj - lambda_acc_old)
 *
 * **Energy safety proof**: Phase A injects controlled restitution energy
 * bounded by e and v_pre. Phase B with b=-v_err and Coulomb cone projection
 * is provably dissipative — it can never inject energy. Combined: total energy
 * change = restitution budget + dissipation (always non-positive for e < 1).
 *
 * ## Physical Meaning of the 3x3 Block
 *
 * The 3x3 effective mass K = J_block * M^{-1} * J_block^T captures coupling
 * between normal and friction directions. The off-diagonal K_nt terms represent
 * how friction impulses affect normal relative velocity (and vice versa). These
 * terms are non-zero when lever arms create rotational coupling between the
 * contact directions — the effect the decoupled solver ignores.
 *
 * ## vRes_ (Velocity Residual)
 *
 * vRes_ is a 6*numBodies vector accumulating M^{-1} * J^T * lambda for all
 * constraint impulses applied so far. The actual body velocity at any point
 * during the solve is v_pre[i] + vRes_[6i : 6i+6].
 *
 * @see docs/designs/0075_unified_contact_constraint/design.md (Phase 2)
 * @ticket 0075b_block_pgs_solver
 */
class BlockPGSSolver
{
public:
  /// Per-body forces result
  struct BodyForces
  {
    ForceVector linearForce;
    TorqueVector angularTorque;

    BodyForces() = default;
    BodyForces(const ForceVector& lf, const TorqueVector& at)
      : linearForce{lf}, angularTorque{at}
    {
    }
  };

  /// Solve result
  struct SolveResult
  {
    std::vector<BodyForces> bodyForces;  ///< Per-body force/torque (size = numBodies)
    Eigen::VectorXd lambdas;            ///< 3 per contact: [lambda_n, lambda_t1, lambda_t2]
                                        ///< = Phase A bounce + Phase B
    Eigen::VectorXd phaseBLambdas;      ///< Phase B lambdas only (no bounce) — used as
                                        ///< warm-start for next frame. Ticket: 0084 Fix F2.
    bool converged{false};
    int iterations{0};
    double residual{std::numeric_limits<double>::quiet_NaN()};
  };

  // Rule of Zero: all members are value types with correct copy/move semantics.

  /**
   * @brief Solve contact + friction constraint system using two-phase Block PGS.
   *
   * All constraints must be ContactConstraint with hasFriction() == true.
   * For frictionless contacts, use ConstraintSolver's ASM path instead.
   *
   * @param constraints ContactConstraint* pointers (must all have hasFriction())
   * @param states Per-body kinematic state
   * @param inverseMasses Per-body inverse mass (size = numBodies)
   * @param inverseInertias Per-body inverse inertia tensor (size = numBodies)
   * @param numBodies Number of bodies in the system
   * @param dt Timestep [s]
   * @param initialLambda Warm-start vector (3*N for N contacts); zeros if empty
   * @return SolveResult with bodyForces, lambdas, iterations, converged, residual
   */
  [[nodiscard]] SolveResult solve(
    const std::vector<Constraint*>& constraints,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    size_t numBodies,
    double dt,
    const std::optional<Eigen::VectorXd>& initialLambda = std::nullopt);

  /// Set maximum sweep count (default: 50)
  void setMaxSweeps(int n) { maxSweeps_ = n; }

  /// Set convergence tolerance: early exit when max ||delta|| < tol (default: 1e-6)
  void setConvergenceTolerance(double tol) { convergenceTolerance_ = tol; }

  [[nodiscard]] int getMaxSweeps() const { return maxSweeps_; }
  [[nodiscard]] double getConvergenceTolerance() const { return convergenceTolerance_; }

private:
  /**
   * @brief Build 3x3 block effective mass: K = J_block * M^{-1} * J_block^T
   *
   * K encodes how a 3D impulse (normal + 2 tangent) at this contact changes
   * the 3D relative constraint velocity. Off-diagonal entries K_nt capture
   * the coupling between friction impulses and normal velocity change.
   *
   * CFM regularization (kCFMEpsilon) is added to the diagonal to prevent
   * singularity at extreme mass ratios.
   *
   * @param c Unified contact constraint (provides 3x12 Jacobian block)
   * @param inverseMasses Per-body inverse mass
   * @param inverseInertias Per-body inverse inertia tensor
   * @return 3x3 effective mass matrix for this contact block
   */
  [[nodiscard]] Eigen::Matrix3d buildBlockK(
    const ContactConstraint& c,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias) const;

  /**
   * @brief Project 3D impulse block onto Coulomb friction cone.
   *
   * Cone definition: lambda_n >= 0, ||lambda_t|| <= mu * lambda_n
   *
   * Cases:
   * - lambda_n < 0: return (0, 0, 0)  [contact separating]
   * - ||lambda_t|| <= mu * lambda_n: return unchanged  [static friction]
   * - ||lambda_t|| > mu * lambda_n: scale tangent to cone surface  [sliding]
   *
   * @param lambda_block 3D impulse (lambda_n, lambda_t1, lambda_t2)
   * @param mu Combined Coulomb friction coefficient
   * @return Projected impulse satisfying cone constraint
   */
  [[nodiscard]] static Eigen::Vector3d projectCoulombCone(
    const Eigen::Vector3d& lambda_block, double mu);

  /**
   * @brief Phase A: Apply restitution bounce impulse for a single contact.
   *
   * Iterates contacts sequentially so each contact's bounce updates the
   * velocity state seen by subsequent contacts. Returns per-contact bounce
   * lambdas for accumulation into the final result.
   *
   * For resting contacts (e == 0) or already-separating contacts (Jv_n >= 0),
   * this is a no-op.
   *
   * @param contacts ContactConstraint pointers (friction contacts only)
   * @param blockKs Pre-computed 3x3 K matrices per contact
   * @param states Per-body kinematic state
   * @param inverseMasses Per-body inverse mass
   * @param inverseInertias Per-body inverse inertia tensor
   * @return Per-contact bounce impulse (lambda_n contribution from Phase A)
   */
  [[nodiscard]] std::vector<double> applyRestitutionPreSolve(
    const std::vector<ContactConstraint*>& contacts,
    const std::vector<Eigen::Matrix3d>& blockKs,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias);

  /**
   * @brief Update vRes_ for normal row only (Phase A helper).
   *
   * vRes_ += M^{-1} * J_n^T * deltaLambdaNormal
   *
   * Only the normal direction (row 0 of the 3x12 Jacobian) is used.
   */
  void updateVResNormalOnly(
    const ContactConstraint& c,
    double deltaLambdaNormal,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias);

  /**
   * @brief Phase B: Execute one complete sweep over all contacts.
   *
   * For each contact: solve the 3x3 block using the full coupled K^{-1} solve.
   * Accumulates, projects onto Coulomb cone, updates velocity residual.
   * Returns max ||delta|| for convergence check.
   *
   * Algorithm per contact c (original coupled solve — P5 velocity-gated clamp reverted):
   *   1. v_err = J_block * (v_pre + vRes_[bodyA, bodyB])
   *   2. unconstrained = K_inv * (-v_err)  [full coupled 3x3 solve]
   *   3. lambda_temp = lambda_acc[c] + unconstrained
   *   4. lambda_proj = projectCoulombCone(lambda_temp, mu)
   *   5. delta = lambda_proj - lambda_acc[c]
   *   6. vRes_ += M^{-1} * J_block^T * delta
   *   7. lambda_acc[c] = lambda_proj
   *
   * Energy injection from the nonlinear Coulomb cone projection breaking the
   * pre-projection cancellation symmetry is handled by the post-sweep
   * net-zero redistribution in solve() — not in this per-contact function.
   * See math-formulation.md Section 3 and 6, and Ticket: 0084 Prototype P6.
   *
   * @param contacts ContactConstraint pointers
   * @param blockKInvs Pre-computed K^{-1} per contact (full 3x3 coupled inverse)
   * @param lambda In/out accumulated 3D impulses (3*numContacts)
   * @param states Per-body kinematic state
   * @param inverseMasses Per-body inverse mass
   * @param inverseInertias Per-body inverse inertia tensor
   * @return max ||delta|| across all contacts (convergence metric)
   */
  double sweepOnce(
    const std::vector<ContactConstraint*>& contacts,
    const std::vector<Eigen::Matrix3d>& blockKInvs,
    Eigen::VectorXd& lambda,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias);

  /**
   * @brief Update vRes_ for all 3 block rows.
   *
   * vRes_ += M^{-1} * J_block^T * delta3
   * where delta3 = (delta_n, delta_t1, delta_t2).
   */
  void updateVRes3(
    const ContactConstraint& c,
    const Eigen::Vector3d& delta3,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias);

  /**
   * @brief Compute v_err = J_block * (v_pre + vRes_[bodyA, bodyB])
   *
   * Returns the 3D constraint-space velocity error:
   *   v_err(0) = J_n  * (v_A + vRes_A, v_B + vRes_B)  [normal direction]
   *   v_err(1) = J_t1 * (...)                           [tangent1 direction]
   *   v_err(2) = J_t2 * (...)                           [tangent2 direction]
   *
   * @param c Contact constraint (provides body indices and Jacobian)
   * @param states Per-body pre-solve kinematic state (v_pre)
   * @return 3D velocity error vector
   */
  [[nodiscard]] Eigen::Vector3d computeBlockVelocityError(
    const ContactConstraint& c,
    const std::vector<std::reference_wrapper<const InertialState>>& states) const;

  // Configuration
  int maxSweeps_{50};
  double convergenceTolerance_{1e-6};

  // CFM regularization on K diagonal: prevents singularity at extreme mass ratios
  static constexpr double kCFMEpsilon{1e-8};

  // DOF layout constants
  static constexpr Eigen::Index kLinearDof{3};
  static constexpr Eigen::Index kAngularDof{3};
  static constexpr Eigen::Index kBodyDof{kLinearDof + kAngularDof};  // 6

  // Per-solve workspace members — reused across calls to avoid heap allocation.
  // Ticket: 0071f_solver_workspace_reuse (pattern established for PGS)

  /// Velocity residual workspace: vRes_[6k..6k+5] = accumulated M^{-1}*J^T*lambda for body k.
  Eigen::VectorXd vRes_;

  /// Per-contact bounce impulse from Phase A. Size = numContacts.
  std::vector<double> bounceLambdas_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINTS_BLOCK_PGS_SOLVER_HPP

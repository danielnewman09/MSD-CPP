// Ticket: 0031_generalized_lagrange_constraints
// Ticket: 0032_contact_constraint_refactor
// Ticket: 0035b4_ecos_solve_integration
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md
// Design: docs/designs/0032_contact_constraint_refactor/design.md
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#ifndef MSD_SIM_PHYSICS_CONSTRAINT_SOLVER_HPP
#define MSD_SIM_PHYSICS_CONSTRAINT_SOLVER_HPP

#include <Eigen/Dense>

#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/**
 * @brief Computes Lagrange multipliers for arbitrary constraint systems
 *
 * Uses direct linear solve (LLT decomposition) to compute Lagrange multipliers
 * that enforce constraint equations via generalized forces. Suitable for small
 * constraint counts (n < 100, typical: 1-10 per object).
 *
 * Algorithm:
 * 1. Assemble constraint Jacobian J by stacking all constraint Jacobians
 * 2. Compute mass matrix inverse M^-1 (block diagonal: [m^-1·I₃, I^-1])
 * 3. Form constraint matrix: A = J·M^-1·J^T
 * 4. Build RHS: b = -J·M^-1·F_ext - J̇·q̇ - α·C - β·Ċ
 * 5. Solve A·λ = b using Eigen LLT decomposition
 * 6. Extract forces: F_constraint = J^T·λ
 *
 * Performance: O(n³) where n = total constraint dimension
 *
 * Thread safety: Thread-safe (stateless solver, mutable matrices are local)
 * Error handling: Returns converged=false for singular matrices (no exceptions)
 *
 * @see
 * docs/designs/0031_generalized_lagrange_constraints/0031_generalized_lagrange_constraints.puml
 * @ticket 0031_generalized_lagrange_constraints
 */
class ConstraintSolver
{
public:
  /**
   * @brief Construct solver
   */
  ConstraintSolver() = default;

  ~ConstraintSolver() = default;

  // ===== Multi-Body Contact Constraint Solver (Ticket 0032, 0045) =====

  /**
   * @brief Per-body constraint forces from multi-body solve
   */
  struct BodyForces
  {
    Vector3D linearForce;    // Net linear constraint force [N]
    Vector3D angularTorque;  // Net angular constraint torque [N·m]

    BodyForces() = default;
    BodyForces(const Coordinate& lf, const Coordinate& at)
      : linearForce{lf}, angularTorque{at}
    {
    }
  };

  /**
   * @brief Result of multi-body constraint solve
   * @ticket 0045_constraint_solver_unification
   */
  struct SolveResult
  {
    std::vector<BodyForces> bodyForces;  // Per-body constraint forces
    Eigen::VectorXd lambdas;             // Lagrange multipliers
    bool converged{false};               // Solver convergence flag
    int iterations{0};                   // Active set changes performed
    double residual{std::numeric_limits<double>::quiet_NaN()};

    SolveResult() = default;
  };

  /**
   * @brief Solve contact constraint system using Active Set Method (ASM)
   *
   * Solves a set of two-body contact constraints with lambda >= 0 enforcement.
   * Each contact constraint couples two bodies via a 1×12 Jacobian.
   *
   * ASM algorithm: partitions contacts into active (compressive) and inactive
   * (separating) sets, solving an equality subproblem at each iteration via
   * LLT decomposition until all KKT conditions are satisfied.
   *
   * CRITICAL implementation notes from prototype debugging:
   * - Uses ERP formulation for Baumgarte stabilization (not alpha/beta)
   * - Restitution RHS: b = -(1+e) · J·v_minus (correct for system A·λ = b)
   * - Baumgarte RHS: b += (ERP/dt) · penetration_depth
   *
   * @param contactConstraints Two-body contact constraints (non-owning)
   * @param states Inertial states of all bodies
   * @param inverseMasses Per-body inverse masses [1/kg] (0 for static)
   * @param inverseInertias Per-body inverse inertia tensors
   * @param numBodies Total number of bodies
   * @param dt Timestep [s]
   * @param initialLambda Optional initial lambda guess for warm-starting.
   *        If non-empty, must have size == contactConstraints.size().
   *        Non-zero entries initialize the ASM active set for faster
   *        convergence on persistent contacts.
   * @return SolveResult with per-body forces
   *
   * @ticket 0032_contact_constraint_refactor
   * @ticket 0034_active_set_method_contact_solver
   * @ticket 0040d_contact_persistence_warm_starting
   * @ticket 0045_constraint_solver_unification
   */
  SolveResult solve(
    const std::vector<Constraint*>& contactConstraints,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    size_t numBodies,
    double dt,
    const std::optional<Eigen::VectorXd>& initialLambda = std::nullopt);

  /**
   * @brief Set maximum safety iteration cap for Active Set Method
   *
   * The effective iteration limit per solve is min(2*C, maxIter) where
   * C is the number of contacts. This safety cap prevents infinite loops
   * for degenerate inputs. For typical use, the default (100) should not
   * need adjustment.
   *
   * @param maxIter Maximum safety iterations (default: 100)
   * @ticket 0034_active_set_method_contact_solver
   */
  void setMaxIterations(int maxIter)
  {
    max_safety_iterations_ = maxIter;
  }

  /**
   * @brief Set constraint violation tolerance for Active Set Method
   *
   * Inactive constraints with violation magnitude below this tolerance
   * are considered satisfied. Tighter tolerance improves accuracy but
   * may cause extra iterations for nearly-active constraints.
   *
   * @param tol Violation tolerance (default: 1e-6)
   * @ticket 0034_active_set_method_contact_solver
   */
  void setConvergenceTolerance(double tol)
  {
    convergence_tolerance_ = tol;
  }

  // ===== ECOS Solver Configuration (Ticket 0035b4) =====

  /**
   * @brief Set ECOS solver tolerance for convergence
   *
   * ECOS uses separate absolute and relative tolerances. Default: 1e-6 for
   * both.
   *
   * @param abs_tol Absolute tolerance (primal/dual residuals)
   * @param rel_tol Relative tolerance (gap)
   * @ticket 0035b4_ecos_solve_integration
   */
  void setECOSTolerance(double abs_tol, double rel_tol)
  {
    ecos_abs_tol_ = abs_tol;
    ecos_rel_tol_ = rel_tol;
  }

  /**
   * @brief Set maximum ECOS iterations
   *
   * Safety cap to prevent unbounded solve time. Default: 100.
   * Typical convergence: 5-15 iterations.
   *
   * @param max_iters Maximum iterations
   * @ticket 0035b4_ecos_solve_integration
   */
  void setECOSMaxIterations(int max_iters)
  {
    ecos_max_iters_ = max_iters;
  }

  /**
   * @brief Get ECOS tolerance settings
   * @return Pair of (absolute tolerance, relative tolerance)
   */
  [[nodiscard]] std::pair<double, double> getECOSTolerance() const
  {
    return {ecos_abs_tol_, ecos_rel_tol_};
  }

  /**
   * @brief Get maximum ECOS iterations
   * @return Maximum iterations
   */
  [[nodiscard]] int getECOSMaxIterations() const
  {
    return ecos_max_iters_;
  }

  // ===== Low-Level Solver Results (Ticket 0034, 0035b4) =====

  /**
   * @brief Result of constraint solve (ASM or ECOS)
   *
   * Contains the solution lambda vector, convergence status, iteration count,
   * and solver-specific diagnostics. Used by both Active Set Method and ECOS
   * SOCP solver paths.
   *
   * @ticket 0034_active_set_method_contact_solver
   * @ticket 0035b4_ecos_solve_integration
   */
  struct ActiveSetResult
  {
    Eigen::VectorXd lambda;  // Lagrange multipliers (all >= 0)
    bool converged{false};   // True if all KKT conditions satisfied
    int iterations{0};       // ASM iterations OR ECOS iterations
    int active_set_size{0};  // ASM: active set size, ECOS: unused (set to 0)

    // ECOS-specific diagnostic fields (Ticket 0035b4)
    std::string solver_type{"ASM"};  // "ASM" or "ECOS"
    int ecos_exit_flag{0};  // ECOS exit code (ECOS_OPTIMAL, ECOS_MAXIT, etc.)
    double primal_residual{
      std::numeric_limits<double>::quiet_NaN()};  // ECOS primal feasibility
    double dual_residual{
      std::numeric_limits<double>::quiet_NaN()};  // ECOS dual feasibility
    double gap{std::numeric_limits<double>::quiet_NaN()};  // ECOS duality gap
  };

  /**
   * @brief Solve friction LCP using ECOS SOCP solver
   *
   * Formulates the contact constraint system with friction as a second-order
   * cone program (SOCP) and solves using the ECOS interior-point method.
   * Each contact produces a 3D friction cone constraint:
   *   ||[λ_t1, λ_t2]|| <= μ * λ_n
   *
   * @param A Effective mass matrix (3C x 3C), symmetric positive semi-definite
   * @param b RHS vector (3C x 1) with restitution and Baumgarte terms
   * @param coneSpec Friction cone specification (μ per contact, normal indices)
   * @param numContacts Number of contacts (C)
   * @return ActiveSetResult with lambda, convergence info, ECOS diagnostics
   *
   * @ticket 0035b4_ecos_solve_integration
   */
  [[nodiscard]] ActiveSetResult solveWithECOS(const Eigen::MatrixXd& A,
                                              const Eigen::VectorXd& b,
                                              const FrictionConeSpec& coneSpec,
                                              int numContacts) const;

  // Rule of Five
  ConstraintSolver(const ConstraintSolver&) = default;
  ConstraintSolver& operator=(const ConstraintSolver&) = default;
  ConstraintSolver(ConstraintSolver&&) noexcept = default;
  ConstraintSolver& operator=(ConstraintSolver&&) noexcept = default;

private:
  // ===== Contact solver helpers (Ticket 0032, 0034, 0045) =====

  /**
   * @brief Compute per-contact 1×12 Jacobians
   *
   * Each contact produces a Jacobian [J_A(1×6) | J_B(1×6)] in the
   * velocity-level formulation [v_A, ω_A, v_B, ω_B].
   *
   * @return Vector of per-contact Jacobian matrices (C entries, each 1×12)
   * @ticket 0045_constraint_solver_unification
   */
  [[nodiscard]] static std::vector<Eigen::MatrixXd> assembleJacobians(
    const std::vector<Constraint*>& contactConstraints,
    const std::vector<std::reference_wrapper<const InertialState>>& states);

  /**
   * @brief Build effective mass matrix A = J·M⁻¹·Jᵀ with regularization
   *
   * Constructs per-body 6×6 inverse mass matrices internally and assembles
   * the symmetric effective mass matrix. Adds kRegularizationEpsilon to
   * diagonal.
   *
   * @return Effective mass matrix (C × C)
   * @ticket 0045_constraint_solver_unification
   */
  [[nodiscard]] static Eigen::MatrixXd assembleEffectiveMass(
    const std::vector<Constraint*>& contactConstraints,
    const std::vector<Eigen::MatrixXd>& jacobians,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    size_t numBodies);

  /**
   * @brief Assemble RHS vector with restitution and Baumgarte terms
   *
   * b_i = -(1 + e_i) · (J_i · v⁻) + (ERP_i / dt) · penetration_i
   *
   * @return RHS vector (C × 1)
   * @ticket 0045_constraint_solver_unification
   */
  [[nodiscard]] static Eigen::VectorXd assembleRHS(
    const std::vector<Constraint*>& contactConstraints,
    const std::vector<Eigen::MatrixXd>& jacobians,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    double dt);

  /**
   * @brief Solve contact LCP using Active Set Method
   *
   * Partitions contacts into active (compressive) and inactive (separating)
   * sets, solving an equality subproblem at each iteration via LLT
   * decomposition until all KKT conditions are satisfied.
   *
   * Algorithm:
   * 1. Initialize working set W = {0, 1, ..., C-1} (all contacts active)
   * 2. Solve equality subproblem: A_W * lambda_W = b_W via LLT
   * 3. If any lambda_W[i] < 0: remove most negative from W (Bland's rule for
   * ties)
   * 4. If all lambda_W >= 0: check inactive constraints for violation
   * 5. If violated inactive constraint found: add most violated to W (Bland's
   * rule)
   * 6. If no violations: KKT conditions satisfied, return exact solution
   *
   * Convergence: Finite, typically <= C iterations for non-degenerate systems.
   * Safety cap: min(2*C, max_safety_iterations_) to prevent cycling.
   *
   * @param A Effective mass matrix (C x C), symmetric positive semi-definite
   * @param b RHS vector (C x 1) with restitution and Baumgarte terms
   * @param numContacts Number of contacts (used for safety iteration cap =
   * 2*C). Note: numContacts == b.size() by construction (one constraint row per
   * contact). Passed explicitly rather than derived from b.size() to document
   * intent and decouple the safety cap computation from the Eigen vector
   * internals.
   * @return ActiveSetResult with lambda vector, convergence info, and active
   * set size
   *
   * @ticket 0034_active_set_method_contact_solver
   * @ticket 0040d_contact_persistence_warm_starting
   */
  [[nodiscard]] ActiveSetResult solveActiveSet(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    int numContacts,
    const std::optional<Eigen::VectorXd>& initialLambda = std::nullopt) const;

  /**
   * @brief Extract per-body forces from solved lambda values
   *
   * Computes F_body_k = Σ J_i_k^T · λ_i / dt for each body k touched by
   * contacts.
   *
   * @return Per-body constraint forces (numBodies entries)
   * @ticket 0045_constraint_solver_unification
   */
  [[nodiscard]] static std::vector<BodyForces> extractBodyForces(
    const std::vector<Constraint*>& contactConstraints,
    const std::vector<Eigen::MatrixXd>& jacobians,
    const Eigen::VectorXd& lambda,
    size_t numBodies,
    double dt);

  // ===== ECOS solver helpers (Ticket 0035b4) =====

  /**
   * @brief Extract friction cone specification from contact constraints
   *
   * Scans contact constraint list for FrictionConstraint instances, extracts
   * friction coefficient μ and normal constraint index per contact. Builds
   * FrictionConeSpec for ECOS.
   *
   * The constraint ordering convention is: for each contact i, the normal
   * constraint is at index 3i, tangent1 at 3i+1, tangent2 at 3i+2.
   *
   * @param contactConstraints All contact constraints (normal + friction
   * interleaved)
   * @param numContacts Number of contacts (C, where constraints has 3C rows
   * total)
   * @return FrictionConeSpec with μ values and normal indices
   *
   * @ticket 0035b4_ecos_solve_integration
   */
  [[nodiscard]] static FrictionConeSpec buildFrictionConeSpec(
    const std::vector<Constraint*>& contactConstraints,
    int numContacts);

  // Active Set Method configuration (Ticket 0034)
  int max_safety_iterations_{
    100};  // Safety cap; effective limit = min(2*C, max_safety_iterations_)
  double convergence_tolerance_{
    1e-6};  // Inactive constraint violation threshold
  static constexpr double kRegularizationEpsilon = 1e-8;

  // ECOS solver configuration (Ticket 0035b4)
  double ecos_abs_tol_{1e-6};  // Absolute tolerance (primal/dual residuals)
  double ecos_rel_tol_{1e-6};  // Relative tolerance (gap)
  int ecos_max_iters_{100};    // Maximum iterations (safety cap)
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINT_SOLVER_HPP

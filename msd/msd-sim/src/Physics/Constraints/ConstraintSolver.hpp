// Ticket: 0031_generalized_lagrange_constraints
// Ticket: 0032_contact_constraint_refactor
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#ifndef MSD_SIM_PHYSICS_CONSTRAINT_SOLVER_HPP
#define MSD_SIM_PHYSICS_CONSTRAINT_SOLVER_HPP

#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"
#include <Eigen/Dense>
#include <functional>
#include <vector>
#include <limits>

namespace msd_sim
{

// Forward declarations
struct InertialState;
class TwoBodyConstraint;

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
 * @see docs/designs/0031_generalized_lagrange_constraints/0031_generalized_lagrange_constraints.puml
 * @ticket 0031_generalized_lagrange_constraints
 */
class ConstraintSolver
{
public:
  /**
   * @brief Result of constraint solve containing Lagrange multipliers and forces
   *
   * Provides diagnostic information (condition number, convergence) alongside
   * forces for solver health monitoring.
   */
  struct SolveResult
  {
    Eigen::VectorXd lambdas;               // Lagrange multipliers
    Coordinate linearConstraintForce;      // Net linear force [N]
    Coordinate angularConstraintForce;     // Net angular torque [N·m]
    bool converged{false};                 // Solver convergence flag
    double conditionNumber{std::numeric_limits<double>::quiet_NaN()};  // Matrix conditioning

    SolveResult() = default;

    SolveResult(const Eigen::VectorXd& l,
                const Coordinate& linForce,
                const Coordinate& angForce,
                bool conv,
                double cond)
      : lambdas{l},
        linearConstraintForce{linForce},
        angularConstraintForce{angForce},
        converged{conv},
        conditionNumber{cond}
    {}
  };

  /**
   * @brief Construct solver
   */
  ConstraintSolver() = default;

  ~ConstraintSolver() = default;

  /**
   * @brief Solve constraint system for Lagrange multipliers
   *
   * Computes constraint forces that satisfy all constraints simultaneously.
   * Returns converged=false for singular/ill-conditioned matrices.
   *
   * @param constraints Vector of constraint pointers (non-owning)
   * @param state Current inertial state (position, orientation, velocities)
   * @param externalForce Net external force in world frame [N]
   * @param externalTorque Net external torque in world frame [N·m]
   * @param mass Object mass [kg]
   * @param inverseInertia Inverse inertia tensor in world frame [1/(kg·m²)]
   * @param dt Timestep [s]
   * @return SolveResult with forces and convergence status
   */
  SolveResult solve(const std::vector<Constraint*>& constraints,
                    const InertialState& state,
                    const Coordinate& externalForce,
                    const Coordinate& externalTorque,
                    double mass,
                    const Eigen::Matrix3d& inverseInertia,
                    double dt);

  // ===== Multi-Body Contact Constraint Solver (Ticket 0032) =====

  /**
   * @brief Per-body constraint forces from multi-body solve
   */
  struct BodyForces
  {
    Coordinate linearForce;     // Net linear constraint force [N]
    Coordinate angularTorque;   // Net angular constraint torque [N·m]

    BodyForces() = default;
    BodyForces(const Coordinate& lf, const Coordinate& at)
      : linearForce{lf}, angularTorque{at}
    {}
  };

  /**
   * @brief Result of multi-body contact constraint solve
   */
  struct MultiBodySolveResult
  {
    std::vector<BodyForces> bodyForces;  // Per-body constraint forces
    Eigen::VectorXd lambdas;             // Lagrange multipliers
    bool converged{false};               // Solver convergence flag
    int iterations{0};                   // Active set changes performed
    double residual{std::numeric_limits<double>::quiet_NaN()};

    MultiBodySolveResult() = default;
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
   * @return MultiBodySolveResult with per-body forces
   *
   * @ticket 0032_contact_constraint_refactor
   * @ticket 0034_active_set_method_contact_solver
   */
  MultiBodySolveResult solveWithContacts(
      const std::vector<TwoBodyConstraint*>& contactConstraints,
      const std::vector<std::reference_wrapper<const InertialState>>& states,
      const std::vector<double>& inverseMasses,
      const std::vector<Eigen::Matrix3d>& inverseInertias,
      size_t numBodies,
      double dt);

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
  void setMaxIterations(int maxIter) { max_safety_iterations_ = maxIter; }

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
  void setConvergenceTolerance(double tol) { convergence_tolerance_ = tol; }

  // Rule of Five
  ConstraintSolver(const ConstraintSolver&) = default;
  ConstraintSolver& operator=(const ConstraintSolver&) = default;
  ConstraintSolver(ConstraintSolver&&) noexcept = default;
  ConstraintSolver& operator=(ConstraintSolver&&) noexcept = default;

private:
  /**
   * @brief Assemble constraint matrix A = J·M^-1·J^T
   *
   * Stacks all constraint Jacobians and computes the symmetric positive
   * definite constraint matrix used in the linear solve.
   *
   * @return Constraint matrix (n × n) where n = total constraint dimension
   */
  Eigen::MatrixXd assembleConstraintMatrix(
      const std::vector<Constraint*>& constraints,
      const InertialState& state,
      double time,
      double mass,
      const Eigen::Matrix3d& inverseInertia) const;

  /**
   * @brief Assemble RHS vector b = -J·M^-1·F_ext - α·C - β·Ċ
   *
   * Builds the right-hand side of the linear system incorporating external
   * forces, constraint violations, and Baumgarte stabilization.
   *
   * @return RHS vector (n × 1) where n = total constraint dimension
   */
  Eigen::VectorXd assembleRHS(
      const std::vector<Constraint*>& constraints,
      const InertialState& state,
      const Coordinate& externalForce,
      const Coordinate& externalTorque,
      double mass,
      const Eigen::Matrix3d& inverseInertia,
      double time,
      double dt) const;

  /**
   * @brief Extract constraint forces from Lagrange multipliers
   *
   * Computes F_constraint = J^T·λ and separates into linear and angular
   * components based on Jacobian structure.
   *
   * @return Pair of (linear force, angular torque)
   */
  std::pair<Coordinate, Coordinate> extractConstraintForces(
      const Eigen::VectorXd& lambdas,
      const std::vector<Constraint*>& constraints,
      const InertialState& state,
      double time) const;

  // ===== Contact solver helpers (Ticket 0032, 0034) =====

  /**
   * @brief Result of Active Set Method solve
   *
   * Contains the solution lambda vector, convergence status, iteration count
   * (number of active set changes), and the size of the final active set.
   *
   * @ticket 0034_active_set_method_contact_solver
   */
  struct ActiveSetResult
  {
    Eigen::VectorXd lambda;         // Lagrange multipliers (all >= 0)
    bool converged{false};          // True if all KKT conditions satisfied
    int iterations{0};              // Number of active set changes performed
    int active_set_size{0};         // Number of contacts in final active set (0 = empty active set)
  };

  /**
   * @brief Compute per-contact 1×12 Jacobians
   *
   * Each contact produces a Jacobian [J_A(1×6) | J_B(1×6)] in the velocity-level
   * formulation [v_A, ω_A, v_B, ω_B].
   *
   * @return Vector of per-contact Jacobian matrices (C entries, each 1×12)
   */
  std::vector<Eigen::MatrixXd> assembleContactJacobians(
      const std::vector<TwoBodyConstraint*>& contactConstraints,
      const std::vector<std::reference_wrapper<const InertialState>>& states) const;

  /**
   * @brief Build effective mass matrix A = J·M⁻¹·Jᵀ with regularization
   *
   * Constructs per-body 6×6 inverse mass matrices internally and assembles
   * the symmetric effective mass matrix. Adds kRegularizationEpsilon to diagonal.
   *
   * @return Effective mass matrix (C × C)
   */
  Eigen::MatrixXd assembleContactEffectiveMass(
      const std::vector<TwoBodyConstraint*>& contactConstraints,
      const std::vector<Eigen::MatrixXd>& jacobians,
      const std::vector<double>& inverseMasses,
      const std::vector<Eigen::Matrix3d>& inverseInertias,
      size_t numBodies) const;

  /**
   * @brief Assemble RHS vector with restitution and Baumgarte terms
   *
   * b_i = -(1 + e_i) · (J_i · v⁻) + (ERP_i / dt) · penetration_i
   *
   * @return RHS vector (C × 1)
   */
  Eigen::VectorXd assembleContactRHS(
      const std::vector<TwoBodyConstraint*>& contactConstraints,
      const std::vector<Eigen::MatrixXd>& jacobians,
      const std::vector<std::reference_wrapper<const InertialState>>& states,
      double dt) const;

  /**
   * @brief Solve contact LCP using Active Set Method
   *
   * Partitions contacts into active (compressive) and inactive (separating)
   * sets, solving an equality subproblem at each iteration via LLT decomposition
   * until all KKT conditions are satisfied.
   *
   * Algorithm:
   * 1. Initialize working set W = {0, 1, ..., C-1} (all contacts active)
   * 2. Solve equality subproblem: A_W * lambda_W = b_W via LLT
   * 3. If any lambda_W[i] < 0: remove most negative from W (Bland's rule for ties)
   * 4. If all lambda_W >= 0: check inactive constraints for violation
   * 5. If violated inactive constraint found: add most violated to W (Bland's rule)
   * 6. If no violations: KKT conditions satisfied, return exact solution
   *
   * Convergence: Finite, typically <= C iterations for non-degenerate systems.
   * Safety cap: min(2*C, max_safety_iterations_) to prevent cycling.
   *
   * @param A Effective mass matrix (C x C), symmetric positive semi-definite
   * @param b RHS vector (C x 1) with restitution and Baumgarte terms
   * @param numContacts Number of contacts (used for safety iteration cap = 2*C).
   *        Note: numContacts == b.size() by construction (one constraint row per contact).
   *        Passed explicitly rather than derived from b.size() to document intent and
   *        decouple the safety cap computation from the Eigen vector internals.
   * @return ActiveSetResult with lambda vector, convergence info, and active set size
   *
   * @ticket 0034_active_set_method_contact_solver
   */
  ActiveSetResult solveActiveSet(const Eigen::MatrixXd& A,
                                 const Eigen::VectorXd& b,
                                 int numContacts) const;

  /**
   * @brief Extract per-body forces from solved lambda values
   *
   * Computes F_body_k = Σ J_i_k^T · λ_i / dt for each body k touched by contacts.
   *
   * @return Per-body constraint forces (numBodies entries)
   */
  std::vector<BodyForces> extractContactBodyForces(
      const std::vector<TwoBodyConstraint*>& contactConstraints,
      const std::vector<Eigen::MatrixXd>& jacobians,
      const Eigen::VectorXd& lambda,
      size_t numBodies,
      double dt) const;

  // Active Set Method configuration (Ticket 0034)
  int max_safety_iterations_{100};     // Safety cap; effective limit = min(2*C, max_safety_iterations_)
  double convergence_tolerance_{1e-6}; // Inactive constraint violation threshold
  static constexpr double kRegularizationEpsilon = 1e-8;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINT_SOLVER_HPP

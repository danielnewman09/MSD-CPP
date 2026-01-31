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
    int iterations{0};                   // PGS iterations used
    double residual{std::numeric_limits<double>::quiet_NaN()};

    MultiBodySolveResult() = default;
  };

  /**
   * @brief Solve contact constraint system using Projected Gauss-Seidel (PGS)
   *
   * Solves a set of two-body contact constraints with lambda >= 0 clamping.
   * Each contact constraint couples two bodies via a 1×12 Jacobian.
   *
   * PGS algorithm: for each iteration, update each lambda_i using
   * the latest values of other lambdas, then clamp to non-negative.
   *
   * CRITICAL implementation notes from prototype debugging:
   * - Uses ERP formulation for Baumgarte stabilization (not alpha/beta)
   * - Restitution RHS: b = -(1+e) · J·v_minus (correct for PGS solve)
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
   */
  MultiBodySolveResult solveWithContacts(
      const std::vector<TwoBodyConstraint*>& contactConstraints,
      const std::vector<std::reference_wrapper<const InertialState>>& states,
      const std::vector<double>& inverseMasses,
      const std::vector<Eigen::Matrix3d>& inverseInertias,
      size_t numBodies,
      double dt);

  /**
   * @brief Set maximum PGS iterations
   * @param maxIter Maximum iterations (default: 10)
   * @ticket 0032_contact_constraint_refactor
   */
  void setMaxIterations(int maxIter) { max_iterations_ = maxIter; }

  /**
   * @brief Set PGS convergence tolerance
   * @param tol Convergence tolerance (default: 1e-4)
   * @ticket 0032_contact_constraint_refactor
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

  // ===== Contact solver helpers (Ticket 0032) =====

  /**
   * @brief Result of Projected Gauss-Seidel iteration
   */
  struct PGSResult
  {
    Eigen::VectorXd lambda;
    bool converged{false};
    int iterations{0};
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
   * @brief Perform Projected Gauss-Seidel iteration with λ ≥ 0 clamping
   *
   * @return PGSResult with lambda vector and convergence info
   */
  PGSResult solvePGS(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) const;

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

  // PGS configuration (Ticket 0032)
  int max_iterations_{10};
  double convergence_tolerance_{1e-4};
  static constexpr double kRegularizationEpsilon = 1e-8;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINT_SOLVER_HPP

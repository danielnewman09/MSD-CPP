// Ticket: 0031_generalized_lagrange_constraints
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md

#ifndef MSD_SIM_PHYSICS_CONSTRAINT_SOLVER_HPP
#define MSD_SIM_PHYSICS_CONSTRAINT_SOLVER_HPP

#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"
#include <Eigen/Dense>
#include <vector>
#include <limits>

namespace msd_sim
{

// Forward declaration
struct InertialState;

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
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINT_SOLVER_HPP

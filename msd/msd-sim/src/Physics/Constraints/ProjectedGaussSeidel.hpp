// Ticket: 0073_hybrid_pgs_large_islands
// Design: docs/designs/0073_hybrid_pgs_large_islands/design.md

#ifndef MSD_SIM_PHYSICS_CONSTRAINTS_PROJECTED_GAUSS_SEIDEL_HPP
#define MSD_SIM_PHYSICS_CONSTRAINTS_PROJECTED_GAUSS_SEIDEL_HPP

#include <Eigen/Dense>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

// Forward declaration — avoids circular include with ConstraintSolver.hpp.
// ConstraintSolver::SolveResult and ConstraintSolver::FlattenedConstraints
// are used only in the .cpp implementation.
class ConstraintSolver;

/**
 * @brief O(n)-per-sweep iterative LCP solver using Projected Gauss-Seidel.
 *
 * Used by ConstraintSolver as the solver backend for large constraint islands
 * (numRows > kASMThreshold). PGS has O(n) cost per sweep and scales linearly
 * with problem size, making it ~40x cheaper per sweep than a single ASM pivot
 * at 160 constraints.
 *
 * Algorithm (Catto 2005):
 * For each sweep, for each constraint row i:
 *   1. delta_i = (b_i - J_i * v_res) / A_ii  using velocity-residual approach
 *   2. For normal rows: lambda_i = max(0, lambda_i + delta_i)
 *   3. For friction rows: solve both tangent components, then ball-project the
 *      2D tangent vector onto the friction disk ||lambda_t|| <= mu * lambda_n
 *
 * Velocity-residual approach (O(n) workspace):
 * - v_res (6 * numBodies doubles) accumulates M^{-1} * J^T * lambda
 * - After each lambda update: v_res += M^{-1} * J_i^T * delta_lambda_i
 * - Off-diagonal contribution for row i: J_i * v_res
 *
 * Friction coupling (R1 from design review):
 * PGS applies ball-projection directly on friction rows, bypassing
 * FrictionConstraint::lambdaBounds(). The ball-projection cone
 * (||lambda_t|| <= mu*lambda_n) is consistent with the decoupled solver from
 * ticket 0070. setNormalLambda() is NOT called during PGS sweeps.
 *
 * @see docs/designs/0073_hybrid_pgs_large_islands/design.md
 * @ticket 0073_hybrid_pgs_large_islands
 */
class ProjectedGaussSeidel
{
public:
  /// Per-body forces result (mirrors ConstraintSolver::BodyForces)
  struct BodyForces
  {
    Coordinate linearForce;
    Coordinate angularTorque;

    BodyForces() = default;
    BodyForces(const Coordinate& lf, const Coordinate& at)
      : linearForce{lf}, angularTorque{at}
    {
    }
  };

  /// Solve result (mirrors ConstraintSolver::SolveResult fields)
  struct SolveResult
  {
    std::vector<BodyForces> bodyForces;
    Eigen::VectorXd lambdas;
    bool converged{false};
    int iterations{0};
    double residual{std::numeric_limits<double>::quiet_NaN()};

    SolveResult() = default;
  };

  ProjectedGaussSeidel() = default;
  ~ProjectedGaussSeidel() = default;

  ProjectedGaussSeidel(const ProjectedGaussSeidel&) = default;
  ProjectedGaussSeidel& operator=(const ProjectedGaussSeidel&) = default;
  ProjectedGaussSeidel(ProjectedGaussSeidel&&) noexcept = default;
  ProjectedGaussSeidel& operator=(ProjectedGaussSeidel&&) noexcept = default;

  /**
   * @brief Solve mixed LCP using Projected Gauss-Seidel.
   *
   * Accepts an interleaved [CC, FC, CC, FC, ...] constraint list.
   * Uses ConstraintSolver::flattenConstraints() to expand multi-row constraints
   * into per-row entries, then performs PGS sweeps.
   *
   * Friction rows use ball-projection (not lambdaBounds() box bounds).
   * setNormalLambda() is NOT called during PGS sweeps (bypassed by design).
   *
   * @param constraints Interleaved ContactConstraint and FrictionConstraint ptrs
   * @param states Per-body kinematic state
   * @param inverseMasses Per-body inverse mass (size = numBodies)
   * @param inverseInertias Per-body inverse inertia tensor (size = numBodies)
   * @param numBodies Number of bodies in the system
   * @param dt Timestep [s]
   * @param initialLambda Warm-start vector (same size as active rows); zeros if
   *                      empty or wrong size.
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
  void setMaxSweeps(int n)
  {
    maxSweeps_ = n;
  }

  /// Set convergence tolerance: early exit when max |delta_lambda| < tol
  /// (default: 1e-6)
  void setConvergenceTolerance(double tol)
  {
    convergenceTolerance_ = tol;
  }

  [[nodiscard]] int getMaxSweeps() const
  {
    return maxSweeps_;
  }

  [[nodiscard]] double getConvergenceTolerance() const
  {
    return convergenceTolerance_;
  }

private:
  // Regularization epsilon added to diagonal (matches ConstraintSolver pattern)
  static constexpr double kRegularizationEpsilon = 1e-8;

  // Internal flat row types (mirrors ConstraintSolver::RowType)
  enum class RowType
  {
    Normal,
    Tangent
  };

  // Flattened per-row data for PGS sweep
  struct FlatRow
  {
    Eigen::Matrix<double, 1, 12> jacobianRow;
    size_t bodyAIndex{0};
    size_t bodyBIndex{0};
    RowType rowType{RowType::Normal};
    double restitution{0.0};
  };

  /**
   * @brief Execute one Gauss-Seidel sweep using velocity-residual approach.
   *
   * Processes rows in flat order. When a Normal row is followed by two Tangent
   * rows (standard friction pair), they are solved together with ball-projection.
   *
   * @param rows Flattened per-row data
   * @param muPerContact Friction coefficient per contact (indexed by contact)
   * @param diag Diagonal effective-mass elements A_ii (size = numRows)
   * @param b RHS vector (size = numRows)
   * @param lambda In/out lambda vector updated in-place
   * @param vRes In/out velocity residual (size = 6 * numBodies), updated in-place
   * @param inverseMasses Per-body inverse mass
   * @param inverseInertias Per-body inverse inertia
   * @return max |delta_lambda| this sweep (for convergence check)
   */
  static double sweepOnce(
    const std::vector<FlatRow>& rows,
    const std::vector<double>& muPerContact,
    const Eigen::VectorXd& diag,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& lambda,
    Eigen::VectorXd& vRes,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias);

  /**
   * @brief Update velocity residual after a lambda change on row rowIdx.
   * v_res += M^{-1} * J_i^T * dLambda
   */
  static void updateVRes(
    const std::vector<FlatRow>& rows,
    size_t rowIdx,
    double dLambda,
    Eigen::VectorXd& vRes,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias);

  int maxSweeps_{50};
  double convergenceTolerance_{1e-6};
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINTS_PROJECTED_GAUSS_SEIDEL_HPP

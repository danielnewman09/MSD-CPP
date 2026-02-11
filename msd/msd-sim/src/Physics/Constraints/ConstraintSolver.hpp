// Ticket: 0031_generalized_lagrange_constraints
// Ticket: 0032_contact_constraint_refactor
// Ticket: 0052d_solver_integration_ecos_removal
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md
// Design: docs/designs/0032_contact_constraint_refactor/design.md
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

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
#include "msd-sim/src/Physics/Constraints/FrictionConeSolver.hpp"
#include "msd-sim/src/Physics/Constraints/FrictionSpec.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/**
 * @brief Computes Lagrange multipliers for arbitrary constraint systems
 *
 * Dispatches between Active Set Method (no friction) and FrictionConeSolver
 * (with friction) based on constraint types present. Uses constraint
 * flattening to expand multi-row constraints (FrictionConstraint dim=2) into
 * per-row entries for the 3C x 3C friction system.
 *
 * @ticket 0031_generalized_lagrange_constraints
 * @ticket 0052d_solver_integration_ecos_removal
 */
class ConstraintSolver
{
public:
  ConstraintSolver() = default;
  ~ConstraintSolver() = default;

  // ===== Multi-Body Contact Constraint Solver =====

  struct BodyForces
  {
    Vector3D linearForce;
    Vector3D angularTorque;

    BodyForces() = default;
    BodyForces(const Coordinate& lf, const Coordinate& at)
      : linearForce{lf}, angularTorque{at}
    {
    }
  };

  struct SolveResult
  {
    std::vector<BodyForces> bodyForces;
    Eigen::VectorXd lambdas;
    bool converged{false};
    int iterations{0};
    double residual{std::numeric_limits<double>::quiet_NaN()};

    SolveResult() = default;
  };

  /**
   * @brief Solve contact constraint system
   *
   * Detects friction constraints and dispatches to either ASM (no friction)
   * or FrictionConeSolver (with friction). For the friction path, constraints
   * are flattened from [CC, FC, CC, FC, ...] to [n, t1, t2, n, t1, t2, ...]
   * rows before assembly.
   */
  SolveResult solve(
    const std::vector<Constraint*>& contactConstraints,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    size_t numBodies,
    double dt,
    const std::optional<Eigen::VectorXd>& initialLambda = std::nullopt);

  void setMaxIterations(int maxIter)
  {
    max_safety_iterations_ = maxIter;
  }

  void setConvergenceTolerance(double tol)
  {
    convergence_tolerance_ = tol;
  }

  // ===== Low-Level Solver Results =====

  struct ActiveSetResult
  {
    Eigen::VectorXd lambda;
    bool converged{false};
    int iterations{0};
    int active_set_size{0};
  };

  // ===== Constraint Flattening (Ticket 0052d) =====

  /// Row type for flattened constraints
  enum class RowType
  {
    Normal,
    Tangent
  };

  /// Flattened constraint representation: one Jacobian row per entry
  struct FlattenedConstraints
  {
    std::vector<Eigen::Matrix<double, 1, 12>> jacobianRows;
    std::vector<size_t> bodyAIndices;
    std::vector<size_t> bodyBIndices;
    std::vector<RowType> rowTypes;
    std::vector<double> restitutions;
    int numContacts{0};
  };

  /**
   * @brief Flatten interleaved [CC, FC, CC, FC, ...] into per-row entries
   *
   * Expands each FrictionConstraint (dim=2) into two separate 1x12 rows.
   * ContactConstraint (dim=1) produces one row. Output ordering is
   * [n_0, t1_0, t2_0, n_1, t1_1, t2_1, ...] for the friction cone solver.
   *
   * @ticket 0052d_solver_integration_ecos_removal
   */
  [[nodiscard]] static FlattenedConstraints flattenConstraints(
    const std::vector<Constraint*>& contactConstraints,
    const std::vector<std::reference_wrapper<const InertialState>>& states);

  /**
   * @brief Build FrictionSpec from flattened constraint metadata
   * @ticket 0052d_solver_integration_ecos_removal
   */
  [[nodiscard]] static FrictionSpec buildFrictionSpec(
    const std::vector<Constraint*>& contactConstraints);

  // Rule of Five
  ConstraintSolver(const ConstraintSolver&) = default;
  ConstraintSolver& operator=(const ConstraintSolver&) = default;
  ConstraintSolver(ConstraintSolver&&) noexcept = default;
  ConstraintSolver& operator=(ConstraintSolver&&) noexcept = default;

private:
  // ===== Contact solver helpers =====

  [[nodiscard]] static std::vector<Eigen::MatrixXd> assembleJacobians(
    const std::vector<Constraint*>& contactConstraints,
    const std::vector<std::reference_wrapper<const InertialState>>& states);

  [[nodiscard]] static Eigen::MatrixXd assembleEffectiveMass(
    const std::vector<Constraint*>& contactConstraints,
    const std::vector<Eigen::MatrixXd>& jacobians,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    size_t numBodies);

  [[nodiscard]] static Eigen::VectorXd assembleRHS(
    const std::vector<Constraint*>& contactConstraints,
    const std::vector<Eigen::MatrixXd>& jacobians,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    double dt);

  [[nodiscard]] ActiveSetResult solveActiveSet(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    int numContacts,
    const std::optional<Eigen::VectorXd>& initialLambda = std::nullopt);

  [[nodiscard]] static std::vector<BodyForces> extractBodyForces(
    const std::vector<Constraint*>& contactConstraints,
    const std::vector<Eigen::MatrixXd>& jacobians,
    const Eigen::VectorXd& lambda,
    size_t numBodies,
    double dt);

  // ===== Friction solver helpers (Ticket 0052d) =====

  /**
   * @brief Assemble effective mass matrix from flattened constraints
   */
  [[nodiscard]] static Eigen::MatrixXd assembleFlatEffectiveMass(
    const FlattenedConstraints& flat,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    size_t numBodies);

  /**
   * @brief Assemble RHS vector from flattened constraints
   */
  [[nodiscard]] static Eigen::VectorXd assembleFlatRHS(
    const FlattenedConstraints& flat,
    const std::vector<std::reference_wrapper<const InertialState>>& states);

  /**
   * @brief Solve with friction cone constraints via FrictionConeSolver
   */
  [[nodiscard]] ActiveSetResult solveWithFriction(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const FrictionSpec& spec,
    const std::optional<Eigen::VectorXd>& initialLambda);

  /**
   * @brief Extract per-body forces from flat lambda (no negative-lambda skip)
   *
   * Unlike extractBodyForces(), this does NOT skip negative lambda values
   * because friction forces can be negative (pushing in negative tangent
   * direction).
   *
   * @ticket 0052d_solver_integration_ecos_removal
   */
  [[nodiscard]] static std::vector<BodyForces> extractBodyForcesFlat(
    const FlattenedConstraints& flat,
    const Eigen::VectorXd& lambda,
    size_t numBodies,
    double dt);

  // Active Set Method configuration
  int max_safety_iterations_{100};
  double convergence_tolerance_{1e-6};
  static constexpr double kRegularizationEpsilon = 1e-8;

  // ASM workspace — reused across calls to avoid per-call heap allocations
  // Ticket: 0053f_wire_solver_workspace
  Eigen::MatrixXd asmAw_;
  Eigen::VectorXd asmBw_;
  Eigen::VectorXd asmW_;

  // Friction cone solver instance
  FrictionConeSolver frictionConeSolver_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINT_SOLVER_HPP

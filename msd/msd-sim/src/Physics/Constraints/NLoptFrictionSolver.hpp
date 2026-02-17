// Ticket: 0068b_nlopt_friction_solver_class
// Design: docs/designs/0068_nlopt_friction_cone_solver/design.md

#ifndef MSD_SIM_PHYSICS_CONSTRAINTS_NLOPT_FRICTION_SOLVER_HPP
#define MSD_SIM_PHYSICS_CONSTRAINTS_NLOPT_FRICTION_SOLVER_HPP

#include <Eigen/Dense>
#include <limits>
#include <vector>

namespace msd_sim
{

/**
 * @brief NLopt-based solver for the friction cone constrained QP
 *
 * Solves:
 *   minimize f(lambda) = (1/2) lambda^T A lambda - b^T lambda
 *   subject to mu_i^2 * lambda_n_i^2 - lambda_t1_i^2 - lambda_t2_i^2 >= 0  (cone constraint)
 *              lambda_n_i >= 0                                            (non-penetration)
 *
 * Uses NLopt's SLSQP (Sequential Quadratic Programming) by default for efficient handling
 * of quadratic objectives with nonlinear constraints. Supports warm-starting from previous
 * frame's solution via ContactCache.
 *
 * @see docs/designs/0068_nlopt_friction_cone_solver/0068_nlopt_friction_cone_solver.puml
 * @ticket 0068b_nlopt_friction_solver_class
 */
class NLoptFrictionSolver
{
public:
  /// Algorithm selection for NLopt
  enum class Algorithm
  {
    SLSQP,          ///< Sequential Quadratic Programming (gradient-based, default)
    COBYLA,         ///< Constrained Optimization BY Linear Approximation (derivative-free)
    MMA,            ///< Method of Moving Asymptotes (gradient-based)
    AUGLAG_SLSQP    ///< Augmented Lagrangian with SLSQP sub-solver
  };

  /// Result of the friction cone solve
  struct SolveResult
  {
    Eigen::VectorXd lambda;           ///< Optimal impulse vector (3C x 1)
    bool converged{false};            ///< True if NLopt returned success/ftol_reached
    int iterations{0};                ///< Iterations performed (if available)
    double residual{std::numeric_limits<double>::quiet_NaN()};        ///< KKT residual (not used by NLopt)
    double objective_value{std::numeric_limits<double>::quiet_NaN()}; ///< Final objective function value
    std::vector<double> constraint_violations;  ///< Per-contact cone constraint values (>= 0 satisfied)
  };

  /**
   * @brief Construct with default settings (SLSQP, 1e-6 tolerance, 100 max iterations)
   */
  NLoptFrictionSolver();

  /**
   * @brief Construct with specified algorithm
   * @param algo NLopt algorithm to use
   */
  explicit NLoptFrictionSolver(Algorithm algo);

  /// Default destructor
  ~NLoptFrictionSolver() = default;

  /**
   * @brief Solve the friction cone constrained QP
   *
   * @param A  Effective mass matrix (3C x 3C), symmetric positive definite
   * @param b  RHS vector (3C x 1) with restitution/velocity terms
   * @param mu Per-contact friction coefficients (C entries). Invalid values (< 0) clamped to 0.
   * @param lambda0  Warm-start vector (3C x 1). If empty or wrong size, cold start from zero.
   * @param normalUpperBounds  Per-contact upper bounds on normal impulses (C entries).
   *        If non-empty, constrains lambda_n_i <= normalUpperBounds[i]. Used to prevent
   *        the coupled QP from inflating normal impulses beyond non-penetration needs.
   * @param tangent1LowerBounds  Per-contact lower bounds on tangent1 impulses (C entries).
   *        If non-empty, constrains lambda_t1_i >= tangent1LowerBounds[i]. Used to prevent
   *        friction from reversing sliding direction (typically 0.0 for sliding mode, omitted
   *        for bilateral friction). Default: empty (bilateral).
   * @return SolveResult with optimal lambda and diagnostics
   * @ticket 0069_friction_velocity_reversal
   */
  [[nodiscard]] SolveResult solve(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const std::vector<double>& mu,
    const Eigen::VectorXd& lambda0 = Eigen::VectorXd{},
    const std::vector<double>& normalUpperBounds = {},
    const std::vector<double>& tangent1LowerBounds = {});

  /// Set convergence tolerance (default: 1e-6)
  void setTolerance(double tol) { tolerance_ = tol; }

  /// Set maximum iterations (default: 100)
  void setMaxIterations(int n) { max_iterations_ = n; }

  /// Set NLopt algorithm (default: SLSQP)
  void setAlgorithm(Algorithm algo) { algorithm_ = algo; }

  /// Get current tolerance
  [[nodiscard]] double getTolerance() const { return tolerance_; }

  /// Get current max iterations
  [[nodiscard]] int getMaxIterations() const { return max_iterations_; }

  /// Get current algorithm
  [[nodiscard]] Algorithm getAlgorithm() const { return algorithm_; }

  // Rule of Zero: compiler-generated special members
  NLoptFrictionSolver(const NLoptFrictionSolver&) = default;
  NLoptFrictionSolver& operator=(const NLoptFrictionSolver&) = default;
  NLoptFrictionSolver(NLoptFrictionSolver&&) noexcept = default;
  NLoptFrictionSolver& operator=(NLoptFrictionSolver&&) noexcept = default;

private:
  /**
   * @brief Objective function callback for NLopt: f(lambda) = (1/2) lambda^T A lambda - b^T lambda
   * @param lambda Current iterate (3C x 1)
   * @param grad Output gradient (3C x 1): grad = A*lambda - b
   * @param data Pointer to ObjectiveData struct
   * @return Objective function value
   */
  static double objective(const std::vector<double>& lambda,
                          std::vector<double>& grad,
                          void* data);

  /**
   * @brief Cone constraint callback for contact i: c_i(lambda) = mu^2*n^2 - t1^2 - t2^2
   *
   * Note: NLopt expects constraints as c(x) <= 0, so we negate to -c(x) <= 0
   * which is equivalent to c(x) >= 0.
   *
   * @param lambda Current iterate (3C x 1)
   * @param grad Output gradient (3 x 1): [2*mu^2*n, -2*t1, -2*t2]
   * @param data Pointer to ConstraintData struct
   * @return Constraint value (negated for NLopt convention)
   */
  static double coneConstraint(const std::vector<double>& lambda,
                               std::vector<double>& grad,
                               void* data);

  /// Data passed to objective function
  struct ObjectiveData
  {
    const Eigen::MatrixXd* A;
    const Eigen::VectorXd* b;
  };

  /// Data passed to constraint function
  struct ConstraintData
  {
    int contactIndex;  ///< Which contact (0..C-1)
    double mu;         ///< Friction coefficient for this contact
  };

  double tolerance_{1e-6};                ///< Convergence tolerance
  int max_iterations_{100};               ///< Maximum iterations
  Algorithm algorithm_{Algorithm::SLSQP}; ///< NLopt algorithm
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINTS_NLOPT_FRICTION_SOLVER_HPP

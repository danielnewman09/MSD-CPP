// Ticket: 0052c_newton_solver_core
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#pragma once

#include <Eigen/Dense>
#include <limits>
#include <vector>

namespace msd_sim
{

/// Projected Newton solver for the friction cone QP:
///   min (1/2) lambda^T A lambda - b^T lambda
///   s.t. ||lambda_t_i|| <= mu_i * lambda_n_i, lambda_n_i >= 0
///
/// Uses Cholesky factorization with regularization, Armijo line search with
/// cone projection, and reduced-space Newton direction for cone surface contacts.
///
/// @see docs/designs/0052_custom_friction_cone_solver/design.md
/// @ticket 0052c_newton_solver_core
class FrictionConeSolver
{
public:
    /// Result of the friction cone solve
    struct SolveResult
    {
        Eigen::VectorXd lambda;           // Optimal impulse vector (3C x 1)
        bool converged{false};            // True if projected gradient residual < tolerance
        int iterations{0};               // Newton iterations performed
        double residual{std::numeric_limits<double>::quiet_NaN()};  // Final projected gradient norm
    };

    FrictionConeSolver() = default;
    // Rule of Zero: all members are scalars, compiler-generated defaults are correct

    /// Solve the friction cone QP.
    ///
    /// @param A  Effective mass matrix (3C x 3C), symmetric positive semi-definite
    /// @param b  RHS vector (3C x 1) with restitution terms
    /// @param mu Per-contact friction coefficients (C entries)
    /// @param lambda0  Optional warm-start vector (3C x 1). If empty or wrong size,
    ///                 cold start from unconstrained optimum.
    /// @return SolveResult with optimal lambda and diagnostics
    [[nodiscard]] SolveResult solve(
        const Eigen::MatrixXd& A,
        const Eigen::VectorXd& b,
        const std::vector<double>& mu,
        const Eigen::VectorXd& lambda0 = Eigen::VectorXd{}) const;

    void setTolerance(double eps) { tolerance_ = eps; }
    void setMaxIterations(int n) { max_iterations_ = n; }

    [[nodiscard]] double getTolerance() const { return tolerance_; }
    [[nodiscard]] int getMaxIterations() const { return max_iterations_; }

private:
    double tolerance_{1e-8};
    int max_iterations_{50};
    double armijo_c1_{1e-4};
    double armijo_beta_{0.5};
    int max_line_search_{20};
    static constexpr double kRegularizationEpsilon = 1e-10;
};

}  // namespace msd_sim

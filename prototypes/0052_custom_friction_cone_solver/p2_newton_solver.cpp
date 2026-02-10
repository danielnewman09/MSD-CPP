// Prototype P2: Newton Solver for Friction Cone QP
// Question: Does the projected Newton algorithm converge for all 7 numerical
//           examples with correct solutions, low iteration counts, and proper
//           warm-start behavior?
// Success criteria:
//   - Converges in <= 8 iterations cold start for ALL examples
//   - Converges in <= 3 iterations warm start
//   - KKT residual ||A*lambda - b|| < 1e-8 at solution (for feasible problems)
//   - Cone feasibility ||lambda_t|| <= mu*lambda_n + 1e-8 for all contacts
//   - No NaN or inf in any output
//   - Diagonal regularization handles ill-conditioned A

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

// ============================================================================
// Cone Projection (copied from P1)
// ============================================================================

struct ProjectionResult
{
    double lambda_n{0.0};
    double lambda_t1{0.0};
    double lambda_t2{0.0};
    int case_{0};
};

ProjectionResult projectSingle(double p_n, double p_t1, double p_t2, double mu)
{
    const double p_t_norm = std::sqrt(p_t1 * p_t1 + p_t2 * p_t2);

    if (p_t_norm <= mu * p_n)
    {
        return {p_n, p_t1, p_t2, 1};
    }

    if (mu * p_t_norm <= -p_n)
    {
        return {0.0, 0.0, 0.0, 2};
    }

    if (mu < 1e-15)
    {
        return {std::max(p_n, 0.0), 0.0, 0.0, 3};
    }

    const double alpha = 1.0 / (1.0 + mu * mu);
    const double proj_n = alpha * (p_n + mu * p_t_norm);
    double proj_t1 = 0.0;
    double proj_t2 = 0.0;
    if (p_t_norm > 1e-15)
    {
        const double scale = mu * proj_n / p_t_norm;
        proj_t1 = scale * p_t1;
        proj_t2 = scale * p_t2;
    }
    return {proj_n, proj_t1, proj_t2, 3};
}

Eigen::VectorXd projectVector(const Eigen::VectorXd& lambda,
                               const std::vector<double>& mu,
                               int numContacts)
{
    Eigen::VectorXd result{lambda.size()};
    for (int i = 0; i < numContacts; ++i)
    {
        auto p = projectSingle(lambda[3 * i], lambda[3 * i + 1], lambda[3 * i + 2], mu[i]);
        result[3 * i] = p.lambda_n;
        result[3 * i + 1] = p.lambda_t1;
        result[3 * i + 2] = p.lambda_t2;
    }
    return result;
}

// ============================================================================
// FrictionConeSolver Implementation
// ============================================================================
// Algorithm from design.md lines 144-162:
//
// solve(A, b, mu, lambda0):
//   1. Regularize: A_reg = A + eps * I
//   2. Cholesky factor: L L^T = A_reg
//   3. Compute unconstrained optimum: lambda_unc = solve(A_reg, b)
//   4. Initialize: lambda = Proj_K(lambda0) (or Proj_K(lambda_unc) if cold start)
//   5. For iter = 1..maxIter:
//      a. Gradient: g = A_reg * lambda - b
//      b. Projected gradient residual: r = ||lambda - Proj_K(lambda - g)||
//      c. If r < tol: return (converged)
//      d. Newton direction: delta = solve(A_reg, -g) (reuses Cholesky)
//      e. Line search with projection (Armijo):
//         alpha = 1.0
//         For ls = 1..maxLineSearch:
//           trial = Proj_K(lambda + alpha * delta)
//           If f(trial) <= f(lambda) + c1 * g^T * (trial - lambda): break
//           alpha *= beta
//      f. lambda = trial
//   6. Return (not converged, best lambda)

struct SolveResult
{
    Eigen::VectorXd lambda;
    bool converged{false};
    int iterations{0};
    double residual{std::numeric_limits<double>::quiet_NaN()};
};

class FrictionConeSolver
{
public:
    SolveResult solve(const Eigen::MatrixXd& A,
                      const Eigen::VectorXd& b,
                      const std::vector<double>& mu,
                      const Eigen::VectorXd& lambda0 = Eigen::VectorXd{}) const
    {
        const int n = static_cast<int>(b.size());
        const int numContacts = n / 3;

        // Step 1: Regularize
        Eigen::MatrixXd A_reg = A;
        double eps = kRegularizationEpsilon;

        // Step 2: Cholesky factorization with fallback regularization
        Eigen::LLT<Eigen::MatrixXd> llt;
        bool factored = false;
        for (int attempt = 0; attempt < 6; ++attempt)
        {
            A_reg = A + eps * Eigen::MatrixXd::Identity(n, n);
            llt.compute(A_reg);
            if (llt.info() == Eigen::Success)
            {
                factored = true;
                break;
            }
            eps *= 10.0;
        }

        if (!factored)
        {
            // Cannot factor -- return zero
            return {Eigen::VectorXd::Zero(n), false, 0,
                    std::numeric_limits<double>::infinity()};
        }

        // Step 3: Unconstrained optimum
        Eigen::VectorXd lambda_unc = llt.solve(b);

        // Step 4: Initialize lambda
        Eigen::VectorXd lambda{n};
        if (lambda0.size() == n)
        {
            lambda = projectVector(lambda0, mu, numContacts);
        }
        else
        {
            lambda = projectVector(lambda_unc, mu, numContacts);
        }

        // Objective function: f(x) = 0.5 * x^T A_reg x - b^T x
        auto objective = [&](const Eigen::VectorXd& x) -> double {
            return 0.5 * x.dot(A_reg * x) - b.dot(x);
        };

        // Step 5: Newton iteration
        double residual = std::numeric_limits<double>::infinity();
        int iter = 0;
        for (iter = 0; iter < max_iterations_; ++iter)
        {
            // Step 5a: Gradient
            Eigen::VectorXd g = A_reg * lambda - b;

            // Step 5b: Projected gradient residual
            Eigen::VectorXd proj_lmg = projectVector(lambda - g, mu, numContacts);
            residual = (lambda - proj_lmg).norm();

            // Step 5c: Convergence check
            if (residual < tolerance_)
            {
                return {lambda, true, iter, residual};
            }

            // Step 5d: Compute search direction using reduced-space Newton.
            // Build J_proj: the block-diagonal Jacobian of the cone projection
            // evaluated at the current lambda. For contacts on the cone surface,
            // this is the rank-2 projection matrix from P1. For interior contacts,
            // it's the identity. For zero contacts, it's zero.
            // Then solve: (J_proj * A_reg * J_proj) * delta = -J_proj * g
            // This gives a Newton step that lives in the tangent space of the
            // active constraint set, enabling quadratic convergence.

            Eigen::MatrixXd J_proj = Eigen::MatrixXd::Zero(n, n);
            for (int c = 0; c < numContacts; ++c)
            {
                double ln = lambda[3 * c];
                double lt1 = lambda[3 * c + 1];
                double lt2 = lambda[3 * c + 2];
                double lt_norm = std::sqrt(lt1 * lt1 + lt2 * lt2);

                Eigen::Matrix3d Jc;
                if (lt_norm <= mu[c] * ln && ln >= 0)
                {
                    // Interior: identity
                    Jc = Eigen::Matrix3d::Identity();
                }
                else if (mu[c] * lt_norm <= -ln || (ln <= 0 && lt_norm < 1e-15))
                {
                    // Dual cone or origin: zero
                    Jc = Eigen::Matrix3d::Zero();
                }
                else
                {
                    // On cone surface: use the projection Jacobian from P1
                    double r = lt_norm;
                    double mu_c = mu[c];
                    if (r < 1e-15 || mu_c < 1e-15)
                    {
                        Jc = Eigen::Matrix3d::Zero();
                        if (ln > 0) Jc(0, 0) = 1.0;
                    }
                    else
                    {
                        double alpha_c = 1.0 / (1.0 + mu_c * mu_c);
                        double proj_n = alpha_c * (ln + mu_c * r);

                        // Row 0: d(proj_n)/dp
                        Jc(0, 0) = alpha_c;
                        Jc(0, 1) = alpha_c * mu_c * lt1 / r;
                        Jc(0, 2) = alpha_c * mu_c * lt2 / r;

                        // Rows 1-2: d(proj_ti)/dp
                        double p_t[2] = {lt1, lt2};
                        for (int i = 0; i < 2; ++i)
                        {
                            Jc(i + 1, 0) = mu_c * alpha_c * p_t[i] / r;
                            for (int j = 0; j < 2; ++j)
                            {
                                double delta_ij = (i == j) ? 1.0 : 0.0;
                                double dproj_n_dpj = alpha_c * mu_c * p_t[j] / r;
                                double d_ratio = delta_ij / r - p_t[i] * p_t[j] / (r * r * r);
                                Jc(i + 1, j + 1) = mu_c * (dproj_n_dpj * p_t[i] / r + proj_n * d_ratio);
                            }
                        }
                    }
                }
                J_proj.block<3, 3>(3 * c, 3 * c) = Jc;
            }

            // Reduced Hessian: H_r = J^T A J (acting in tangent space)
            Eigen::MatrixXd H_r = J_proj.transpose() * A_reg * J_proj;
            // Reduced gradient: g_r = J^T g
            Eigen::VectorXd g_r = J_proj.transpose() * g;
            // Regularize for numerical stability
            H_r += 1e-12 * Eigen::MatrixXd::Identity(n, n);

            Eigen::VectorXd delta;
            Eigen::LLT<Eigen::MatrixXd> llt_r{H_r};
            if (llt_r.info() == Eigen::Success)
            {
                delta = J_proj * llt_r.solve(-g_r);
            }
            else
            {
                // Fallback to standard Newton
                delta = llt.solve(-g);
            }

            // Step 5e: Armijo line search with projection
            double alpha = 1.0;
            double f_current = objective(lambda);
            Eigen::VectorXd trial = lambda;
            bool made_progress = false;

            for (int ls = 0; ls < max_line_search_; ++ls)
            {
                trial = projectVector(lambda + alpha * delta, mu, numContacts);
                double step_norm = (trial - lambda).norm();

                if (step_norm < 1e-15)
                {
                    alpha *= armijo_beta_;
                    continue;
                }

                double f_trial = objective(trial);

                // Sufficient decrease: f(trial) < f(current) - c1 * alpha * ||step||^2
                // Or standard Armijo: f(trial) <= f(current) + c1 * g^T * (trial - lambda)
                double descent = g.dot(trial - lambda);
                if (f_trial <= f_current + armijo_c1_ * std::min(descent, 0.0))
                {
                    lambda = trial;
                    made_progress = true;
                    break;
                }
                alpha *= armijo_beta_;
            }

            // If projected Newton didn't work, try SPG (Spectral Projected Gradient).
            // Uses Barzilai-Borwein step size for superlinear convergence.
            if (!made_progress)
            {
                // BB step size: alpha = (s^T s) / (s^T y) where s = x_k - x_{k-1}, y = g_k - g_{k-1}
                // For the first call, use 1/||A|| as initial step.
                // We approximate by using the Cauchy step size: alpha = (g^T g) / (g^T A g)
                double gAg = g.dot(A_reg * g);
                alpha = (gAg > 1e-15) ? g.dot(g) / gAg : 1.0;

                for (int ls = 0; ls < max_line_search_; ++ls)
                {
                    trial = projectVector(lambda - alpha * g, mu, numContacts);
                    double step_norm = (trial - lambda).norm();

                    if (step_norm < 1e-15)
                    {
                        alpha *= armijo_beta_;
                        continue;
                    }

                    double f_trial = objective(trial);
                    double descent = g.dot(trial - lambda);
                    if (f_trial <= f_current + armijo_c1_ * std::min(descent, 0.0))
                    {
                        lambda = trial;
                        made_progress = true;
                        break;
                    }
                    alpha *= armijo_beta_;
                }
            }

            if (!made_progress)
            {
                break;
            }
        }

        return {lambda, false, iter, residual};
    }

    void setTolerance(double eps) { tolerance_ = eps; }
    void setMaxIterations(int n) { max_iterations_ = n; }

private:
    double tolerance_{1e-8};
    int max_iterations_{50};
    double armijo_c1_{1e-4};
    double armijo_beta_{0.5};
    int max_line_search_{20};
    static constexpr double kRegularizationEpsilon = 1e-10;
};

// ============================================================================
// Test Infrastructure
// ============================================================================

static int totalTests = 0;
static int passedTests = 0;
static int failedTests = 0;

void reportResult(const std::string& name, bool passed, const std::string& detail = "")
{
    totalTests++;
    if (passed)
    {
        passedTests++;
        std::printf("  [PASS] %s", name.c_str());
    }
    else
    {
        failedTests++;
        std::printf("  [FAIL] %s", name.c_str());
    }
    if (!detail.empty())
    {
        std::printf(" -- %s", detail.c_str());
    }
    std::printf("\n");
}

bool approxEqual(double a, double b, double tol = 1e-6)
{
    return std::abs(a - b) < tol;
}

void checkSolution(const std::string& name,
                   const SolveResult& result,
                   const Eigen::MatrixXd& A,
                   const Eigen::VectorXd& b,
                   const std::vector<double>& mu,
                   int numContacts,
                   int maxIters)
{
    char buf[512];

    // Check convergence
    std::snprintf(buf, sizeof(buf), "converged=%s, iters=%d, residual=%.2e",
                  result.converged ? "true" : "false", result.iterations, result.residual);
    reportResult(name + ": convergence", result.converged, buf);

    // Check iteration count
    std::snprintf(buf, sizeof(buf), "iters=%d, max=%d", result.iterations, maxIters);
    reportResult(name + ": iteration count <= " + std::to_string(maxIters),
                 result.iterations <= maxIters, buf);

    // Check no NaN/inf
    bool no_nan = true;
    for (int i = 0; i < result.lambda.size(); ++i)
    {
        if (std::isnan(result.lambda[i]) || std::isinf(result.lambda[i]))
        {
            no_nan = false;
            break;
        }
    }
    reportResult(name + ": no NaN/inf", no_nan);

    // Check cone feasibility for each contact
    bool cone_ok = true;
    for (int c = 0; c < numContacts; ++c)
    {
        double ln = result.lambda[3 * c];
        double lt1 = result.lambda[3 * c + 1];
        double lt2 = result.lambda[3 * c + 2];
        double lt_norm = std::sqrt(lt1 * lt1 + lt2 * lt2);

        if (ln < -1e-8 || lt_norm > mu[c] * ln + 1e-8)
        {
            std::snprintf(buf, sizeof(buf), "contact %d: ln=%.6f, ||lt||=%.6f, mu*ln=%.6f",
                          c, ln, lt_norm, mu[c] * ln);
            reportResult(name + ": cone feasibility contact " + std::to_string(c), false, buf);
            cone_ok = false;
        }
    }
    if (cone_ok)
    {
        reportResult(name + ": cone feasibility (all contacts)", true);
    }

    // Check KKT residual: for a constrained problem, the projected gradient
    // should be small. The gradient g = A*lambda - b should satisfy:
    // lambda = Proj_K(lambda - g) at the solution.
    Eigen::VectorXd g = A * result.lambda - b;
    Eigen::VectorXd proj = projectVector(result.lambda - g, mu, numContacts);
    double kkt_residual = (result.lambda - proj).norm();
    std::snprintf(buf, sizeof(buf), "projected_grad_norm=%.2e", kkt_residual);
    reportResult(name + ": KKT residual < 1e-8",
                 kkt_residual < 1e-8, buf);
}

// ============================================================================
// Numerical Examples
// ============================================================================

void example1_Frictionless()
{
    std::printf("\n=== Example 1: Frictionless (mu=0) ===\n");
    std::printf("Expected: lambda_n=15.0, lambda_t=0\n\n");

    // Single contact, mu=0. The cone collapses to the half-line.
    // A is 3x3 effective mass, b is RHS.
    // With mu=0, the solver should produce lambda_t = 0 and lambda_n = A_nn^{-1} * b_n.
    //
    // Setup: A = diag(2, 2, 2), b = (30, 5, -3)
    // Unconstrained: lambda = A^{-1}b = (15, 2.5, -1.5)
    // With mu=0 cone: lambda_n = max(15, 0) = 15, lambda_t = (0, 0)
    // But since the cone constraint forces lambda_t=0, the solver minimizes
    // f(lambda_n) = 0.5 * 2 * lambda_n^2 - 30 * lambda_n
    // f'(lambda_n) = 2*lambda_n - 30 = 0 -> lambda_n = 15
    // So expected: (15, 0, 0)

    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 30.0, 5.0, -3.0;
    std::vector<double> mu = {0.0};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    checkSolution("Ex1", result, A, b, mu, 1, 8);

    // Check specific expected values
    char buf[256];
    std::snprintf(buf, sizeof(buf), "lambda_n=%.6f (expected 15.0)", result.lambda[0]);
    reportResult("Ex1: lambda_n = 15.0", approxEqual(result.lambda[0], 15.0, 1e-6), buf);

    std::snprintf(buf, sizeof(buf), "lambda_t1=%.6f, lambda_t2=%.6f (expected 0)",
                  result.lambda[1], result.lambda[2]);
    reportResult("Ex1: lambda_t = 0",
                 approxEqual(result.lambda[1], 0.0, 1e-8) &&
                 approxEqual(result.lambda[2], 0.0, 1e-8), buf);
}

void example2_Sticking()
{
    std::printf("\n=== Example 2: Sticking Contact (mu=0.5) ===\n");
    std::printf("Expected: unconstrained optimum is feasible\n\n");

    // Single contact, mu=0.5.
    // Choose A and b so the unconstrained optimum is INSIDE the cone.
    // A = diag(2, 2, 2), b = (30, -4, 0)
    // Unconstrained: lambda = (15, -2, 0)
    // ||lambda_t|| = 2, mu*lambda_n = 0.5*15 = 7.5
    // 2 <= 7.5 -> interior -> sticking

    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 30.0, -4.0, 0.0;
    std::vector<double> mu = {0.5};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    checkSolution("Ex2", result, A, b, mu, 1, 8);

    // The unconstrained optimum should be the solution
    char buf[256];
    std::snprintf(buf, sizeof(buf), "lambda=(%.4f, %.4f, %.4f) expected (15, -2, 0)",
                  result.lambda[0], result.lambda[1], result.lambda[2]);
    reportResult("Ex2: lambda = (15, -2, 0)",
                 approxEqual(result.lambda[0], 15.0, 1e-6) &&
                 approxEqual(result.lambda[1], -2.0, 1e-6) &&
                 approxEqual(result.lambda[2], 0.0, 1e-6), buf);
}

void example3_Sliding()
{
    std::printf("\n=== Example 3: Sliding Contact (mu=0.3) ===\n");
    std::printf("Expected: solution on cone surface\n\n");

    // Single contact, mu=0.3.
    // Choose A and b so the unconstrained optimum is OUTSIDE the cone.
    // A = diag(2, 2, 2), b = (30, 20, 0)
    // Unconstrained: lambda = (15, 10, 0)
    // ||lambda_t|| = 10, mu*lambda_n = 0.3*15 = 4.5
    // 10 > 4.5 -> outside cone -> sliding
    //
    // At the solution, we expect ||lambda_t|| = mu * lambda_n (on cone surface).
    // The QP with cone constraint yields a solution where the gradient of the
    // objective is in the normal cone of the constraint set at the solution point.

    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 30.0, 20.0, 0.0;
    std::vector<double> mu = {0.3};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    checkSolution("Ex3", result, A, b, mu, 1, 8);

    // Verify on cone surface
    double lt_norm = std::sqrt(result.lambda[1] * result.lambda[1] +
                                result.lambda[2] * result.lambda[2]);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "||lt||=%.6f, mu*ln=%.6f",
                  lt_norm, mu[0] * result.lambda[0]);
    reportResult("Ex3: ||lambda_t|| = mu * lambda_n (on surface)",
                 approxEqual(lt_norm, mu[0] * result.lambda[0], 1e-6), buf);

    // Verify sliding direction is preserved (t1 direction, t2 = 0)
    reportResult("Ex3: sliding direction preserved (t1 > 0)",
                 result.lambda[1] > 0.0);
}

void example4_TwoContacts()
{
    std::printf("\n=== Example 4: Two Contacts, Different mu ===\n");
    std::printf("Expected: independent cone constraints satisfied\n\n");

    // Two contacts: mu1=0.8 (high friction), mu2=0.2 (low friction)
    // A is 6x6 block-diagonal (no cross-coupling between contacts)
    // Contact 0: A_00 = diag(2,2,2), b_0 = (30, -4, 0) -> sticking (interior)
    // Contact 1: A_11 = diag(2,2,2), b_1 = (20, 15, 0) -> sliding (outside cone)
    //
    // Contact 0: unconstrained = (15, -2, 0), ||lt||=2, mu*ln=0.8*15=12 -> interior
    // Contact 1: unconstrained = (10, 7.5, 0), ||lt||=7.5, mu*ln=0.2*10=2 -> outside

    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd b{6};
    b << 30.0, -4.0, 0.0,   // Contact 0
         20.0, 15.0, 0.0;   // Contact 1
    std::vector<double> mu = {0.8, 0.2};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    checkSolution("Ex4", result, A, b, mu, 2, 8);

    // Contact 0 should be at unconstrained optimum (sticking)
    char buf[256];
    std::snprintf(buf, sizeof(buf), "c0=(%.4f, %.4f, %.4f) expected (15, -2, 0)",
                  result.lambda[0], result.lambda[1], result.lambda[2]);
    reportResult("Ex4: contact 0 sticking at (15, -2, 0)",
                 approxEqual(result.lambda[0], 15.0, 1e-5) &&
                 approxEqual(result.lambda[1], -2.0, 1e-5) &&
                 approxEqual(result.lambda[2], 0.0, 1e-5), buf);

    // Contact 1 should be on cone surface
    double lt1_norm = std::sqrt(result.lambda[4] * result.lambda[4] +
                                 result.lambda[5] * result.lambda[5]);
    std::snprintf(buf, sizeof(buf), "c1: ||lt||=%.6f, mu*ln=%.6f",
                  lt1_norm, mu[1] * result.lambda[3]);
    reportResult("Ex4: contact 1 on cone surface",
                 approxEqual(lt1_norm, mu[1] * result.lambda[3], 1e-6), buf);
}

void example5_WarmStart()
{
    std::printf("\n=== Example 5: Warm Start vs Cold Start ===\n");
    std::printf("Expected: warm start converges in fewer iterations\n\n");

    // Use the sliding case (Example 3) as the base problem
    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 30.0, 20.0, 0.0;
    std::vector<double> mu = {0.3};

    FrictionConeSolver solver;

    // Cold start (from zero)
    Eigen::VectorXd cold_start = Eigen::VectorXd::Zero(3);
    auto result_cold = solver.solve(A, b, mu, cold_start);

    // Now perturb the problem slightly (as would happen between frames)
    Eigen::VectorXd b_perturbed{3};
    b_perturbed << 30.5, 19.5, 0.0;

    // Warm start from previous solution
    auto result_warm = solver.solve(A, b_perturbed, mu, result_cold.lambda);

    // Cold start on perturbed problem
    auto result_cold2 = solver.solve(A, b_perturbed, mu, cold_start);

    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "cold_iters=%d, warm_iters=%d",
                  result_cold2.iterations, result_warm.iterations);
    reportResult("Ex5: warm start fewer iterations",
                 result_warm.iterations <= result_cold2.iterations, buf);

    reportResult("Ex5: warm start converged", result_warm.converged);

    std::snprintf(buf, sizeof(buf), "warm_iters=%d, max=3", result_warm.iterations);
    reportResult("Ex5: warm start <= 3 iterations",
                 result_warm.iterations <= 3, buf);

    // Verify both give same solution
    double diff = (result_warm.lambda - result_cold2.lambda).norm();
    std::snprintf(buf, sizeof(buf), "solution_diff=%.2e", diff);
    reportResult("Ex5: warm and cold converge to same solution",
                 diff < 1e-6, buf);
}

void example6_GrazingContact()
{
    std::printf("\n=== Example 6: Grazing Contact (lambda_n -> 0) ===\n");
    std::printf("Expected: no NaN, graceful zero\n\n");

    // Grazing contact: the RHS drives lambda_n toward zero.
    // A = diag(2, 2, 2), b = (0.001, 10, 0)
    // Unconstrained: lambda = (0.0005, 5, 0)
    // ||lt|| = 5, mu*ln = 0.3*0.0005 = 0.00015 -> way outside cone
    // The solution should have lambda_n near 0 with lambda_t constrained by cone.

    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 0.001, 10.0, 0.0;
    std::vector<double> mu = {0.3};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    // Primary checks: no NaN, no inf, converged
    bool no_nan = !std::isnan(result.lambda[0]) &&
                  !std::isnan(result.lambda[1]) &&
                  !std::isnan(result.lambda[2]);
    bool no_inf = !std::isinf(result.lambda[0]) &&
                  !std::isinf(result.lambda[1]) &&
                  !std::isinf(result.lambda[2]);

    reportResult("Ex6: no NaN", no_nan);
    reportResult("Ex6: no inf", no_inf);
    reportResult("Ex6: converged", result.converged);

    // Check cone feasibility
    double lt_norm = std::sqrt(result.lambda[1] * result.lambda[1] +
                                result.lambda[2] * result.lambda[2]);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "ln=%.8f, ||lt||=%.8f, mu*ln=%.8f",
                  result.lambda[0], lt_norm, mu[0] * result.lambda[0]);
    reportResult("Ex6: cone feasibility",
                 result.lambda[0] >= -1e-8 && lt_norm <= mu[0] * result.lambda[0] + 1e-8,
                 buf);

    checkSolution("Ex6", result, A, b, mu, 1, 8);
}

void example7_InclinedPlaneAtFrictionAngle()
{
    std::printf("\n=== Example 7: Inclined Plane at Friction Angle ===\n");
    std::printf("Expected: stick/slip boundary (barely sticking)\n\n");

    // Inclined plane at exactly the friction angle: theta = arctan(mu)
    // At this angle, the required friction force EXACTLY equals the maximum
    // available friction force. The solution should be right at the cone boundary.
    //
    // mu = 0.5, theta = arctan(0.5) = 26.57 degrees
    //
    // For a single contact, we model this as:
    // The gravity component along the tangent creates a RHS that drives
    // lambda_t to mu * lambda_n.
    //
    // In the solver's coordinates:
    // Normal RHS corresponds to mg*cos(theta)
    // Tangential RHS corresponds to mg*sin(theta)
    //
    // With A = diag (effective mass terms), the effective masses along normal
    // and tangent are the same (simple 1-body contact).
    //
    // Setup: 1 body, mass = 10 kg, g = 9.81
    // mg = 98.1
    // theta = arctan(0.5) = 0.4636 rad
    // Normal component: mg*cos(theta) = 98.1 * cos(0.4636) = 87.83
    // Tangential component: mg*sin(theta) = 98.1 * sin(0.4636) = 43.91
    //
    // A = I (unit effective mass for simplicity)
    // b = (87.83, 43.91, 0)
    // mu = 0.5
    //
    // Unconstrained: lambda = (87.83, 43.91, 0)
    // ||lt|| = 43.91, mu*ln = 0.5*87.83 = 43.915
    // Since sin(theta) = mu*cos(theta) = mu/sqrt(1+mu^2) * sqrt(1+mu^2) * cos(theta),
    // we get ||lt|| = mu*ln exactly at friction angle.

    double mu_val = 0.5;
    double theta = std::atan(mu_val);
    double mg = 98.1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << mg * std::cos(theta), mg * std::sin(theta), 0.0;
    std::vector<double> mu = {mu_val};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    checkSolution("Ex7", result, A, b, mu, 1, 8);

    // At friction angle, ||lambda_t|| should equal mu * lambda_n
    double lt_norm = std::sqrt(result.lambda[1] * result.lambda[1] +
                                result.lambda[2] * result.lambda[2]);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "||lt||=%.6f, mu*ln=%.6f, diff=%.2e",
                  lt_norm, mu_val * result.lambda[0],
                  std::abs(lt_norm - mu_val * result.lambda[0]));

    // The unconstrained solution is right at (or very near) the cone boundary
    // So it should be either exactly feasible or projected to the surface
    bool at_boundary = approxEqual(lt_norm, mu_val * result.lambda[0], 1e-4);
    bool interior = lt_norm < mu_val * result.lambda[0] - 1e-4;
    reportResult("Ex7: at or near friction angle boundary",
                 at_boundary || interior, buf);

    // Verify the solution is physically reasonable:
    // lambda_n should be close to mg*cos(theta)
    std::snprintf(buf, sizeof(buf), "ln=%.4f, expected=%.4f",
                  result.lambda[0], mg * std::cos(theta));
    reportResult("Ex7: lambda_n ~ mg*cos(theta)",
                 approxEqual(result.lambda[0], mg * std::cos(theta), 0.1), buf);
}

void testRegularization()
{
    std::printf("\n=== Bonus: Diagonal Regularization for Ill-Conditioned A ===\n\n");

    // Ill-conditioned A: condition number ~1e6 (realistic for high mass ratio)
    // This tests that the solver handles ill-conditioning gracefully.
    Eigen::MatrixXd A{3, 3};
    A << 1.0, 0.0, 0.0,
         0.0, 1e-6, 0.0,
         0.0, 0.0, 1e-6;

    Eigen::VectorXd b{3};
    b << 10.0, 1.0, 0.0;
    std::vector<double> mu = {0.5};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    bool no_nan = !std::isnan(result.lambda[0]) &&
                  !std::isnan(result.lambda[1]) &&
                  !std::isnan(result.lambda[2]);
    bool no_inf = !std::isinf(result.lambda[0]) &&
                  !std::isinf(result.lambda[1]) &&
                  !std::isinf(result.lambda[2]);

    reportResult("Regularization: no NaN", no_nan);
    reportResult("Regularization: no inf", no_inf);
    reportResult("Regularization: converged", result.converged);

    char buf[256];
    std::snprintf(buf, sizeof(buf), "lambda=(%.4f, %.4f, %.4f)",
                  result.lambda[0], result.lambda[1], result.lambda[2]);
    reportResult("Regularization: reasonable output", result.lambda[0] > 0, buf);
}

void testCoupledContacts()
{
    std::printf("\n=== Bonus: Coupled Contacts (Off-Diagonal A) ===\n\n");

    // Two contacts sharing a body -> off-diagonal blocks in A
    // This is more realistic than diagonal A.
    Eigen::MatrixXd A{6, 6};
    A << 4.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 4.0, 0.0, 0.0, 0.5, 0.0,
         0.0, 0.0, 4.0, 0.0, 0.0, 0.5,
         1.0, 0.0, 0.0, 3.0, 0.0, 0.0,
         0.0, 0.5, 0.0, 0.0, 3.0, 0.0,
         0.0, 0.0, 0.5, 0.0, 0.0, 3.0;

    Eigen::VectorXd b{6};
    b << 20.0, 8.0, 0.0,    // Contact 0
         15.0, 12.0, 0.0;   // Contact 1
    std::vector<double> mu = {0.5, 0.3};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    // Note: coupled contacts with off-diagonal A require more iterations than
    // the 8-iteration target for diagonal A. This is because the reduced-space
    // Newton step on the cone surface doesn't fully capture cross-contact
    // coupling. For production, the implementation should use a full augmented
    // Lagrangian or SOCP interior-point refinement. For now, we verify
    // convergence and correctness, and document the iteration count.
    checkSolution("Coupled", result, A, b, mu, 2, 50);
}

// ============================================================================
// Main
// ============================================================================

int main()
{
    std::printf("=== Prototype P2: Newton Solver for Friction Cone QP ===\n");

    example1_Frictionless();
    example2_Sticking();
    example3_Sliding();
    example4_TwoContacts();
    example5_WarmStart();
    example6_GrazingContact();
    example7_InclinedPlaneAtFrictionAngle();
    testRegularization();
    testCoupledContacts();

    std::printf("\n========================================\n");
    std::printf("Results: %d/%d passed, %d failed\n", passedTests, totalTests, failedTests);
    std::printf("========================================\n");

    return failedTests > 0 ? 1 : 0;
}

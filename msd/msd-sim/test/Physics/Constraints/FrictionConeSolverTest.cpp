// Ticket: 0052c_newton_solver_core
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Constraints/FrictionConeSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ConeProjection.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <vector>

using namespace msd_sim;

namespace
{

// Helper: verify cone feasibility for all contacts
void expectConeFeasible(const Eigen::VectorXd& lambda, const std::vector<double>& mu, int numContacts)
{
    for (int c = 0; c < numContacts; ++c)
    {
        double ln = lambda[3 * c];
        double lt1 = lambda[3 * c + 1];
        double lt2 = lambda[3 * c + 2];
        double lt_norm = std::sqrt(lt1 * lt1 + lt2 * lt2);

        EXPECT_GE(ln, -1e-8) << "Contact " << c << ": negative normal force";
        EXPECT_LE(lt_norm, mu[static_cast<size_t>(c)] * ln + 1e-8)
            << "Contact " << c << ": friction exceeds cone";
    }
}

// Helper: verify KKT residual
void expectKKTSatisfied(const Eigen::VectorXd& lambda, const Eigen::MatrixXd& A,
                        const Eigen::VectorXd& b, const std::vector<double>& mu,
                        int numContacts, double tol = 1e-8)
{
    Eigen::VectorXd g = A * lambda - b;
    Eigen::VectorXd proj = ConeProjection::projectVector(lambda - g, mu, numContacts);
    double kkt_residual = (lambda - proj).norm();
    EXPECT_LT(kkt_residual, tol) << "KKT residual: " << kkt_residual;
}

// Helper: verify no NaN/inf
void expectFinite(const Eigen::VectorXd& lambda)
{
    for (int i = 0; i < lambda.size(); ++i)
    {
        EXPECT_FALSE(std::isnan(lambda[i])) << "NaN at index " << i;
        EXPECT_FALSE(std::isinf(lambda[i])) << "inf at index " << i;
    }
}

}  // namespace

// ============================================================================
// M8 Example 1: Frictionless (mu=0)
// ============================================================================

TEST(FrictionConeSolver, M8Ex1_Frictionless_0052c)
{
    // A = diag(2,2,2), b = (30, 5, -3), mu = 0
    // With mu=0, lambda_t forced to 0, lambda_n = 15
    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 30.0, 5.0, -3.0;
    std::vector<double> mu = {0.0};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    EXPECT_TRUE(result.converged);
    EXPECT_LE(result.iterations, 8);
    expectFinite(result.lambda);
    expectConeFeasible(result.lambda, mu, 1);
    expectKKTSatisfied(result.lambda, A, b, mu, 1);

    EXPECT_NEAR(result.lambda[0], 15.0, 1e-6);
    EXPECT_NEAR(result.lambda[1], 0.0, 1e-8);
    EXPECT_NEAR(result.lambda[2], 0.0, 1e-8);
}

// ============================================================================
// M8 Example 2: Sticking Contact (mu=0.5)
// ============================================================================

TEST(FrictionConeSolver, M8Ex2_Sticking_0052c)
{
    // A = diag(2,2,2), b = (30, -4, 0), mu = 0.5
    // Unconstrained: (15, -2, 0), ||lt||=2, mu*ln=7.5 -> interior
    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 30.0, -4.0, 0.0;
    std::vector<double> mu = {0.5};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    EXPECT_TRUE(result.converged);
    EXPECT_LE(result.iterations, 8);
    expectFinite(result.lambda);
    expectConeFeasible(result.lambda, mu, 1);
    expectKKTSatisfied(result.lambda, A, b, mu, 1);

    EXPECT_NEAR(result.lambda[0], 15.0, 1e-6);
    EXPECT_NEAR(result.lambda[1], -2.0, 1e-6);
    EXPECT_NEAR(result.lambda[2], 0.0, 1e-6);
}

// ============================================================================
// M8 Example 3: Sliding Contact (mu=0.3)
// ============================================================================

TEST(FrictionConeSolver, M8Ex3_Sliding_0052c)
{
    // A = diag(2,2,2), b = (30, 20, 0), mu = 0.3
    // Unconstrained: (15, 10, 0), ||lt||=10 > mu*ln=4.5 -> outside
    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 30.0, 20.0, 0.0;
    std::vector<double> mu = {0.3};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    EXPECT_TRUE(result.converged);
    EXPECT_LE(result.iterations, 8);
    expectFinite(result.lambda);
    expectConeFeasible(result.lambda, mu, 1);
    expectKKTSatisfied(result.lambda, A, b, mu, 1);

    // Verify on cone surface
    double lt_norm = std::sqrt(result.lambda[1] * result.lambda[1] +
                                result.lambda[2] * result.lambda[2]);
    EXPECT_NEAR(lt_norm, mu[0] * result.lambda[0], 1e-6);

    // Sliding direction preserved (positive t1)
    EXPECT_GT(result.lambda[1], 0.0);
}

// ============================================================================
// M8 Example 4: Two Contacts, Different mu
// ============================================================================

TEST(FrictionConeSolver, M8Ex4_TwoContacts_0052c)
{
    // Two contacts: mu1=0.8 (sticking), mu2=0.2 (sliding)
    // A block-diagonal, no cross-coupling
    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd b{6};
    b << 30.0, -4.0, 0.0,   // Contact 0: (15,-2,0) interior for mu=0.8
         20.0, 15.0, 0.0;   // Contact 1: (10,7.5,0) outside for mu=0.2
    std::vector<double> mu = {0.8, 0.2};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    EXPECT_TRUE(result.converged);
    EXPECT_LE(result.iterations, 8);
    expectFinite(result.lambda);
    expectConeFeasible(result.lambda, mu, 2);
    expectKKTSatisfied(result.lambda, A, b, mu, 2);

    // Contact 0: sticking at unconstrained optimum
    EXPECT_NEAR(result.lambda[0], 15.0, 1e-5);
    EXPECT_NEAR(result.lambda[1], -2.0, 1e-5);
    EXPECT_NEAR(result.lambda[2], 0.0, 1e-5);

    // Contact 1: on cone surface
    double lt1_norm = std::sqrt(result.lambda[4] * result.lambda[4] +
                                 result.lambda[5] * result.lambda[5]);
    EXPECT_NEAR(lt1_norm, mu[1] * result.lambda[3], 1e-6);
}

// ============================================================================
// M8 Example 5: Warm Start Convergence
// ============================================================================

TEST(FrictionConeSolver, M8Ex5_WarmStartFewerIterations_0052c)
{
    // Base problem: sliding
    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 30.0, 20.0, 0.0;
    std::vector<double> mu = {0.3};

    FrictionConeSolver solver;

    // Cold start
    Eigen::VectorXd cold_start = Eigen::VectorXd::Zero(3);
    auto result_cold = solver.solve(A, b, mu, cold_start);
    EXPECT_TRUE(result_cold.converged);

    // Perturb problem slightly
    Eigen::VectorXd b_perturbed{3};
    b_perturbed << 30.5, 19.5, 0.0;

    // Warm start from previous solution
    auto result_warm = solver.solve(A, b_perturbed, mu, result_cold.lambda);

    // Cold start on perturbed problem
    auto result_cold2 = solver.solve(A, b_perturbed, mu, cold_start);

    EXPECT_TRUE(result_warm.converged);
    EXPECT_LE(result_warm.iterations, result_cold2.iterations);
    EXPECT_LE(result_warm.iterations, 3);

    // Both should give same solution
    double diff = (result_warm.lambda - result_cold2.lambda).norm();
    EXPECT_LT(diff, 1e-6);
}

// ============================================================================
// M8 Example 6: Grazing Contact
// ============================================================================

TEST(FrictionConeSolver, M8Ex6_GrazingContact_0052c)
{
    // A = diag(2,2,2), b = (0.001, 10, 0), mu=0.3
    // lambda_n near 0, high tangential drive
    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 0.001, 10.0, 0.0;
    std::vector<double> mu = {0.3};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    EXPECT_TRUE(result.converged);
    expectFinite(result.lambda);
    expectConeFeasible(result.lambda, mu, 1);
    expectKKTSatisfied(result.lambda, A, b, mu, 1);
}

// ============================================================================
// M8 Example 7: Inclined Plane at Friction Angle
// ============================================================================

TEST(FrictionConeSolver, M8Ex7_InclinedPlane_0052c)
{
    // theta = arctan(mu), so the unconstrained solution is right at the cone boundary
    double mu_val = 0.5;
    double theta = std::atan(mu_val);
    double mg = 98.1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << mg * std::cos(theta), mg * std::sin(theta), 0.0;
    std::vector<double> mu = {mu_val};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    EXPECT_TRUE(result.converged);
    EXPECT_LE(result.iterations, 8);
    expectFinite(result.lambda);
    expectConeFeasible(result.lambda, mu, 1);
    expectKKTSatisfied(result.lambda, A, b, mu, 1);

    // At friction angle, solution is at or near cone boundary
    double lt_norm = std::sqrt(result.lambda[1] * result.lambda[1] +
                                result.lambda[2] * result.lambda[2]);
    bool at_boundary = std::abs(lt_norm - mu_val * result.lambda[0]) < 1e-4;
    bool interior = lt_norm < mu_val * result.lambda[0] - 1e-4;
    EXPECT_TRUE(at_boundary || interior);

    // lambda_n ~ mg*cos(theta)
    EXPECT_NEAR(result.lambda[0], mg * std::cos(theta), 0.1);
}

// ============================================================================
// Cholesky Failure Recovery
// ============================================================================

TEST(FrictionConeSolver, CholeskyFailureRecovery_0052c)
{
    // Ill-conditioned A: condition number ~1e6
    Eigen::MatrixXd A{3, 3};
    A << 1.0, 0.0, 0.0,
         0.0, 1e-6, 0.0,
         0.0, 0.0, 1e-6;

    Eigen::VectorXd b{3};
    b << 10.0, 1.0, 0.0;
    std::vector<double> mu = {0.5};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    expectFinite(result.lambda);
    EXPECT_TRUE(result.converged);
    EXPECT_GT(result.lambda[0], 0.0);
}

// ============================================================================
// Max Iterations Cap
// ============================================================================

TEST(FrictionConeSolver, MaxIterationsCap_0052c)
{
    // Use coupled contacts that require more iterations
    Eigen::MatrixXd A{6, 6};
    A << 4.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 4.0, 0.0, 0.0, 0.5, 0.0,
         0.0, 0.0, 4.0, 0.0, 0.0, 0.5,
         1.0, 0.0, 0.0, 3.0, 0.0, 0.0,
         0.0, 0.5, 0.0, 0.0, 3.0, 0.0,
         0.0, 0.0, 0.5, 0.0, 0.0, 3.0;

    Eigen::VectorXd b{6};
    b << 20.0, 8.0, 0.0,
         15.0, 12.0, 0.0;
    std::vector<double> mu = {0.5, 0.3};

    FrictionConeSolver solver;
    solver.setMaxIterations(3);  // Artificially low
    auto result = solver.solve(A, b, mu);

    // Should either converge quickly (if initial projection is good) or
    // return false if 3 iterations was not enough
    EXPECT_LE(result.iterations, 3);
    expectFinite(result.lambda);
    expectConeFeasible(result.lambda, mu, 2);
}

// ============================================================================
// Coupled Contacts (off-diagonal A)
// ============================================================================

TEST(FrictionConeSolver, CoupledContacts_0052c)
{
    Eigen::MatrixXd A{6, 6};
    A << 4.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 4.0, 0.0, 0.0, 0.5, 0.0,
         0.0, 0.0, 4.0, 0.0, 0.0, 0.5,
         1.0, 0.0, 0.0, 3.0, 0.0, 0.0,
         0.0, 0.5, 0.0, 0.0, 3.0, 0.0,
         0.0, 0.0, 0.5, 0.0, 0.0, 3.0;

    Eigen::VectorXd b{6};
    b << 20.0, 8.0, 0.0,
         15.0, 12.0, 0.0;
    std::vector<double> mu = {0.5, 0.3};

    FrictionConeSolver solver;
    auto result = solver.solve(A, b, mu);

    EXPECT_TRUE(result.converged);
    expectFinite(result.lambda);
    expectConeFeasible(result.lambda, mu, 2);
    expectKKTSatisfied(result.lambda, A, b, mu, 2);
}

// ============================================================================
// Cold Start from Zero
// ============================================================================

TEST(FrictionConeSolver, ColdStartFromZero_0052c)
{
    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 30.0, 20.0, 0.0;
    std::vector<double> mu = {0.3};

    FrictionConeSolver solver;
    Eigen::VectorXd zero_start = Eigen::VectorXd::Zero(3);
    auto result = solver.solve(A, b, mu, zero_start);

    EXPECT_TRUE(result.converged);
    expectConeFeasible(result.lambda, mu, 1);
    expectKKTSatisfied(result.lambda, A, b, mu, 1);
}

// ============================================================================
// Determinism: Same Input -> Same Output
// ============================================================================

TEST(FrictionConeSolver, Deterministic_0052c)
{
    Eigen::MatrixXd A = 2.0 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd b{3};
    b << 30.0, 20.0, 0.0;
    std::vector<double> mu = {0.3};

    FrictionConeSolver solver;
    auto result1 = solver.solve(A, b, mu);
    auto result2 = solver.solve(A, b, mu);

    EXPECT_EQ(result1.converged, result2.converged);
    EXPECT_EQ(result1.iterations, result2.iterations);
    EXPECT_DOUBLE_EQ(result1.residual, result2.residual);
    EXPECT_EQ(result1.lambda.size(), result2.lambda.size());
    for (int i = 0; i < result1.lambda.size(); ++i)
    {
        EXPECT_DOUBLE_EQ(result1.lambda[i], result2.lambda[i]);
    }
}

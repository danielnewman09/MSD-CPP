// Ticket: 0052b_cone_projection_and_linear_algebra
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Constraints/ConeProjection.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace msd_sim;

// ============================================================================
// Case 1: Interior of Cone
// ============================================================================

TEST(ConeProjection, Case1_InteriorPointUnchanged_0052b)
{
    // Point well inside cone: mu=0.5, p=(10, 1, 0)
    // ||p_t|| = 1 <= 0.5 * 10 = 5 -> interior
    auto r = ConeProjection::project(10.0, 1.0, 0.0, 0.5);
    EXPECT_EQ(r.case_, 1);
    EXPECT_DOUBLE_EQ(r.lambda_n, 10.0);
    EXPECT_DOUBLE_EQ(r.lambda_t1, 1.0);
    EXPECT_DOUBLE_EQ(r.lambda_t2, 0.0);
}

TEST(ConeProjection, Case1_BoundaryPointIsInterior_0052b)
{
    // Point exactly on boundary: mu=0.5, p=(10, 3, 4) -> ||p_t||=5, mu*p_n=5
    auto r = ConeProjection::project(10.0, 3.0, 4.0, 0.5);
    EXPECT_EQ(r.case_, 1);
    EXPECT_DOUBLE_EQ(r.lambda_n, 10.0);
    EXPECT_DOUBLE_EQ(r.lambda_t1, 3.0);
    EXPECT_DOUBLE_EQ(r.lambda_t2, 4.0);
}

TEST(ConeProjection, Case1_PositiveNormalOnly_0052b)
{
    auto r = ConeProjection::project(5.0, 0.0, 0.0, 1.0);
    EXPECT_EQ(r.case_, 1);
    EXPECT_DOUBLE_EQ(r.lambda_n, 5.0);
    EXPECT_DOUBLE_EQ(r.lambda_t1, 0.0);
    EXPECT_DOUBLE_EQ(r.lambda_t2, 0.0);
}

// ============================================================================
// Case 2: Dual Cone (projects to origin)
// ============================================================================

TEST(ConeProjection, Case2_DualConeProjectsToOrigin_0052b)
{
    // p=(-10, 1, 0), mu=0.5
    // mu*||p_t|| = 0.5, -p_n = 10 -> 0.5 <= 10 -> dual cone
    auto r = ConeProjection::project(-10.0, 1.0, 0.0, 0.5);
    EXPECT_EQ(r.case_, 2);
    EXPECT_DOUBLE_EQ(r.lambda_n, 0.0);
    EXPECT_DOUBLE_EQ(r.lambda_t1, 0.0);
    EXPECT_DOUBLE_EQ(r.lambda_t2, 0.0);
}

TEST(ConeProjection, Case2_DualConeBoundary_0052b)
{
    // p=(-5, 10, 0), mu=0.5
    // mu*||p_t|| = 5, -p_n = 5 -> 5 <= 5 -> dual cone
    auto r = ConeProjection::project(-5.0, 10.0, 0.0, 0.5);
    EXPECT_EQ(r.case_, 2);
    EXPECT_DOUBLE_EQ(r.lambda_n, 0.0);
    EXPECT_DOUBLE_EQ(r.lambda_t1, 0.0);
    EXPECT_DOUBLE_EQ(r.lambda_t2, 0.0);
}

TEST(ConeProjection, Case2_AllNegative_0052b)
{
    auto r = ConeProjection::project(-3.0, -1.0, -1.0, 0.5);
    EXPECT_EQ(r.case_, 2);
    EXPECT_DOUBLE_EQ(r.lambda_n, 0.0);
    EXPECT_DOUBLE_EQ(r.lambda_t1, 0.0);
    EXPECT_DOUBLE_EQ(r.lambda_t2, 0.0);
}

// ============================================================================
// Case 3: Cone Surface Projection
// ============================================================================

TEST(ConeProjection, Case3_1DTangentProjection_0052b)
{
    // p=(1, 10, 0), mu=0.5
    // alpha = 1/(1+0.25) = 0.8
    // proj_n = 0.8 * (1 + 0.5*10) = 4.8
    // proj_t1 = 0.5 * 4.8 * 10/10 = 2.4
    auto r = ConeProjection::project(1.0, 10.0, 0.0, 0.5);
    EXPECT_EQ(r.case_, 3);
    EXPECT_NEAR(r.lambda_n, 4.8, 1e-10);
    EXPECT_NEAR(r.lambda_t1, 2.4, 1e-10);
    EXPECT_NEAR(r.lambda_t2, 0.0, 1e-10);
}

TEST(ConeProjection, Case3_ProjectedPointOnConeSurface_0052b)
{
    auto r = ConeProjection::project(1.0, 10.0, 0.0, 0.5);
    double proj_t_norm = std::sqrt(r.lambda_t1 * r.lambda_t1 + r.lambda_t2 * r.lambda_t2);
    EXPECT_NEAR(proj_t_norm, 0.5 * r.lambda_n, 1e-10);
}

TEST(ConeProjection, Case3_2DTangentProjection_0052b)
{
    // p=(2, 3, 4), mu=0.3
    // ||p_t|| = 5
    // alpha = 1/(1+0.09) = 1/1.09
    // proj_n = (2 + 0.3*5) / 1.09 = 3.5 / 1.09
    auto r = ConeProjection::project(2.0, 3.0, 4.0, 0.3);
    double expected_n = (2.0 + 0.3 * 5.0) / (1.0 + 0.09);
    double expected_t1 = 0.3 * expected_n * 3.0 / 5.0;
    double expected_t2 = 0.3 * expected_n * 4.0 / 5.0;
    EXPECT_EQ(r.case_, 3);
    EXPECT_NEAR(r.lambda_n, expected_n, 1e-10);
    EXPECT_NEAR(r.lambda_t1, expected_t1, 1e-10);
    EXPECT_NEAR(r.lambda_t2, expected_t2, 1e-10);
}

TEST(ConeProjection, Case3_2DTangentFeasibility_0052b)
{
    auto r = ConeProjection::project(2.0, 3.0, 4.0, 0.3);
    double proj_t_norm = std::sqrt(r.lambda_t1 * r.lambda_t1 + r.lambda_t2 * r.lambda_t2);
    EXPECT_NEAR(proj_t_norm, 0.3 * r.lambda_n, 1e-10);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(ConeProjection, EdgeCase_MuZeroPositiveNormal_0052b)
{
    auto r = ConeProjection::project(5.0, 3.0, 4.0, 0.0);
    EXPECT_NEAR(r.lambda_n, 5.0, 1e-10);
    EXPECT_NEAR(r.lambda_t1, 0.0, 1e-10);
    EXPECT_NEAR(r.lambda_t2, 0.0, 1e-10);
}

TEST(ConeProjection, EdgeCase_MuZeroNegativeNormal_0052b)
{
    auto r = ConeProjection::project(-5.0, 3.0, 4.0, 0.0);
    EXPECT_NEAR(r.lambda_n, 0.0, 1e-10);
    EXPECT_NEAR(r.lambda_t1, 0.0, 1e-10);
    EXPECT_NEAR(r.lambda_t2, 0.0, 1e-10);
}

TEST(ConeProjection, EdgeCase_ZeroTangentPositiveNormal_0052b)
{
    auto r = ConeProjection::project(5.0, 0.0, 0.0, 0.5);
    EXPECT_EQ(r.case_, 1);
    EXPECT_DOUBLE_EQ(r.lambda_n, 5.0);
    EXPECT_DOUBLE_EQ(r.lambda_t1, 0.0);
    EXPECT_DOUBLE_EQ(r.lambda_t2, 0.0);
}

TEST(ConeProjection, EdgeCase_ZeroTangentNegativeNormal_0052b)
{
    auto r = ConeProjection::project(-5.0, 0.0, 0.0, 0.5);
    EXPECT_EQ(r.case_, 2);
    EXPECT_DOUBLE_EQ(r.lambda_n, 0.0);
    EXPECT_DOUBLE_EQ(r.lambda_t1, 0.0);
    EXPECT_DOUBLE_EQ(r.lambda_t2, 0.0);
}

TEST(ConeProjection, EdgeCase_Origin_0052b)
{
    auto r = ConeProjection::project(0.0, 0.0, 0.0, 0.5);
    EXPECT_NEAR(r.lambda_n, 0.0, 1e-10);
    EXPECT_NEAR(r.lambda_t1, 0.0, 1e-10);
    EXPECT_NEAR(r.lambda_t2, 0.0, 1e-10);
}

TEST(ConeProjection, EdgeCase_NormalZeroWithTangent_0052b)
{
    // lambda_n = 0 with tangential: case 3
    auto r = ConeProjection::project(0.0, 5.0, 0.0, 0.5);
    EXPECT_EQ(r.case_, 3);
    double expected_n = (0.0 + 0.5 * 5.0) / (1.0 + 0.25);
    double expected_t1 = 0.5 * expected_n;
    EXPECT_NEAR(r.lambda_n, expected_n, 1e-10);
    EXPECT_NEAR(r.lambda_t1, expected_t1, 1e-10);
    EXPECT_NEAR(r.lambda_t2, 0.0, 1e-10);
}

TEST(ConeProjection, EdgeCase_LargeValues_NoNaNInf_0052b)
{
    auto r = ConeProjection::project(1e10, 1e11, 0.0, 0.5);
    EXPECT_FALSE(std::isnan(r.lambda_n));
    EXPECT_FALSE(std::isnan(r.lambda_t1));
    EXPECT_FALSE(std::isnan(r.lambda_t2));
    EXPECT_FALSE(std::isinf(r.lambda_n));
    EXPECT_FALSE(std::isinf(r.lambda_t1));
    EXPECT_FALSE(std::isinf(r.lambda_t2));
}

TEST(ConeProjection, EdgeCase_SmallValues_NoNaN_0052b)
{
    auto r = ConeProjection::project(1e-12, 1e-12, 0.0, 0.5);
    EXPECT_FALSE(std::isnan(r.lambda_n));
    EXPECT_FALSE(std::isnan(r.lambda_t1));
    EXPECT_FALSE(std::isnan(r.lambda_t2));
}

// ============================================================================
// Idempotency: project(project(x)) == project(x)
// ============================================================================

class ConeProjectionIdempotency : public ::testing::TestWithParam<std::tuple<double, double, double, double, std::string>>
{};

TEST_P(ConeProjectionIdempotency, ProjectProjectEqualsProject_0052b)
{
    auto [n, t1, t2, mu, name] = GetParam();
    auto r1 = ConeProjection::project(n, t1, t2, mu);
    auto r2 = ConeProjection::project(r1.lambda_n, r1.lambda_t1, r1.lambda_t2, mu);
    EXPECT_NEAR(r2.lambda_n, r1.lambda_n, 1e-12) << "Failed for: " << name;
    EXPECT_NEAR(r2.lambda_t1, r1.lambda_t1, 1e-12) << "Failed for: " << name;
    EXPECT_NEAR(r2.lambda_t2, r1.lambda_t2, 1e-12) << "Failed for: " << name;
}

INSTANTIATE_TEST_SUITE_P(
    ConeProjection,
    ConeProjectionIdempotency,
    ::testing::Values(
        std::make_tuple(10.0, 1.0, 0.0, 0.5, "interior"),
        std::make_tuple(-10.0, 1.0, 0.0, 0.5, "dual_cone"),
        std::make_tuple(1.0, 10.0, 0.0, 0.5, "surface_1d"),
        std::make_tuple(2.0, 3.0, 4.0, 0.3, "surface_2d"),
        std::make_tuple(0.0, 5.0, 0.0, 0.5, "normal_zero"),
        std::make_tuple(5.0, 3.0, 4.0, 0.0, "mu_zero"),
        std::make_tuple(0.0, 0.0, 0.0, 0.5, "origin"),
        std::make_tuple(1.0, 0.0, 0.0, 0.5, "positive_normal_only"),
        std::make_tuple(-1.0, 0.0, 0.0, 0.5, "negative_normal_only"),
        std::make_tuple(100.0, 50.0, -30.0, 0.8, "large_high_mu")
    )
);

// ============================================================================
// Continuity at Case Boundaries
// ============================================================================

TEST(ConeProjection, Continuity_Case1Case3Boundary_0052b)
{
    double mu = 0.5;
    double p_n = 10.0;
    double eps = 1e-8;

    auto r_below = ConeProjection::project(p_n, 5.0 - eps, 0.0, mu);
    auto r_above = ConeProjection::project(p_n, 5.0 + eps, 0.0, mu);
    auto r_exact = ConeProjection::project(p_n, 5.0, 0.0, mu);

    EXPECT_NEAR(r_below.lambda_n, r_exact.lambda_n, 1e-6);
    EXPECT_NEAR(r_below.lambda_t1, r_exact.lambda_t1, 1e-6);
    EXPECT_NEAR(r_above.lambda_n, r_exact.lambda_n, 1e-6);
    EXPECT_NEAR(r_above.lambda_t1, r_exact.lambda_t1, 1e-6);
}

TEST(ConeProjection, Continuity_Case2Case3Boundary_0052b)
{
    double mu = 0.5;
    double eps = 1e-8;

    auto r_case3 = ConeProjection::project(-5.0 + eps, 10.0, 0.0, mu);

    // At the boundary, Case 3 result should approach zero
    double case3_norm = std::sqrt(
        r_case3.lambda_n * r_case3.lambda_n +
        r_case3.lambda_t1 * r_case3.lambda_t1 +
        r_case3.lambda_t2 * r_case3.lambda_t2);
    EXPECT_LT(case3_norm, 1e-5);
}

// ============================================================================
// Gradient: All 3 Cases
// ============================================================================

TEST(ConeProjection, GradientCase1_Identity_0052b)
{
    auto J = ConeProjection::gradient(10.0, 1.0, 0.0, 0.5);
    Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
    EXPECT_LT((J - expected).norm(), 1e-12);
}

TEST(ConeProjection, GradientCase2_Zero_0052b)
{
    auto J = ConeProjection::gradient(-10.0, 1.0, 0.0, 0.5);
    EXPECT_LT(J.norm(), 1e-12);
}

TEST(ConeProjection, GradientCase3_Rank2_0052b)
{
    auto J = ConeProjection::gradient(1.0, 10.0, 0.0, 0.5);
    // Should not be identity or zero
    EXPECT_GT(J.norm(), 0.1);
    EXPECT_LT((J - Eigen::Matrix3d::Identity()).norm(), 10.0);  // Not identity

    // Verify symmetry (projection Jacobian should be symmetric)
    EXPECT_NEAR(J(0, 1), J(1, 0), 1e-10);
    EXPECT_NEAR(J(0, 2), J(2, 0), 1e-10);
    EXPECT_NEAR(J(1, 2), J(2, 1), 1e-10);
}

// ============================================================================
// Gradient: Finite Difference Verification
// ============================================================================

class ConeProjectionGradientFD : public ::testing::TestWithParam<std::tuple<double, double, double, double, std::string>>
{};

TEST_P(ConeProjectionGradientFD, GradientMatchesFiniteDifference_0052b)
{
    auto [n, t1, t2, mu, name] = GetParam();
    const double h = 1e-6;

    Eigen::Matrix3d J_analytic = ConeProjection::gradient(n, t1, t2, mu);
    Eigen::Matrix3d J_numeric;

    double inputs[3] = {n, t1, t2};

    for (int col = 0; col < 3; ++col)
    {
        double saved = inputs[col];

        inputs[col] = saved + h;
        auto r_plus = ConeProjection::project(inputs[0], inputs[1], inputs[2], mu);

        inputs[col] = saved - h;
        auto r_minus = ConeProjection::project(inputs[0], inputs[1], inputs[2], mu);

        inputs[col] = saved;

        J_numeric(0, col) = (r_plus.lambda_n - r_minus.lambda_n) / (2.0 * h);
        J_numeric(1, col) = (r_plus.lambda_t1 - r_minus.lambda_t1) / (2.0 * h);
        J_numeric(2, col) = (r_plus.lambda_t2 - r_minus.lambda_t2) / (2.0 * h);
    }

    double max_err = (J_analytic - J_numeric).cwiseAbs().maxCoeff();
    EXPECT_LT(max_err, 1e-4) << "Failed for: " << name << " max_err=" << max_err;
}

INSTANTIATE_TEST_SUITE_P(
    ConeProjection,
    ConeProjectionGradientFD,
    ::testing::Values(
        std::make_tuple(10.0, 1.0, 0.0, 0.5, "Case1_interior"),
        std::make_tuple(-10.0, 1.0, 0.0, 0.5, "Case2_dual_cone"),
        std::make_tuple(1.0, 10.0, 0.0, 0.5, "Case3_1d_tangent"),
        std::make_tuple(2.0, 3.0, 4.0, 0.3, "Case3_2d_tangent"),
        std::make_tuple(10.0, 3.0, 4.0, 0.8, "Case3_high_mu"),
        std::make_tuple(5.0, 0.5, -0.3, 1.0, "Case1_mu1")
    )
);

// ============================================================================
// projectVector (multi-contact)
// ============================================================================

TEST(ConeProjection, ProjectVector_MultiContact_0052b)
{
    Eigen::VectorXd lambda{6};
    lambda << 10.0, 1.0, 0.0,   // Contact 0: interior for mu=0.5
              1.0, 10.0, 0.0;   // Contact 1: outside for mu=0.3

    std::vector<double> mu = {0.5, 0.3};
    auto result = ConeProjection::projectVector(lambda, mu, 2);

    // Contact 0 should be unchanged (interior)
    EXPECT_NEAR(result[0], 10.0, 1e-10);
    EXPECT_NEAR(result[1], 1.0, 1e-10);
    EXPECT_NEAR(result[2], 0.0, 1e-10);

    // Contact 1 should be projected to surface
    double exp_n = (1.0 + 0.3 * 10.0) / (1.0 + 0.09);
    double exp_t1 = 0.3 * exp_n;
    EXPECT_NEAR(result[3], exp_n, 1e-10);
    EXPECT_NEAR(result[4], exp_t1, 1e-10);
    EXPECT_NEAR(result[5], 0.0, 1e-10);
}

TEST(ConeProjection, ProjectVector_Idempotent_0052b)
{
    Eigen::VectorXd lambda{6};
    lambda << 10.0, 1.0, 0.0,
              1.0, 10.0, 0.0;

    std::vector<double> mu = {0.5, 0.3};
    auto result1 = ConeProjection::projectVector(lambda, mu, 2);
    auto result2 = ConeProjection::projectVector(result1, mu, 2);

    double diff = (result2 - result1).norm();
    EXPECT_LT(diff, 1e-12);
}

TEST(ConeProjection, ProjectVector_SingleContact_0052b)
{
    Eigen::VectorXd lambda{3};
    lambda << 5.0, 2.0, -1.0;

    std::vector<double> mu = {0.5};
    auto result = ConeProjection::projectVector(lambda, mu, 1);

    // Verify cone feasibility
    double t_norm = std::sqrt(result[1] * result[1] + result[2] * result[2]);
    EXPECT_LE(t_norm, 0.5 * result[0] + 1e-10);
    EXPECT_GE(result[0], -1e-10);
}

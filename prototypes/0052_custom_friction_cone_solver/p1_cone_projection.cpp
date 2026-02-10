// Prototype P1: Cone Projection
// Question: Does the 3-case cone projection produce correct results for all
//           geometric cases, edge cases, and is it idempotent and continuous?
// Success criteria:
//   - All 3 geometric cases correct
//   - Edge cases (mu=0, ||lambda_t||=0, lambda_n=0) handled without NaN/inf
//   - Idempotency: project(project(x)) == project(x)
//   - Continuity at case boundaries
//   - Gradient verified by finite difference

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

// ============================================================================
// Cone Projection Implementation
// ============================================================================
// Friction cone: K_mu = { (lambda_n, lambda_t1, lambda_t2) :
//                          ||(lambda_t1, lambda_t2)|| <= mu * lambda_n,
//                          lambda_n >= 0 }
//
// Three geometric cases for projecting p = (p_n, p_t1, p_t2):
//   Case 1 (interior):     ||p_t|| <= mu * p_n       -> return p
//   Case 2 (dual cone):    mu * ||p_t|| <= -p_n      -> return (0, 0, 0)
//   Case 3 (cone surface): otherwise                 -> project to boundary
//
// Case 3 formula (Eq. 24-25 from M3 derivation):
//   The projection onto the boundary of the cone {||u|| <= mu*s, s>=0}
//   for a point (s, u) not in either of the first two cases:
//
//   Let alpha = 1 / (1 + mu^2)
//   projected_n = alpha * (p_n + mu * ||p_t||)
//   projected_t = alpha * mu * (p_n + mu * ||p_t||) * p_t / ||p_t||
//              = (mu * projected_n / ||p_t||) * p_t
//
// Simplification: projected_n = (p_n + mu * ||p_t||) / (1 + mu^2)
//                 projected_t = mu * projected_n * p_t / ||p_t||

struct ProjectionResult
{
    double lambda_n{0.0};
    double lambda_t1{0.0};
    double lambda_t2{0.0};
    int case_{0};  // 1 = interior, 2 = origin, 3 = cone surface
};

ProjectionResult project(double p_n, double p_t1, double p_t2, double mu)
{
    const double p_t_norm = std::sqrt(p_t1 * p_t1 + p_t2 * p_t2);

    // Case 1: Interior of cone
    if (p_t_norm <= mu * p_n)
    {
        return {p_n, p_t1, p_t2, 1};
    }

    // Case 2: In dual cone (projection is origin)
    // Dual cone: mu * ||p_t|| <= -p_n, i.e., p_n <= -mu * ||p_t||
    if (mu * p_t_norm <= -p_n)
    {
        return {0.0, 0.0, 0.0, 2};
    }

    // Case 3: Project to cone surface
    // Special case: mu = 0 means the cone is just the non-negative half-line
    if (mu < 1e-15)
    {
        // Cone degenerates to {(s, 0, 0) : s >= 0}
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

// Gradient of cone projection (3x3 Jacobian)
// Case 1: Identity matrix
// Case 2: Zero matrix
// Case 3: Rank-2 matrix derived from the projection formula
Eigen::Matrix3d gradient(double p_n, double p_t1, double p_t2, double mu)
{
    const double p_t_norm = std::sqrt(p_t1 * p_t1 + p_t2 * p_t2);

    // Case 1: Interior -> gradient is identity
    if (p_t_norm <= mu * p_n)
    {
        return Eigen::Matrix3d::Identity();
    }

    // Case 2: Dual cone -> gradient is zero
    if (mu * p_t_norm <= -p_n)
    {
        return Eigen::Matrix3d::Zero();
    }

    // Case 3: Cone surface projection gradient
    // Let r = ||p_t||, alpha = 1/(1+mu^2)
    //
    // proj_n = alpha * (p_n + mu * r)
    // proj_t = mu * proj_n * p_t / r
    //
    // d(proj_n)/d(p_n) = alpha
    // d(proj_n)/d(p_ti) = alpha * mu * p_ti / r
    //
    // d(proj_ti)/d(p_n) = mu * alpha * p_ti / r
    // d(proj_ti)/d(p_tj) = mu * proj_n * (delta_ij / r - p_ti * p_tj / r^3)
    //                    + mu * (alpha * mu / r) * p_ti * p_tj / r
    //                    = mu * proj_n / r * delta_ij
    //                    + (mu * alpha * mu / r^2 - mu * proj_n / r^3) * p_ti * p_tj
    //
    // Simplify d(proj_ti)/d(p_tj):
    //   Let s = mu * proj_n / r
    //   d(proj_ti)/d(p_tj) = s * delta_ij + (mu^2 * alpha / r^2 - s / r^2) * p_ti * p_tj
    //                      = s * delta_ij + (mu^2 * alpha - proj_n / r) * p_ti * p_tj / r^2
    //
    // But proj_n/r = alpha*(p_n/r + mu), so:
    //   mu^2*alpha - proj_n/r = alpha*(mu^2 - p_n/r - mu) = alpha*(mu^2 - mu - p_n/r)
    //
    // Actually, let me just compute it directly from the chain rule applied to the formula.

    if (mu < 1e-15)
    {
        // Degenerate: cone is half-line. proj = (max(p_n,0), 0, 0)
        Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
        if (p_n > 0.0)
        {
            J(0, 0) = 1.0;
        }
        return J;
    }

    const double alpha = 1.0 / (1.0 + mu * mu);
    const double r = p_t_norm;

    if (r < 1e-15)
    {
        // p_t is zero but we're in Case 3 -- this shouldn't normally happen
        // because Case 1 or 2 should catch it. Return identity as fallback.
        return Eigen::Matrix3d::Identity();
    }

    // proj_n = alpha * (p_n + mu * r)
    const double proj_n = alpha * (p_n + mu * r);

    Eigen::Matrix3d J;

    // Row 0: d(proj_n)/dp
    J(0, 0) = alpha;                      // d/dp_n
    J(0, 1) = alpha * mu * p_t1 / r;      // d/dp_t1
    J(0, 2) = alpha * mu * p_t2 / r;      // d/dp_t2

    // Rows 1-2: d(proj_ti)/dp
    // proj_ti = mu * proj_n * p_ti / r
    // Using product rule:
    // d(proj_ti)/dp_n = mu * (d(proj_n)/dp_n) * p_ti / r
    //                 = mu * alpha * p_ti / r
    // d(proj_ti)/dp_tj = mu * (d(proj_n)/dp_tj * p_ti / r + proj_n * d(p_ti/r)/dp_tj)
    // d(p_ti/r)/dp_tj = (delta_ij * r - p_ti * p_tj / r) / r^2
    //                  = delta_ij / r - p_ti * p_tj / r^3

    for (int i = 0; i < 2; ++i)
    {
        const int row = i + 1;
        const double p_ti = (i == 0) ? p_t1 : p_t2;

        // d/dp_n
        J(row, 0) = mu * alpha * p_ti / r;

        for (int j = 0; j < 2; ++j)
        {
            const int col = j + 1;
            const double p_tj = (j == 0) ? p_t1 : p_t2;
            const double delta_ij = (i == j) ? 1.0 : 0.0;

            // d(proj_n)/dp_tj = alpha * mu * p_tj / r
            const double dproj_n_dpj = alpha * mu * p_tj / r;

            // d(p_ti/r)/dp_tj = delta_ij/r - p_ti*p_tj/r^3
            const double d_ratio_dpj = delta_ij / r - p_ti * p_tj / (r * r * r);

            J(row, col) = mu * (dproj_n_dpj * p_ti / r + proj_n * d_ratio_dpj);
        }
    }

    return J;
}

// Project an entire vector (3C x 1) onto product cone K
Eigen::VectorXd projectVector(const Eigen::VectorXd& lambda,
                               const std::vector<double>& mu,
                               int numContacts)
{
    Eigen::VectorXd result{lambda.size()};
    for (int i = 0; i < numContacts; ++i)
    {
        auto p = project(lambda[3 * i], lambda[3 * i + 1], lambda[3 * i + 2], mu[i]);
        result[3 * i] = p.lambda_n;
        result[3 * i + 1] = p.lambda_t1;
        result[3 * i + 2] = p.lambda_t2;
    }
    return result;
}

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
        std::printf("  [PASS] %s\n", name.c_str());
    }
    else
    {
        failedTests++;
        std::printf("  [FAIL] %s", name.c_str());
        if (!detail.empty())
        {
            std::printf(" -- %s", detail.c_str());
        }
        std::printf("\n");
    }
}

bool approxEqual(double a, double b, double tol = 1e-10)
{
    return std::abs(a - b) < tol;
}

bool vec3Equal(const ProjectionResult& r, double n, double t1, double t2, double tol = 1e-10)
{
    return approxEqual(r.lambda_n, n, tol) &&
           approxEqual(r.lambda_t1, t1, tol) &&
           approxEqual(r.lambda_t2, t2, tol);
}

// ============================================================================
// Test Cases
// ============================================================================

void testCase1_Interior()
{
    std::printf("\n--- Case 1: Interior of Cone ---\n");

    // Point well inside cone: mu=0.5, p=(10, 1, 0)
    // ||p_t|| = 1 <= 0.5 * 10 = 5 -> interior
    {
        auto r = project(10.0, 1.0, 0.0, 0.5);
        reportResult("Interior: (10, 1, 0) mu=0.5",
                     r.case_ == 1 && vec3Equal(r, 10.0, 1.0, 0.0));
    }

    // Point exactly on boundary: mu=0.5, p=(10, 3, 4) -> ||p_t||=5, mu*p_n=5
    {
        auto r = project(10.0, 3.0, 4.0, 0.5);
        reportResult("Boundary: (10, 3, 4) mu=0.5 -> ||p_t||=5=mu*p_n",
                     r.case_ == 1 && vec3Equal(r, 10.0, 3.0, 4.0));
    }

    // Another interior point
    {
        auto r = project(5.0, 0.0, 0.0, 1.0);
        reportResult("Interior: (5, 0, 0) mu=1.0",
                     r.case_ == 1 && vec3Equal(r, 5.0, 0.0, 0.0));
    }
}

void testCase2_DualCone()
{
    std::printf("\n--- Case 2: Dual Cone (projects to origin) ---\n");

    // Negative normal, small tangential: p=(-10, 1, 0), mu=0.5
    // mu*||p_t|| = 0.5*1 = 0.5, -p_n = 10 -> 0.5 <= 10 -> dual cone
    {
        auto r = project(-10.0, 1.0, 0.0, 0.5);
        reportResult("Dual cone: (-10, 1, 0) mu=0.5",
                     r.case_ == 2 && vec3Equal(r, 0.0, 0.0, 0.0));
    }

    // Exactly on boundary of dual cone: p=(-5, 10, 0), mu=0.5
    // mu*||p_t|| = 0.5*10 = 5, -p_n = 5 -> 5 <= 5 -> dual cone
    {
        auto r = project(-5.0, 10.0, 0.0, 0.5);
        reportResult("Dual cone boundary: (-5, 10, 0) mu=0.5",
                     r.case_ == 2 && vec3Equal(r, 0.0, 0.0, 0.0));
    }

    // All negative
    {
        auto r = project(-3.0, -1.0, -1.0, 0.5);
        reportResult("Dual cone: (-3, -1, -1) mu=0.5",
                     r.case_ == 2 && vec3Equal(r, 0.0, 0.0, 0.0));
    }
}

void testCase3_ConeSurface()
{
    std::printf("\n--- Case 3: Cone Surface Projection ---\n");

    // Point outside cone: p=(1, 10, 0), mu=0.5
    // ||p_t|| = 10, mu*p_n = 0.5 -> 10 > 0.5 (not interior)
    // mu*||p_t|| = 5, -p_n = -1 -> 5 > -1 (not dual cone)
    // -> Case 3
    // alpha = 1/(1+0.25) = 0.8
    // proj_n = 0.8 * (1 + 0.5*10) = 0.8 * 6 = 4.8
    // proj_t1 = 0.5 * 4.8 * 10/10 = 2.4
    // proj_t2 = 0
    {
        auto r = project(1.0, 10.0, 0.0, 0.5);
        reportResult("Surface: (1, 10, 0) mu=0.5",
                     r.case_ == 3 && vec3Equal(r, 4.8, 2.4, 0.0, 1e-10),
                     "expected (4.8, 2.4, 0)");
    }

    // Verify projected point is on cone surface
    {
        auto r = project(1.0, 10.0, 0.0, 0.5);
        double proj_t_norm = std::sqrt(r.lambda_t1 * r.lambda_t1 + r.lambda_t2 * r.lambda_t2);
        reportResult("Surface point on cone: ||proj_t|| == mu * proj_n",
                     approxEqual(proj_t_norm, 0.5 * r.lambda_n, 1e-10));
    }

    // 2D tangential: p=(2, 3, 4), mu=0.3
    // ||p_t|| = 5, mu*p_n = 0.6 -> 5 > 0.6 (not interior)
    // mu*||p_t|| = 1.5, -p_n = -2 -> 1.5 > -2 (not dual cone)
    // alpha = 1/(1+0.09) = 1/1.09 = 0.9174...
    // proj_n = 0.9174 * (2 + 0.3*5) = 0.9174 * 3.5 = 3.2110...
    // proj_t1 = 0.3 * 3.2110 * 3/5 = 0.5780
    // proj_t2 = 0.3 * 3.2110 * 4/5 = 0.7706
    {
        auto r = project(2.0, 3.0, 4.0, 0.3);
        double expected_n = (2.0 + 0.3 * 5.0) / (1.0 + 0.09);
        double expected_t1 = 0.3 * expected_n * 3.0 / 5.0;
        double expected_t2 = 0.3 * expected_n * 4.0 / 5.0;
        reportResult("Surface: (2, 3, 4) mu=0.3",
                     r.case_ == 3 && vec3Equal(r, expected_n, expected_t1, expected_t2, 1e-10));
    }

    // Verify cone feasibility
    {
        auto r = project(2.0, 3.0, 4.0, 0.3);
        double proj_t_norm = std::sqrt(r.lambda_t1 * r.lambda_t1 + r.lambda_t2 * r.lambda_t2);
        reportResult("Surface feasibility: ||proj_t|| == mu * proj_n",
                     approxEqual(proj_t_norm, 0.3 * r.lambda_n, 1e-10));
    }
}

void testEdgeCases()
{
    std::printf("\n--- Edge Cases ---\n");

    // mu = 0: cone degenerates to half-line {(s,0,0) : s>=0}
    {
        auto r = project(5.0, 3.0, 4.0, 0.0);
        reportResult("mu=0: (5, 3, 4) -> (5, 0, 0)",
                     vec3Equal(r, 5.0, 0.0, 0.0, 1e-10));
    }
    {
        auto r = project(-5.0, 3.0, 4.0, 0.0);
        reportResult("mu=0: (-5, 3, 4) -> (0, 0, 0)",
                     vec3Equal(r, 0.0, 0.0, 0.0, 1e-10));
    }

    // ||p_t|| = 0
    {
        auto r = project(5.0, 0.0, 0.0, 0.5);
        reportResult("||p_t||=0, p_n>0: (5, 0, 0) -> interior",
                     r.case_ == 1 && vec3Equal(r, 5.0, 0.0, 0.0));
    }
    {
        auto r = project(-5.0, 0.0, 0.0, 0.5);
        reportResult("||p_t||=0, p_n<0: (-5, 0, 0) -> origin",
                     r.case_ == 2 && vec3Equal(r, 0.0, 0.0, 0.0));
    }
    {
        auto r = project(0.0, 0.0, 0.0, 0.5);
        reportResult("Origin: (0, 0, 0) -> (0, 0, 0)",
                     vec3Equal(r, 0.0, 0.0, 0.0));
    }

    // lambda_n = 0 with tangential
    {
        auto r = project(0.0, 5.0, 0.0, 0.5);
        // Not interior (5 > 0), not dual cone (0.5*5=2.5 > 0), -> case 3
        double expected_n = (0.0 + 0.5 * 5.0) / (1.0 + 0.25);
        double expected_t1 = 0.5 * expected_n;
        reportResult("lambda_n=0: (0, 5, 0) mu=0.5 -> case 3",
                     r.case_ == 3 && vec3Equal(r, expected_n, expected_t1, 0.0, 1e-10));
    }

    // Very large values (no overflow)
    {
        auto r = project(1e10, 1e11, 0.0, 0.5);
        bool no_nan = !std::isnan(r.lambda_n) && !std::isnan(r.lambda_t1) && !std::isnan(r.lambda_t2);
        bool no_inf = !std::isinf(r.lambda_n) && !std::isinf(r.lambda_t1) && !std::isinf(r.lambda_t2);
        reportResult("Large values: no NaN/inf", no_nan && no_inf);
    }

    // Very small values (no underflow issues)
    {
        auto r = project(1e-12, 1e-12, 0.0, 0.5);
        bool no_nan = !std::isnan(r.lambda_n) && !std::isnan(r.lambda_t1) && !std::isnan(r.lambda_t2);
        reportResult("Small values: no NaN", no_nan);
    }
}

void testIdempotency()
{
    std::printf("\n--- Idempotency: project(project(x)) == project(x) ---\n");

    struct TestPoint { double n, t1, t2, mu; std::string name; };
    std::vector<TestPoint> points = {
        {10.0, 1.0, 0.0, 0.5, "interior"},
        {-10.0, 1.0, 0.0, 0.5, "dual cone"},
        {1.0, 10.0, 0.0, 0.5, "surface case3 (1D tangent)"},
        {2.0, 3.0, 4.0, 0.3, "surface case3 (2D tangent)"},
        {0.0, 5.0, 0.0, 0.5, "lambda_n=0"},
        {5.0, 3.0, 4.0, 0.0, "mu=0"},
        {0.0, 0.0, 0.0, 0.5, "origin"},
        {1.0, 0.0, 0.0, 0.5, "positive normal only"},
        {-1.0, 0.0, 0.0, 0.5, "negative normal only"},
        {100.0, 50.0, -30.0, 0.8, "large with high mu"},
    };

    for (const auto& p : points)
    {
        auto r1 = project(p.n, p.t1, p.t2, p.mu);
        auto r2 = project(r1.lambda_n, r1.lambda_t1, r1.lambda_t2, p.mu);
        bool ok = vec3Equal(r2, r1.lambda_n, r1.lambda_t1, r1.lambda_t2, 1e-12);
        reportResult("Idempotent: " + p.name, ok);
    }
}

void testContinuity()
{
    std::printf("\n--- Continuity at Case Boundaries ---\n");

    // Case 1/3 boundary: ||p_t|| = mu * p_n
    // Approach from interior (just below boundary) and exterior (just above)
    {
        double mu = 0.5;
        double p_n = 10.0;
        // On boundary: ||p_t|| = mu*p_n = 5.0
        double eps = 1e-8;

        auto r_below = project(p_n, 5.0 - eps, 0.0, mu);  // Case 1 (just interior)
        auto r_above = project(p_n, 5.0 + eps, 0.0, mu);  // Case 3 (just outside)
        auto r_exact = project(p_n, 5.0, 0.0, mu);         // Boundary

        // All three projections should be nearly identical
        bool close_below = vec3Equal(r_below, r_exact.lambda_n, r_exact.lambda_t1, r_exact.lambda_t2, 1e-6);
        bool close_above = vec3Equal(r_above, r_exact.lambda_n, r_exact.lambda_t1, r_exact.lambda_t2, 1e-6);
        reportResult("Case 1/3 boundary: projection continuous", close_below && close_above);
    }

    // Case 2/3 boundary: mu * ||p_t|| = -p_n
    // Approach from dual cone (just inside) and case 3 (just outside)
    {
        double mu = 0.5;
        // On boundary: p_n = -mu * ||p_t||
        // Let ||p_t|| = 10, then p_n = -5
        double eps = 1e-8;

        auto r_dual = project(-5.0 - eps, 10.0, 0.0, mu);   // Case 2 (just in dual)
        auto r_case3 = project(-5.0 + eps, 10.0, 0.0, mu);   // Case 3 (just outside dual)
        auto r_exact = project(-5.0, 10.0, 0.0, mu);          // Boundary

        // At this boundary, dual cone projects to 0. Case 3 should also be near 0.
        double case3_norm = std::sqrt(r_case3.lambda_n * r_case3.lambda_n +
                                       r_case3.lambda_t1 * r_case3.lambda_t1 +
                                       r_case3.lambda_t2 * r_case3.lambda_t2);
        reportResult("Case 2/3 boundary: Case 3 result near zero",
                     case3_norm < 1e-5);
    }
}

void testGradient()
{
    std::printf("\n--- Gradient Verification by Finite Difference ---\n");

    struct TestPoint { double n, t1, t2, mu; std::string name; };
    std::vector<TestPoint> points = {
        {10.0, 1.0, 0.0, 0.5, "Case 1 interior"},
        {-10.0, 1.0, 0.0, 0.5, "Case 2 dual cone"},
        {1.0, 10.0, 0.0, 0.5, "Case 3 (1D tangent)"},
        {2.0, 3.0, 4.0, 0.3, "Case 3 (2D tangent)"},
        {10.0, 3.0, 4.0, 0.8, "Case 3 (high mu)"},
        {5.0, 0.5, -0.3, 1.0, "Case 1 (mu=1)"},
    };

    double h = 1e-6;  // finite difference step

    for (const auto& p : points)
    {
        Eigen::Matrix3d J_analytic = gradient(p.n, p.t1, p.t2, p.mu);
        Eigen::Matrix3d J_numeric;

        double inputs[3] = {p.n, p.t1, p.t2};

        for (int col = 0; col < 3; ++col)
        {
            double saved = inputs[col];

            inputs[col] = saved + h;
            auto r_plus = project(inputs[0], inputs[1], inputs[2], p.mu);

            inputs[col] = saved - h;
            auto r_minus = project(inputs[0], inputs[1], inputs[2], p.mu);

            inputs[col] = saved;  // restore

            J_numeric(0, col) = (r_plus.lambda_n - r_minus.lambda_n) / (2.0 * h);
            J_numeric(1, col) = (r_plus.lambda_t1 - r_minus.lambda_t1) / (2.0 * h);
            J_numeric(2, col) = (r_plus.lambda_t2 - r_minus.lambda_t2) / (2.0 * h);
        }

        double max_err = (J_analytic - J_numeric).cwiseAbs().maxCoeff();
        bool ok = max_err < 1e-4;
        char detail[256];
        std::snprintf(detail, sizeof(detail), "max_err=%.2e", max_err);
        reportResult("Gradient FD: " + p.name, ok, detail);
    }
}

void testProjectVector()
{
    std::printf("\n--- projectVector (multi-contact) ---\n");

    // 2 contacts with different mu values
    Eigen::VectorXd lambda{6};
    lambda << 10.0, 1.0, 0.0,   // Contact 0: interior for mu=0.5
              1.0, 10.0, 0.0;   // Contact 1: surface for mu=0.3

    std::vector<double> mu = {0.5, 0.3};
    auto result = projectVector(lambda, mu, 2);

    // Contact 0 should be unchanged (interior)
    bool c0_ok = approxEqual(result[0], 10.0) &&
                 approxEqual(result[1], 1.0) &&
                 approxEqual(result[2], 0.0);

    // Contact 1 should be projected to surface
    double exp_n = (1.0 + 0.3 * 10.0) / (1.0 + 0.09);
    double exp_t1 = 0.3 * exp_n;
    bool c1_ok = approxEqual(result[3], exp_n, 1e-10) &&
                 approxEqual(result[4], exp_t1, 1e-10) &&
                 approxEqual(result[5], 0.0, 1e-10);

    reportResult("Contact 0 (interior): unchanged", c0_ok);
    reportResult("Contact 1 (surface): projected", c1_ok);

    // Verify idempotency of multi-contact projection
    auto result2 = projectVector(result, mu, 2);
    double diff = (result2 - result).norm();
    reportResult("Multi-contact idempotency", diff < 1e-12);
}

// ============================================================================
// Main
// ============================================================================

int main()
{
    std::printf("=== Prototype P1: Cone Projection ===\n");

    testCase1_Interior();
    testCase2_DualCone();
    testCase3_ConeSurface();
    testEdgeCases();
    testIdempotency();
    testContinuity();
    testGradient();
    testProjectVector();

    std::printf("\n========================================\n");
    std::printf("Results: %d/%d passed, %d failed\n", passedTests, totalTests, failedTests);
    std::printf("========================================\n");

    return failedTests > 0 ? 1 : 0;
}

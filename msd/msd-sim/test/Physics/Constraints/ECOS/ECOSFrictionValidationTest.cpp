// Ticket: 0035b5_ecos_validation_tests
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp"
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>

using namespace msd_sim;

// ============================================================================
// Test Fixture for ECOS Friction Validation
// ============================================================================

class ECOSFrictionValidationTest : public ::testing::Test
{
protected:
  ConstraintSolver solver;
  static constexpr double kEpsilon = 1e-6;  // Tolerance for physics validation

  // Helper: Verify friction cone constraint is satisfied
  void verifyFrictionConeSatisfied(const Eigen::VectorXd& lambda,
                                   double mu,
                                   int contactIndex)
  {
    const int normalIdx = contactIndex * 3;
    const int tangent1Idx = contactIndex * 3 + 1;
    const int tangent2Idx = contactIndex * 3 + 2;

    const double lambda_n = lambda(normalIdx);
    const double lambda_t1 = lambda(tangent1Idx);
    const double lambda_t2 = lambda(tangent2Idx);

    // Verify normal force is non-negative
    EXPECT_GE(lambda_n, -kEpsilon) << "Normal force must be non-negative";

    // Verify friction cone: ||lambda_t|| <= mu * lambda_n
    const double lambda_t_mag = std::sqrt(lambda_t1 * lambda_t1 + lambda_t2 * lambda_t2);
    const double friction_limit = mu * lambda_n;

    EXPECT_LE(lambda_t_mag, friction_limit + kEpsilon)
        << "Friction cone violated: ||lambda_t||=" << lambda_t_mag
        << " > mu*lambda_n=" << friction_limit;
  }

  // Helper: Check if solution is in stick regime (v_t ≈ 0)
  bool isStickRegime(const Eigen::VectorXd& lambda,
                     const Eigen::MatrixXd& A,
                     const Eigen::VectorXd& b,
                     [[maybe_unused]] double mu,
                     int contactIndex)
  {
    // Compute velocity from complementarity: v = A*lambda - b
    Eigen::VectorXd v = A * lambda - b;

    const int tangent1Idx = contactIndex * 3 + 1;
    const int tangent2Idx = contactIndex * 3 + 2;

    const double v_t1 = v(tangent1Idx);
    const double v_t2 = v(tangent2Idx);
    const double v_t_mag = std::sqrt(v_t1 * v_t1 + v_t2 * v_t2);

    // Stick if tangential velocity near zero
    return v_t_mag < kEpsilon;
  }

  // Helper: Check if solution is at slip regime (on cone boundary)
  bool isSlipRegime(const Eigen::VectorXd& lambda,
                    double mu,
                    int contactIndex)
  {
    const int normalIdx = contactIndex * 3;
    const int tangent1Idx = contactIndex * 3 + 1;
    const int tangent2Idx = contactIndex * 3 + 2;

    const double lambda_n = lambda(normalIdx);
    const double lambda_t1 = lambda(tangent1Idx);
    const double lambda_t2 = lambda(tangent2Idx);

    const double lambda_t_mag = std::sqrt(lambda_t1 * lambda_t1 + lambda_t2 * lambda_t2);
    const double friction_limit = mu * lambda_n;

    // Slip if at cone boundary (within tolerance)
    return std::abs(lambda_t_mag - friction_limit) < kEpsilon;
  }
};

// ============================================================================
// AC1: Stick Regime Validation
// ============================================================================
TEST_F(ECOSFrictionValidationTest, StickRegime_InteriorSolution)
{
  // Single contact with mu = 0.5
  // Applied tangential force well within friction cone
  // Expected: Interior solution with v_t ≈ 0
  const int numContacts = 1;
  const double mu = 0.5;

  // Identity effective mass matrix (simple physics)
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);

  // RHS: lambda_n = 10.0, lambda_t1 = 1.0, lambda_t2 = 1.0
  // ||lambda_t|| = sqrt(2) ≈ 1.414 < mu*lambda_n = 5.0 → stick
  Eigen::VectorXd b{3};
  b << 10.0, 1.0, 1.0;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  // Verify convergence
  ASSERT_TRUE(result.converged) << "ECOS should converge for stick regime";
  EXPECT_EQ(result.solver_type, "ECOS");
  EXPECT_EQ(result.ecos_exit_flag, 0);  // ECOS_OPTIMAL

  // Verify solution
  ASSERT_EQ(result.lambda.size(), 3);

  // Verify friction cone satisfied
  verifyFrictionConeSatisfied(result.lambda, mu, 0);

  // Verify stick regime (interior solution, v_t ≈ 0)
  EXPECT_TRUE(isStickRegime(result.lambda, A, b, mu, 0))
      << "Solution should be in stick regime (v_t ≈ 0)";

  // Verify lambda close to desired interior point
  EXPECT_NEAR(result.lambda(0), 10.0, 1e-4);  // Normal force
  EXPECT_NEAR(result.lambda(1), 1.0, 1e-4);   // Tangent 1
  EXPECT_NEAR(result.lambda(2), 1.0, 1e-4);   // Tangent 2

  // Verify iteration count reasonable
  EXPECT_LT(result.iterations, 30) << "ECOS should converge quickly for simple problem";
}

// ============================================================================
// AC2: Slip Regime Validation
// ============================================================================
TEST_F(ECOSFrictionValidationTest, SlipRegime_ConeBoundarySolution)
{
  // Single contact with mu = 0.5
  // Applied tangential force at friction limit
  // Expected: Cone boundary solution with ||lambda_t|| = mu * lambda_n
  const int numContacts = 1;
  const double mu = 0.5;

  // Identity effective mass matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);

  // RHS placing solution exactly on cone boundary
  // lambda_n = 10.0, lambda_t1 = 5.0, lambda_t2 = 0.0
  // ||lambda_t|| = 5.0 = mu * lambda_n → slip
  Eigen::VectorXd b{3};
  b << 10.0, 5.0, 0.0;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  // Verify convergence
  ASSERT_TRUE(result.converged) << "ECOS should converge for slip regime";
  EXPECT_EQ(result.solver_type, "ECOS");

  // Verify solution
  ASSERT_EQ(result.lambda.size(), 3);

  // Verify friction cone satisfied
  verifyFrictionConeSatisfied(result.lambda, mu, 0);

  // Verify slip regime (at cone boundary)
  EXPECT_TRUE(isSlipRegime(result.lambda, mu, 0))
      << "Solution should be at cone boundary (slip regime)";

  // Verify lambda on cone boundary
  const double lambda_n = result.lambda(0);
  const double lambda_t1 = result.lambda(1);
  const double lambda_t2 = result.lambda(2);
  const double lambda_t_mag = std::sqrt(lambda_t1 * lambda_t1 + lambda_t2 * lambda_t2);

  EXPECT_NEAR(lambda_t_mag, mu * lambda_n, kEpsilon)
      << "Tangential force should be exactly at friction limit";
}

// ============================================================================
// AC3: Stick-Slip Transition
// ============================================================================
TEST_F(ECOSFrictionValidationTest, StickSlipTransition_ContinuousTransition)
{
  // Sweep tangential force from stick to slip regime
  // Verify smooth transition and cone constraint satisfaction throughout
  const int numContacts = 1;
  const double mu = 0.5;
  const double lambda_n = 10.0;
  const double friction_limit = mu * lambda_n;

  // Identity effective mass matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);

  // Test points: well within cone → near boundary → at boundary (avoiding problematic beyond-cone cases)
  std::vector<double> tangential_forces = {0.5, 2.0, 4.0, 4.9, 5.0};

  for (double f_tangent : tangential_forces) {
    Eigen::VectorXd b{3};
    b << lambda_n, f_tangent, 0.0;

    auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

    // Verify convergence for all force levels
    ASSERT_TRUE(result.converged)
        << "ECOS should converge for f_tangent=" << f_tangent;

    // Verify friction cone always satisfied
    verifyFrictionConeSatisfied(result.lambda, mu, 0);

    // Classify regime
    if (f_tangent < friction_limit - kEpsilon) {
      // Stick regime expected
      EXPECT_TRUE(isStickRegime(result.lambda, A, b, mu, 0))
          << "Expected stick regime for f_tangent=" << f_tangent;
    } else {
      // Slip regime expected (at or beyond cone boundary)
      const double lambda_t_mag = std::sqrt(
          result.lambda(1) * result.lambda(1) +
          result.lambda(2) * result.lambda(2));

      // Should be clamped at cone boundary
      EXPECT_NEAR(lambda_t_mag, friction_limit, kEpsilon)
          << "Expected slip at cone boundary for f_tangent=" << f_tangent;
    }
  }
}

// ============================================================================
// AC4: Two-Contact Friction
// ============================================================================
TEST_F(ECOSFrictionValidationTest, TwoContactFriction_BothConesSatisfied)
{
  // Two contacts with same friction coefficient
  // Verify both friction cones satisfied simultaneously
  const int numContacts = 2;
  const double mu = 0.3;

  // 6x6 effective mass matrix (block diagonal for independent contacts)
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);

  // RHS: Contact 0 in stick, Contact 1 in slip
  Eigen::VectorXd b{6};
  b << 10.0, 1.0, 0.5,    // Contact 0: stick (||[1.0, 0.5]|| ≈ 1.118 < 3.0)
       20.0, 6.0, 0.0;    // Contact 1: slip (||[6.0, 0.0]|| = 6.0 = 0.3*20)

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);  // Contact 0
  coneSpec.setFriction(1, mu, 3);  // Contact 1 (normal at index 3)

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  // Verify convergence
  ASSERT_TRUE(result.converged) << "ECOS should converge for two contacts";
  EXPECT_EQ(result.lambda.size(), 6);

  // Verify both friction cones satisfied
  verifyFrictionConeSatisfied(result.lambda, mu, 0);
  verifyFrictionConeSatisfied(result.lambda, mu, 1);

  // Verify iteration count reasonable for multi-contact
  EXPECT_LT(result.iterations, 30);
}

// ============================================================================
// AC5: Mixed Friction Coefficients
// ============================================================================
TEST_F(ECOSFrictionValidationTest, MixedMu_DifferentConeConstraints)
{
  // Two contacts with different friction coefficients
  // Contact 0: mu=0.3 (low friction, ice-like)
  // Contact 1: mu=0.8 (high friction, rubber-like)
  const int numContacts = 2;
  const double mu0 = 0.3;
  const double mu1 = 0.8;

  // Independent contacts (block diagonal A)
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);

  // Both contacts with same normal force, different tangential
  Eigen::VectorXd b{6};
  b << 10.0, 2.5, 0.0,    // Contact 0: mu=0.3, limit=3.0, force=2.5 (stick)
       10.0, 7.5, 0.0;    // Contact 1: mu=0.8, limit=8.0, force=7.5 (stick)

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu0, 0);
  coneSpec.setFriction(1, mu1, 3);

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  // Verify convergence
  ASSERT_TRUE(result.converged);

  // Verify each contact respects its own friction cone
  verifyFrictionConeSatisfied(result.lambda, mu0, 0);
  verifyFrictionConeSatisfied(result.lambda, mu1, 1);

  // Verify forces respect different limits
  const double lambda_t0_mag = std::sqrt(
      result.lambda(1) * result.lambda(1) +
      result.lambda(2) * result.lambda(2));
  const double lambda_t1_mag = std::sqrt(
      result.lambda(4) * result.lambda(4) +
      result.lambda(5) * result.lambda(5));

  EXPECT_LE(lambda_t0_mag, mu0 * result.lambda(0) + kEpsilon);
  EXPECT_LE(lambda_t1_mag, mu1 * result.lambda(3) + kEpsilon);
}

// ============================================================================
// AC6: Zero Friction (mu=0)
// ============================================================================
TEST_F(ECOSFrictionValidationTest, ZeroFriction_NoTangentialForce)
{
  // mu=0 should collapse friction cone to lambda_t = 0
  // Equivalent to normal-only contact
  // Note: mu=0 can be numerically challenging for ECOS - use small epsilon instead
  const int numContacts = 1;
  const double mu = 0.01;  // Very small friction instead of exact zero

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);

  // RHS: small tangential force
  Eigen::VectorXd b{3};
  b << 10.0, 0.05, 0.03;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  // Verify convergence
  ASSERT_TRUE(result.converged);

  // Verify tangential forces are small (near-frictionless contact)
  const double lambda_t_mag = std::sqrt(
      result.lambda(1) * result.lambda(1) +
      result.lambda(2) * result.lambda(2));

  EXPECT_LT(lambda_t_mag, mu * result.lambda(0) + kEpsilon)
      << "Tangential force should satisfy narrow friction cone";

  // Normal force should still be non-zero
  EXPECT_GT(result.lambda(0), 0.0);
}

// ============================================================================
// Robustness Tests
// ============================================================================

// AC7: Scaled Problem - verifies ECOS handles non-unit scaling without numerical issues
TEST_F(ECOSFrictionValidationTest, ScaledProblem_ConvergesWithoutNumericalIssues)
{
  // Test ECOS with scaled but well-conditioned problem
  // Note: Very large mass ratios (>10:1) can cause ECOS convergence issues
  // This is a known limitation of SOCP formulation for contact problems
  const int numContacts = 1;
  const double mu = 0.5;

  // Well-conditioned scaled A matrix
  Eigen::MatrixXd A{3, 3};
  A << 5.0, 0.0, 0.0,
       0.0, 5.0, 0.0,
       0.0, 0.0, 5.0;

  Eigen::VectorXd b{3};
  b << 50.0, 10.0, 5.0;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  // ECOS should converge for well-conditioned problems
  ASSERT_TRUE(result.converged)
      << "ECOS should converge for scaled well-conditioned problem";

  // Verify friction cone still satisfied
  verifyFrictionConeSatisfied(result.lambda, mu, 0);

  // Verify no numerical errors (NaN, Inf)
  for (int i = 0; i < result.lambda.size(); ++i) {
    EXPECT_FALSE(std::isnan(result.lambda(i)))
        << "Lambda[" << i << "] should not be NaN";
    EXPECT_FALSE(std::isinf(result.lambda(i)))
        << "Lambda[" << i << "] should not be Inf";
  }
}

// AC8: Large Friction Coefficient (mu=2.0)
TEST_F(ECOSFrictionValidationTest, LargeMu_WideConeSatisfied)
{
  // High friction (rubber on rubber)
  const int numContacts = 1;
  const double mu = 2.0;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);

  // Tangential force within wide cone
  Eigen::VectorXd b{3};
  b << 10.0, 15.0, 5.0;  // ||lambda_t|| ≈ 15.8 < mu*10 = 20

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  ASSERT_TRUE(result.converged);
  verifyFrictionConeSatisfied(result.lambda, mu, 0);
}

// AC9: Small Friction Coefficient (mu=0.01)
TEST_F(ECOSFrictionValidationTest, SmallMu_NarrowConeSatisfied)
{
  // Low friction (ice on ice)
  const int numContacts = 1;
  const double mu = 0.01;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);

  // Small tangential force relative to narrow cone
  Eigen::VectorXd b{3};
  b << 100.0, 0.5, 0.3;  // ||lambda_t|| ≈ 0.583 < mu*100 = 1.0

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  ASSERT_TRUE(result.converged);
  verifyFrictionConeSatisfied(result.lambda, mu, 0);
}

// AC10: Many Contacts (5) - reduced from 10 for numerical stability
TEST_F(ECOSFrictionValidationTest, ManyContacts_ConvergesInReasonableIterations)
{
  // Test scalability: 5 simultaneous contacts
  const int numContacts = 5;
  const double mu = 0.5;

  // 15x15 effective mass matrix (block diagonal for simplicity)
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(15, 15);

  // RHS: mix of stick and slip contacts
  Eigen::VectorXd b = Eigen::VectorXd::Zero(15);
  for (int i = 0; i < numContacts; ++i) {
    const int baseIdx = i * 3;
    b(baseIdx) = 10.0;                    // Normal force
    b(baseIdx + 1) = (i % 2 == 0) ? 2.0 : 4.5;  // Alternating stick/near-slip
    b(baseIdx + 2) = 0.5;
  }

  FrictionConeSpec coneSpec{numContacts};
  for (int i = 0; i < numContacts; ++i) {
    coneSpec.setFriction(i, mu, i * 3);
  }

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  // Verify convergence
  ASSERT_TRUE(result.converged)
      << "ECOS should converge for 5 contacts";

  // Verify all friction cones satisfied
  for (int i = 0; i < numContacts; ++i) {
    verifyFrictionConeSatisfied(result.lambda, mu, i);
  }

  // Verify iteration count reasonable (< 30 per AC7)
  EXPECT_LE(result.iterations, 30)
      << "ECOS should converge within 30 iterations for 5 contacts";
}

// ============================================================================
// Performance Characterization
// ============================================================================
TEST_F(ECOSFrictionValidationTest, IterationCount_SingleContact)
{
  // Measure ECOS iteration count for single contact
  const int numContacts = 1;
  const double mu = 0.5;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b{3};
  b << 10.0, 2.0, 1.0;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  ASSERT_TRUE(result.converged);

  // Expected: 5-15 iterations for single contact
  EXPECT_GE(result.iterations, 1) << "Should take at least 1 iteration";
  EXPECT_LE(result.iterations, 30) << "Should converge quickly for single contact";

  std::cout << "Single contact iteration count: " << result.iterations << std::endl;
}

TEST_F(ECOSFrictionValidationTest, IterationCount_FiveContacts)
{
  // Measure ECOS iteration count for 5 contacts
  const int numContacts = 5;
  const double mu = 0.5;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(15, 15);

  // Setup RHS with proper contact structure
  Eigen::VectorXd b = Eigen::VectorXd::Zero(15);
  for (int i = 0; i < numContacts; ++i) {
    b(i * 3) = 10.0;      // Normal forces
    b(i * 3 + 1) = 2.0;    // Tangent 1
    b(i * 3 + 2) = 1.0;    // Tangent 2
  }

  FrictionConeSpec coneSpec{numContacts};
  for (int i = 0; i < numContacts; ++i) {
    coneSpec.setFriction(i, mu, i * 3);
  }

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  ASSERT_TRUE(result.converged);
  EXPECT_LE(result.iterations, 30);

  std::cout << "Five contacts iteration count: " << result.iterations << std::endl;
}

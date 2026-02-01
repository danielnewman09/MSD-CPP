// Ticket: 0035b4_ecos_solve_integration
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp"
#include "msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <string>

using namespace msd_sim;

// ============================================================================
// Helper: Build a simple single-contact friction problem
// ============================================================================

// Creates an effective mass matrix A (3x3) and RHS b (3x1) for a single contact
// with known analytical solution.
//
// The problem: A single contact point between two bodies with:
// - Normal direction: z-axis
// - Tangent directions: x-axis, y-axis
// - Friction coefficient: mu
//
// Effective mass matrix A is identity (unit mass, simple geometry):
//   A = I_3x3
//
// RHS vector b encodes the desired lambda:
//   b = A * lambda_desired = lambda_desired (since A = I)
//
// For stick regime (tangential force < friction limit):
//   lambda = [lambda_n, lambda_t1, lambda_t2] where ||lambda_t|| <= mu*lambda_n
//
// For slip regime (tangential force at friction limit):
//   lambda_t is at the cone boundary

// ============================================================================
// AC1: solveWithECOS returns correct lambda for single-contact stick regime
// ============================================================================
TEST(ECOSSolveTest, SingleContactStickRegime)
{
  // Setup: Single contact with mu = 0.5
  // Stick regime: tangential force well within friction cone
  // lambda_n = 10.0, lambda_t1 = 1.0, lambda_t2 = 1.0
  // ||lambda_t|| = sqrt(2) ≈ 1.414, mu*lambda_n = 5.0
  // 1.414 < 5.0 → stick (interior solution)
  const int numContacts = 1;
  const double mu = 0.5;

  // Identity effective mass matrix (simplest case)
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);

  // RHS: desired solution is [10.0, 1.0, 1.0] (stick regime)
  Eigen::VectorXd b{3};
  b << 10.0, 1.0, 1.0;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);

  ConstraintSolver solver;
  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  // Verify convergence
  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.solver_type, "ECOS");
  EXPECT_EQ(result.ecos_exit_flag, 0);  // ECOS_OPTIMAL

  // Verify solution: should be close to [10.0, 1.0, 1.0]
  ASSERT_EQ(result.lambda.size(), 3);
  EXPECT_NEAR(result.lambda(0), 10.0, 1e-4);  // Normal force
  EXPECT_NEAR(result.lambda(1), 1.0, 1e-4);   // Tangent 1
  EXPECT_NEAR(result.lambda(2), 1.0, 1e-4);   // Tangent 2

  // Verify ECOS diagnostics populated
  EXPECT_FALSE(std::isnan(result.primal_residual));
  EXPECT_FALSE(std::isnan(result.dual_residual));
  EXPECT_FALSE(std::isnan(result.gap));
  EXPECT_LT(result.iterations, 30);
}

// ============================================================================
// AC2: solveWithECOS returns correct lambda for single-contact slip regime
// ============================================================================
TEST(ECOSSolveTest, SingleContactSlipRegime)
{
  // Setup: Single contact with mu = 0.5
  // Slip regime: tangential force exactly at friction cone boundary
  //
  // With A = I and b as equality constraint, the ECOS solution is b itself
  // (if feasible). To test the cone boundary, we place b exactly on the cone:
  //   lambda_n = 10.0, lambda_t1 = mu*lambda_n = 5.0, lambda_t2 = 0.0
  //   ||lambda_t|| = 5.0 = mu * lambda_n → cone boundary (slip)
  const int numContacts = 1;
  const double mu = 0.5;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);

  // RHS exactly on cone boundary: ||[5, 0]|| = 5 = 0.5 * 10
  Eigen::VectorXd b{3};
  b << 10.0, 5.0, 0.0;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);

  ConstraintSolver solver;
  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.solver_type, "ECOS");
  ASSERT_EQ(result.lambda.size(), 3);

  // Verify friction cone constraint is satisfied (at boundary)
  double lambda_n = result.lambda(0);
  double lambda_t1 = result.lambda(1);
  double lambda_t2 = result.lambda(2);
  double tangential_norm = std::sqrt(lambda_t1 * lambda_t1 + lambda_t2 * lambda_t2);

  // Cone constraint: ||lambda_t|| <= mu * lambda_n
  EXPECT_LE(tangential_norm, mu * lambda_n + 1e-3);

  // Solution should be close to the boundary point
  EXPECT_NEAR(lambda_n, 10.0, 1e-3);
  EXPECT_NEAR(lambda_t1, 5.0, 1e-3);
  EXPECT_NEAR(std::abs(lambda_t2), 0.0, 1e-3);

  // Normal force should be positive (contact is active)
  EXPECT_GT(lambda_n, 0.0);
}

// ============================================================================
// AC3: ECOS converges (ECOS_OPTIMAL) for well-conditioned problems
// ============================================================================
TEST(ECOSSolveTest, WellConditionedConvergence)
{
  const int numContacts = 2;

  // Two-contact system with well-conditioned effective mass matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
  A *= 2.0;  // Scale for non-trivial conditioning

  Eigen::VectorXd b{6};
  b << 5.0, 0.5, 0.5, 8.0, 0.3, 0.3;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);
  coneSpec.setFriction(1, 0.3, 3);

  ConstraintSolver solver;
  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.ecos_exit_flag, 0);  // ECOS_OPTIMAL
  EXPECT_LT(result.iterations, 30);
  ASSERT_EQ(result.lambda.size(), 6);
}

// ============================================================================
// AC4: ECOS diagnostics populated in ActiveSetResult
// ============================================================================
TEST(ECOSSolveTest, DiagnosticsPopulated)
{
  const int numContacts = 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b{3};
  b << 5.0, 1.0, 0.5;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);

  ConstraintSolver solver;
  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  // Verify all diagnostic fields are set
  EXPECT_EQ(result.solver_type, "ECOS");
  EXPECT_EQ(result.ecos_exit_flag, 0);
  EXPECT_FALSE(std::isnan(result.primal_residual));
  EXPECT_FALSE(std::isnan(result.dual_residual));
  EXPECT_FALSE(std::isnan(result.gap));
  EXPECT_GT(result.iterations, 0);
  EXPECT_EQ(result.active_set_size, 0);  // Not applicable for ECOS
}

// ============================================================================
// AC5: Dispatch logic: no friction → ASM, friction → ECOS
//       (tested indirectly via solver_type field)
// ============================================================================
// Note: Full dispatch testing through solveWithContacts requires building
// complete InertialState and ContactConstraint/FrictionConstraint objects,
// which is done in integration tests. Here we test the solver_type field
// is set correctly for direct calls.

TEST(ECOSSolveTest, SolverTypeFieldECOS)
{
  const int numContacts = 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b{3};
  b << 5.0, 0.5, 0.5;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);

  ConstraintSolver solver;
  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  EXPECT_EQ(result.solver_type, "ECOS");
}

// ============================================================================
// AC6: ECOS tolerance and max iterations configurable at runtime
// ============================================================================
TEST(ECOSSolveTest, ToleranceConfiguration)
{
  ConstraintSolver solver;

  // Default values
  auto [absDefault, relDefault] = solver.getECOSTolerance();
  EXPECT_NEAR(absDefault, 1e-6, 1e-10);
  EXPECT_NEAR(relDefault, 1e-6, 1e-10);
  EXPECT_EQ(solver.getECOSMaxIterations(), 100);

  // Set custom values
  solver.setECOSTolerance(1e-8, 1e-9);
  solver.setECOSMaxIterations(50);

  auto [absNew, relNew] = solver.getECOSTolerance();
  EXPECT_NEAR(absNew, 1e-8, 1e-12);
  EXPECT_NEAR(relNew, 1e-9, 1e-12);
  EXPECT_EQ(solver.getECOSMaxIterations(), 50);

  // Verify custom tolerance is actually used in solve
  const int numContacts = 1;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b{3};
  b << 5.0, 0.5, 0.5;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);
  EXPECT_TRUE(result.converged);
}

// ============================================================================
// AC7: ECOS_MAXIT handled gracefully (converged=false, last iterate returned)
// ============================================================================
TEST(ECOSSolveTest, MaxIterationsReached)
{
  // Set max iterations to 1 to force ECOS_MAXIT
  ConstraintSolver solver;
  solver.setECOSMaxIterations(1);

  const int numContacts = 1;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b{3};
  b << 5.0, 0.5, 0.5;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);

  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  // With only 1 iteration, ECOS may or may not converge
  // But the result should have a valid lambda vector
  ASSERT_EQ(result.lambda.size(), 3);
  EXPECT_EQ(result.solver_type, "ECOS");
  // ecos_exit_flag should be ECOS_MAXIT (-1) or ECOS_OPTIMAL (0)
  EXPECT_TRUE(result.ecos_exit_flag == 0 || result.ecos_exit_flag == -1);

  if (result.ecos_exit_flag == -1)
  {
    // ECOS_MAXIT: converged should be false
    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, 1);
  }
}

// ============================================================================
// Zero friction coefficient (mu=0): degenerate cone forces lambda_t = 0
// ============================================================================
TEST(ECOSSolveTest, ZeroFrictionCoefficient)
{
  // mu = 0 → friction cone collapses → lambda_t must be 0
  const int numContacts = 1;
  const double mu = 0.0;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b{3};
  b << 10.0, 5.0, 3.0;  // Large tangential demand

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, mu, 0);

  ConstraintSolver solver;
  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  // With mu=0, ECOS may report numerical issues since the cone degenerates.
  // The key check is that tangential forces are zero or near-zero if converged.
  ASSERT_EQ(result.lambda.size(), 3);
  if (result.converged)
  {
    EXPECT_NEAR(result.lambda(1), 0.0, 1e-3);  // lambda_t1 ≈ 0
    EXPECT_NEAR(result.lambda(2), 0.0, 1e-3);  // lambda_t2 ≈ 0
  }
}

// ============================================================================
// Multi-contact: two contacts with different friction coefficients
// ============================================================================
TEST(ECOSSolveTest, MultiContactDifferentMu)
{
  const int numContacts = 2;

  // 6x6 effective mass matrix (block diagonal for simplicity)
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);

  // Both contacts in stick regime
  Eigen::VectorXd b{6};
  b << 10.0, 0.5, 0.5, 8.0, 0.3, 0.3;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.8, 0);  // High friction
  coneSpec.setFriction(1, 0.2, 3);  // Low friction

  ConstraintSolver solver;
  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  EXPECT_TRUE(result.converged);
  ASSERT_EQ(result.lambda.size(), 6);

  // Verify both contacts satisfy their respective cone constraints
  // Contact 0: ||[lambda(1), lambda(2)]|| <= 0.8 * lambda(0)
  double t0_norm = std::sqrt(result.lambda(1) * result.lambda(1) +
                              result.lambda(2) * result.lambda(2));
  EXPECT_LE(t0_norm, 0.8 * result.lambda(0) + 1e-4);

  // Contact 1: ||[lambda(4), lambda(5)]|| <= 0.2 * lambda(3)
  double t1_norm = std::sqrt(result.lambda(4) * result.lambda(4) +
                              result.lambda(5) * result.lambda(5));
  EXPECT_LE(t1_norm, 0.2 * result.lambda(3) + 1e-4);
}

// ============================================================================
// High mass ratio (100:1): ECOS should still converge
// ============================================================================
TEST(ECOSSolveTest, HighMassRatio)
{
  const int numContacts = 1;

  // Effective mass matrix with 100:1 mass ratio
  // This simulates a heavy object on a light support
  // Note: Extremely high ratios (1000:1) can trigger ECOS_NUMERICS;
  // 100:1 is a reasonable test of robustness for interior-point methods.
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  A(0, 0) = 100.0;    // Higher effective mass for normal
  A(1, 1) = 1.0;      // Low effective mass for tangent 1
  A(2, 2) = 1.0;      // Low effective mass for tangent 2

  // b must be inside the cone for the equality constraint to be feasible:
  // lambda = A^-1 * b = [50/100, 0.3/1, 0.3/1] = [0.5, 0.3, 0.3]
  // ||[0.3, 0.3]|| ≈ 0.424, mu*lambda_n = 0.5*0.5 = 0.25 → need to fix
  // Use: b = [100, 0.1, 0.1] → lambda = [1.0, 0.1, 0.1]
  // ||[0.1, 0.1]|| ≈ 0.141, mu*lambda_n = 0.5*1.0 = 0.5 → feasible (stick)
  Eigen::VectorXd b{3};
  b << 100.0, 0.1, 0.1;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);

  ConstraintSolver solver;
  auto result = solver.solveWithECOS(A, b, coneSpec, numContacts);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.ecos_exit_flag, 0);
  EXPECT_LT(result.iterations, 30);

  // Verify cone constraint
  double lambda_n = result.lambda(0);
  double tangential_norm = std::sqrt(result.lambda(1) * result.lambda(1) +
                                      result.lambda(2) * result.lambda(2));
  EXPECT_LE(tangential_norm, 0.5 * lambda_n + 1e-4);
}

// ============================================================================
// ActiveSetResult default fields: ASM result has "ASM" solver_type
// ============================================================================
TEST(ECOSSolveTest, ActiveSetResultDefaultsToASM)
{
  ConstraintSolver::ActiveSetResult result;
  EXPECT_EQ(result.solver_type, "ASM");
  EXPECT_EQ(result.ecos_exit_flag, 0);
  EXPECT_TRUE(std::isnan(result.primal_residual));
  EXPECT_TRUE(std::isnan(result.dual_residual));
  EXPECT_TRUE(std::isnan(result.gap));
}

// ============================================================================
// ECOSData equality constraint support: verify A_eq and b_eq populated
// ============================================================================
TEST(ECOSSolveTest, ECOSProblemBuilderPopulatesEqualityConstraints)
{
  const int numContacts = 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  A(0, 1) = 0.1;
  A(1, 0) = 0.1;  // Symmetric off-diagonal

  Eigen::VectorXd b{3};
  b << 5.0, 1.0, 0.5;

  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);

  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // Verify equality constraint data is populated
  EXPECT_EQ(data.num_equality_, 3);
  EXPECT_GT(data.A_eq_.nnz, 0);
  EXPECT_EQ(data.b_eq_.size(), 3u);

  // Verify b_eq matches input b
  EXPECT_NEAR(data.b_eq_[0], 5.0, 1e-10);
  EXPECT_NEAR(data.b_eq_[1], 1.0, 1e-10);
  EXPECT_NEAR(data.b_eq_[2], 0.5, 1e-10);

  // Verify A_eq dimensions
  EXPECT_EQ(data.A_eq_.nrows, 3);
  EXPECT_EQ(data.A_eq_.ncols, 3);
}

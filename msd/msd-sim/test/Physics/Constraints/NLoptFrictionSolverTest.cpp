// Ticket: 0068b_nlopt_friction_solver_class
// Design: docs/designs/0068_nlopt_friction_cone_solver/design.md

#include "msd-sim/src/Physics/Constraints/NLoptFrictionSolver.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

namespace msd_sim {

TEST(NLoptFrictionSolverTest, ConstructorSetsDefaultParameters)
{
  NLoptFrictionSolver solver{};

  EXPECT_NEAR(solver.getTolerance(), 1e-6, 1e-12);
  EXPECT_EQ(solver.getMaxIterations(), 100);
  EXPECT_EQ(solver.getAlgorithm(), NLoptFrictionSolver::Algorithm::SLSQP);
}

TEST(NLoptFrictionSolverTest, AlgorithmSelectionConstructor)
{
  NLoptFrictionSolver solver{NLoptFrictionSolver::Algorithm::COBYLA};

  EXPECT_EQ(solver.getAlgorithm(), NLoptFrictionSolver::Algorithm::COBYLA);
  EXPECT_NEAR(solver.getTolerance(), 1e-6, 1e-12);
  EXPECT_EQ(solver.getMaxIterations(), 100);
}

TEST(NLoptFrictionSolverTest, SettersUpdateConfiguration)
{
  NLoptFrictionSolver solver{};

  solver.setTolerance(1e-8);
  solver.setMaxIterations(200);
  solver.setAlgorithm(NLoptFrictionSolver::Algorithm::MMA);

  EXPECT_NEAR(solver.getTolerance(), 1e-8, 1e-12);
  EXPECT_EQ(solver.getMaxIterations(), 200);
  EXPECT_EQ(solver.getAlgorithm(), NLoptFrictionSolver::Algorithm::MMA);
}

TEST(NLoptFrictionSolverTest, UnconstrainedOptimum)
{
  // With zero friction, the QP reduces to an unconstrained problem
  // minimize (1/2) lambda^T A lambda - b^T lambda
  // Optimal solution: lambda = A^{-1} b

  NLoptFrictionSolver solver{};

  // 1-contact problem (3 variables): lambda_n, lambda_t1, lambda_t2
  Eigen::MatrixXd A(3, 3);
  A << 2.0, 0.0, 0.0,
       0.0, 2.0, 0.0,
       0.0, 0.0, 2.0;

  Eigen::VectorXd b(3);
  b << 4.0, 0.0, 0.0;  // Should converge to lambda = [2, 0, 0]

  std::vector<double> mu{0.0};  // Zero friction

  auto result = solver.solve(A, b, mu);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.lambda.size(), 3);
  EXPECT_NEAR(result.lambda[0], 2.0, 1e-4);
  EXPECT_NEAR(result.lambda[1], 0.0, 1e-4);
  EXPECT_NEAR(result.lambda[2], 0.0, 1e-4);
  EXPECT_EQ(result.constraint_violations.size(), 1);
  EXPECT_GE(result.constraint_violations[0], 0.0);  // Cone constraint satisfied
}

TEST(NLoptFrictionSolverTest, ConeInteriorSolution)
{
  // With small friction and no tangential bias, solution should be at cone interior

  NLoptFrictionSolver solver{};

  // 1-contact problem
  Eigen::MatrixXd A(3, 3);
  A << 2.0, 0.0, 0.0,
       0.0, 2.0, 0.0,
       0.0, 0.0, 2.0;

  Eigen::VectorXd b(3);
  b << 4.0, 1.0, 0.5;  // Small tangential components

  std::vector<double> mu{0.3};  // Small friction coefficient

  auto result = solver.solve(A, b, mu);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.lambda.size(), 3);
  EXPECT_GT(result.lambda[0], 0.0);  // Normal component positive

  // Check cone constraint: mu^2 * n^2 - t1^2 - t2^2 >= 0
  const double n = result.lambda[0];
  const double t1 = result.lambda[1];
  const double t2 = result.lambda[2];
  const double cone_val = mu[0] * mu[0] * n * n - t1 * t1 - t2 * t2;

  EXPECT_GE(cone_val, -1e-5);  // Constraint satisfied (within tolerance)
  EXPECT_NEAR(result.constraint_violations[0], cone_val, 1e-4);
}

TEST(NLoptFrictionSolverTest, ConeSurfaceSolution)
{
  // With large friction and strong tangential bias, solution should saturate at cone surface

  NLoptFrictionSolver solver{};

  // 1-contact problem
  Eigen::MatrixXd A(3, 3);
  A << 2.0, 0.0, 0.0,
       0.0, 2.0, 0.0,
       0.0, 0.0, 2.0;

  Eigen::VectorXd b(3);
  b << 4.0, 10.0, 0.0;  // Strong tangential component

  std::vector<double> mu{1.0};  // High friction coefficient

  auto result = solver.solve(A, b, mu);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.lambda.size(), 3);
  EXPECT_GT(result.lambda[0], 0.0);  // Normal component positive

  // Check cone constraint is satisfied
  const double n = result.lambda[0];
  const double t1 = result.lambda[1];
  const double t2 = result.lambda[2];
  const double tangent_mag = std::sqrt(t1 * t1 + t2 * t2);
  const double max_tangent = mu[0] * n;

  // Should be at or near cone surface
  EXPECT_LE(tangent_mag, max_tangent + 1e-5);
  EXPECT_GE(result.constraint_violations[0], -1e-5);
}

TEST(NLoptFrictionSolverTest, WarmStartProducesSameSolution)
{
  NLoptFrictionSolver solver{};

  // 1-contact problem
  Eigen::MatrixXd A(3, 3);
  A << 2.0, 0.0, 0.0,
       0.0, 2.0, 0.0,
       0.0, 0.0, 2.0;

  Eigen::VectorXd b(3);
  b << 4.0, 2.0, 1.0;

  std::vector<double> mu{0.5};

  // Cold start
  auto result_cold = solver.solve(A, b, mu);
  EXPECT_TRUE(result_cold.converged);

  // Warm start from solution
  auto result_warm = solver.solve(A, b, mu, result_cold.lambda);
  EXPECT_TRUE(result_warm.converged);

  // Warm start should produce same solution (tests that warm-start mechanism works)
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(result_warm.lambda[i], result_cold.lambda[i], 1e-5);
  }
}

TEST(NLoptFrictionSolverTest, MultipleContacts)
{
  NLoptFrictionSolver solver{};

  // 2-contact problem (6 variables)
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6) * 2.0;

  Eigen::VectorXd b(6);
  b << 4.0, 1.0, 0.5,   // Contact 1
       6.0, 2.0, 1.0;   // Contact 2

  std::vector<double> mu{0.4, 0.6};

  auto result = solver.solve(A, b, mu);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.lambda.size(), 6);
  EXPECT_EQ(result.constraint_violations.size(), 2);

  // Check both normal forces positive
  EXPECT_GT(result.lambda[0], 0.0);
  EXPECT_GT(result.lambda[3], 0.0);

  // Check both cone constraints satisfied
  for (size_t i = 0; i < 2; ++i)
  {
    EXPECT_GE(result.constraint_violations[i], -1e-5);
  }
}

TEST(NLoptFrictionSolverTest, AlgorithmVariantsProduceSimilarResults)
{
  // Test that different algorithms converge to similar solutions

  // 1-contact problem
  Eigen::MatrixXd A(3, 3);
  A << 2.0, 0.0, 0.0,
       0.0, 2.0, 0.0,
       0.0, 0.0, 2.0;

  Eigen::VectorXd b(3);
  b << 4.0, 2.0, 1.0;

  std::vector<double> mu{0.5};

  // Solve with SLSQP (default)
  NLoptFrictionSolver solver_slsqp{NLoptFrictionSolver::Algorithm::SLSQP};
  auto result_slsqp = solver_slsqp.solve(A, b, mu);
  EXPECT_TRUE(result_slsqp.converged);

  // Solve with COBYLA (derivative-free)
  NLoptFrictionSolver solver_cobyla{NLoptFrictionSolver::Algorithm::COBYLA};
  auto result_cobyla = solver_cobyla.solve(A, b, mu);
  EXPECT_TRUE(result_cobyla.converged);

  // Solutions should be close (within 1% tolerance for different algorithms)
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(result_cobyla.lambda[i], result_slsqp.lambda[i],
                std::abs(result_slsqp.lambda[i]) * 0.01);
  }
}

TEST(NLoptFrictionSolverTest, ConstraintViolationDiagnosticAccuracy)
{
  NLoptFrictionSolver solver{};

  // 1-contact problem
  Eigen::MatrixXd A(3, 3);
  A << 2.0, 0.0, 0.0,
       0.0, 2.0, 0.0,
       0.0, 0.0, 2.0;

  Eigen::VectorXd b(3);
  b << 4.0, 2.0, 1.0;

  std::vector<double> mu{0.5};

  auto result = solver.solve(A, b, mu);
  EXPECT_TRUE(result.converged);

  // Manually compute constraint violation
  const double n = result.lambda[0];
  const double t1 = result.lambda[1];
  const double t2 = result.lambda[2];
  const double manual_cone_val = mu[0] * mu[0] * n * n - t1 * t1 - t2 * t2;

  EXPECT_NEAR(result.constraint_violations[0], manual_cone_val, 1e-10);
  EXPECT_GE(result.constraint_violations[0], -1e-5);  // Should be satisfied
}

TEST(NLoptFrictionSolverTest, InvalidMuClampedToZero)
{
  NLoptFrictionSolver solver{};

  // 1-contact problem with invalid negative mu
  Eigen::MatrixXd A(3, 3);
  A << 2.0, 0.0, 0.0,
       0.0, 2.0, 0.0,
       0.0, 0.0, 2.0;

  Eigen::VectorXd b(3);
  b << 4.0, 1.0, 0.5;

  std::vector<double> mu{-0.5};  // Invalid negative friction

  // Should clamp to 0.0 and solve (logged warning expected)
  auto result = solver.solve(A, b, mu);
  EXPECT_TRUE(result.converged);

  // With mu=0, tangential components should be zero (unconstrained)
  EXPECT_GT(result.lambda[0], 0.0);
}

TEST(NLoptFrictionSolverTest, DimensionMismatchThrows)
{
  NLoptFrictionSolver solver{};

  Eigen::MatrixXd A(3, 3);
  A.setIdentity();

  Eigen::VectorXd b(6);  // Wrong size
  b.setZero();

  std::vector<double> mu{0.5};

  EXPECT_THROW(solver.solve(A, b, mu), std::runtime_error);
}

TEST(NLoptFrictionSolverTest, SolveResultStructurePopulated)
{
  NLoptFrictionSolver solver{};

  Eigen::MatrixXd A(3, 3);
  A << 2.0, 0.0, 0.0,
       0.0, 2.0, 0.0,
       0.0, 0.0, 2.0;

  Eigen::VectorXd b(3);
  b << 4.0, 1.0, 0.5;

  std::vector<double> mu{0.3};

  auto result = solver.solve(A, b, mu);

  // Check all fields are populated
  EXPECT_EQ(result.lambda.size(), 3);
  EXPECT_TRUE(result.converged);  // Should converge for this well-conditioned problem
  EXPECT_GT(result.iterations, 0);
  EXPECT_FALSE(std::isnan(result.objective_value));
  EXPECT_TRUE(std::isnan(result.residual));  // residual not used by NLopt
  EXPECT_EQ(result.constraint_violations.size(), 1);
}

}  // namespace msd_sim

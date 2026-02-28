// Ticket: 0087_collision_solver_lambda_test_suite
// Design: docs/designs/0087_collision_solver_lambda_test_suite/design.md

#ifndef MSD_SIM_TEST_HELPERS_LAMBDA_ASSERTIONS_HPP
#define MSD_SIM_TEST_HELPERS_LAMBDA_ASSERTIONS_HPP

#include <cmath>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"

namespace msd_sim::test
{

/**
 * @brief Reusable assertion helpers for verifying solver lambda structure,
 * values, and Coulomb cone compliance.
 *
 * All functions use EXPECT_* (not ASSERT_*) so that all violations are
 * reported in a single test run rather than aborting on the first failure.
 *
 * Units: lambdas have units of N·s (impulse). Expected values should be
 * passed in the same units.
 *
 * @ticket 0087_collision_solver_lambda_test_suite
 */

// ============================================================================
// Convergence
// ============================================================================

/**
 * @brief Assert that the solver converged (result.converged == true).
 *
 * @param result SolveResult from CollisionScenario::stepOnce()
 * @param context Optional description for failure messages
 */
inline void assertConverged(const ConstraintSolver::SolveResult& result,
                             const std::string& context = "")
{
  EXPECT_TRUE(result.converged)
    << "Solver did not converge"
    << (context.empty() ? "" : " [" + context + "]")
    << ". iterations=" << result.iterations
    << ", residual=" << result.residual;
}

// ============================================================================
// Normal lambda checks
// ============================================================================

/**
 * @brief Assert that a specific lambda entry is non-negative.
 *
 * Non-negative normal impulses are a fundamental constraint invariant —
 * objects cannot pull each other together.
 *
 * @param result SolveResult from CollisionScenario::stepOnce()
 * @param lambdaIdx Index into result.lambdas for the normal component
 * @param context Optional description for failure messages
 */
inline void assertNonNegativeNormal(const ConstraintSolver::SolveResult& result,
                                    int lambdaIdx,
                                    const std::string& context = "")
{
  ASSERT_LT(lambdaIdx, static_cast<int>(result.lambdas.size()))
    << "lambdaIdx " << lambdaIdx << " out of range (size="
    << result.lambdas.size() << ")"
    << (context.empty() ? "" : " [" + context + "]");

  EXPECT_GE(result.lambdas[lambdaIdx], 0.0)
    << "Normal lambda at index " << lambdaIdx << " is negative: "
    << result.lambdas[lambdaIdx]
    << (context.empty() ? "" : " [" + context + "]");
}

/**
 * @brief Assert a normal lambda is within relative tolerance of expected.
 *
 * Checks: |result.lambdas[lambdaIdx] - expectedLambda| <= tol
 *
 * @param result SolveResult from CollisionScenario::stepOnce()
 * @param lambdaIdx Index into result.lambdas for the normal component
 * @param expectedLambda Analytically derived expected value [N·s]
 * @param tol Absolute tolerance [N·s]
 * @param context Optional description for failure messages
 */
inline void assertNormalLambda(const ConstraintSolver::SolveResult& result,
                                int lambdaIdx,
                                double expectedLambda,
                                double tol,
                                const std::string& context = "")
{
  ASSERT_LT(lambdaIdx, static_cast<int>(result.lambdas.size()))
    << "lambdaIdx " << lambdaIdx << " out of range (size="
    << result.lambdas.size() << ")"
    << (context.empty() ? "" : " [" + context + "]");

  const double actual{result.lambdas[lambdaIdx]};
  EXPECT_NEAR(actual, expectedLambda, tol)
    << "Normal lambda at index " << lambdaIdx
    << ": actual=" << actual
    << ", expected=" << expectedLambda
    << ", tol=" << tol
    << (context.empty() ? "" : " [" + context + "]");
}

// ============================================================================
// Coulomb cone compliance
// ============================================================================

/**
 * @brief Assert Coulomb cone compliance for one contact's lambda triplet.
 *
 * For Block PGS with friction, each contact contributes three lambda values
 * [λ_n, λ_t1, λ_t2] at consecutive indices. This function verifies:
 *
 *   ||[λ_t1, λ_t2]|| <= frictionCoeff * λ_n  (Coulomb cone condition)
 *
 * And additionally:
 *   λ_n >= 0  (non-negative normal impulse)
 *
 * @param result SolveResult from CollisionScenario::stepOnce()
 * @param normalIdx Index of λ_n in result.lambdas
 * @param tangentIdx Index of λ_t1 in result.lambdas (λ_t2 = tangentIdx + 1)
 * @param frictionCoeff Coulomb friction coefficient μ
 * @param context Optional description for failure messages
 */
inline void assertCoulombCone(const ConstraintSolver::SolveResult& result,
                               int normalIdx,
                               int tangentIdx,
                               double frictionCoeff,
                               const std::string& context = "")
{
  const int minRequired{std::max(normalIdx, tangentIdx + 1) + 1};
  ASSERT_GE(static_cast<int>(result.lambdas.size()), minRequired)
    << "lambdas vector too small for Coulomb cone check"
    << " (size=" << result.lambdas.size()
    << ", need>=" << minRequired << ")"
    << (context.empty() ? "" : " [" + context + "]");

  const double lambdaN{result.lambdas[normalIdx]};
  const double lambdaT1{result.lambdas[tangentIdx]};
  const double lambdaT2{result.lambdas[tangentIdx + 1]};

  // Non-negative normal impulse
  EXPECT_GE(lambdaN, 0.0)
    << "Normal lambda negative (idx=" << normalIdx << "): " << lambdaN
    << (context.empty() ? "" : " [" + context + "]");

  // Coulomb cone: ||[t1, t2]|| <= mu * n
  const double tangentMag{std::hypot(lambdaT1, lambdaT2)};
  const double coneLimit{frictionCoeff * lambdaN};

  // Allow a small numerical tolerance for solver residuals
  constexpr double kConeTol{1e-6};
  EXPECT_LE(tangentMag, coneLimit + kConeTol)
    << "Coulomb cone violated: ||[lambda_t1, lambda_t2]|| = " << tangentMag
    << " > mu * lambda_n = " << frictionCoeff << " * " << lambdaN
    << " = " << coneLimit
    << " (excess: " << (tangentMag - coneLimit) << ")"
    << (context.empty() ? "" : " [" + context + "]");
}

// ============================================================================
// Block PGS layout
// ============================================================================

/**
 * @brief Assert the Block PGS lambda vector has the expected layout.
 *
 * For Block PGS with friction: 3 values per contact [λ_n, λ_t1, λ_t2].
 * Verifies: result.lambdas.size() == numContacts * 3
 *
 * @param result SolveResult from CollisionScenario::stepOnce()
 * @param numContacts Expected number of contact points
 * @param context Optional description for failure messages
 */
inline void assertBlockPGSLayout(const ConstraintSolver::SolveResult& result,
                                  int numContacts,
                                  const std::string& context = "")
{
  const int expectedSize{numContacts * 3};
  EXPECT_EQ(static_cast<int>(result.lambdas.size()), expectedSize)
    << "Block PGS lambda vector size mismatch: actual="
    << result.lambdas.size()
    << ", expected=" << expectedSize
    << " (" << numContacts << " contacts × 3 components)"
    << (context.empty() ? "" : " [" + context + "]");
}

/**
 * @brief Assert the lambda vector has at least one entry and size is a
 * multiple of 3 (Block PGS layout: [n, t1, t2] triplets).
 *
 * More lenient than assertBlockPGSLayout — useful when the exact contact
 * count is not known ahead of time (e.g., cube-on-floor may generate
 * 1–4 contacts depending on penetration geometry).
 *
 * @param result SolveResult from CollisionScenario::stepOnce()
 * @param context Optional description for failure messages
 */
inline void assertBlockPGSLayoutMultipleOf3(
  const ConstraintSolver::SolveResult& result,
  const std::string& context = "")
{
  EXPECT_GT(result.lambdas.size(), 0u)
    << "Lambda vector is empty — no constraints solved"
    << (context.empty() ? "" : " [" + context + "]");

  EXPECT_EQ(result.lambdas.size() % 3, 0u)
    << "Lambda vector size " << result.lambdas.size()
    << " is not a multiple of 3 (Block PGS [n,t1,t2] layout)"
    << (context.empty() ? "" : " [" + context + "]");
}

// ============================================================================
// Convenience: assert Coulomb cone for all contacts in a Block PGS result
// ============================================================================

/**
 * @brief Assert Coulomb cone compliance for every contact in a Block PGS
 * result.
 *
 * Iterates triplets [0,1,2], [3,4,5], ... and calls assertCoulombCone for
 * each. Asserts Block PGS layout (size % 3 == 0) first.
 *
 * @param result SolveResult from CollisionScenario::stepOnce()
 * @param frictionCoeff Coulomb friction coefficient μ (same for all contacts)
 * @param context Optional description for failure messages
 */
inline void assertAllCoulombCones(const ConstraintSolver::SolveResult& result,
                                   double frictionCoeff,
                                   const std::string& context = "")
{
  assertBlockPGSLayoutMultipleOf3(result, context);

  const int numContacts{static_cast<int>(result.lambdas.size()) / 3};
  for (int i = 0; i < numContacts; ++i)
  {
    const int normalIdx{i * 3};
    const int tangentIdx{i * 3 + 1};
    std::ostringstream contactCtx;
    contactCtx << "contact " << i;
    if (!context.empty())
    {
      contactCtx << " [" << context << "]";
    }
    assertCoulombCone(result, normalIdx, tangentIdx, frictionCoeff,
                      contactCtx.str());
  }
}

}  // namespace msd_sim::test

#endif  // MSD_SIM_TEST_HELPERS_LAMBDA_ASSERTIONS_HPP

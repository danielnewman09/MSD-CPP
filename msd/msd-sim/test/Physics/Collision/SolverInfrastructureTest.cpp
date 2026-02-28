// Ticket: 0087_collision_solver_lambda_test_suite (Stage A)
// Design: docs/designs/0087_collision_solver_lambda_test_suite/design.md
//
// Stage A acceptance gate: prove that CollisionScenarioBuilder eliminates
// boilerplate by rewriting 3 existing CollisionPipelineTest scenarios in
// significantly fewer lines of code.
//
// This file also exercises CollisionScenario, CollisionScenarioBuilder, and
// LambdaAssertions directly to validate the infrastructure before dependent
// Stage B-E tests are written.

#include <gtest/gtest.h>

#include <span>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/test/Helpers/CollisionScenario.hpp"
#include "msd-sim/test/Helpers/CollisionScenarioBuilder.hpp"
#include "msd-sim/test/Helpers/LambdaAssertions.hpp"

using namespace msd_sim;
using namespace msd_sim::test;

// ============================================================================
// Stage A Acceptance Gate: 3 CollisionPipelineTest scenarios rewritten
//
// Original tests (CollisionPipelineTest.cpp):
//   1. execute_SeparatedObjects_NoForceApplied
//   2. execute_OverlappingObjects_ForcesApplied
//   3. execute_InertialVsEnvironment_OnlyInertialGetsForce
//
// Each rewrite reduces setup to 1–3 lines using builder/scenario APIs,
// compared to 10–15 lines of manual hull construction and asset setup.
// ============================================================================

// ============================================================================
// Rewrite 1: Separated objects — no collision, no force
//
// Original: ~15 lines of manual hull/frame/asset construction
// Rewrite : 3 lines using twoCubes() with large separation
// ============================================================================

TEST(SolverInfrastructureTest, Rewrite1_SeparatedCubes_NoCollision)
{
  // Cubes 10 m apart — well outside contact range (cube half-extent = 0.5 m)
  auto scenario = CollisionScenarioBuilder::twoCubes(
    10.0,  // separation: 10 m between centres
    0.0,   // velA
    0.0,   // velB
    1.0,   // massA
    1.0,   // massB
    0.5,   // restitution
    0.5);  // friction

  const auto result = scenario.stepOnce();

  // No collision: lambda vector is empty (no constraints created)
  EXPECT_FALSE(scenario.hadCollisions())
    << "Cubes 10 m apart should not collide";

  EXPECT_EQ(result.lambdas.size(), 0u)
    << "No lambdas expected when objects do not collide";
}

// ============================================================================
// Rewrite 2: Overlapping objects — collision detected, lambdas produced
//
// Original: ~18 lines of manual construction with setCoefficientOfRestitution
// Rewrite : 2 lines using overlappingCubes()
// ============================================================================

TEST(SolverInfrastructureTest, Rewrite2_OverlappingCubes_CollisionDetected)
{
  // 0.2 m overlap between two unit cubes
  auto scenario = CollisionScenarioBuilder::overlappingCubes(
    0.2,   // overlap [m]
    1.0,   // mass [kg]
    0.5,   // restitution
    0.5);  // friction

  const auto result = scenario.stepOnce();

  EXPECT_TRUE(scenario.hadCollisions())
    << "Overlapping cubes must register a collision";

  EXPECT_GT(result.lambdas.size(), 0u)
    << "Overlapping cubes should produce at least one lambda";

  // Solver should converge for a simple two-body overlap
  assertConverged(result, "overlapping cubes");

  // All normal lambdas (every 3rd entry starting at 0) must be non-negative
  for (int i = 0; i < static_cast<int>(result.lambdas.size()); i += 3)
  {
    assertNonNegativeNormal(result, i, "overlapping cubes normal lambda");
  }
}

// ============================================================================
// Rewrite 3: Inertial vs environment — cube hits floor, gets force
//
// Original: ~20 lines with separate inertials/environments vectors
// Rewrite : 3 lines using cubeOnFloor()
// ============================================================================

TEST(SolverInfrastructureTest, Rewrite3_CubeOnFloor_CollisionDetected)
{
  // Cube centre at z=0.3 — penetrating the floor (half-extent = 0.5)
  // so the cube bottom face (at z=0.3-0.5=-0.2) overlaps the floor top (z=0)
  // giving 0.2m penetration depth.
  auto scenario = CollisionScenarioBuilder::cubeOnFloor(
    0.3,   // height: cube centre at z=0.3 (bottom at z=-0.2, penetrating floor by 0.2m)
    0.0,   // velocityZ
    0.0,   // velocityX
    1.0,   // mass
    0.0,   // restitution
    0.5);  // friction

  const auto result = scenario.stepOnce();

  EXPECT_TRUE(scenario.hadCollisions())
    << "Cube penetrating floor must register collision";

  EXPECT_GT(result.lambdas.size(), 0u)
    << "Cube-on-floor should produce at least one lambda";

  assertConverged(result, "cube on floor");

  // All normal lambdas must be non-negative (repulsive contact)
  for (int i = 0; i < static_cast<int>(result.lambdas.size()); i += 3)
  {
    assertNonNegativeNormal(result, i, "cube on floor normal lambda");
  }
}

// ============================================================================
// Infrastructure smoke tests
// ============================================================================

// Verify CollisionScenario can be constructed and stepOnce called on empty scene
TEST(SolverInfrastructureTest, EmptyScenario_StepOnce_NoError)
{
  CollisionScenario scenario{0.016};
  EXPECT_NO_THROW(scenario.stepOnce());
  EXPECT_FALSE(scenario.hadCollisions());
  EXPECT_EQ(scenario.getLastSolveResult().lambdas.size(), 0u);
}

// Verify addInertial returns correct indices
TEST(SolverInfrastructureTest, AddInertial_ReturnsCorrectIndices)
{
  CollisionScenario scenario{0.016};
  const auto pts = CollisionScenarioBuilder::unitCubePoints();

  const size_t idxA = scenario.addInertial(
    pts, ReferenceFrame{Coordinate{0.0, 0.0, 0.0}}, 1.0);
  const size_t idxB = scenario.addInertial(
    pts, ReferenceFrame{Coordinate{5.0, 0.0, 0.0}}, 2.0);

  EXPECT_EQ(idxA, 0u);
  EXPECT_EQ(idxB, 1u);
  EXPECT_EQ(scenario.numInertials(), 2u);
}

// Verify initial velocity set via addInertial is preserved pre-step
TEST(SolverInfrastructureTest, AddInertialWithVelocity_VelocityPreserved)
{
  CollisionScenario scenario{0.016};
  const auto pts = CollisionScenarioBuilder::unitCubePoints();

  scenario.addInertial(pts,
                       ReferenceFrame{Coordinate{0.0, 0.0, 5.0}},
                       1.0,
                       0.0,
                       0.5,
                       Coordinate{0.0, 0.0, -2.0});  // downward velocity

  const auto& state = scenario.getInertialState(0);
  EXPECT_NEAR(state.velocity.z(), -2.0, 1e-10)
    << "Initial z-velocity should be preserved";
}

// Verify out-of-range index throws
TEST(SolverInfrastructureTest, GetInertialState_OutOfRange_Throws)
{
  CollisionScenario scenario{0.016};
  EXPECT_THROW(scenario.getInertialState(0), std::out_of_range);
}

// Verify getLastSolveResult() matches return value of stepOnce()
TEST(SolverInfrastructureTest, GetLastSolveResult_MatchesStepOnce)
{
  auto scenario = CollisionScenarioBuilder::overlappingCubes(
    0.2, 1.0, 0.5, 0.5);

  const auto returned = scenario.stepOnce();
  const auto& stored = scenario.getLastSolveResult();

  EXPECT_EQ(returned.converged, stored.converged);
  EXPECT_EQ(returned.lambdas.size(), stored.lambdas.size());
  for (Eigen::Index i = 0; i < returned.lambdas.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(returned.lambdas[i], stored.lambdas[i]);
  }
}

// ============================================================================
// LambdaAssertions unit tests
// ============================================================================

// assertConverged: passes when solver converged
TEST(LambdaAssertionsTest, AssertConverged_Passes)
{
  ConstraintSolver::SolveResult result;
  result.converged = true;
  EXPECT_NO_FATAL_FAILURE(assertConverged(result));
}

// assertNonNegativeNormal: passes for non-negative lambda
TEST(LambdaAssertionsTest, AssertNonNegativeNormal_Passes)
{
  ConstraintSolver::SolveResult result;
  result.lambdas.resize(3);
  result.lambdas[0] = 1.5;   // normal
  result.lambdas[1] = 0.2;   // tangent 1
  result.lambdas[2] = -0.1;  // tangent 2 (negative OK for friction)
  EXPECT_NO_FATAL_FAILURE(assertNonNegativeNormal(result, 0));
}

// assertCoulombCone: passes when friction within cone
TEST(LambdaAssertionsTest, AssertCoulombCone_WithinCone_Passes)
{
  ConstraintSolver::SolveResult result;
  result.lambdas.resize(3);
  result.lambdas[0] = 2.0;   // lambda_n
  result.lambdas[1] = 0.3;   // lambda_t1
  result.lambdas[2] = 0.4;   // lambda_t2  => ||(0.3, 0.4)|| = 0.5 <= 0.5 * 2.0 = 1.0
  EXPECT_NO_FATAL_FAILURE(assertCoulombCone(result, 0, 1, 0.5));
}

// assertBlockPGSLayout: passes for correct count
TEST(LambdaAssertionsTest, AssertBlockPGSLayout_CorrectCount_Passes)
{
  ConstraintSolver::SolveResult result;
  result.lambdas.resize(6);  // 2 contacts × 3 components
  EXPECT_NO_FATAL_FAILURE(assertBlockPGSLayout(result, 2));
}

// assertAllCoulombCones: passes for two contacts both within cone
TEST(LambdaAssertionsTest, AssertAllCoulombCones_BothWithinCone_Passes)
{
  ConstraintSolver::SolveResult result;
  result.lambdas.resize(6);
  result.lambdas[0] = 2.0;  result.lambdas[1] = 0.3;  result.lambdas[2] = 0.4;
  result.lambdas[3] = 1.5;  result.lambdas[4] = 0.2;  result.lambdas[5] = 0.1;
  EXPECT_NO_FATAL_FAILURE(assertAllCoulombCones(result, 0.5));
}

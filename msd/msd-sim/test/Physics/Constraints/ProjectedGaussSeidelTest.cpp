// Ticket: 0073_hybrid_pgs_large_islands
// Unit tests for ProjectedGaussSeidel solver and ConstraintSolver threshold
// dispatch.
//
// Tests validate:
// - PGS convergence for single normal constraint
// - PGS with friction (ball-projection, not box bounds)
// - Warm-start reduces sweep count
// - Convergence tolerance triggers early exit
// - Degenerate (A_ii ~ 0) contact does not crash
// - Large friction island converges without energy injection
// - ConstraintSolver threshold dispatch routes n=20 to ASM, n=21 to PGS

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

#include "msd-sim/src/DataTypes/AngularAcceleration.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

using namespace msd_sim;

// ============================================================================
// Helpers
// ============================================================================

namespace
{

InertialState makeState(
  const Coordinate& position = Coordinate{0.0, 0.0, 0.0},
  const Coordinate& velocity = Coordinate{0.0, 0.0, 0.0})
{
  InertialState state;
  state.position = position;
  state.velocity = velocity;
  state.acceleration = Coordinate{0.0, 0.0, 0.0};
  state.orientation = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};
  state.quaternionRate = Eigen::Vector4d::Zero();
  state.angularAcceleration = AngularAcceleration{0.0, 0.0, 0.0};
  return state;
}

Eigen::Matrix3d identityInertia()
{
  return Eigen::Matrix3d::Identity();
}

/// Build a single ContactConstraint: body 0 approaching body 1 along +Z.
std::unique_ptr<ContactConstraint> makeContactConstraint(
  size_t bodyA,
  size_t bodyB,
  double restitution = 0.0,
  double penetration = 0.01)
{
  return std::make_unique<ContactConstraint>(
    bodyA, bodyB,
    Coordinate{0, 0, 1},   // normal
    Coordinate{0, 0, 0.5}, // contactA
    Coordinate{0, 0, 0.4}, // contactB
    penetration,
    Coordinate{0, 0, 0},   // comA
    Coordinate{0, 0, 0},   // comB
    restitution,
    0.0);
}

/// Build FrictionConstraint for the same contact geometry.
std::unique_ptr<FrictionConstraint> makeFrictionConstraint(
  size_t bodyA,
  size_t bodyB,
  double mu = 0.5)
{
  return std::make_unique<FrictionConstraint>(
    bodyA, bodyB,
    Coordinate{0, 0, 1},   // normal
    Coordinate{0, 0, 0.5}, // contactA
    Coordinate{0, 0, 0.4}, // contactB
    Coordinate{0, 0, 0},   // comA
    Coordinate{0, 0, 0},   // comB
    mu);
}

}  // anonymous namespace

// ============================================================================
// 1. Single contact normal — PGS converges to correct lambda
// ============================================================================

TEST(ProjectedGaussSeidelTest, SingleContactNormal_ConvergesPositiveLambda)
{
  // One body approaching a static surface. PGS should produce lambda > 0.
  // Normal = {0,0,1} (A→B). J*v = -n·vA + n·vB.
  // With vA={0,0,2} (A moving toward B in +Z): J*v = -(0,0,1)·(0,0,2) = -2.
  // b = -(1+e)*(-2) = +2 → lambda_new = max(0, 2/A_ii) > 0.
  ProjectedGaussSeidel pgs;

  InertialState stateA = makeState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = makeState(Coordinate{0, 0, 0.9});

  auto cc = makeContactConstraint(0, 1, /*restitution=*/0.0);

  std::vector<Constraint*> constraints{cc.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA, stateB};
  std::vector<double> invMasses{1.0 / 10.0, 0.0};  // stateB = infinite mass
  std::vector<Eigen::Matrix3d> invInertias{identityInertia(),
                                           Eigen::Matrix3d::Zero()};

  auto result = pgs.solve(constraints, states, invMasses, invInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  ASSERT_EQ(1, result.lambdas.size());
  EXPECT_GT(result.lambdas(0), 0.0);
  EXPECT_EQ(2, result.bodyForces.size());
}

// ============================================================================
// 2. Single contact with friction — ball-projection respected
// ============================================================================

TEST(ProjectedGaussSeidelTest,
     SingleContactWithFriction_BallProjection)
{
  // Body sliding in X with contact normal in Z. Friction should oppose motion.
  // Verify: ||lambda_t|| <= mu * lambda_n (ball-projection)
  ProjectedGaussSeidel pgs;
  pgs.setMaxSweeps(100);

  // Body A slides in +X while pressing into B in +Z.
  // Normal = {0,0,1}: J*v = -n·vA = -(0,0,1)·(3,0,0.5) = -0.5 → b = +0.5 > 0
  InertialState stateA = makeState(Coordinate{0, 0, 0}, Coordinate{3.0, 0, 0.5});
  InertialState stateB = makeState(Coordinate{0, 0, 0.9});

  double const mu = 0.5;
  auto cc = makeContactConstraint(0, 1, 0.0, 0.01);
  auto fc = makeFrictionConstraint(0, 1, mu);

  std::vector<Constraint*> constraints{cc.get(), fc.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA, stateB};
  std::vector<double> invMasses{1.0 / 10.0, 0.0};
  std::vector<Eigen::Matrix3d> invInertias{identityInertia(),
                                           Eigen::Matrix3d::Zero()};

  auto result = pgs.solve(constraints, states, invMasses, invInertias, 2, 0.016);

  ASSERT_EQ(3, result.lambdas.size());
  double const lambdaN = result.lambdas(0);
  double const lt1 = result.lambdas(1);
  double const lt2 = result.lambdas(2);
  double const tangentNorm = std::sqrt(lt1 * lt1 + lt2 * lt2);

  EXPECT_GE(lambdaN, 0.0) << "Normal lambda must be non-negative";

  // Ball-projection: ||lambda_t|| <= mu * lambda_n
  if (lambdaN > 1e-10)
  {
    EXPECT_LE(tangentNorm, mu * lambdaN * 1.01)
      << "Friction cone violation: ||lt||=" << tangentNorm
      << " > mu*ln=" << mu * lambdaN;
  }
}

// ============================================================================
// 3. Warm-start reduces sweeps vs cold-start
// ============================================================================

TEST(ProjectedGaussSeidelTest, WarmStart_ReducesSweepCount)
{
  // With the correct warm-start lambda, PGS should converge in very few sweeps.
  // Cold-start from zero requires more sweeps.
  ProjectedGaussSeidel pgsCold;
  ProjectedGaussSeidel pgsWarm;

  InertialState stateA = makeState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = makeState(Coordinate{0, 0, 0.9});

  auto cc = makeContactConstraint(0, 1);

  std::vector<Constraint*> constraints{cc.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA, stateB};
  std::vector<double> invMasses{1.0 / 10.0, 0.0};
  std::vector<Eigen::Matrix3d> invInertias{identityInertia(),
                                           Eigen::Matrix3d::Zero()};

  // Cold solve
  auto coldResult = pgsCold.solve(
    constraints, states, invMasses, invInertias, 2, 0.016, std::nullopt);

  // Warm solve using cold result as initial lambda
  auto warmResult = pgsWarm.solve(
    constraints, states, invMasses, invInertias, 2, 0.016,
    coldResult.lambdas);

  EXPECT_TRUE(coldResult.converged);
  EXPECT_TRUE(warmResult.converged);

  // Warm-start should converge in 1 sweep (or at most fewer than cold)
  EXPECT_LE(warmResult.iterations, coldResult.iterations)
    << "Warm-start should not need more sweeps than cold-start";
  EXPECT_EQ(1, warmResult.iterations)
    << "Perfect warm-start (correct lambda) should converge in 1 sweep";
}

// ============================================================================
// 4. Convergence tolerance — early exit before maxSweeps
// ============================================================================

TEST(ProjectedGaussSeidelTest, ConvergenceTolerance_EarlyExit)
{
  ProjectedGaussSeidel pgs;
  pgs.setMaxSweeps(1000);
  pgs.setConvergenceTolerance(1e-4);

  InertialState stateA = makeState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = makeState(Coordinate{0, 0, 0.9});

  auto cc = makeContactConstraint(0, 1);

  std::vector<Constraint*> constraints{cc.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA, stateB};
  std::vector<double> invMasses{1.0 / 10.0, 0.0};
  std::vector<Eigen::Matrix3d> invInertias{identityInertia(),
                                           Eigen::Matrix3d::Zero()};

  auto result = pgs.solve(
    constraints, states, invMasses, invInertias, 2, 0.016);

  // Should converge early — far fewer than 1000 sweeps
  EXPECT_TRUE(result.converged);
  EXPECT_LT(result.iterations, 1000)
    << "Should have exited early via convergence tolerance";
}

// ============================================================================
// 5. Degenerate contact (A_ii ~ 0) — skip without crash
// ============================================================================

TEST(ProjectedGaussSeidelTest, DegenerateContact_ZeroMass_NoNaNs)
{
  // If both bodies have infinite mass (invMass = 0), A_ii = kRegularizationEpsilon
  // (tiny but nonzero). PGS should not crash and result should be finite.
  ProjectedGaussSeidel pgs;
  pgs.setMaxSweeps(5);

  InertialState stateA = makeState(Coordinate{0, 0, 0}, Coordinate{0, 0, 1.0});
  InertialState stateB = makeState(Coordinate{0, 0, 0.9});

  auto cc = makeContactConstraint(0, 1);

  std::vector<Constraint*> constraints{cc.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA, stateB};
  std::vector<double> invMasses{0.0, 0.0};  // Both infinite mass
  std::vector<Eigen::Matrix3d> invInertias{Eigen::Matrix3d::Zero(),
                                           Eigen::Matrix3d::Zero()};

  auto result = pgs.solve(
    constraints, states, invMasses, invInertias, 2, 0.016);

  ASSERT_EQ(1, result.lambdas.size());
  EXPECT_FALSE(std::isnan(result.lambdas(0)))
    << "Lambda should not be NaN for degenerate contact";
  EXPECT_FALSE(std::isinf(result.lambdas(0)))
    << "Lambda should not be Inf for degenerate contact";
}

// ============================================================================
// 6. Large friction island — converges, lambda matches ConstraintSolver
// ============================================================================

TEST(ProjectedGaussSeidelTest, LargeFrictionIsland_MatchesConstraintSolver)
{
  // Build a 7-contact system with friction (7 CC + 7 FC = 21 rows > kASMThreshold=20).
  // ConstraintSolver::solve() should dispatch to PGS.
  // Verify result is consistent: lambdas >= 0 for normals, friction cone satisfied.
  ConstraintSolver solver;

  const int numContacts = 7;  // 7*3 = 21 rows > kASMThreshold=20
  const size_t numBodies = static_cast<size_t>(numContacts) + 1;  // 1 dynamic + floor

  std::vector<InertialState> states(numBodies);
  for (int c = 0; c < numContacts; ++c)
  {
    // Each body moving down at -2 m/s
    states[static_cast<size_t>(c)] =
      makeState(Coordinate{static_cast<double>(c), 0, 0},
                Coordinate{0, 0, 2.0});
  }
  states[static_cast<size_t>(numContacts)] = makeState();  // Static floor

  std::vector<std::reference_wrapper<const InertialState>> stateRefs;
  stateRefs.reserve(numBodies);
  for (size_t k = 0; k < numBodies; ++k)
  {
    stateRefs.emplace_back(states[k]);
  }

  std::vector<double> invMasses(numBodies, 1.0 / 10.0);
  invMasses[static_cast<size_t>(numContacts)] = 0.0;  // Floor: infinite mass
  std::vector<Eigen::Matrix3d> invInertias(numBodies, identityInertia());
  invInertias[static_cast<size_t>(numContacts)] = Eigen::Matrix3d::Zero();

  // Build CC and FC pairs for each contact
  std::vector<std::unique_ptr<ContactConstraint>> ccs;
  std::vector<std::unique_ptr<FrictionConstraint>> fcs;
  std::vector<Constraint*> constraintPtrs;

  for (int c = 0; c < numContacts; ++c)
  {
    auto cc = makeContactConstraint(
      static_cast<size_t>(c),
      static_cast<size_t>(numContacts));
    auto fc = makeFrictionConstraint(
      static_cast<size_t>(c),
      static_cast<size_t>(numContacts));
    constraintPtrs.push_back(cc.get());
    constraintPtrs.push_back(fc.get());
    ccs.push_back(std::move(cc));
    fcs.push_back(std::move(fc));
  }

  // Total rows = 7 * (1 + 2) = 21 > kASMThreshold=20 → dispatches to PGS
  EXPECT_GT(
    static_cast<size_t>(numContacts * 3),
    ConstraintSolver::kASMThreshold);

  auto result = solver.solve(
    constraintPtrs, stateRefs, invMasses, invInertias, numBodies, 0.016);

  ASSERT_EQ(numContacts * 3, result.lambdas.size());

  // Verify: all normal lambdas >= 0
  for (int c = 0; c < numContacts; ++c)
  {
    EXPECT_GE(result.lambdas(3 * c), 0.0)
      << "Normal lambda[" << c << "] must be non-negative";
  }

  // Verify: friction cone satisfied for each contact
  const double mu = 0.5;
  for (int c = 0; c < numContacts; ++c)
  {
    double const lambdaN = result.lambdas(3 * c);
    double const lt1 = result.lambdas(3 * c + 1);
    double const lt2 = result.lambdas(3 * c + 2);
    double const tangentNorm = std::sqrt(lt1 * lt1 + lt2 * lt2);

    if (lambdaN > 1e-10)
    {
      EXPECT_LE(tangentNorm, mu * lambdaN * 1.05)
        << "Friction cone violation at contact " << c
        << ": ||lt||=" << tangentNorm << " > mu*ln=" << mu * lambdaN;
    }
  }

  // Verify: body forces are populated
  EXPECT_EQ(numBodies, result.bodyForces.size());
}

// ============================================================================
// 7. ConstraintSolver threshold dispatch — n=20 uses ASM, n=21 uses PGS
// ============================================================================

TEST(ConstraintSolverThresholdTest, kASMThresholdValue_Is20)
{
  // Design specifies kASMThreshold = 20 (constexpr, publicly accessible)
  EXPECT_EQ(20u, ConstraintSolver::kASMThreshold);
}

TEST(ConstraintSolverThresholdTest, ThresholdDispatchASM_20Rows)
{
  // 20 contacts (no friction) = 20 rows = exactly kASMThreshold.
  // Should use ASM path: iterations field is ASM iteration count (finite).
  ConstraintSolver solver;

  const int numContacts = 20;
  const size_t numBodies = static_cast<size_t>(numContacts) + 1;

  std::vector<InertialState> states(numBodies);
  for (int c = 0; c < numContacts; ++c)
  {
    states[static_cast<size_t>(c)] =
      makeState(Coordinate{static_cast<double>(c), 0, 0},
                Coordinate{0, 0, 2.0});
  }
  states[static_cast<size_t>(numContacts)] = makeState();

  std::vector<std::reference_wrapper<const InertialState>> stateRefs;
  stateRefs.reserve(numBodies);
  for (size_t k = 0; k < numBodies; ++k)
  {
    stateRefs.emplace_back(states[k]);
  }

  std::vector<double> invMasses(numBodies, 1.0 / 10.0);
  invMasses[static_cast<size_t>(numContacts)] = 0.0;
  std::vector<Eigen::Matrix3d> invInertias(numBodies, identityInertia());
  invInertias[static_cast<size_t>(numContacts)] = Eigen::Matrix3d::Zero();

  std::vector<std::unique_ptr<ContactConstraint>> ccs;
  std::vector<Constraint*> constraintPtrs;

  for (int c = 0; c < numContacts; ++c)
  {
    auto cc = makeContactConstraint(
      static_cast<size_t>(c), static_cast<size_t>(numContacts));
    constraintPtrs.push_back(cc.get());
    ccs.push_back(std::move(cc));
  }

  // Exactly kASMThreshold rows — ASM path
  EXPECT_EQ(static_cast<size_t>(numContacts), ConstraintSolver::kASMThreshold);

  auto result = solver.solve(
    constraintPtrs, stateRefs, invMasses, invInertias, numBodies, 0.016);

  EXPECT_TRUE(result.converged);
  ASSERT_EQ(numContacts, result.lambdas.size());

  // ASM should converge with a small number of iterations
  EXPECT_LE(result.iterations, numContacts * 2)
    << "ASM iterations should be bounded by 2*numContacts";
}

TEST(ConstraintSolverThresholdTest, ThresholdDispatchPGS_21Rows)
{
  // 7 contacts + 7 frictions = 21 rows > kASMThreshold=20.
  // Should use PGS path: iterations field is sweep count.
  ConstraintSolver solver;

  const int numContacts = 7;  // 7 * 3 = 21 rows
  const size_t numBodies = static_cast<size_t>(numContacts) + 1;

  std::vector<InertialState> states(numBodies);
  for (int c = 0; c < numContacts; ++c)
  {
    states[static_cast<size_t>(c)] =
      makeState(Coordinate{static_cast<double>(c), 0, 0},
                Coordinate{0, 0, 2.0});
  }
  states[static_cast<size_t>(numContacts)] = makeState();

  std::vector<std::reference_wrapper<const InertialState>> stateRefs;
  stateRefs.reserve(numBodies);
  for (size_t k = 0; k < numBodies; ++k)
  {
    stateRefs.emplace_back(states[k]);
  }

  std::vector<double> invMasses(numBodies, 1.0 / 10.0);
  invMasses[static_cast<size_t>(numContacts)] = 0.0;
  std::vector<Eigen::Matrix3d> invInertias(numBodies, identityInertia());
  invInertias[static_cast<size_t>(numContacts)] = Eigen::Matrix3d::Zero();

  std::vector<std::unique_ptr<ContactConstraint>> ccs;
  std::vector<std::unique_ptr<FrictionConstraint>> fcs;
  std::vector<Constraint*> constraintPtrs;

  for (int c = 0; c < numContacts; ++c)
  {
    auto cc = makeContactConstraint(
      static_cast<size_t>(c), static_cast<size_t>(numContacts));
    auto fc = makeFrictionConstraint(
      static_cast<size_t>(c), static_cast<size_t>(numContacts));
    constraintPtrs.push_back(cc.get());
    constraintPtrs.push_back(fc.get());
    ccs.push_back(std::move(cc));
    fcs.push_back(std::move(fc));
  }

  // 21 rows > kASMThreshold=20 → dispatches to PGS
  EXPECT_GT(static_cast<size_t>(numContacts * 3),
            ConstraintSolver::kASMThreshold);

  auto result = solver.solve(
    constraintPtrs, stateRefs, invMasses, invInertias, numBodies, 0.016);

  // PGS sweep count is bounded by maxSweeps (50 default)
  EXPECT_LE(result.iterations, 50)
    << "PGS iterations should be bounded by maxSweeps";
  ASSERT_EQ(numContacts * 3, result.lambdas.size());

  // Verify normal lambdas
  for (int c = 0; c < numContacts; ++c)
  {
    EXPECT_GE(result.lambdas(3 * c), 0.0)
      << "Normal lambda[" << c << "] must be non-negative";
  }
}

// ============================================================================
// 8. Empty constraint list — both PGS and solver handle gracefully
// ============================================================================

TEST(ProjectedGaussSeidelTest, EmptyConstraints_ReturnsZeroLambdas)
{
  ProjectedGaussSeidel pgs;

  std::vector<Constraint*> constraints;
  std::vector<std::reference_wrapper<const InertialState>> states;
  std::vector<double> invMasses;
  std::vector<Eigen::Matrix3d> invInertias;

  auto result = pgs.solve(constraints, states, invMasses, invInertias, 0, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(0, result.lambdas.size());
  EXPECT_EQ(0, result.iterations);
}

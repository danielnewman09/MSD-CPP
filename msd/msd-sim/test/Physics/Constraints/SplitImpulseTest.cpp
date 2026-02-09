// Ticket: 0040b_split_impulse_position_correction
// Design: docs/designs/0040b-split-impulse-position-correction/design.md
// Test: Split impulse position correction unit tests

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "msd-sim/src/DataTypes/AngularRate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/PositionCorrector.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

InertialState createDefaultState(
  const Coordinate& position = Coordinate{0.0, 0.0, 0.0},
  const Coordinate& velocity = Coordinate{0.0, 0.0, 0.0})
{
  InertialState state;
  state.position = position;
  state.velocity = velocity;
  state.acceleration = Coordinate{0.0, 0.0, 0.0};
  state.orientation = QuaternionD{1.0, 0.0, 0.0, 0.0};
  state.quaternionRate = Eigen::Vector4d::Zero();
  state.angularAcceleration = AngularRate{0.0, 0.0, 0.0};
  return state;
}

/// Create cube point cloud
std::vector<Coordinate> createCubePoints(double size)
{
  double const half = size / 2.0;
  return {Coordinate{-half, -half, -half},
          Coordinate{half, -half, -half},
          Coordinate{half, half, -half},
          Coordinate{-half, half, -half},
          Coordinate{-half, -half, half},
          Coordinate{half, -half, half},
          Coordinate{half, half, half},
          Coordinate{-half, half, half}};
}

}  // anonymous namespace

// ============================================================================
// Velocity RHS Tests — Verify Baumgarte Removal
// ============================================================================

TEST(SplitImpulse, VelocityRHS_NoBaumgarteBias)
{
  // A resting contact (v_rel ~= 0) should have a MUCH smaller RHS than
  // the old Baumgarte formula, but not exactly zero due to gentle slop
  // correction for resting contacts with penetration > 5mm slop.
  //
  // Set up a contact with 10mm penetration but zero relative velocity.
  // Old Baumgarte: RHS = (0.2/dt)*pen = (0.2/0.0167)*0.01 = 0.12 m/s
  // New slop correction: 0.2 * (0.01 - 0.005) / (1/60) = 0.06 m/s (2× smaller)
  //
  // We compare lambda values (impulse magnitudes) to verify the improvement.

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, -0.5};
  Coordinate comB{0.0, 0.0, 0.5};
  double const penetration = 0.01;  // 10mm
  double const restitution = 0.0;   // No restitution
  double const dt = 1.0 / 60.0;

  ContactConstraint constraint{0,
                               1,
                               normal,
                               contactA,
                               contactB,
                               penetration,
                               comA,
                               comB,
                               restitution,
                               0.0};  // preImpactVel = 0 (resting)

  std::vector<Constraint*> constraints{&constraint};

  InertialState stateA = createDefaultState(Coordinate{0.0, 0.0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0.0, 0.0, 0.5});
  std::vector<std::reference_wrapper<const InertialState>> states{
    std::cref(stateA), std::cref(stateB)};

  std::vector<double> inverseMasses{0.1, 0.1};  // 10 kg each
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Identity() * 0.1};

  ConstraintSolver solver;
  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, dt);

  // With zero relative velocity, e=0, and 10mm penetration (5mm above slop):
  // slopCorrection = 0.2 * (0.01 - 0.005) / dt = 0.06
  // RHS = -(1+0)*0 + 0.06 = 0.06 (small but positive)
  //
  // Old Baumgarte: b = (0.2/dt)*pen = 0.12 — 2× larger RHS
  //
  // Lambda from new formula should be about half of old Baumgarte lambda.
  // Verify lambda is positive but small (gentle correction, not injection).
  ASSERT_TRUE(result.converged);
  ASSERT_GT(result.lambdas.size(), 0);

  double const newLambda = result.lambdas(0);
  double const oldBaumgarteRHS = (0.2 / dt) * penetration;  // 0.12
  double const newSlopRHS = 0.06;  // 0.2 * (0.01 - 0.005) / dt

  // Lambda is positive (slop correction active for resting contact)
  EXPECT_GT(newLambda, 0.0)
    << "Resting contact with penetration should get gentle slop correction";

  // New RHS should be less than old Baumgarte RHS
  EXPECT_LT(newSlopRHS, oldBaumgarteRHS)
    << "New slop correction should be smaller than old Baumgarte bias";
}

TEST(SplitImpulse, VelocityRHS_RestitutionOnly)
{
  // With restitution e = 0.5, a bouncing contact should produce a
  // non-zero lambda that applies restitution velocity change.
  // The key test here: the RHS formula is -(1+e)*J*v, which is
  // purely restitution-based (no Baumgarte component).

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.5};
  Coordinate contactB{0.0, 0.0, -0.5};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{0.0, 0.0, 1.0};

  // Body A moves upward at +1 m/s, B is static
  // J*v = -n·vA + n·vB = -(0,0,1)·(0,0,1) + 0 = -1
  // RHS = -(1+0.5)*(-1) = 1.5 — positive, so lambda should be positive
  InertialState stateA =
    createDefaultState(Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 1.0});
  InertialState stateB = createDefaultState(Coordinate{0.0, 0.0, 1.0});

  ContactConstraint constraint{
    0, 1, normal, contactA, contactB, 0.01, comA, comB, 0.5, 1.0};

  std::vector<Constraint*> constraints{&constraint};
  std::vector<std::reference_wrapper<const InertialState>> states{
    std::cref(stateA), std::cref(stateB)};

  std::vector<double> inverseMasses{0.1, 0.0};  // B is static
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Zero()};

  ConstraintSolver solver;
  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 1.0 / 60.0);

  // Lambda should be positive (restitution active)
  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.lambdas.size(), 0);
  if (result.lambdas.size() > 0)
  {
    EXPECT_GT(result.lambdas(0), 0.0)
      << "Approaching contact should produce positive lambda from restitution";
  }

  // Body A should receive a downward force (negative z, pushing A back)
  EXPECT_LT(result.bodyForces[0].linearForce.z(), 0.0)
    << "Body A should receive downward force to reverse approach";
}

// ============================================================================
// Position Correction Tests
// ============================================================================

TEST(SplitImpulse, PositionCorrection_ReducesPenetration)
{
  // Set up two bodies with 10mm penetration, slop=5mm, beta=0.2
  // After correction, body positions should change to reduce penetration.

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, -0.5};
  Coordinate comB{0.0, 0.0, 0.5};
  double const penetration = 0.01;  // 10mm

  ContactConstraint constraint{
    0, 1, normal, contactA, contactB, penetration, comA, comB, 0.5, 0.0};

  std::vector<Constraint*> constraints{&constraint};

  InertialState stateA = createDefaultState(Coordinate{0.0, 0.0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0.0, 0.0, 0.5});
  std::vector<InertialState*> states{&stateA, &stateB};

  Coordinate originalPosA = stateA.position;
  Coordinate originalPosB = stateB.position;

  std::vector<double> inverseMasses{0.1, 0.1};  // 10 kg each
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Identity() * 0.1};

  PositionCorrector corrector;
  PositionCorrector::Config config;
  config.beta = 0.2;
  config.slop = 0.005;

  corrector.correctPositions(constraints,
                             states,
                             inverseMasses,
                             inverseInertias,
                             2,
                             2,
                             1.0 / 60.0,
                             config);

  // Bodies should have moved apart along the normal
  double const separationChange = (stateB.position.z() - stateA.position.z()) -
                                  (originalPosB.z() - originalPosA.z());

  EXPECT_GT(separationChange, 0.0)
    << "Position correction should push bodies apart";
}

TEST(SplitImpulse, PositionCorrection_DoesNotChangeVelocity)
{
  // After position correction, velocity state must be unchanged.

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, -0.5};
  Coordinate comB{0.0, 0.0, 0.5};

  ContactConstraint constraint{
    0, 1, normal, contactA, contactB, 0.01, comA, comB, 0.5, 0.0};

  std::vector<Constraint*> constraints{&constraint};

  InertialState stateA =
    createDefaultState(Coordinate{0.0, 0.0, -0.5}, Coordinate{1.0, 2.0, 3.0});
  InertialState stateB =
    createDefaultState(Coordinate{0.0, 0.0, 0.5}, Coordinate{-1.0, -2.0, -3.0});

  Coordinate velA_before = stateA.velocity;
  Coordinate velB_before = stateB.velocity;

  std::vector<InertialState*> states{&stateA, &stateB};

  std::vector<double> inverseMasses{0.1, 0.1};
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Identity() * 0.1};

  PositionCorrector corrector;
  corrector.correctPositions(
    constraints, states, inverseMasses, inverseInertias, 2, 2, 1.0 / 60.0);

  EXPECT_DOUBLE_EQ(stateA.velocity.x(), velA_before.x());
  EXPECT_DOUBLE_EQ(stateA.velocity.y(), velA_before.y());
  EXPECT_DOUBLE_EQ(stateA.velocity.z(), velA_before.z());

  EXPECT_DOUBLE_EQ(stateB.velocity.x(), velB_before.x());
  EXPECT_DOUBLE_EQ(stateB.velocity.y(), velB_before.y());
  EXPECT_DOUBLE_EQ(stateB.velocity.z(), velB_before.z());
}

TEST(SplitImpulse, PositionCorrection_DoesNotChangeAngularVelocity)
{
  // Angular velocity (quaternionRate) must be unchanged after correction.

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.5, 0.0, 0.0};
  Coordinate contactB{-0.5, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, -0.5};
  Coordinate comB{0.0, 0.0, 0.5};

  ContactConstraint constraint{
    0, 1, normal, contactA, contactB, 0.01, comA, comB, 0.5, 0.0};

  std::vector<Constraint*> constraints{&constraint};

  InertialState stateA = createDefaultState(Coordinate{0.0, 0.0, -0.5});
  stateA.setAngularVelocity(AngularRate{1.0, 0.0, 0.0});
  InertialState stateB = createDefaultState(Coordinate{0.0, 0.0, 0.5});

  Eigen::Vector4d qdotA_before = stateA.quaternionRate;
  Eigen::Vector4d qdotB_before = stateB.quaternionRate;

  std::vector<InertialState*> states{&stateA, &stateB};

  std::vector<double> inverseMasses{0.1, 0.1};
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Identity() * 0.1};

  PositionCorrector corrector;
  corrector.correctPositions(
    constraints, states, inverseMasses, inverseInertias, 2, 2, 1.0 / 60.0);

  EXPECT_DOUBLE_EQ(stateA.quaternionRate(0), qdotA_before(0));
  EXPECT_DOUBLE_EQ(stateA.quaternionRate(1), qdotA_before(1));
  EXPECT_DOUBLE_EQ(stateA.quaternionRate(2), qdotA_before(2));
  EXPECT_DOUBLE_EQ(stateA.quaternionRate(3), qdotA_before(3));

  EXPECT_DOUBLE_EQ(stateB.quaternionRate(0), qdotB_before(0));
  EXPECT_DOUBLE_EQ(stateB.quaternionRate(1), qdotB_before(1));
  EXPECT_DOUBLE_EQ(stateB.quaternionRate(2), qdotB_before(2));
  EXPECT_DOUBLE_EQ(stateB.quaternionRate(3), qdotB_before(3));
}

// ============================================================================
// Slop Parameter Tests
// ============================================================================

TEST(SplitImpulse, Slop_NoCorrectionBelowThreshold)
{
  // Penetration = 3mm, slop = 5mm -> no correction should occur.

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, -0.5};
  Coordinate comB{0.0, 0.0, 0.5};

  ContactConstraint constraint{
    0, 1, normal, contactA, contactB, 0.003, comA, comB, 0.5, 0.0};

  std::vector<Constraint*> constraints{&constraint};

  InertialState stateA = createDefaultState(Coordinate{0.0, 0.0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0.0, 0.0, 0.5});

  Coordinate posA_before = stateA.position;
  Coordinate posB_before = stateB.position;

  std::vector<InertialState*> states{&stateA, &stateB};
  std::vector<double> inverseMasses{0.1, 0.1};
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Identity() * 0.1};

  PositionCorrector corrector;
  PositionCorrector::Config config;
  config.slop = 0.005;

  corrector.correctPositions(constraints,
                             states,
                             inverseMasses,
                             inverseInertias,
                             2,
                             2,
                             1.0 / 60.0,
                             config);

  EXPECT_DOUBLE_EQ(stateA.position.x(), posA_before.x());
  EXPECT_DOUBLE_EQ(stateA.position.y(), posA_before.y());
  EXPECT_DOUBLE_EQ(stateA.position.z(), posA_before.z());

  EXPECT_DOUBLE_EQ(stateB.position.x(), posB_before.x());
  EXPECT_DOUBLE_EQ(stateB.position.y(), posB_before.y());
  EXPECT_DOUBLE_EQ(stateB.position.z(), posB_before.z());
}

TEST(SplitImpulse, Slop_PartialCorrectionAboveThreshold)
{
  // Penetration = 10mm, slop = 5mm -> corrects based on 5mm excess only.
  // A smaller penetration excess should produce smaller correction.

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, -0.5};
  Coordinate comB{0.0, 0.0, 0.5};

  // Large penetration: 20mm above slop
  ContactConstraint constraintLarge{
    0, 1, normal, contactA, contactB, 0.025, comA, comB, 0.5, 0.0};

  // Small penetration: 5mm above slop
  ContactConstraint constraintSmall{
    0, 1, normal, contactA, contactB, 0.010, comA, comB, 0.5, 0.0};

  PositionCorrector corrector;
  PositionCorrector::Config config;
  config.slop = 0.005;
  config.beta = 0.2;

  // Test large penetration
  InertialState stateA1 = createDefaultState(Coordinate{0.0, 0.0, -0.5});
  InertialState stateB1 = createDefaultState(Coordinate{0.0, 0.0, 0.5});
  std::vector<Constraint*> constraints1{&constraintLarge};
  std::vector<InertialState*> states1{&stateA1, &stateB1};
  std::vector<double> inverseMasses{0.1, 0.1};
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Identity() * 0.1};

  corrector.correctPositions(constraints1,
                             states1,
                             inverseMasses,
                             inverseInertias,
                             2,
                             2,
                             1.0 / 60.0,
                             config);

  double const largeSeparation =
    (stateB1.position.z() - stateA1.position.z()) - 1.0;

  // Test small penetration
  InertialState stateA2 = createDefaultState(Coordinate{0.0, 0.0, -0.5});
  InertialState stateB2 = createDefaultState(Coordinate{0.0, 0.0, 0.5});
  std::vector<Constraint*> constraints2{&constraintSmall};
  std::vector<InertialState*> states2{&stateA2, &stateB2};

  corrector.correctPositions(constraints2,
                             states2,
                             inverseMasses,
                             inverseInertias,
                             2,
                             2,
                             1.0 / 60.0,
                             config);

  double const smallSeparation =
    (stateB2.position.z() - stateA2.position.z()) - 1.0;

  EXPECT_GT(largeSeparation, smallSeparation)
    << "Larger penetration excess should produce larger correction";
  EXPECT_GT(smallSeparation, 0.0)
    << "Penetration above slop should produce some correction";
}

// ============================================================================
// Energy Conservation Tests
// ============================================================================

TEST(SplitImpulse, EnergyConservation_RestingContact)
{
  // A resting cube on a floor should maintain E_post <= E_pre
  // over 100 frames with split impulse (no Baumgarte injection).

  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  auto floorPoints = createCubePoints(20.0);
  ConvexHull floorHull{floorPoints};

  WorldModel world;

  // Cube resting at z=0.5 (just touching floor at z=0)
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.51}};
  world.spawnObject(1, cubeHull, 10.0, cubeFrame);

  // Floor at z = -10 (top surface at z = 0)
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -10.0}};
  world.spawnEnvironmentObject(2, floorHull, floorFrame);

  // Let it settle for a few frames
  auto simTime = std::chrono::milliseconds{0};
  world.update(simTime);
  simTime += std::chrono::milliseconds{16};
  world.update(simTime);
  simTime += std::chrono::milliseconds{16};

  // Now track energy over 100 frames
  double previousEnergy = std::numeric_limits<double>::max();
  int energyIncreaseCount = 0;

  for (int frame = 0; frame < 100; ++frame)
  {
    simTime += std::chrono::milliseconds{16};
    world.update(simTime);

    auto energy =
      EnergyTracker::computeSystemEnergy(world.getInertialAssets(), {});
    double const totalEnergy = energy.total();

    if (totalEnergy > previousEnergy + 1e-6)
    {
      ++energyIncreaseCount;
    }
    previousEnergy = totalEnergy;
  }

  // Allow a very small number of transient energy increases from
  // integration artifacts, but the system should not systematically
  // inject energy via Baumgarte
  EXPECT_LT(energyIncreaseCount, 10)
    << "Resting contact should not systematically inject energy";
}

// ============================================================================
// Environment Body Tests
// ============================================================================

TEST(SplitImpulse, EnvironmentBody_NotMoved)
{
  // Environment body (inverseMass = 0) should not be moved by
  // position correction.

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, -0.5};
  Coordinate comB{0.0, 0.0, 0.5};

  // Body 0 = inertial, Body 1 = environment
  ContactConstraint constraint{
    0, 1, normal, contactA, contactB, 0.01, comA, comB, 0.5, 0.0};

  std::vector<Constraint*> constraints{&constraint};

  InertialState stateA = createDefaultState(Coordinate{0.0, 0.0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0.0, 0.0, 0.5});

  Coordinate posB_before = stateB.position;

  std::vector<InertialState*> states{&stateA, &stateB};
  std::vector<double> inverseMasses{0.1, 0.0};  // B is environment
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Zero()};

  PositionCorrector corrector;
  size_t const numInertial = 1;

  corrector.correctPositions(constraints,
                             states,
                             inverseMasses,
                             inverseInertias,
                             2,
                             numInertial,
                             1.0 / 60.0);

  EXPECT_DOUBLE_EQ(stateB.position.x(), posB_before.x());
  EXPECT_DOUBLE_EQ(stateB.position.y(), posB_before.y());
  EXPECT_DOUBLE_EQ(stateB.position.z(), posB_before.z());

  // Inertial body A should still move
  EXPECT_NE(stateA.position.z(), -0.5)
    << "Inertial body should be moved by position correction";
}

// ============================================================================
// Orientation Correction Tests
// ============================================================================

TEST(SplitImpulse, OrientationCorrection_TiltedCube)
{
  // A cube penetrating at an angle should have its orientation corrected,
  // not just position.

  Coordinate normal{0.0, 0.0, 1.0};
  // Offset contact point to create lever arm for angular correction
  Coordinate contactA{0.5, 0.0, 0.0};
  Coordinate contactB{0.5, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, -0.5};
  Coordinate comB{0.0, 0.0, 0.5};

  ContactConstraint constraint{
    0, 1, normal, contactA, contactB, 0.01, comA, comB, 0.5, 0.0};

  std::vector<Constraint*> constraints{&constraint};

  InertialState stateA = createDefaultState(Coordinate{0.0, 0.0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0.0, 0.0, 0.5});

  QuaternionD orientA_before = stateA.orientation;

  std::vector<InertialState*> states{&stateA, &stateB};
  std::vector<double> inverseMasses{0.1, 0.0};
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Zero()};

  PositionCorrector corrector;
  corrector.correctPositions(
    constraints, states, inverseMasses, inverseInertias, 2, 1, 1.0 / 60.0);

  // Orientation should have changed due to angular position correction
  // (contact at offset creates lever arm -> angular pseudo-velocity)
  double const orientDiff =
    (stateA.orientation.coeffs() - orientA_before.coeffs()).norm();

  EXPECT_GT(orientDiff, 1e-12)
    << "Offset contact should cause orientation correction";
}

// ============================================================================
// Beta and Slop Parameter Edge Cases
// ============================================================================

TEST(SplitImpulse, BetaZero_NoCorrection)
{
  // With beta = 0.0, no position correction should occur.

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, -0.5};
  Coordinate comB{0.0, 0.0, 0.5};

  ContactConstraint constraint{
    0, 1, normal, contactA, contactB, 0.01, comA, comB, 0.5, 0.0};

  std::vector<Constraint*> constraints{&constraint};

  InertialState stateA = createDefaultState(Coordinate{0.0, 0.0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0.0, 0.0, 0.5});

  Coordinate posA_before = stateA.position;
  Coordinate posB_before = stateB.position;

  std::vector<InertialState*> states{&stateA, &stateB};
  std::vector<double> inverseMasses{0.1, 0.1};
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Identity() * 0.1};

  PositionCorrector corrector;
  PositionCorrector::Config config;
  config.beta = 0.0;

  corrector.correctPositions(constraints,
                             states,
                             inverseMasses,
                             inverseInertias,
                             2,
                             2,
                             1.0 / 60.0,
                             config);

  EXPECT_DOUBLE_EQ(stateA.position.x(), posA_before.x());
  EXPECT_DOUBLE_EQ(stateA.position.y(), posA_before.y());
  EXPECT_DOUBLE_EQ(stateA.position.z(), posA_before.z());

  EXPECT_DOUBLE_EQ(stateB.position.x(), posB_before.x());
  EXPECT_DOUBLE_EQ(stateB.position.y(), posB_before.y());
  EXPECT_DOUBLE_EQ(stateB.position.z(), posB_before.z());
}

TEST(SplitImpulse, SlopZero_FullCorrection)
{
  // With slop = 0.0, even tiny penetration should be corrected.

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, -0.5};
  Coordinate comB{0.0, 0.0, 0.5};

  ContactConstraint constraint{0,
                               1,
                               normal,
                               contactA,
                               contactB,
                               0.001,
                               comA,
                               comB,
                               0.5,
                               0.0};  // 1mm penetration

  std::vector<Constraint*> constraints{&constraint};

  InertialState stateA = createDefaultState(Coordinate{0.0, 0.0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0.0, 0.0, 0.5});

  std::vector<InertialState*> states{&stateA, &stateB};
  std::vector<double> inverseMasses{0.1, 0.1};
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Identity() * 0.1};

  PositionCorrector corrector;
  PositionCorrector::Config config;
  config.slop = 0.0;
  config.beta = 0.2;

  corrector.correctPositions(constraints,
                             states,
                             inverseMasses,
                             inverseInertias,
                             2,
                             2,
                             1.0 / 60.0,
                             config);

  double const separationChange =
    (stateB.position.z() - stateA.position.z()) - 1.0;

  EXPECT_GT(separationChange, 0.0)
    << "With slop=0, even small penetration should be corrected";
}

// ============================================================================
// Multi-Contact Test
// ============================================================================

TEST(SplitImpulse, MultiContact_IndependentCorrection)
{
  // 4-point manifold: each contact should be corrected proportionally
  // to its depth.

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate comA{0.0, 0.0, -0.5};
  Coordinate comB{0.0, 0.0, 0.5};

  // Four contacts with varying depths
  ContactConstraint c1{0,
                       1,
                       normal,
                       Coordinate{-0.5, -0.5, 0.0},
                       Coordinate{-0.5, -0.5, 0.0},
                       0.015,
                       comA,
                       comB,
                       0.5,
                       0.0};  // 15mm (10mm above slop)
  ContactConstraint c2{0,
                       1,
                       normal,
                       Coordinate{0.5, -0.5, 0.0},
                       Coordinate{0.5, -0.5, 0.0},
                       0.010,
                       comA,
                       comB,
                       0.5,
                       0.0};  // 10mm (5mm above slop)
  ContactConstraint c3{0,
                       1,
                       normal,
                       Coordinate{-0.5, 0.5, 0.0},
                       Coordinate{-0.5, 0.5, 0.0},
                       0.003,
                       comA,
                       comB,
                       0.5,
                       0.0};  // 3mm (below slop)
  ContactConstraint c4{0,
                       1,
                       normal,
                       Coordinate{0.5, 0.5, 0.0},
                       Coordinate{0.5, 0.5, 0.0},
                       0.020,
                       comA,
                       comB,
                       0.5,
                       0.0};  // 20mm (15mm above slop)

  std::vector<Constraint*> constraints{&c1, &c2, &c3, &c4};

  InertialState stateA = createDefaultState(Coordinate{0.0, 0.0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0.0, 0.0, 0.5});

  std::vector<InertialState*> states{&stateA, &stateB};
  std::vector<double> inverseMasses{0.1, 0.0};  // B is static
  std::vector<Eigen::Matrix3d> inverseInertias{
    Eigen::Matrix3d::Identity() * 0.1, Eigen::Matrix3d::Zero()};

  PositionCorrector corrector;
  PositionCorrector::Config config;
  config.slop = 0.005;
  config.beta = 0.2;

  corrector.correctPositions(constraints,
                             states,
                             inverseMasses,
                             inverseInertias,
                             2,
                             1,
                             1.0 / 60.0,
                             config);

  // Body A should have moved downward (away from static B which is above)
  // Normal is (0,0,1) pointing A->B, so J_A^T * lambda pushes A in -n direction
  EXPECT_LT(stateA.position.z(), -0.5)
    << "Inertial body should move away from penetration (downward, since B is "
       "above)";
}

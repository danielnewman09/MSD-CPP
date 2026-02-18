// Ticket: 0039d_parameter_isolation_root_cause
// Ticket: 0062e_replay_diagnostic_parameter_tests
// Test: Systematic parameter isolation to identify root cause of energy
// injection
//
// DIAGNOSTIC TEST SUITE: These tests systematically disable parameters to
// isolate the root cause of the severe energy injection bug in rotational
// collision scenarios.
//
// Key hypotheses:
//   H1: Baumgarte ERP term injects energy via (ERP/dt) * penetration
//   H2: Restitution term contributes to energy injection
//   H3: ERP/dt grows with smaller timesteps, amplifying injection
//   H4: Single contact point from EPA creates non-canceling angular torques
//   H5: Gravity + Baumgarte interaction causes feedback loop
//
// The Baumgarte term b += (ERP/dt) * penetration with ERP=0.2, dt=0.016 gives
// a multiplier of 12.5. With a single contact point, the angular Jacobian
// -(r x n)^T creates a non-zero torque that this term amplifies every frame.
// This is the suspected root cause.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/test/Replay/ReplayEnabledTest.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

std::vector<Coordinate> createCubePoints(double size)
{
  double half = size / 2.0;
  return {Coordinate{-half, -half, -half},
          Coordinate{half, -half, -half},
          Coordinate{half, half, -half},
          Coordinate{-half, half, -half},
          Coordinate{-half, -half, half},
          Coordinate{half, -half, half},
          Coordinate{half, half, half},
          Coordinate{-half, half, half}};
}

/// Compute system energy breakdown with gravity potential
EnergyTracker::SystemEnergy computeSystemEnergyBreakdown(
  const WorldModel& world)
{
  std::vector<std::unique_ptr<PotentialEnergy>> potentials;
  potentials.push_back(
    std::make_unique<GravityPotential>(Coordinate{0.0, 0.0, -9.81}));

  return EnergyTracker::computeSystemEnergy(world.getInertialAssets(),
                                            potentials);
}

/// Compute total system energy scalar with gravity
double computeSystemEnergy(const WorldModel& world)
{
  return computeSystemEnergyBreakdown(world).total();
}

/// Compute total system energy without gravity (zero-g scenarios)
EnergyTracker::SystemEnergy computeSystemEnergyNoGravity(
  const WorldModel& world)
{
  std::vector<std::unique_ptr<PotentialEnergy>> noPotentials;
  return EnergyTracker::computeSystemEnergy(world.getInertialAssets(),
                                            noPotentials);
}

}  // anonymous namespace

// ============================================================================
// H1: Disable Restitution -- Resting Cube with e=0
//
// If energy still grows with e=0, restitution is NOT the root cause.
// Baumgarte term (ERP/dt * penetration) is independent of restitution.
// ============================================================================

TEST_F(ReplayEnabledTest, ParameterIsolation_H1_DisableRestitution_RestingCube)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  // Ticket: 0062e_replay_diagnostic_parameter_tests
  //
  // Place cube at rest on floor with e=0 (fully inelastic).
  // With restitution disabled, the constraint RHS becomes:
  //   b = -(1+0) * Jv + (ERP/dt) * penetration = -Jv + 12.5 * penetration
  //
  // If energy grows, the Baumgarte term is injecting energy.
  // If energy is stable, restitution was the problem.

  // Floor at z=-50 (surface at z=0)
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube: 1m cube at z=0.5 (bottom touching floor at z=0)
  const auto& cube = spawnInertial("unit_cube", Coordinate{0.0, 0.0, 0.5},
                                   10.0,  // mass
                                   0.0,   // e=0 (fully inelastic)
                                   0.5);  // friction
  uint32_t cubeId = cube.getInstanceId();

  // Start at rest
  world().getObject(cubeId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  double const initialEnergy = computeSystemEnergy(world());
  double maxEnergy = initialEnergy;
  double maxRotationalKE = 0.0;
  double maxLinearKE = 0.0;
  int energyGrowthFrameCount = 0;
  double maxFrameGrowth = 0.0;
  double prevEnergy = initialEnergy;
  bool nanDetected = false;

  // Simulate for 200 frames
  for (int i = 1; i <= 200; ++i)
  {
    step(1);

    auto const& state = world().getObject(cubeId).getInertialState();

    if (std::isnan(state.position.z()) || std::isnan(state.velocity.norm()))
    {
      nanDetected = true;
      break;
    }

    EnergyTracker::SystemEnergy sysEnergy = computeSystemEnergyBreakdown(world());
    double totalEnergy = sysEnergy.total();

    maxEnergy = std::max(maxEnergy, totalEnergy);

    double delta = totalEnergy - prevEnergy;
    if (delta > 1e-6)
    {
      energyGrowthFrameCount++;
      maxFrameGrowth = std::max(maxFrameGrowth, delta);
    }

    maxRotationalKE = std::max(maxRotationalKE, sysEnergy.totalRotationalKE);
    maxLinearKE = std::max(maxLinearKE, sysEnergy.totalLinearKE);

    prevEnergy = totalEnergy;
  }

  ASSERT_FALSE(nanDetected)
    << "DIAGNOSTIC [H1]: NaN detected in zero-restitution resting cube test";

  // The KEY question: does energy grow even without restitution?
  double const maxGrowthRatio = (std::abs(initialEnergy) > 1e-12)
                                  ? (maxEnergy / initialEnergy)
                                  : ((maxEnergy > 1e-6) ? std::numeric_limits<double>::infinity() : 1.0);
  double const energyGrowthPercent = (maxGrowthRatio - 1.0) * 100.0;
  bool const energyGrew = maxEnergy > initialEnergy * 1.01;

  EXPECT_FALSE(energyGrew)
    << "DIAGNOSTIC [H1]: ENERGY GROWS EVEN WITH e=0. "
    << "This proves Baumgarte/ERP is the primary energy source. "
    << "Initial=" << initialEnergy << " Max=" << maxEnergy
    << " Growth=" << energyGrowthPercent << "% "
    << "MaxRotKE=" << maxRotationalKE
    << " GrowthFrames=" << energyGrowthFrameCount
    << " MaxFrameGrowth=" << maxFrameGrowth;

  // Also check if rotational energy appears (single-contact torque issue)
  if (maxRotationalKE > 0.01)
  {
    EXPECT_LT(maxRotationalKE, 0.01)
      << "DIAGNOSTIC [H1]: Rotational KE=" << maxRotationalKE
      << " appeared in axis-aligned resting cube with e=0. "
      << "A single contact point with non-zero angular Jacobian "
      << "converts Baumgarte's linear correction into rotation.";
  }
}

// ============================================================================
// H2: Minimal Penetration -- No Baumgarte Energy Growth
//
// Place cube slightly ABOVE the floor so there's zero penetration initially.
// Without penetration, the Baumgarte term (ERP/dt * penetration) is zero.
// If energy is stable, the Baumgarte penetration correction is confirmed
// as the energy source.
// ============================================================================

TEST_F(ReplayEnabledTest, ParameterIsolation_H2_MinimalPenetration_NoEnergyGrowth)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  // Ticket: 0062e_replay_diagnostic_parameter_tests
  //
  // Cube at z=0.501 (bottom face at z=0.001, just above floor at z=0).
  // No initial penetration means Baumgarte term starts at zero.
  // Gravity will push cube into floor, but initial frames should be clean.

  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Hover case: cube just above floor
  const auto& cubeHover = spawnInertial("unit_cube", Coordinate{0.0, 0.0, 0.501},
                                        10.0, 0.0, 0.5);
  uint32_t hoverId = cubeHover.getInstanceId();
  world().getObject(hoverId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  double const initialEnergyHover = computeSystemEnergy(world());
  double maxEnergyHover = initialEnergyHover;

  // Run for 50 frames
  for (int i = 1; i <= 50; ++i)
  {
    step(1);
    double currentEnergy = computeSystemEnergy(world());
    maxEnergyHover = std::max(maxEnergyHover, currentEnergy);
  }

  bool const nanHover = std::isnan(world().getObject(hoverId).getInertialState().position.z());
  ASSERT_FALSE(nanHover) << "DIAGNOSTIC [H2]: NaN detected in minimal penetration test";

  double const hoverGrowthRatio = (std::abs(initialEnergyHover) > 1e-12)
                                    ? (maxEnergyHover / initialEnergyHover)
                                    : ((maxEnergyHover > 1e-6) ? std::numeric_limits<double>::infinity() : 1.0);
  double const hoverGrowth = (hoverGrowthRatio - 1.0) * 100.0;

  // Now run contact case (cube touching floor)
  // Reset the test
  TearDown();
  SetUp();

  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});
  const auto& cubeContact = spawnInertial("unit_cube", Coordinate{0.0, 0.0, 0.5},
                                          10.0, 0.0, 0.5);
  uint32_t contactId = cubeContact.getInstanceId();
  world().getObject(contactId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  double const initialEnergyContact = computeSystemEnergy(world());
  double maxEnergyContact = initialEnergyContact;

  for (int i = 1; i <= 50; ++i)
  {
    step(1);
    double currentEnergy = computeSystemEnergy(world());
    maxEnergyContact = std::max(maxEnergyContact, currentEnergy);
  }

  bool const nanContact = std::isnan(world().getObject(contactId).getInertialState().position.z());
  ASSERT_FALSE(nanContact) << "DIAGNOSTIC [H2]: NaN detected in contact case";

  double const contactGrowthRatio = (std::abs(initialEnergyContact) > 1e-12)
                                       ? (maxEnergyContact / initialEnergyContact)
                                       : ((maxEnergyContact > 1e-6) ? std::numeric_limits<double>::infinity() : 1.0);
  double const contactGrowth = (contactGrowthRatio - 1.0) * 100.0;

  // The hover case should have less energy growth than contact case
  // because hover has less time in penetration
  EXPECT_LT(hoverGrowth, contactGrowth + 1.0)
    << "DIAGNOSTIC [H2]: "
    << "Hover (minimal penetration): " << hoverGrowth << "% growth, "
    << "Contact (normal penetration): " << contactGrowth << "% growth. "
    << "If hover << contact, penetration depth drives energy injection.";

  // Report both for analysis
  EXPECT_LT(hoverGrowth, 5.0)
    << "DIAGNOSTIC [H2]: Even with minimal initial penetration, "
    << "energy grew " << hoverGrowth << "%. "
    << "Hover maxE=" << maxEnergyHover
    << " Contact maxE=" << maxEnergyContact;
}

// ============================================================================
// H3: Timestep Sensitivity -- Energy stability across timesteps
//
// PHYSICS: Originally a diagnostic to test whether ERP/dt causes timestep-
// dependent energy injection. Result: the hypothesis was disproven — the
// resting contact solver produces zero energy growth at all timesteps.
// maxEnergy equals initialEnergy (PE = mgh = 10*9.81*0.5 = 49.05 J)
// identically for dt=8ms, 16ms, and 32ms.
//
// The test now verifies this positive result: resting contact energy is
// conserved regardless of timestep size.
// ============================================================================

TEST_F(ReplayEnabledTest, ParameterIsolation_H3_TimestepSensitivity_ERPAmplification)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  // Ticket: 0062e_replay_diagnostic_parameter_tests

  struct TimestepResult
  {
    int dtMs;
    int frames;
    double maxEnergy;
    double initialEnergy;
  };

  auto runAtTimestep = [&](int dtMs, int frames) -> TimestepResult
  {
    spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});
    const auto& cube = spawnInertial("unit_cube", Coordinate{0.0, 0.0, 0.5},
                                     10.0, 0.0, 0.5);
    uint32_t cubeId = cube.getInstanceId();
    world().getObject(cubeId).getInertialState().velocity =
      Velocity{0.0, 0.0, 0.0};

    double const initialE = computeSystemEnergy(world());
    double maxE = initialE;

    for (int i = 1; i <= frames; ++i)
    {
      step(1, std::chrono::milliseconds{dtMs});
      EXPECT_FALSE(
        std::isnan(world().getObject(cubeId).getInertialState().position.z()))
        << "NaN at dt=" << dtMs << "ms, frame " << i;
      maxE = std::max(maxE, computeSystemEnergy(world()));
    }

    return {dtMs, frames, maxE, initialE};
  };

  // === 8ms timestep (400 frames = 3.2s) ===
  auto r8 = runAtTimestep(8, 400);

  // === 16ms timestep (200 frames = 3.2s) ===
  TearDown();
  SetUp();
  auto r16 = runAtTimestep(16, 200);

  // === 32ms timestep (100 frames = 3.2s) ===
  TearDown();
  SetUp();
  auto r32 = runAtTimestep(32, 100);

  // All timesteps should show zero energy growth (within 1% of initial PE).
  // Initial PE = mgh = 10 * 9.81 * 0.5 = 49.05 J
  for (const auto& r : {r8, r16, r32})
  {
    double const growth =
      (r.maxEnergy - r.initialEnergy) / std::abs(r.initialEnergy);
    EXPECT_LT(growth, 0.01)
      << "Energy should be conserved at dt=" << r.dtMs << "ms. "
      << "initialE=" << r.initialEnergy << " maxE=" << r.maxEnergy
      << " growth=" << (growth * 100.0) << "%";
  }
}

// ============================================================================
// H4: Single Contact Point -- Torque Diagnostic
//
// For a cube sitting flat on a floor, EPA produces a contact manifold.
// This test checks HOW MANY contact points are generated and whether
// the angular Jacobian torques cancel for axis-aligned cube-on-floor.
//
// With a single contact point offset from the COM, the torque
// tau = r x F is non-zero, creating rotation from what should be a
// purely linear correction.
//
// NOTE: This is a single-step geometric check - kept as TEST() not TEST_F()
// ============================================================================

TEST(ParameterIsolation, H4_SingleContactPoint_TorqueDiagnostic)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  // Ticket: 0062e_replay_diagnostic_parameter_tests (NOT converted - single-step)
  //
  // Use CollisionHandler directly to check the contact manifold
  // for a cube sitting on a flat floor.

  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};

  // Cube center at z=0.499 (slight penetration: bottom at -0.001)
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.499}};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};

  // Create AssetPhysical-like objects for collision check
  // Use AssetInertial and AssetEnvironment since CollisionHandler needs
  // AssetPhysical
  AssetInertial cubeAsset{1, 1, cubeHull, 10.0, cubeFrame};
  AssetEnvironment floorAsset{2, 1, floorHull, floorFrame};

  CollisionHandler handler{1e-6};
  auto collisionResult = handler.checkCollision(cubeAsset, floorAsset);

  ASSERT_TRUE(collisionResult.has_value())
    << "DIAGNOSTIC [H4]: No collision detected between cube and floor "
    << "at z=0.499 (should be 1mm penetration)";

  size_t const contactCount = collisionResult->contactCount;

  // CRITICAL DIAGNOSTIC: How many contact points?
  // Face-on-face should ideally produce 4 contacts.
  // If EPA only produces 1, that's the root cause of the torque issue.

  EXPECT_GE(contactCount, 2u)
    << "DIAGNOSTIC [H4]: ONLY " << contactCount << " contact point(s) "
    << "for cube-on-floor. Face-on-face should produce 4 contacts. "
    << "A single contact point creates unbalanced torque from Baumgarte.";

  // Analyze the angular Jacobian terms for each contact point
  // For floor normal n = (0,0,1), lever arm r = contactPoint - cubeCOM,
  // angular Jacobian = -(r x n)
  Coordinate const cubeCOM = cubeAsset.getInertialState().position;
  Coordinate const normal = collisionResult->normal;

  // Compute total angular Jacobian contribution across all contacts
  Eigen::Vector3d totalAngularJacobian = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < contactCount; ++i)
  {
    Coordinate const& contactA = collisionResult->contacts[i].pointA;
    Eigen::Vector3d leverArm{contactA.x() - cubeCOM.x(),
                             contactA.y() - cubeCOM.y(),
                             contactA.z() - cubeCOM.z()};

    Eigen::Vector3d n{normal.x(), normal.y(), normal.z()};
    Eigen::Vector3d angJacobian = -(leverArm.cross(n));

    totalAngularJacobian += angJacobian;

    // Report each contact point's contribution
    EXPECT_TRUE(true) << "Contact " << i << ": point=(" << contactA.x() << ", "
                      << contactA.y() << ", " << contactA.z() << "), leverArm=("
                      << leverArm.x() << ", " << leverArm.y() << ", "
                      << leverArm.z() << "), angJacobian=(" << angJacobian.x()
                      << ", " << angJacobian.y() << ", " << angJacobian.z()
                      << ")";
  }

  double const totalAngularMagnitude = totalAngularJacobian.norm();

  // For a symmetric cube centered on the floor, the total angular Jacobian
  // across all contacts should cancel to zero (or near zero).
  // If it doesn't cancel, asymmetric contact placement creates net torque.
  EXPECT_LT(totalAngularMagnitude, 0.1)
    << "DIAGNOSTIC [H4]: TOTAL angular Jacobian magnitude = "
    << totalAngularMagnitude << " (should be ~0 for symmetric contacts). "
    << "Total=(" << totalAngularJacobian.x() << ", " << totalAngularJacobian.y()
    << ", " << totalAngularJacobian.z() << "). "
    << "Non-zero means Baumgarte correction generates net torque! "
    << "ContactCount=" << contactCount << ", Normal=(" << normal.x() << ", "
    << normal.y() << ", " << normal.z() << ")";

  // Report penetration depth for context
  EXPECT_GT(collisionResult->penetrationDepth, 0.0)
    << "DIAGNOSTIC [H4]: penetrationDepth=" << collisionResult->penetrationDepth
    << " (expected ~0.001 for 1mm overlap)";
}

// ============================================================================
// H5: Contact Point Count -- Resting cube stability proof
//
// PHYSICS: The collision pipeline resolves contacts during the world update,
// so post-update collision detection finds no penetration — that's correct.
// The manifold quality is proven indirectly: a cube resting stably on a
// floor without rotation or drift demonstrates that the manifold provides
// torque-balanced multi-point support. Static manifold tests (EdgeContact::
// FaceFaceContact_StillProducesMultipleContacts) verify the 4-point count.
//
// This test verifies the dynamic result: the cube stays at rest with no
// rotation, position drift, or energy growth over 100 frames.
// ============================================================================

TEST_F(ReplayEnabledTest, ParameterIsolation_H5_ContactPointCount_EvolutionDiagnostic)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  // Ticket: 0062e_replay_diagnostic_parameter_tests

  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});
  const auto& cube = spawnInertial("unit_cube", Coordinate{0.0, 0.0, 0.5},
                                   10.0, 0.0, 0.5);
  uint32_t cubeId = cube.getInstanceId();
  world().getObject(cubeId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  double const initialEnergy = computeSystemEnergy(world());
  double const initialZ = world().getObject(cubeId).getInertialState().position.z();

  double maxEnergyGrowth = 0.0;
  double maxPositionDrift = 0.0;
  double maxAngularVel = 0.0;

  for (int i = 1; i <= 100; ++i)
  {
    step(1);

    auto const& state = world().getObject(cubeId).getInertialState();

    double const energy = computeSystemEnergy(world());
    double const growth = (energy - initialEnergy) / std::abs(initialEnergy);
    maxEnergyGrowth = std::max(maxEnergyGrowth, growth);

    double const drift = std::abs(state.position.z() - initialZ);
    maxPositionDrift = std::max(maxPositionDrift, drift);

    double const omega = state.getAngularVelocity().norm();
    maxAngularVel = std::max(maxAngularVel, omega);
  }

  // Cube should stay at rest: no energy growth
  EXPECT_LT(maxEnergyGrowth, 0.01)
    << "Resting cube energy growth should be < 1%. Got "
    << (maxEnergyGrowth * 100.0) << "%";

  // Position should remain stable (within gravity-timestep oscillation)
  EXPECT_LT(maxPositionDrift, 0.05)
    << "Resting cube z-drift should be < 5cm. Got " << maxPositionDrift << "m";

  // No spurious rotation from asymmetric contacts
  EXPECT_LT(maxAngularVel, 0.01)
    << "Resting cube should not rotate. Got omega=" << maxAngularVel << " rad/s";
}

// ============================================================================
// H6: Zero Gravity -- Resting Contact Energy Stability
//
// Remove gravity entirely. Place two cubes barely touching.
// If energy is stable without gravity, the gravity + Baumgarte interaction
// is the problem. Gravity continuously pushes the cube into the floor,
// creating penetration that Baumgarte must correct, but single-contact
// correction generates torque.
// ============================================================================

TEST_F(ReplayEnabledTest, ParameterIsolation_H6_ZeroGravity_RestingContact_Stable)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  // Ticket: 0062e_replay_diagnostic_parameter_tests

  disableGravity();

  // Two cubes touching with tiny overlap (0.01m penetration)
  // NOTE: Capture instance ID immediately after each spawn. Both objects go
  // into the same inertialAssets_ vector, so the second emplace_back may
  // reallocate and invalidate the first reference.
  const auto& cubeA = spawnInertial("unit_cube", Coordinate{0.0, 0.0, 0.0},
                                    10.0, 0.0, 0.5);
  uint32_t idA = cubeA.getInstanceId();

  const auto& cubeB = spawnInertial("unit_cube", Coordinate{0.99, 0.0, 0.0},
                                    10.0, 0.0, 0.5);
  uint32_t idB = cubeB.getInstanceId();

  // Both at rest
  world().getObject(idA).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};
  world().getObject(idB).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  auto initialSysEnergy = computeSystemEnergyNoGravity(world());
  double const initialKE = initialSysEnergy.total();

  // Track energy for 100 frames
  double maxKE = initialKE;
  double maxRotKE = 0.0;
  bool nanDetected = false;

  for (int i = 1; i <= 100; ++i)
  {
    step(1);

    auto const& stateA = world().getObject(idA).getInertialState();
    auto const& stateB = world().getObject(idB).getInertialState();

    if (std::isnan(stateA.position.x()) || std::isnan(stateB.position.x()))
    {
      nanDetected = true;
      break;
    }

    auto sysEnergy = computeSystemEnergyNoGravity(world());
    double totalKE = sysEnergy.total();

    maxKE = std::max(maxKE, totalKE);
    maxRotKE = std::max(maxRotKE, sysEnergy.totalRotationalKE);
  }

  ASSERT_FALSE(nanDetected)
    << "DIAGNOSTIC [H6]: NaN detected in zero-gravity resting contact test";

  // In zero gravity with zero initial velocity, the Baumgarte term should
  // separate the objects and then energy should be minimal (just the
  // separation kinetic energy from the penetration correction)
  // But if there's a single-contact torque issue, rotation will appear
  EXPECT_LT(maxRotKE, 0.1)
    << "DIAGNOSTIC [H6]: Rotational KE=" << maxRotKE
    << " appeared in zero-gravity resting contact. "
    << "This indicates the Baumgarte correction is generating torque "
    << "through asymmetric contact points even without gravity.";

  // Energy from Baumgarte separating the objects is acceptable,
  // but it should be bounded
  EXPECT_LT(maxKE, 1.0)
    << "DIAGNOSTIC [H6]: maxKE=" << maxKE
    << " in zero-gravity resting contact. "
    << "Baumgarte should only add minimal separation energy (~0.01J) "
    << "for 0.01m penetration. If much higher, energy feedback loop exists.";
}

// ============================================================================
// H7: Gravity vs No-Gravity Comparison
//
// Run the SAME resting cube scenario with and without gravity.
// If gravity+Baumgarte interaction is the cause, the gravity case
// should show dramatically more energy growth.
// ============================================================================

TEST_F(ReplayEnabledTest, ParameterIsolation_H7_GravityComparison_BaumgarteAmplification)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  // Ticket: 0062e_replay_diagnostic_parameter_tests

  // === WITH gravity ===
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});
  const auto& cubeGrav = spawnInertial("unit_cube", Coordinate{0.0, 0.0, 0.5},
                                       10.0, 0.0, 0.5);
  uint32_t gravId = cubeGrav.getInstanceId();
  world().getObject(gravId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  double const initialEnergyGrav = computeSystemEnergy(world());
  double maxEnergyGrav = initialEnergyGrav;
  double maxRotKEGrav = 0.0;
  bool nanGrav = false;

  for (int i = 1; i <= 200; ++i)
  {
    step(1);
    if (std::isnan(world().getObject(gravId).getInertialState().position.z()))
    {
      nanGrav = true;
      break;
    }
    auto sysE = computeSystemEnergyBreakdown(world());
    maxEnergyGrav = std::max(maxEnergyGrav, sysE.total());
    maxRotKEGrav = std::max(maxRotKEGrav, sysE.totalRotationalKE);
  }
  ASSERT_FALSE(nanGrav) << "DIAGNOSTIC [H7]: NaN in gravity case";

  double const gravityInjected = maxEnergyGrav - initialEnergyGrav;

  // === WITHOUT gravity ===
  TearDown();
  SetUp();
  disableGravity();

  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});
  const auto& cubeNoGrav = spawnInertial("unit_cube", Coordinate{0.0, 0.0, 0.499},
                                         10.0, 0.0, 0.5);
  uint32_t noGravId = cubeNoGrav.getInstanceId();
  world().getObject(noGravId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  auto initialENoGrav = computeSystemEnergyNoGravity(world());
  double const initialEnergyNoGrav = initialENoGrav.total();
  double maxEnergyNoGrav = initialEnergyNoGrav;
  double maxRotKENoGrav = 0.0;
  bool nanNoGrav = false;

  for (int i = 1; i <= 200; ++i)
  {
    step(1);
    if (std::isnan(world().getObject(noGravId).getInertialState().position.z()))
    {
      nanNoGrav = true;
      break;
    }
    auto sysE = computeSystemEnergyNoGravity(world());
    maxEnergyNoGrav = std::max(maxEnergyNoGrav, sysE.total());
    maxRotKENoGrav = std::max(maxRotKENoGrav, sysE.totalRotationalKE);
  }
  ASSERT_FALSE(nanNoGrav) << "DIAGNOSTIC [H7]: NaN in no-gravity case";

  double const noGravityInjected = maxEnergyNoGrav - initialEnergyNoGrav;

  // If gravity amplifies the problem, gravity case should inject much more
  bool const gravityAmplifies =
    (gravityInjected > 1.0) && (gravityInjected > noGravityInjected * 10.0);

  EXPECT_TRUE(gravityAmplifies ||
              (gravityInjected < 1.0 && noGravityInjected < 1.0))
    << "DIAGNOSTIC [H7]: Gravity amplification analysis:\n"
    << "  WITH gravity: injected=" << gravityInjected << " J, "
    << "maxE=" << maxEnergyGrav
    << " initialE=" << initialEnergyGrav
    << " rotKE=" << maxRotKEGrav << "\n"
    << "  WITHOUT gravity: injected=" << noGravityInjected << " J, "
    << "maxE=" << maxEnergyNoGrav << " initialE=" << initialEnergyNoGrav
    << " rotKE=" << maxRotKENoGrav << "\n"
    << (gravityAmplifies
          ? "CONFIRMED: Gravity continuously drives penetration, "
            "amplifying Baumgarte energy injection"
          : "NOT CONFIRMED: Gravity does not significantly amplify the issue");
}

// ============================================================================
// H8: Tilted Cube -- Verify Rotation Onset Mechanism
//
// A slightly tilted cube on a floor should settle. If instead it enters
// a feedback loop (tilt -> asymmetric penetration -> Baumgarte torque ->
// more tilt), this confirms the single-contact feedback hypothesis.
// ============================================================================

TEST_F(ReplayEnabledTest, ParameterIsolation_H8_TiltedCube_FeedbackLoop)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  // Ticket: 0062e_replay_diagnostic_parameter_tests

  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube with 2-degree initial tilt
  // NOTE: ReplayEnabledTest doesn't support orientation in spawn methods yet,
  // so we need to manually create the cube with tilted orientation
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  double const tiltAngle = 2.0 * M_PI / 180.0;  // 2 degrees
  Eigen::Quaterniond tiltQ =
    Eigen::AngleAxisd{tiltAngle, Eigen::Vector3d::UnitX()} *
    Eigen::Quaterniond::Identity();

  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.6}, tiltQ};
  world().spawnObject(1, cubeHull, 10.0, cubeFrame);

  uint32_t cubeId = 1;
  world().getObject(cubeId).setCoefficientOfRestitution(0.0);
  world().getObject(cubeId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  double const initialEnergy = computeSystemEnergy(world());

  // Track angular velocity magnitude over time
  double maxAngVel = 0.0;
  double maxEnergy = initialEnergy;
  bool diverging = false;
  double prevAngVel = 0.0;
  int growthStreak = 0;

  for (int i = 1; i <= 300; ++i)
  {
    step(1);

    auto const& state = world().getObject(cubeId).getInertialState();

    if (std::isnan(state.position.z()))
    {
      break;
    }

    AngularVelocity omega = state.getAngularVelocity();
    double angVelMag = Eigen::Vector3d{omega.x(), omega.y(), omega.z()}.norm();

    maxAngVel = std::max(maxAngVel, angVelMag);

    double currentEnergy = computeSystemEnergy(world());
    maxEnergy = std::max(maxEnergy, currentEnergy);

    // Check for monotonic angular velocity growth (feedback loop signature)
    if (angVelMag > prevAngVel + 1e-4)
    {
      growthStreak++;
    }
    else
    {
      growthStreak = 0;
    }

    // If angular velocity grows for 20+ consecutive frames, that's a feedback
    // loop
    if (growthStreak > 20)
    {
      diverging = true;
    }

    prevAngVel = angVelMag;
  }

  double const energyGrowthPercent = (maxEnergy / initialEnergy - 1.0) * 100.0;

  EXPECT_FALSE(diverging)
    << "DIAGNOSTIC [H8]: FEEDBACK LOOP DETECTED. "
    << "Angular velocity grew monotonically for 20+ consecutive frames. "
    << "MaxAngVel=" << maxAngVel << " rad/s. "
    << "This confirms: tilt -> asymmetric penetration -> Baumgarte torque "
    << "-> more tilt. Energy growth=" << energyGrowthPercent << "%";

  EXPECT_LT(energyGrowthPercent, 10.0)
    << "DIAGNOSTIC [H8]: Tilted cube energy grew " << energyGrowthPercent
    << "% (threshold: 10%). "
    << "MaxAngVel=" << maxAngVel << " MaxEnergy=" << maxEnergy
    << " InitialEnergy=" << initialEnergy;
}

// ============================================================================
// H9: Baumgarte ERP Contribution Analysis
//
// Directly compute the Baumgarte energy injection per frame for a
// known penetration depth. This is a purely analytical check of the
// mathematical formula.
//
// For a single contact with lever arm r:
//   Angular Jacobian = -(r x n)
//   Baumgarte velocity bias = (ERP/dt) * penetration
//   Force = lambda / dt, where lambda satisfies the constraint system
//
// The power injected by Baumgarte = F * v_correction
// With v_correction = (ERP/dt) * penetration and the effective mass
// m_eff = 1 / (J * M^-1 * J^T), the impulse magnitude is approximately:
//   lambda = m_eff * (ERP/dt) * penetration
//   Energy_injected = lambda^2 / (2 * m_eff)
//                   = 0.5 * m_eff * [(ERP/dt) * penetration]^2
//
// NOTE: This is a purely analytical calculation - kept as TEST() not TEST_F()
// ============================================================================

TEST(ParameterIsolation, H9_BaumgarteEnergyInjectionAnalysis)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  // Ticket: 0062e_replay_diagnostic_parameter_tests (NOT converted - analytical)
  //
  // Analytical computation: How much energy does Baumgarte inject per frame?

  double const erp = 0.2;
  double const dt = 0.016;
  double const penetration = 0.001;  // 1mm penetration
  double const mass = 10.0;

  // ERP/dt multiplier
  double const erpOverDt = erp / dt;

  // Baumgarte velocity bias (the "target" correction velocity)
  double const baumgarteBias = erpOverDt * penetration;

  // For a single contact point on a 10kg cube:
  // Linear effective mass contribution = 1/m = 0.1
  // If contact is offset from COM by r, angular contribution adds more
  // For cube with side 1m, r ~ 0.5m from COM to contact
  double const leverArm = 0.5;

  // For a 1m uniform cube, I = (1/6) * m * s^2 = 10/6 = 1.667 kg*m^2
  double const cubeSide = 1.0;
  double const I = (1.0 / 6.0) * mass * cubeSide * cubeSide;
  double const Iinv = 1.0 / I;

  // Effective mass: m_eff = 1 / (1/m + r^2 * Iinv)
  double const invMeff = (1.0 / mass) + (leverArm * leverArm * Iinv);
  double const mEff = 1.0 / invMeff;

  // Impulse from Baumgarte: lambda = m_eff * baumgarteBias
  double const lambda = mEff * baumgarteBias;

  // Energy injected per frame: E = lambda^2 / (2 * m_eff)
  // This equals 0.5 * m_eff * baumgarteBias^2
  double const energyPerFrame = 0.5 * mEff * baumgarteBias * baumgarteBias;

  // Over 200 frames (3.2 seconds)
  double const totalInjected = energyPerFrame * 200;

  // Report analytical results
  EXPECT_LT(energyPerFrame, 0.01)
    << "DIAGNOSTIC [H9]: Analytical Baumgarte energy injection per frame:\n"
    << "  ERP/dt = " << erpOverDt << "\n"
    << "  Baumgarte bias velocity = " << baumgarteBias << " m/s\n"
    << "  Lever arm = " << leverArm << " m\n"
    << "  Effective mass = " << mEff << " kg\n"
    << "  Impulse per frame = " << lambda << " N*s\n"
    << "  Energy per frame = " << energyPerFrame << " J\n"
    << "  Total over 200 frames = " << totalInjected << " J\n"
    << "  NOTE: This analysis assumes constant 1mm penetration. "
    << "In practice, the feedback loop increases penetration over time, "
    << "making the injection grow exponentially.";

  // Compare: initial PE of a 10kg cube at z=0.5 with gravity
  double const initialPE = mass * 9.81 * 0.5;  // ~49 J

  EXPECT_LT(totalInjected, initialPE * 0.01)
    << "DIAGNOSTIC [H9]: Baumgarte injection (" << totalInjected
    << " J) vs initial PE (" << initialPE << " J). "
    << "If injection is > 1% of initial PE, it's energetically significant. "
    << "Ratio: " << (totalInjected / initialPE * 100.0) << "%";

  // The KEY insight: even small per-frame injection compounds
  // because the injected energy increases penetration, which increases
  // the next frame's injection
  double const compoundedEnergy = energyPerFrame * (1.0 + erpOverDt * dt);
  EXPECT_GT(compoundedEnergy, energyPerFrame)
    << "DIAGNOSTIC [H9]: Compounding factor analysis: "
    << "Each frame's injection increases penetration, which increases "
    << "next frame's injection by factor " << (1.0 + erpOverDt * dt);
}

// ============================================================================
// H10: Integration Order Verification
//
// WorldModel::update() calls updateCollisions(dt) THEN updatePhysics(dt).
// This means collision constraint forces are computed using CURRENT velocities,
// then those forces are integrated in the physics step.
//
// Verify this ordering is consistent by checking that a falling cube
// has reasonable behavior (doesn't fall through floor or explode).
// ============================================================================

TEST_F(ReplayEnabledTest, ParameterIsolation_H10_IntegrationOrder_ConsistencyCheck)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  // Ticket: 0062e_replay_diagnostic_parameter_tests

  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Drop cube from height
  const auto& cube = spawnInertial("unit_cube", Coordinate{0.0, 0.0, 2.0},
                                   10.0, 0.0, 0.5);
  uint32_t cubeId = cube.getInstanceId();

  double minZ = 2.0;
  double maxSpeed = 0.0;

  for (int i = 1; i <= 300; ++i)
  {
    step(1);

    auto const& state = world().getObject(cubeId).getInertialState();

    if (std::isnan(state.position.z()))
    {
      FAIL() << "DIAGNOSTIC [H10]: NaN at frame " << i;
    }

    minZ = std::min(minZ, state.position.z());
    maxSpeed = std::max(maxSpeed, state.velocity.norm());
  }

  // Cube should not fall through floor (z should not go below ~0.0)
  EXPECT_GT(minZ, -0.5)
    << "DIAGNOSTIC [H10]: Cube fell through floor to z=" << minZ
    << ". This indicates collision forces are not applied before integration "
    << "or the constraint solver failed.";

  // Cube should not explode (speed should be bounded)
  EXPECT_LT(maxSpeed, 100.0)
    << "DIAGNOSTIC [H10]: Maximum speed=" << maxSpeed
    << " m/s. An inelastic cube dropped from 2m should not exceed "
    << "~6 m/s impact velocity. Speed > 100 indicates energy injection.";

  // After 300 frames (4.8s) with e=0, cube should be roughly settled
  auto const& finalState = world().getObject(cubeId).getInertialState();
  double const finalSpeed = finalState.velocity.norm();

  EXPECT_LT(finalSpeed, 1.0)
    << "DIAGNOSTIC [H10]: After 4.8s with e=0, cube still moving at "
    << finalSpeed << " m/s. Should be settled near zero.";
}

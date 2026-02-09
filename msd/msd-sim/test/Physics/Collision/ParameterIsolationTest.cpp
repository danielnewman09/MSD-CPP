// Ticket: 0039d_parameter_isolation_root_cause
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

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

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

/// Run a resting-cube-on-floor simulation for a given number of frames
/// and track energy. Returns max energy growth ratio relative to initial.
struct SimulationResult
{
  double initialEnergy;
  double maxEnergy;
  double finalEnergy;
  double maxGrowthRatio;  // maxEnergy / initialEnergy
  int energyGrowthFrameCount;
  double maxFrameGrowth;
  double maxRotationalKE;
  double maxLinearKE;
  bool nanDetected;
};

SimulationResult runRestingCubeSimulation(double cubeStartZ,
                                          double restitution,
                                          int numFrames,
                                          int frameStepMs,
                                          bool useGravity = true)
{
  WorldModel world;

  if (!useGravity)
  {
    world.clearPotentialEnergies();
  }

  // Floor: large cube at z = -50 (top surface at z = 0)
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Cube: 1m x 1m x 1m (top face at cubeStartZ + 0.5)
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, cubeStartZ}};
  world.spawnObject(1, cubeHull, 10.0, cubeFrame);

  uint32_t cubeId = 1;
  world.getObject(cubeId).setCoefficientOfRestitution(restitution);
  // Start at rest
  world.getObject(cubeId).getInertialState().velocity = Vector3D{0.0, 0.0, 0.0};

  double const initialEnergy = useGravity
                                 ? computeSystemEnergy(world)
                                 : computeSystemEnergyNoGravity(world).total();

  SimulationResult result{};
  result.initialEnergy = initialEnergy;
  result.maxEnergy = initialEnergy;
  result.finalEnergy = initialEnergy;
  result.maxGrowthRatio = 1.0;
  result.energyGrowthFrameCount = 0;
  result.maxFrameGrowth = 0.0;
  result.maxRotationalKE = 0.0;
  result.maxLinearKE = 0.0;
  result.nanDetected = false;

  double prevEnergy = initialEnergy;

  for (int i = 1; i <= numFrames; ++i)
  {
    world.update(std::chrono::milliseconds{i * frameStepMs});

    auto const& state = world.getObject(cubeId).getInertialState();

    if (std::isnan(state.position.z()) || std::isnan(state.velocity.norm()))
    {
      result.nanDetected = true;
      break;
    }

    EnergyTracker::SystemEnergy sysEnergy =
      useGravity ? computeSystemEnergyBreakdown(world)
                 : computeSystemEnergyNoGravity(world);

    double totalEnergy = sysEnergy.total();

    result.maxEnergy = std::max(result.maxEnergy, totalEnergy);
    result.finalEnergy = totalEnergy;

    double delta = totalEnergy - prevEnergy;
    if (delta > 1e-6)
    {
      result.energyGrowthFrameCount++;
      result.maxFrameGrowth = std::max(result.maxFrameGrowth, delta);
    }

    result.maxRotationalKE =
      std::max(result.maxRotationalKE, sysEnergy.totalRotationalKE);
    result.maxLinearKE = std::max(result.maxLinearKE, sysEnergy.totalLinearKE);

    prevEnergy = totalEnergy;
  }

  if (std::abs(result.initialEnergy) > 1e-12)
  {
    result.maxGrowthRatio = result.maxEnergy / result.initialEnergy;
  }
  else
  {
    result.maxGrowthRatio =
      (result.maxEnergy > 1e-6) ? std::numeric_limits<double>::infinity() : 1.0;
  }

  return result;
}

}  // anonymous namespace

// ============================================================================
// H1: Disable Restitution -- Resting Cube with e=0
//
// If energy still grows with e=0, restitution is NOT the root cause.
// Baumgarte term (ERP/dt * penetration) is independent of restitution.
// ============================================================================

TEST(ParameterIsolation, H1_DisableRestitution_RestingCube)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  //
  // Place cube at rest on floor with e=0 (fully inelastic).
  // With restitution disabled, the constraint RHS becomes:
  //   b = -(1+0) * Jv + (ERP/dt) * penetration = -Jv + 12.5 * penetration
  //
  // If energy grows, the Baumgarte term is injecting energy.
  // If energy is stable, restitution was the problem.

  auto result = runRestingCubeSimulation(
    0.5,  // z position: cube center at 0.5, bottom at 0.0 (touching floor)
    0.0,  // e = 0 (fully inelastic)
    200,  // frames
    16);  // 16ms per frame

  ASSERT_FALSE(result.nanDetected)
    << "DIAGNOSTIC [H1]: NaN detected in zero-restitution resting cube test";

  // The KEY question: does energy grow even without restitution?
  double const energyGrowthPercent = (result.maxGrowthRatio - 1.0) * 100.0;

  bool const energyGrew = result.maxEnergy > result.initialEnergy * 1.01;

  EXPECT_FALSE(energyGrew)
    << "DIAGNOSTIC [H1]: ENERGY GROWS EVEN WITH e=0. "
    << "This proves Baumgarte/ERP is the primary energy source. "
    << "Initial=" << result.initialEnergy << " Max=" << result.maxEnergy
    << " Growth=" << energyGrowthPercent << "% "
    << "MaxRotKE=" << result.maxRotationalKE
    << " GrowthFrames=" << result.energyGrowthFrameCount
    << " MaxFrameGrowth=" << result.maxFrameGrowth;

  // Also check if rotational energy appears (single-contact torque issue)
  if (result.maxRotationalKE > 0.01)
  {
    EXPECT_LT(result.maxRotationalKE, 0.01)
      << "DIAGNOSTIC [H1]: Rotational KE=" << result.maxRotationalKE
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

TEST(ParameterIsolation, H2_MinimalPenetration_NoEnergyGrowth)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  //
  // Cube at z=0.501 (bottom face at z=0.001, just above floor at z=0).
  // No initial penetration means Baumgarte term starts at zero.
  // Gravity will push cube into floor, but initial frames should be clean.

  auto result = runRestingCubeSimulation(
    0.501,  // Just above floor
    0.0,    // e=0
    50,     // fewer frames (before deep penetration develops)
    16);

  ASSERT_FALSE(result.nanDetected)
    << "DIAGNOSTIC [H2]: NaN detected in minimal penetration test";

  // Compare with cube sitting exactly at floor level
  auto contactResult =
    runRestingCubeSimulation(0.5,  // Exactly at floor (touching)
                             0.0,  // e=0
                             50,   // same frame count
                             16);

  ASSERT_FALSE(contactResult.nanDetected)
    << "DIAGNOSTIC [H2]: NaN detected in contact case";

  double const hoverGrowth = (result.maxGrowthRatio - 1.0) * 100.0;
  double const contactGrowth = (contactResult.maxGrowthRatio - 1.0) * 100.0;

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
    << "Hover maxE=" << result.maxEnergy
    << " Contact maxE=" << contactResult.maxEnergy;
}

// ============================================================================
// H3: Timestep Sensitivity -- ERP/dt amplification
//
// If Baumgarte ERP/dt is the culprit, SMALLER timesteps should produce
// WORSE energy injection (because ERP/dt grows as dt shrinks).
// ERP=0.2, dt=0.008 -> ERP/dt=25
// ERP=0.2, dt=0.016 -> ERP/dt=12.5
// ERP=0.2, dt=0.032 -> ERP/dt=6.25
//
// If energy growth is worse with smaller dt, the ERP/dt term is confirmed.
// ============================================================================

TEST(ParameterIsolation, H3_TimestepSensitivity_ERPAmplification)
{
  // Ticket: 0039d_parameter_isolation_root_cause

  // Run same scenario at different timesteps
  // Use same total simulation time (~3.2 seconds)
  auto result8ms = runRestingCubeSimulation(
    0.5, 0.0, 400, 8);  // 400 frames * 8ms = 3.2s, ERP/dt = 25.0

  auto result16ms = runRestingCubeSimulation(
    0.5, 0.0, 200, 16);  // 200 frames * 16ms = 3.2s, ERP/dt = 12.5

  auto result32ms = runRestingCubeSimulation(
    0.5, 0.0, 100, 32);  // 100 frames * 32ms = 3.2s, ERP/dt = 6.25

  ASSERT_FALSE(result8ms.nanDetected) << "DIAGNOSTIC [H3]: NaN at 8ms timestep";
  ASSERT_FALSE(result16ms.nanDetected)
    << "DIAGNOSTIC [H3]: NaN at 16ms timestep";
  ASSERT_FALSE(result32ms.nanDetected)
    << "DIAGNOSTIC [H3]: NaN at 32ms timestep";

  double const growth8ms = (result8ms.maxGrowthRatio - 1.0) * 100.0;
  double const growth16ms = (result16ms.maxGrowthRatio - 1.0) * 100.0;
  double const growth32ms = (result32ms.maxGrowthRatio - 1.0) * 100.0;

  // If ERP/dt is the culprit: growth8ms > growth16ms > growth32ms
  // If NOT ERP/dt: no clear ordering

  bool const erpPattern = (growth8ms > growth16ms) && (growth16ms > growth32ms);

  // Report the pattern regardless of pass/fail
  EXPECT_TRUE(erpPattern)
    << "DIAGNOSTIC [H3]: Timestep sensitivity analysis:\n"
    << "  dt=8ms  (ERP/dt=25.0): " << growth8ms << "% energy growth, "
    << "maxE=" << result8ms.maxEnergy << "\n"
    << "  dt=16ms (ERP/dt=12.5): " << growth16ms << "% energy growth, "
    << "maxE=" << result16ms.maxEnergy << "\n"
    << "  dt=32ms (ERP/dt=6.25): " << growth32ms << "% energy growth, "
    << "maxE=" << result32ms.maxEnergy << "\n"
    << (erpPattern
          ? "CONFIRMED: Smaller dt = worse growth (ERP/dt is the cause)"
          : "NOT CONFIRMED: No clear ERP/dt correlation");

  // Additional diagnostic: the ratio of energy growths should roughly
  // match the ratio of ERP/dt values if the relationship is linear
  if (growth16ms > 1.0 && growth32ms > 1.0)
  {
    double const ratio_8_16 = growth8ms / growth16ms;
    double const ratio_16_32 = growth16ms / growth32ms;

    // ERP/dt ratio: 25/12.5 = 2.0, 12.5/6.25 = 2.0
    // If the relationship is linear, we expect ratio ~2.0
    EXPECT_GT(ratio_8_16, 1.0)
      << "DIAGNOSTIC [H3]: growth_8ms/growth_16ms = " << ratio_8_16
      << " (expected > 1.0 if ERP/dt matters)";
    EXPECT_GT(ratio_16_32, 1.0)
      << "DIAGNOSTIC [H3]: growth_16ms/growth_32ms = " << ratio_16_32
      << " (expected > 1.0 if ERP/dt matters)";
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
// ============================================================================

TEST(ParameterIsolation, H4_SingleContactPoint_TorqueDiagnostic)
{
  // Ticket: 0039d_parameter_isolation_root_cause
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
// H5: Contact Point Count Diagnostic
//
// Run full simulation and track contact manifold sizes over time.
// This uses CollisionHandler to inspect what EPA actually produces
// for a settling cube.
// ============================================================================

TEST(ParameterIsolation, H5_ContactPointCount_EvolutionDiagnostic)
{
  // Ticket: 0039d_parameter_isolation_root_cause
  //
  // Track how many contact points are generated per frame as a cube
  // settles on a floor. If contact count stays at 1, the manifold
  // generator is not providing enough contacts for stability.

  WorldModel world;

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.5}};
  world.spawnObject(1, cubeHull, 10.0, cubeFrame);

  uint32_t cubeId = 1;
  world.getObject(cubeId).setCoefficientOfRestitution(0.0);
  world.getObject(cubeId).getInertialState().velocity = Vector3D{0.0, 0.0, 0.0};

  // Use a separate CollisionHandler to inspect contacts each frame
  CollisionHandler handler{1e-6};

  int singleContactFrames = 0;
  int multiContactFrames = 0;
  int noContactFrames = 0;
  int maxContacts = 0;

  for (int i = 1; i <= 100; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});

    // Check collision state AFTER the world update
    auto const& assets = world.getInertialAssets();
    auto const& envAssets = world.getEnvironmentalObjects();

    if (!assets.empty() && !envAssets.empty())
    {
      auto result = handler.checkCollision(assets[0], envAssets[0]);
      if (result)
      {
        int const count = static_cast<int>(result->contactCount);
        maxContacts = std::max(maxContacts, count);

        if (count == 1)
        {
          singleContactFrames++;
        }
        else
        {
          multiContactFrames++;
        }
      }
      else
      {
        noContactFrames++;
      }
    }
  }

  // CRITICAL: If single contact dominates, the manifold is under-constrained
  EXPECT_GT(multiContactFrames, singleContactFrames)
    << "DIAGNOSTIC [H5]: Single-contact frames dominate ("
    << singleContactFrames << " single vs " << multiContactFrames << " multi, "
    << noContactFrames << " none). "
    << "Max contact count seen: " << maxContacts << ". "
    << "Single contacts cannot provide torque-balanced support for a cube.";

  EXPECT_GE(maxContacts, 4)
    << "DIAGNOSTIC [H5]: Max contacts per frame = " << maxContacts
    << " (expected 4 for cube-on-floor face contact). "
    << "Insufficient contact points prevent stable resting.";
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

TEST(ParameterIsolation, H6_ZeroGravity_RestingContact_Stable)
{
  // Ticket: 0039d_parameter_isolation_root_cause

  WorldModel world;
  world.clearPotentialEnergies();  // No gravity

  // Two cubes touching at x=0.5
  auto cubePointsA = createCubePoints(1.0);
  auto cubePointsB = createCubePoints(1.0);
  ConvexHull hullA{cubePointsA};
  ConvexHull hullB{cubePointsB};

  // Place them with a tiny overlap (0.01m penetration)
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.49, 0.0, 0.0}};

  world.spawnObject(1, hullA, 10.0, frameA);
  world.spawnObject(2, hullB, 10.0, frameB);

  uint32_t idA = 1;
  uint32_t idB = 2;

  world.getObject(idA).setCoefficientOfRestitution(0.0);
  world.getObject(idB).setCoefficientOfRestitution(0.0);

  // Both at rest
  world.getObject(idA).getInertialState().velocity = Vector3D{0.0, 0.0, 0.0};
  world.getObject(idB).getInertialState().velocity = Vector3D{0.0, 0.0, 0.0};

  auto initialSysEnergy = computeSystemEnergyNoGravity(world);
  double const initialKE = initialSysEnergy.total();

  // Track energy for 100 frames
  double maxKE = initialKE;
  double maxRotKE = 0.0;
  bool nanDetected = false;

  for (int i = 1; i <= 100; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});

    auto const& stateA = world.getObject(idA).getInertialState();
    auto const& stateB = world.getObject(idB).getInertialState();

    if (std::isnan(stateA.position.x()) || std::isnan(stateB.position.x()))
    {
      nanDetected = true;
      break;
    }

    auto sysEnergy = computeSystemEnergyNoGravity(world);
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

TEST(ParameterIsolation, H7_GravityComparison_BaumgarteAmplification)
{
  // Ticket: 0039d_parameter_isolation_root_cause

  // With gravity: cube pushed into floor continuously
  auto withGravity = runRestingCubeSimulation(0.5,    // resting on floor
                                              0.0,    // e=0
                                              200,    // frames
                                              16,     // 16ms
                                              true);  // gravity ON

  // Without gravity: cube just resting (initial penetration only)
  // Slightly overlapping at z=0.499 to create initial contact
  auto noGravity = runRestingCubeSimulation(0.499,   // slight penetration
                                            0.0,     // e=0
                                            200,     // frames
                                            16,      // 16ms
                                            false);  // gravity OFF

  ASSERT_FALSE(withGravity.nanDetected)
    << "DIAGNOSTIC [H7]: NaN in gravity case";
  ASSERT_FALSE(noGravity.nanDetected)
    << "DIAGNOSTIC [H7]: NaN in no-gravity case";

  // Compare absolute energy injected (not ratios, since no-gravity starts at
  // ~0)
  double const gravityInjected =
    withGravity.maxEnergy - withGravity.initialEnergy;
  double const noGravityInjected =
    noGravity.maxEnergy - noGravity.initialEnergy;

  // If gravity amplifies the problem, gravity case should inject much more
  bool const gravityAmplifies =
    (gravityInjected > 1.0) && (gravityInjected > noGravityInjected * 10.0);

  EXPECT_TRUE(gravityAmplifies ||
              (gravityInjected < 1.0 && noGravityInjected < 1.0))
    << "DIAGNOSTIC [H7]: Gravity amplification analysis:\n"
    << "  WITH gravity: injected=" << gravityInjected << " J, "
    << "maxE=" << withGravity.maxEnergy
    << " initialE=" << withGravity.initialEnergy
    << " rotKE=" << withGravity.maxRotationalKE << "\n"
    << "  WITHOUT gravity: injected=" << noGravityInjected << " J, "
    << "maxE=" << noGravity.maxEnergy << " initialE=" << noGravity.initialEnergy
    << " rotKE=" << noGravity.maxRotationalKE << "\n"
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

TEST(ParameterIsolation, H8_TiltedCube_FeedbackLoop)
{
  // Ticket: 0039d_parameter_isolation_root_cause

  WorldModel world;

  // Floor
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Cube with 2-degree initial tilt
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  double const tiltAngle = 2.0 * M_PI / 180.0;  // 2 degrees
  Eigen::Quaterniond tiltQ =
    Eigen::AngleAxisd{tiltAngle, Eigen::Vector3d::UnitX()} *
    Eigen::Quaterniond::Identity();

  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.6}, tiltQ};
  world.spawnObject(1, cubeHull, 10.0, cubeFrame);

  uint32_t cubeId = 1;
  world.getObject(cubeId).setCoefficientOfRestitution(0.0);
  world.getObject(cubeId).getInertialState().velocity = Vector3D{0.0, 0.0, 0.0};

  double const initialEnergy = computeSystemEnergy(world);

  // Track angular velocity magnitude over time
  double maxAngVel = 0.0;
  double maxEnergy = initialEnergy;
  bool diverging = false;
  double prevAngVel = 0.0;
  int growthStreak = 0;

  for (int i = 1; i <= 300; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});

    auto const& state = world.getObject(cubeId).getInertialState();

    if (std::isnan(state.position.z()))
    {
      break;
    }

    AngularRate omega = state.getAngularVelocity();
    double angVelMag = Eigen::Vector3d{omega.x(), omega.y(), omega.z()}.norm();

    maxAngVel = std::max(maxAngVel, angVelMag);

    double currentEnergy = computeSystemEnergy(world);
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
// ============================================================================

TEST(ParameterIsolation, H9_BaumgarteEnergyInjectionAnalysis)
{
  // Ticket: 0039d_parameter_isolation_root_cause
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

TEST(ParameterIsolation, H10_IntegrationOrder_ConsistencyCheck)
{
  // Ticket: 0039d_parameter_isolation_root_cause

  WorldModel world;

  // Floor
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Drop cube from height
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 2.0}};
  world.spawnObject(1, cubeHull, 10.0, cubeFrame);

  uint32_t cubeId = 1;
  world.getObject(cubeId).setCoefficientOfRestitution(0.0);

  double minZ = 2.0;
  double maxSpeed = 0.0;

  for (int i = 1; i <= 300; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});

    auto const& state = world.getObject(cubeId).getInertialState();

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
  auto const& finalState = world.getObject(cubeId).getInertialState();
  double const finalSpeed = finalState.velocity.norm();

  EXPECT_LT(finalSpeed, 1.0)
    << "DIAGNOSTIC [H10]: After 4.8s with e=0, cube still moving at "
    << finalSpeed << " m/s. Should be settled near zero.";
}

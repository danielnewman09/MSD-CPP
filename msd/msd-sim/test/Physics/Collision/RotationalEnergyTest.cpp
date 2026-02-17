// Ticket: 0039c_rotational_coupling_test_suite
// Ticket: 0062c_replay_rotational_collision_tests
// Test: Scenario F4 -- Energy transfer with rotation
// Converted to ReplayEnabledTest fixture for automatic replay recording
//
// DIAGNOSTIC TEST SUITE: This test specifically investigates the known
// energy injection bug in rotational collisions. Failure is EXPECTED and
// provides diagnostic data about the bug's magnitude.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp"
#include "msd-sim/test/Replay/ReplayEnabledTest.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

/// Compute system energy breakdown using EnergyTracker with gravity potential
EnergyTracker::SystemEnergy computeSystemEnergyBreakdown(
  const WorldModel& world)
{
  std::vector<std::unique_ptr<PotentialEnergy>> potentials;
  potentials.push_back(
    std::make_unique<GravityPotential>(Coordinate{0.0, 0.0, -9.81}));

  return EnergyTracker::computeSystemEnergy(world.getInertialAssets(),
                                            potentials);
}

/// Compute total system energy scalar
double computeSystemEnergy(const WorldModel& world)
{
  return computeSystemEnergyBreakdown(world).total();
}

}  // anonymous namespace

// ============================================================================
// F4: Rotation energy transfer -- cube dropped at angle with e=1.0
// Validates: Rotational energy accounting
//
// Pre-impact:  linear KE > 0, rotational KE = 0
// Post-impact: linear KE + rotational KE should equal pre-impact total KE
//              (within tolerance for discrete time-stepping)
//
// DIAGNOSTIC: Specifically flag energy GROWTH (the known bug direction).
// In discrete time-stepping, exact conservation is impossible; the test
// looks for energy growth, not just inequality.
// ============================================================================

TEST_F(ReplayEnabledTest,
       RotationalEnergyTest_F4_RotationEnergyTransfer_EnergyConserved)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  // Ticket: 0062c_replay_rotational_collision_tests

  // Floor
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube: 1m x 1m x 1m, dropped at 45 degrees with e=1.0 (elastic)
  Eigen::Quaterniond q =
    Eigen::AngleAxisd{M_PI / 4.0, Eigen::Vector3d::UnitX()} *
    Eigen::AngleAxisd{M_PI / 4.0, Eigen::Vector3d::UnitY()};

  double const halfDiag = std::sqrt(3.0) / 2.0;

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, 2.0 + halfDiag},
                                   1.0,   // mass (kg)
                                   1.0,   // restitution (elastic)
                                   0.5);  // friction
  uint32_t cubeId = cube.getInstanceId();

  // Manually set orientation
  world().getObject(cubeId).getInertialState().orientation = q;

  double const initialTotalEnergy = computeSystemEnergy(world());

  // Track energy over 100 frames post-impact
  bool impactOccurred = false;
  bool rotationalKEDetected = false;
  bool nanDetected = false;

  double maxTotalEnergy = initialTotalEnergy;
  double minTotalEnergy = initialTotalEnergy;
  double maxRotationalKE = 0.0;
  double maxLinearKE = 0.0;
  int energyGrowthCount = 0;
  double maxEnergyGrowth = 0.0;
  double prevEnergy = initialTotalEnergy;

  for (int i = 1; i <= 500; ++i)
  {
    step(1);

    auto const& state = world().getObject(cubeId).getInertialState();

    if (std::isnan(state.position.z()) || std::isnan(state.velocity.norm()))
    {
      nanDetected = true;
      break;
    }

    auto sysEnergy = computeSystemEnergyBreakdown(world());
    double totalEnergy = sysEnergy.total();

    maxTotalEnergy = std::max(maxTotalEnergy, totalEnergy);
    minTotalEnergy = std::min(minTotalEnergy, totalEnergy);

    // Track frame-to-frame energy growth
    double delta = totalEnergy - prevEnergy;
    if (delta > 1e-6)
    {
      energyGrowthCount++;
      maxEnergyGrowth = std::max(maxEnergyGrowth, delta);
    }
    prevEnergy = totalEnergy;

    // Detect impact: sphere near floor and velocity reverses
    double vz = state.velocity.z();
    if (!impactOccurred && state.position.z() < 2.0 && vz > 0.0)
    {
      impactOccurred = true;
    }

    if (impactOccurred)
    {
      // Track energy breakdown
      maxRotationalKE = std::max(maxRotationalKE, sysEnergy.totalRotationalKE);
      maxLinearKE = std::max(maxLinearKE, sysEnergy.totalLinearKE);

      if (sysEnergy.totalRotationalKE > 0.001)
      {
        rotationalKEDetected = true;
      }
    }
  }

  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in rotational energy transfer test";

  // DIAGNOSTIC: Impact should have occurred
  EXPECT_TRUE(impactOccurred)
    << "DIAGNOSTIC: Cube should have hit the floor within 300 frames";

  // DIAGNOSTIC: Rotational KE should appear after impact
  // (energy transfers from linear to rotational mode)
  EXPECT_TRUE(rotationalKEDetected)
    << "DIAGNOSTIC: Rotational KE should be non-zero after corner impact. "
    << "maxRotationalKE=" << maxRotationalKE
    << " This indicates rotation was NOT initiated by the collision.";

  // DIAGNOSTIC: Total energy drift over 100 post-impact frames
  // For e=1.0 elastic collision, energy should be approximately conserved.
  // We use a 1% tolerance as specified in the ticket.
  if (impactOccurred && !nanDetected)
  {
    double const energyDrift =
      (maxTotalEnergy - initialTotalEnergy) / std::abs(initialTotalEnergy);

    EXPECT_LT(energyDrift, 0.01)
      << "DIAGNOSTIC: Total energy drift=" << (energyDrift * 100.0)
      << "% over simulation (threshold: 1%). "
      << "InitialE=" << initialTotalEnergy << " MaxE=" << maxTotalEnergy
      << " EnergyGrowthFrames=" << energyGrowthCount
      << " MaxFrameGrowth=" << maxEnergyGrowth;

    // DIAGNOSTIC: Specifically flag energy GROWTH (the known bug)
    bool const energyGrew = maxTotalEnergy > initialTotalEnergy * 1.001;
    EXPECT_FALSE(energyGrew)
      << "DIAGNOSTIC: ENERGY GROWTH DETECTED. "
      << "This is the signature of the rotational energy injection bug. "
      << "E_initial=" << initialTotalEnergy << " E_max=" << maxTotalEnergy
      << " Growth=" << ((maxTotalEnergy / initialTotalEnergy - 1.0) * 100.0)
      << "%";
  }

  // DIAGNOSTIC: Log energy breakdown for analysis
  if (impactOccurred)
  {
    // Use ADD_FAILURE for informational diagnostic output only if there's
    // interesting data to report
    if (maxRotationalKE > 0.0 || maxLinearKE > 0.0)
    {
      // This is informational, not a failure condition
      // maxRotationalKE and maxLinearKE are tracked for diagnostic purposes
    }
  }
}

// ============================================================================
// F4b: Zero-gravity elastic cube collision -- energy conservation check
// A simpler variant that removes gravity to isolate rotational energy transfer
// ============================================================================

TEST_F(ReplayEnabledTest,
       RotationalEnergyTest_F4b_ZeroGravity_RotationalEnergyTransfer_Conserved)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  // Ticket: 0062c_replay_rotational_collision_tests

  disableGravity();

  // Two cubes colliding with offset to induce rotation
  // Place cubes with slight vertical offset so contact is off-center
  const auto& cubeA =
    spawnInertialWithVelocity("unit_cube",
                              Coordinate{-0.05, 0.0, 0.0},
                              Coordinate{2.0, 0.0, 0.0},  // velocity
                              1.0,                        // mass (kg)
                              1.0,   // restitution (elastic)
                              0.5);  // friction
  uint32_t idA = cubeA.getInstanceId();

  const auto& cubeB =
    spawnInertial("unit_cube",
                  Coordinate{1.0, 0.0, 0.3},  // Offset in z for torque
                  1.0,                        // mass (kg)
                  1.0,                        // restitution (elastic)
                  0.5);                       // friction
  uint32_t idB = cubeB.getInstanceId();

  // Compute initial KE (no gravity, so no PE)
  std::vector<std::unique_ptr<PotentialEnergy>> noPotentials;
  auto initialSysEnergy = EnergyTracker::computeSystemEnergy(
    world().getInertialAssets(), noPotentials);
  double const initialKE = initialSysEnergy.total();

  ASSERT_GT(initialKE, 0.0) << "Initial KE should be non-zero";

  bool nanDetected = false;
  bool rotationalKEDetected = false;
  double maxRotKE = 0.0;

  for (int i = 1; i <= 50; ++i)
  {
    step(1);

    auto const& stateA = world().getObject(idA).getInertialState();
    auto const& stateB = world().getObject(idB).getInertialState();

    if (std::isnan(stateA.position.x()) || std::isnan(stateB.position.x()))
    {
      nanDetected = true;
      break;
    }

    auto sysEnergy = EnergyTracker::computeSystemEnergy(
      world().getInertialAssets(), noPotentials);

    if (sysEnergy.totalRotationalKE > 0.001)
    {
      rotationalKEDetected = true;
    }
    maxRotKE = std::max(maxRotKE, sysEnergy.totalRotationalKE);
  }

  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in zero-gravity rotation test";

  // DIAGNOSTIC: Off-center collision should transfer energy to rotation
  EXPECT_TRUE(rotationalKEDetected)
    << "DIAGNOSTIC: Off-center collision should produce rotational KE. "
    << "maxRotKE=" << maxRotKE;

  // DIAGNOSTIC: Total KE should be conserved (elastic, no gravity)
  if (!nanDetected)
  {
    auto finalSysEnergy = EnergyTracker::computeSystemEnergy(
      world().getInertialAssets(), noPotentials);
    double const finalKE = finalSysEnergy.total();

    double const drift = std::abs(finalKE - initialKE) / initialKE;
    EXPECT_LT(drift, 0.05) << "DIAGNOSTIC: KE drift=" << (drift * 100.0)
                           << "% (threshold: 5%). "
                           << "initialKE=" << initialKE
                           << " finalKE=" << finalKE
                           << " (linearKE=" << finalSysEnergy.totalLinearKE
                           << " rotKE=" << finalSysEnergy.totalRotationalKE
                           << ")";

    // DIAGNOSTIC: Specifically flag energy growth
    bool const energyGrew = finalKE > initialKE * 1.001;
    EXPECT_FALSE(energyGrew)
      << "DIAGNOSTIC: ENERGY GROWTH in zero-gravity elastic collision. "
      << "initialKE=" << initialKE << " finalKE=" << finalKE
      << " growth=" << ((finalKE / initialKE - 1.0) * 100.0) << "%";
  }
}

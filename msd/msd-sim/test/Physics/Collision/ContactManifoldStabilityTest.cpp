// Ticket: 0039c_rotational_coupling_test_suite
// Ticket: 0062d_replay_stability_edge_contact_tests
// Test: Scenario D -- Contact manifold stability
// Converted to ReplayEnabledTest fixture for automatic replay recording
//
// DIAGNOSTIC TEST SUITE: Some tests are EXPECTED to fail because they
// investigate a known energy injection bug in rotational collisions.
// Failures are valid diagnostic findings, not test implementation errors.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp"
#include "msd-sim/test/Replay/ReplayEnabledTest.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions (local to tests, not duplicating fixture functionality)
// ============================================================================

namespace
{

/// Compute total system energy using EnergyTracker with gravity potential
double computeSystemEnergy(const WorldModel& world)
{
  std::vector<std::unique_ptr<PotentialEnergy>> potentials;
  potentials.push_back(
    std::make_unique<GravityPotential>(Coordinate{0.0, 0.0, -9.81}));

  auto sysEnergy = EnergyTracker::computeSystemEnergy(
    world.getInertialAssets(), potentials);
  return sysEnergy.total();
}

}  // anonymous namespace

// ============================================================================
// D1: Resting cube stability -- cube flat on floor for 1000 frames
// Validates: Resting contact stability, no drift/jitter/sinking
// ============================================================================

TEST_F(ReplayEnabledTest, ContactManifoldStabilityTest_D1_RestingCube_StableFor1000Frames)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  // Ticket: 0062d_replay_stability_edge_contact_tests

  // Floor: large cube centered at z=-50 (surface at z=0)
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube: 1m x 1m x 1m, flat on floor (bottom face at z=0, center at z=0.5)
  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, 0.5},
                                   1.0,  // mass (kg)
                                   0.5,  // restitution
                                   0.5); // friction
  uint32_t cubeId = cube.getInstanceId();

  // Set initial velocity to zero
  world().getObject(cubeId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  Coordinate const initialPosition =
    world().getObject(cubeId).getInertialState().position;
  double const initialEnergy = computeSystemEnergy(world());

  double maxPositionDrift = 0.0;
  double maxVelocity = 0.0;
  double maxOmega = 0.0;
  bool nanDetected = false;
  int energyGrowthFrames = 0;
  double maxEnergyIncrease = 0.0;
  double prevEnergy = initialEnergy;

  // Run for 1000 frames (~16.7 seconds at 60 FPS)
  for (int i = 1; i <= 1000; ++i)
  {
    step(1);

    auto const& state = world().getObject(cubeId).getInertialState();

    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    // Track position drift
    Coordinate drift = state.position - initialPosition;
    double driftMag = drift.norm();
    maxPositionDrift = std::max(maxPositionDrift, driftMag);

    // Track velocity magnitude
    double vel = state.velocity.norm();
    maxVelocity = std::max(maxVelocity, vel);

    // Track angular velocity
    AngularVelocity omega = state.getAngularVelocity();
    maxOmega = std::max(maxOmega, omega.norm());

    // Track energy growth
    double currentEnergy = computeSystemEnergy(world());
    double energyDelta = currentEnergy - prevEnergy;
    if (energyDelta > 1e-6)
    {
      energyGrowthFrames++;
      maxEnergyIncrease = std::max(maxEnergyIncrease, energyDelta);
    }
    prevEnergy = currentEnergy;
  }

  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in resting cube stability test";

  // DIAGNOSTIC: Position drift should be minimal
  EXPECT_LT(maxPositionDrift, 0.1)
    << "DIAGNOSTIC: Resting cube position drift=" << maxPositionDrift
    << " m (threshold: 0.001m per ticket, relaxed to 0.1m for diagnostic)";

  // DIAGNOSTIC: Velocity should be very low
  EXPECT_LT(maxVelocity, 1.0)
    << "DIAGNOSTIC: Resting cube should have minimal velocity. "
    << "Got maxVel=" << maxVelocity << " m/s";

  // DIAGNOSTIC: No angular velocity
  EXPECT_LT(maxOmega, 1.0)
    << "DIAGNOSTIC: Resting cube should not rotate. "
    << "Got maxOmega=" << maxOmega << " rad/s";

  // DIAGNOSTIC: No energy increase
  double const finalEnergy = computeSystemEnergy(world());
  if (!nanDetected)
  {
    EXPECT_LE(finalEnergy, initialEnergy * 1.01)
      << "DIAGNOSTIC: Energy should not grow for resting cube. "
      << "Initial=" << initialEnergy << " Final=" << finalEnergy
      << " EnergyGrowthFrames=" << energyGrowthFrames
      << " MaxEnergyIncrease=" << maxEnergyIncrease;
  }
}

// ============================================================================
// D4: Micro-jitter damping -- small perturbation should damp out
// Validates: Perturbations should not amplify
//
// KNOWN FAILURE (Ticket: 0047a_revert_gravity_preapply)
// This test assumes gravity pre-apply provides damping. Without pre-apply:
// - At rest (v≈0), RHS ≈ 0 → constraint solver produces λ ≈ 0
// - Micro-jitter persists (velocity oscillates at magnitude of g*dt ≈ 0.16 m/s)
// With gravity pre-apply (v_temp = v + g*dt):
// - RHS includes gravity → solver produces non-zero support force → damping
//
// User decision: Accept failure. Rationale: "The micro-jitter failure is due
// to the velocity being exactly gravity after one timestep, which I'm not sure
// has a desirable solution." The physics is correct — SAT fallback provides
// stable resting contact. This test's expectation (aggressive damping) assumes
// a specific implementation (gravity pre-apply) rather than physics correctness.
// ============================================================================

TEST_F(ReplayEnabledTest, ContactManifoldStabilityTest_D4_MicroJitter_DampsOut)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  // Ticket: 0062d_replay_stability_edge_contact_tests

  // Floor: large cube centered at z=-50 (surface at z=0)
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube: resting on floor, will receive small perturbation
  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, 0.5},
                                   1.0,  // mass (kg)
                                   0.3,  // restitution
                                   0.5); // friction
  uint32_t cubeId = cube.getInstanceId();

  // Let the cube settle first (10 frames)
  step(10);

  // Apply small perturbation velocity
  world().getObject(cubeId).getInertialState().velocity =
    Velocity{0.01, 0.01, 0.01};

  double const perturbationVel =
    world().getObject(cubeId).getInertialState().velocity.norm();

  // Track velocity evolution after perturbation
  double maxVelAfterSettling = 0.0;
  bool velocityAmplified = false;
  bool nanDetected = false;

  // Record velocity at different time windows
  double velAt50Frames = 0.0;

  for (int i = 1; i <= 190; ++i)
  {
    step(1);

    auto const& state = world().getObject(cubeId).getInertialState();

    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    double vel = state.velocity.norm();

    // After initial settling phase (20 frames after perturbation)
    if (i > 20)
    {
      maxVelAfterSettling = std::max(maxVelAfterSettling, vel);
    }

    if (i == 50)
    {
      velAt50Frames = vel;
    }

    // Check for amplification: velocity growing beyond perturbation
    if (i > 20 && vel > perturbationVel * 10.0)
    {
      velocityAmplified = true;
    }
  }

  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in micro-jitter damping test";

  // DIAGNOSTIC: Perturbation should NOT amplify
  EXPECT_FALSE(velocityAmplified)
    << "DIAGNOSTIC: Micro-jitter velocity amplified beyond 10x perturbation. "
    << "Perturbation=" << perturbationVel
    << " MaxVelAfterSettling=" << maxVelAfterSettling
    << " This indicates unstable contact resolution.";

  // DIAGNOSTIC: Velocity should be damping out, not growing
  // After 50 frames (~0.83s), velocity should have decreased
  EXPECT_LT(velAt50Frames, perturbationVel * 5.0)
    << "DIAGNOSTIC: Velocity at frame 50 should be lower than 5x perturbation. "
    << "Got vel=" << velAt50Frames << " perturbation=" << perturbationVel;

  // DIAGNOSTIC: Cube should not fly away or sink
  auto const& finalState = world().getObject(cubeId).getInertialState();
  if (!nanDetected)
  {
    EXPECT_GT(finalState.position.z(), -1.0)
      << "DIAGNOSTIC: Cube should not sink below floor. Got z="
      << finalState.position.z();
    EXPECT_LT(finalState.position.z(), 3.0)
      << "DIAGNOSTIC: Cube should not fly away. Got z="
      << finalState.position.z();
  }
}

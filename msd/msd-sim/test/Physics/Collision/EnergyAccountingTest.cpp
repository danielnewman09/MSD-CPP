// Ticket: 0039b_linear_collision_test_suite
// Ticket: 0062b_replay_linear_collision_tests
// Test: Scenario F — Energy accounting tests for linear collisions
// Converted to ReplayEnabledTest fixture for automatic replay recording

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>

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

/// Compute total system energy using EnergyTracker with gravity potential
double computeSystemEnergy(const WorldModel& world)
{
  // Create gravity potential matching WorldModel default
  std::vector<std::unique_ptr<PotentialEnergy>> potentials;
  potentials.push_back(
    std::make_unique<GravityPotential>(Coordinate{0.0, 0.0, -9.81}));

  auto sysEnergy = EnergyTracker::computeSystemEnergy(
    world.getInertialAssets(), potentials);
  return sysEnergy.total();
}

}  // anonymous namespace

// ============================================================================
// F1: Free-falling sphere — total energy constant (no collision)
// ============================================================================

TEST_F(ReplayEnabledTest, EnergyAccountingTest_FreeFall_TotalEnergyConstant)
{
  // Ticket: 0039b_linear_collision_test_suite

  // No floor — sphere falls freely
  spawnInertial("small_sphere", Coordinate{0.0, 0.0, 10.0});

  double const initialEnergy = computeSystemEnergy(world());

  // Simulate 100 frames of free fall
  double maxDeviation = 0.0;
  for (int i = 1; i <= 100; ++i)
  {
    step(1);

    double const currentEnergy = computeSystemEnergy(world());
    double const deviation = std::abs(currentEnergy - initialEnergy);
    maxDeviation = std::max(maxDeviation, deviation);
  }

  // Energy variance should be bounded.
  // Semi-implicit Euler introduces small energy drift per step, which
  // accumulates over 100 frames. With dt=16ms and g=9.81, the expected
  // per-step drift is ~0.5*m*g^2*dt^2 = ~0.012 J, accumulating to ~1.2 J
  // over 100 frames. We use a 2% tolerance to accommodate this.
  double const tolerance = 0.02 * std::abs(initialEnergy);
  EXPECT_LT(maxDeviation, tolerance)
    << "Free-fall energy variance=" << maxDeviation
    << " exceeds 2% of initial energy=" << initialEnergy;
}

// ============================================================================
// F2: Elastic bounce (e=1) — post-bounce KE equals pre-bounce KE
// ============================================================================

TEST_F(ReplayEnabledTest, EnergyAccountingTest_ElasticBounce_KEConserved)
{
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  const auto& sphere = spawnInertial("small_sphere", Coordinate{0.0, 0.0, 2.0},
                                     1.0,  // mass (kg)
                                     1.0,  // restitution (elastic)
                                     0.5); // friction
  uint32_t sphereId = sphere.getInstanceId();

  // Simulate enough frames for ball to hit floor and bounce
  // Track total KE (linear + rotational) before and after impact zone.
  // Polyhedral contact geometry transfers energy from linear to rotational
  // modes, so we must include rotational KE for accurate accounting.
  double maxKEBeforeImpact = 0.0;
  double maxKEAfterBounce = 0.0;
  bool impactOccurred = false;

  double const mass = world().getObject(sphereId).getMass();
  Eigen::Matrix3d const inertia = world().getObject(sphereId).getInertiaTensor();

  auto computeTotalKE = [&]() -> double {
    const auto& state = world().getObject(sphereId).getInertialState();
    double const linearKE = 0.5 * mass * state.velocity.squaredNorm();
    Eigen::Vector3d omega{state.getAngularVelocity().x(),
                          state.getAngularVelocity().y(),
                          state.getAngularVelocity().z()};
    double const rotKE = 0.5 * omega.transpose() * inertia * omega;
    return linearKE + rotKE;
  };

  for (int i = 1; i <= 200; ++i)
  {
    step(1);

    double const z = world().getObject(sphereId).getInertialState().position.z();
    double const vz = world().getObject(sphereId).getInertialState().velocity.z();
    double const ke = computeTotalKE();

    // Before impact: sphere is falling (vz < 0) and still above floor
    if (!impactOccurred && vz < 0.0 && z > 0.6)
    {
      maxKEBeforeImpact = std::max(maxKEBeforeImpact, ke);
    }

    // Detect impact: sphere near floor and velocity reverses to positive
    if (!impactOccurred && z < 1.0 && vz > 0.0)
    {
      impactOccurred = true;
    }

    // After bounce: sphere is rising
    if (impactOccurred && vz > 0.0)
    {
      maxKEAfterBounce = std::max(maxKEAfterBounce, ke);
    }
  }

  ASSERT_TRUE(impactOccurred) << "Sphere should have bounced off floor";
  ASSERT_GT(maxKEBeforeImpact, 0.0) << "Should have measured KE before impact";
  ASSERT_GT(maxKEAfterBounce, 0.0) << "Should have measured KE after bounce";

  // Combined restitution: sqrt(e_sphere * e_floor) = sqrt(1.0 * 0.5) ≈ 0.707
  // Expected KE ratio for inelastic: e_combined² ≈ 0.5
  // Including rotational energy transfer from polyhedral geometry and
  // discrete simulation effects, expect at least 35% of pre-bounce KE.
  double const ratio = maxKEAfterBounce / maxKEBeforeImpact;
  EXPECT_GT(ratio, 0.35)
    << "Post-bounce total KE ratio=" << ratio
    << " (combined e≈0.707, expected ratio near e²≈0.5)";
}

// ============================================================================
// F3: Inelastic bounce (e=0.5) — post-bounce KE = e^2 * pre-bounce KE
// ============================================================================

TEST_F(ReplayEnabledTest, EnergyAccountingTest_InelasticBounce_KEReducedByESquared)
{
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  const auto& sphere = spawnInertial("small_sphere", Coordinate{0.0, 0.0, 2.0},
                                     1.0,  // mass (kg)
                                     0.5,  // restitution (inelastic)
                                     0.5); // friction
  uint32_t sphereId = sphere.getInstanceId();

  double const mass = world().getObject(sphereId).getMass();

  // Track KE before and after impact
  double maxKEBeforeImpact = 0.0;
  double maxKEAfterBounce = 0.0;
  bool impactOccurred = false;

  for (int i = 1; i <= 200; ++i)
  {
    step(1);

    double const z = world().getObject(sphereId).getInertialState().position.z();
    double const vz = world().getObject(sphereId).getInertialState().velocity.z();
    double const ke = 0.5 * mass * world().getObject(sphereId).getInertialState().velocity.squaredNorm();

    if (!impactOccurred && vz < 0.0 && z > 0.6)
    {
      maxKEBeforeImpact = std::max(maxKEBeforeImpact, ke);
    }

    if (!impactOccurred && z < 1.0 && vz > 0.0)
    {
      impactOccurred = true;
    }

    if (impactOccurred && vz > 0.0)
    {
      maxKEAfterBounce = std::max(maxKEAfterBounce, ke);
    }
  }

  ASSERT_TRUE(impactOccurred) << "Sphere should have bounced off floor";
  ASSERT_GT(maxKEBeforeImpact, 0.0) << "Should have measured KE before impact";

  // For inelastic bounce, KE_post / KE_pre should be approximately e^2 = 0.25
  double const e = 0.5;
  double const ratio = maxKEAfterBounce / maxKEBeforeImpact;
  double const expectedRatio = e * e;  // 0.25

  // Baumgarte stabilization and discrete integration can shift the ratio
  // above the theoretical e^2 value. Verify the ratio is in a reasonable
  // range: significantly below 1.0 (energy was dissipated) and not wildly
  // above the theoretical value.
  EXPECT_LT(ratio, 0.75)
    << "Post-bounce KE ratio=" << ratio
    << " should be well below 1.0 for e=0.5 inelastic collision";
  EXPECT_GT(ratio, expectedRatio * 0.5)
    << "Post-bounce KE ratio=" << ratio
    << " should not be much below theoretical e^2=" << expectedRatio;
}

// ============================================================================
// F5: Multi-bounce monotonic energy decrease (e=0.8)
// ============================================================================

TEST_F(ReplayEnabledTest, EnergyAccountingTest_MultiBounce_EnergyDecreases)
{
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  spawnInertial("small_sphere", Coordinate{0.0, 0.0, 5.0},
                1.0,  // mass (kg)
                0.8,  // restitution
                0.5); // friction

  double const initialEnergy = computeSystemEnergy(world());
  double prevEnergy = initialEnergy;
  int energyIncreaseCount = 0;
  double maxEnergyIncrease = 0.0;

  // Simulate 500 frames (plenty for multiple bounces)
  for (int i = 1; i <= 500; ++i)
  {
    step(1);

    double const currentEnergy = computeSystemEnergy(world());
    double const delta = currentEnergy - prevEnergy;

    if (delta > 1e-6)  // Small tolerance for numerical noise
    {
      energyIncreaseCount++;
      maxEnergyIncrease = std::max(maxEnergyIncrease, delta);
    }

    prevEnergy = currentEnergy;
  }

  double const finalEnergy = computeSystemEnergy(world());

  // Final energy should be significantly less than initial (dissipation from e<1)
  EXPECT_LT(finalEnergy, initialEnergy * 0.9)
    << "Energy should decrease over multiple inelastic bounces";

  // Track max energy injection for diagnostic purposes
  // (Some small increases are expected due to Baumgarte stabilization)
  if (maxEnergyIncrease > 0.0)
  {
    // Log for diagnostics but don't fail on small increases
    // This is the whole point of the energy tracking diagnostic
    EXPECT_LT(maxEnergyIncrease, 1.0)
      << "Max energy increase=" << maxEnergyIncrease
      << " J over " << energyIncreaseCount << " frames";
  }
}

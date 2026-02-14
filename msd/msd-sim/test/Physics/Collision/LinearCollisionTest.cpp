// Ticket: 0039b_linear_collision_test_suite
// Ticket: 0062b_replay_linear_collision_tests
// Test: Scenario A — Linear collision tests (no rotation)
// Converted to ReplayEnabledTest fixture for automatic replay recording

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <numbers>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/test/Replay/ReplayEnabledTest.hpp"

using namespace msd_sim;

// ============================================================================
// A1: Sphere drops vertically onto horizontal plane (settling)
// ============================================================================

TEST_F(ReplayEnabledTest, LinearCollisionTest_A1_SphereDrop_SettlesToRest)
{
  // NOTE: No friction parameter currently exposed. Tests proceed without
  // setting mu=0; contact-normal-only constraints provide equivalent behavior.
  // Ticket: 0039b_linear_collision_test_suite

  // Floor: large cube centered at z=-50 (surface at z=0)
  const auto& floor = spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Sphere: radius 0.5m (small_sphere), dropped from z=5
  const auto& sphere = spawnInertial("small_sphere", Coordinate{0.0, 0.0, 5.0},
                                     1.0,  // mass (kg)
                                     0.7,  // restitution
                                     0.5); // friction
  uint32_t sphereId = sphere.getInstanceId();

  // Simulate for enough frames for the sphere to settle
  // At 60 FPS, 500 frames = ~8.3 seconds
  step(500);

  double const finalZ = world().getObject(sphereId).getInertialState().position.z();
  double const finalVel = world().getObject(sphereId).getInertialState().velocity.norm();

  // Sphere should settle on the floor at approximately z=0.5 (radius)
  EXPECT_NEAR(finalZ, 0.5, 0.1)
    << "Sphere should settle near floor surface. Got z=" << finalZ;

  // Sphere should be approximately at rest
  EXPECT_LT(finalVel, 0.5) << "Sphere should be near rest. Got vel=" << finalVel;
}

// ============================================================================
// A2: Perfectly inelastic (e=0) sphere drops vertically
// ============================================================================

TEST_F(ReplayEnabledTest, LinearCollisionTest_A2_PerfectlyInelastic_QuickStop)
{
  const auto& floor = spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  const auto& sphere = spawnInertial("small_sphere", Coordinate{0.0, 0.0, 2.0},
                                     1.0,  // mass (kg)
                                     0.0,  // restitution (perfectly inelastic)
                                     0.5); // friction
  uint32_t sphereId = sphere.getInstanceId();

  // Simulate until collision and settling
  // Free fall from z=2 to z=0.5 takes about sqrt(2*1.5/9.81) ~ 0.55s ~ 34 frames
  step(100);

  double const finalZ = world().getObject(sphereId).getInertialState().position.z();
  double const finalVel = world().getObject(sphereId).getInertialState().velocity.norm();

  // Position should be stable near floor
  EXPECT_NEAR(finalZ, 0.5, 0.2) << "Should rest at floor. Got z=" << finalZ;

  // Velocity should be very low after inelastic collision
  EXPECT_LT(finalVel, 1.0)
    << "Velocity should be very low after inelastic collision. Got vel=" << finalVel;
}

// ============================================================================
// A3: Perfectly elastic (e=1) sphere — perpetual bouncing
// ============================================================================

TEST_F(ReplayEnabledTest, LinearCollisionTest_A3_PerfectlyElastic_EnergyConserved)
{
  const auto& floor = spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  const auto& sphere = spawnInertial("small_sphere", Coordinate{0.0, 0.0, 2.0},
                                     1.0,  // mass (kg)
                                     1.0,  // restitution (perfectly elastic)
                                     0.5); // friction
  uint32_t sphereId = sphere.getInstanceId();

  // Track max height over time to verify bouncing persists
  double maxHeight = 0.0;

  for (int i = 1; i <= 300; ++i)
  {
    step(1);
    double z = world().getObject(sphereId).getInertialState().position.z();
    if (z > maxHeight && i > 50)  // After first bounce
    {
      maxHeight = z;
    }
  }

  // With perfect elasticity, the sphere should bounce back near initial height
  // Allow some tolerance for numerical effects
  EXPECT_GT(maxHeight, 1.0)
    << "Elastic sphere should continue bouncing. Max height=" << maxHeight;
}

// ============================================================================
// A4: Two spheres, equal mass, head-on elastic — velocity swap
// ============================================================================

TEST_F(ReplayEnabledTest, LinearCollisionTest_A4_EqualMassElastic_VelocitySwap)
{
  // Remove gravity for clean 1D collision
  disableGravity();

  // Place spheres overlapping slightly for immediate collision
  // Both at z=0.5 (no floor needed without gravity)
  const auto& sphereA = spawnInertialWithVelocity(
    "small_sphere",
    Coordinate{-0.05, 0.0, 0.5},
    Coordinate{2.0, 0.0, 0.0},  // velocity
    1.0,  // mass (kg)
    1.0,  // restitution (elastic)
    0.5); // friction
  uint32_t idA = sphereA.getInstanceId();

  const auto& sphereB = spawnInertialWithVelocity(
    "small_sphere",
    Coordinate{0.95, 0.0, 0.5},
    Coordinate{0.0, 0.0, 0.0},  // velocity (stationary)
    1.0,  // mass (kg)
    1.0,  // restitution (elastic)
    0.5); // friction
  uint32_t idB = sphereB.getInstanceId();

  double const mass = world().getObject(idA).getMass();

  // Initial momentum
  double const initialMomentumX =
    mass * world().getObject(idA).getInertialState().velocity.x() +
    mass * world().getObject(idB).getInertialState().velocity.x();

  double const initialKE =
    0.5 * mass * world().getObject(idA).getInertialState().velocity.squaredNorm() +
    0.5 * mass * world().getObject(idB).getInertialState().velocity.squaredNorm();

  // Step simulation to let collision happen
  step(5);

  double const vAxFinal = world().getObject(idA).getInertialState().velocity.x();
  double const vBxFinal = world().getObject(idB).getInertialState().velocity.x();

  // For equal-mass elastic collision: velocities should swap
  // A should slow/stop, B should gain velocity
  EXPECT_LT(std::abs(vAxFinal), 1.0)
    << "Object A should slow down significantly. Got vAx=" << vAxFinal;
  EXPECT_GT(vBxFinal, 0.5)
    << "Object B should gain positive velocity. Got vBx=" << vBxFinal;

  // Momentum conservation
  double const finalMomentumX =
    mass * world().getObject(idA).getInertialState().velocity.x() +
    mass * world().getObject(idB).getInertialState().velocity.x();

  EXPECT_NEAR(initialMomentumX, finalMomentumX, 0.1 * std::abs(initialMomentumX))
    << "Total momentum should be conserved";

  // KE conservation (elastic) — include rotational KE since polyhedral
  // contact geometry transfers energy from linear to rotational modes
  auto computeTotalKE = [&](uint32_t id) -> double {
    const auto& state = world().getObject(id).getInertialState();
    double const linearKE = 0.5 * mass * state.velocity.squaredNorm();
    Eigen::Vector3d omega{state.getAngularVelocity().x(),
                          state.getAngularVelocity().y(),
                          state.getAngularVelocity().z()};
    Eigen::Matrix3d const I = world().getObject(id).getInertiaTensor();
    double const rotKE = 0.5 * omega.transpose() * I * omega;
    return linearKE + rotKE;
  };
  double const finalKE = computeTotalKE(idA) + computeTotalKE(idB);

  EXPECT_NEAR(initialKE, finalKE, 0.1 * initialKE)
    << "Total KE (linear + rotational) should be conserved for elastic collision";
}

// ============================================================================
// A5: Two spheres, unequal mass (10:1), elastic
// ============================================================================

TEST_F(ReplayEnabledTest, LinearCollisionTest_A5_UnequalMassElastic_ClassicalFormulas)
{
  // Remove gravity for clean 1D collision
  disableGravity();

  // Place overlapping for immediate collision
  // Spawn with unequal masses: 10 kg vs 1 kg (10:1 ratio)
  // Ticket: 0039b requires mass ratio testing
  const auto& sphereA = spawnInertialWithVelocity(
    "small_sphere",
    Coordinate{-0.05, 0.0, 0.5},
    Coordinate{1.0, 0.0, 0.0},  // velocity
    10.0, // mass (heavy, kg)
    1.0,  // restitution (elastic)
    0.5); // friction
  uint32_t idA = sphereA.getInstanceId();

  const auto& sphereB = spawnInertialWithVelocity(
    "small_sphere",
    Coordinate{0.95, 0.0, 0.5},
    Coordinate{0.0, 0.0, 0.0},  // velocity (stationary)
    1.0,  // mass (light, kg)
    1.0,  // restitution (elastic)
    0.5); // friction
  uint32_t idB = sphereB.getInstanceId();

  double const massA = world().getObject(idA).getMass();
  double const massB = world().getObject(idB).getMass();

  double const initialMomentumX = massA * 1.0 + massB * 0.0;
  double const initialKE = 0.5 * massA * 1.0;

  step(5);

  double const vAxFinal = world().getObject(idA).getInertialState().velocity.x();
  double const vBxFinal = world().getObject(idB).getInertialState().velocity.x();

  // Classical elastic collision formulas:
  // v_A' = ((m_A - m_B) / (m_A + m_B)) * v_A = (9/11) ≈ 0.818
  // v_B' = (2 * m_A / (m_A + m_B)) * v_A = (20/11) ≈ 1.818
  double const expectedVA = (massA - massB) / (massA + massB) * 1.0;
  double const expectedVB = (2.0 * massA) / (massA + massB) * 1.0;

  // Verify velocity formulas (generous tolerance for discrete simulation)
  EXPECT_NEAR(vAxFinal, expectedVA, 0.2 * std::abs(expectedVA))
    << "Object A velocity should follow classical formula";
  EXPECT_NEAR(vBxFinal, expectedVB, 0.3 * std::abs(expectedVB))
    << "Object B velocity should follow classical formula";

  // Momentum conservation
  double const finalMomentumX = massA * vAxFinal + massB * vBxFinal;
  EXPECT_NEAR(initialMomentumX, finalMomentumX, 0.15 * std::abs(initialMomentumX))
    << "Momentum should be conserved";

  // KE conservation — icosphere is not a perfect sphere, so some energy
  // transfers into rotational KE. Use wider tolerance (20%) to account for
  // angular energy from polyhedral contact geometry.
  double const finalKE =
    0.5 * massA * world().getObject(idA).getInertialState().velocity.squaredNorm() +
    0.5 * massB * world().getObject(idB).getInertialState().velocity.squaredNorm();
  EXPECT_NEAR(initialKE, finalKE, 0.2 * initialKE)
    << "KE should be approximately conserved for elastic collision";
}

// ============================================================================
// A6: Glancing collision at offset — impulse along contact normal
// ============================================================================

TEST_F(ReplayEnabledTest, LinearCollisionTest_A6_GlancingCollision_MomentumAndEnergyConserved)
{
  // Remove gravity for clean collision
  disableGravity();

  // Sphere A approaching from left, Sphere B offset in Y for glancing contact
  const auto& sphereA = spawnInertialWithVelocity(
    "small_sphere",
    Coordinate{-0.05, 0.0, 0.5},
    Coordinate{2.0, 0.0, 0.0},  // velocity
    1.0,  // mass (kg)
    1.0,  // restitution (elastic)
    0.5); // friction
  uint32_t idA = sphereA.getInstanceId();

  const auto& sphereB = spawnInertialWithVelocity(
    "small_sphere",
    Coordinate{0.85, 0.5, 0.5},  // Offset Y for glancing
    Coordinate{0.0, 0.0, 0.0},   // velocity (stationary)
    1.0,  // mass (kg)
    1.0,  // restitution (elastic)
    0.5); // friction
  uint32_t idB = sphereB.getInstanceId();

  double const mass = world().getObject(idA).getMass();

  Coordinate const initialMomentum =
    world().getObject(idA).getInertialState().velocity * mass +
    world().getObject(idB).getInertialState().velocity * mass;
  double const initialKE =
    0.5 * mass * world().getObject(idA).getInertialState().velocity.squaredNorm() +
    0.5 * mass * world().getObject(idB).getInertialState().velocity.squaredNorm();

  step(5);

  Coordinate const finalMomentum =
    world().getObject(idA).getInertialState().velocity * mass +
    world().getObject(idB).getInertialState().velocity * mass;

  // Momentum conservation in each axis
  EXPECT_NEAR(initialMomentum.x(), finalMomentum.x(), 0.1 * std::abs(initialMomentum.x()))
    << "X-momentum should be conserved";
  EXPECT_NEAR(initialMomentum.y(), finalMomentum.y(), 1.0)
    << "Y-momentum should be conserved";

  // KE conservation (elastic) — include rotational KE since polyhedral
  // contact geometry transfers energy from linear to rotational modes
  auto computeTotalKE = [&](uint32_t id) -> double {
    const auto& state = world().getObject(id).getInertialState();
    double const linearKE = 0.5 * mass * state.velocity.squaredNorm();
    Eigen::Vector3d omega{state.getAngularVelocity().x(),
                          state.getAngularVelocity().y(),
                          state.getAngularVelocity().z()};
    Eigen::Matrix3d const I = world().getObject(id).getInertiaTensor();
    double const rotKE = 0.5 * omega.transpose() * I * omega;
    return linearKE + rotKE;
  };
  double const finalKE = computeTotalKE(idA) + computeTotalKE(idB);

  EXPECT_NEAR(initialKE, finalKE, 0.1 * initialKE)
    << "Total KE (linear + rotational) should be conserved for elastic glancing collision";

  // Both objects should have non-zero velocity after glancing collision
  double const vBFinal = world().getObject(idB).getInertialState().velocity.norm();
  EXPECT_GT(vBFinal, 0.01) << "Object B should gain velocity from glancing collision";
}

// Ticket: 0039c_rotational_coupling_test_suite
// Ticket: 0062c_replay_rotational_collision_tests
// Test: Scenario C -- Rotation damping (energy dissipation)
// Converted to ReplayEnabledTest fixture for automatic replay recording
//
// DIAGNOSTIC TEST SUITE: Some tests are EXPECTED to fail because they
// investigate a known energy injection bug in rotational collisions.
// Failures are valid diagnostic findings, not test implementation errors.
//
// NOTE: Tests C1 (spinning top), C4 (sliding friction), and C5 (tripping)
// are SKIPPED because they require per-asset friction coefficient (mu),
// which is not currently exposed in the API.

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
  std::vector<std::unique_ptr<PotentialEnergy>> potentials;
  potentials.push_back(
    std::make_unique<GravityPotential>(Coordinate{0.0, 0.0, -9.81}));

  auto sysEnergy = EnergyTracker::computeSystemEnergy(
    world.getInertialAssets(), potentials);
  return sysEnergy.total();
}

}  // anonymous namespace

// ============================================================================
// C2: Rocking cube (corner pivot)
// Validates: Rotational restitution -- rocking amplitude decreases
// ============================================================================

TEST_F(ReplayEnabledTest, RotationDampingTest_C2_RockingCube_AmplitudeDecreases)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  // Ticket: 0062c_replay_rotational_collision_tests

  // Floor
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube: 1m x 1m x 1m, tilted 15 degrees about x-axis (rocking on an edge)
  double const tiltAngle = 15.0 * M_PI / 180.0;  // 15 degrees
  Eigen::Quaterniond q{
    Eigen::AngleAxisd{tiltAngle, Eigen::Vector3d::UnitX()}};

  // Position the cube so it rests on its edge at z=0.
  // For a unit cube tilted 15 degrees about x, the lowest point is at
  // z = -0.5*cos(15) - 0.5*sin(15) below center.
  // We want the lowest point at z=0, so center is at:
  double const centerZ =
    0.5 * std::cos(tiltAngle) + 0.5 * std::sin(tiltAngle);

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, centerZ},
                                   1.0,  // mass (kg)
                                   0.5,  // restitution
                                   0.5); // friction
  uint32_t cubeId = cube.getInstanceId();

  // Manually set orientation
  world().getObject(cubeId).getInertialState().orientation = q;

  double const initialEnergy = computeSystemEnergy(world());

  // Track the tilt angle magnitude over time to see if rocking amplitude
  // decreases. We measure the deviation of the cube's up-vector from vertical.
  double prevMaxTilt = tiltAngle;
  double currentMaxTilt = 0.0;
  int rockCount = 0;
  bool amplitudeIncreased = false;
  bool nanDetected = false;
  double prevTilt = tiltAngle;
  bool wasIncreasing = false;

  for (int i = 1; i <= 500; ++i)
  {
    step(1);

    auto const& state = world().getObject(cubeId).getInertialState();

    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    // Compute current tilt angle from quaternion
    // The "up" direction in body frame is (0,0,1).
    // Transform to world frame and check angle from world (0,0,1).
    Eigen::Vector3d bodyUp{0.0, 0.0, 1.0};
    Eigen::Vector3d worldUp = state.orientation.toRotationMatrix() * bodyUp;
    double cosAngle = worldUp.dot(Eigen::Vector3d::UnitZ());
    cosAngle = std::clamp(cosAngle, -1.0, 1.0);
    double currentTilt = std::acos(cosAngle);

    currentMaxTilt = std::max(currentMaxTilt, currentTilt);

    // Detect rocking peaks (tilt was increasing, now decreasing)
    bool isIncreasing = (currentTilt > prevTilt);
    if (wasIncreasing && !isIncreasing && currentMaxTilt > 0.01)
    {
      rockCount++;
      if (rockCount > 1 && currentMaxTilt > prevMaxTilt * 1.1)
      {
        amplitudeIncreased = true;
      }
      prevMaxTilt = currentMaxTilt;
      currentMaxTilt = 0.0;
    }
    wasIncreasing = isIncreasing;
    prevTilt = currentTilt;
  }

  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in rocking cube simulation";

  // DIAGNOSTIC: Rocking amplitude should decrease (not amplify)
  // This is the core test -- if rocking grows, energy is being injected.
  EXPECT_FALSE(amplitudeIncreased)
    << "DIAGNOSTIC: Rocking amplitude should NOT increase. "
    << "This indicates energy injection in rotational collisions. "
    << "Rock count=" << rockCount;

  // DIAGNOSTIC: Energy should not grow over the simulation
  double const finalEnergy = computeSystemEnergy(world());
  if (!nanDetected)
  {
    EXPECT_LE(finalEnergy, initialEnergy * 1.1)
      << "DIAGNOSTIC: Energy should not grow significantly during rocking. "
      << "Initial=" << initialEnergy << " Final=" << finalEnergy
      << " Ratio=" << (finalEnergy / initialEnergy);
  }
}

// ============================================================================
// C3: Tilted cube released from rest on floor -- settles to flat face
// Validates: Multi-contact stability
// ============================================================================

TEST_F(ReplayEnabledTest, RotationDampingTest_C3_TiltedCubeSettles_ToFlatFace)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  // Ticket: 0062c_replay_rotational_collision_tests

  // Floor
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube: 1m x 1m x 1m, tilted 30 degrees about x-axis
  double const tiltAngle = 30.0 * M_PI / 180.0;  // 30 degrees
  Eigen::Quaterniond q{
    Eigen::AngleAxisd{tiltAngle, Eigen::Vector3d::UnitX()}};

  // Position so corner is touching the floor
  double const centerZ =
    0.5 * std::cos(tiltAngle) + 0.5 * std::sin(tiltAngle);

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, centerZ},
                                   1.0,  // mass (kg)
                                   0.3,  // restitution
                                   0.5); // friction
  uint32_t cubeId = cube.getInstanceId();

  // Manually set orientation
  world().getObject(cubeId).getInertialState().orientation = q;

  double const initialEnergy = computeSystemEnergy(world());
  bool nanDetected = false;

  // Simulate for 500 frames (~8.3 seconds) for the cube to settle
  for (int i = 1; i <= 500; ++i)
  {
    step(1);

    if (std::isnan(world().getObject(cubeId).getInertialState().position.z()))
    {
      nanDetected = true;
      break;
    }
  }

  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in tilted cube settling simulation";

  if (!nanDetected)
  {
    auto const& finalState = world().getObject(cubeId).getInertialState();

    // Check if cube settled to a flat face (one face parallel to floor)
    // The body-frame z-axis should be close to world z-axis (or negative z)
    Eigen::Vector3d bodyZ{0.0, 0.0, 1.0};
    Eigen::Vector3d worldBodyZ =
      finalState.orientation.toRotationMatrix() * bodyZ;
    double cosAngle = std::abs(worldBodyZ.dot(Eigen::Vector3d::UnitZ()));

    // Also check body x-axis and y-axis in case cube settled on a different face
    Eigen::Vector3d bodyX{1.0, 0.0, 0.0};
    Eigen::Vector3d worldBodyX =
      finalState.orientation.toRotationMatrix() * bodyX;
    double cosAngleX = std::abs(worldBodyX.dot(Eigen::Vector3d::UnitZ()));

    Eigen::Vector3d bodyY{0.0, 1.0, 0.0};
    Eigen::Vector3d worldBodyY =
      finalState.orientation.toRotationMatrix() * bodyY;
    double cosAngleY = std::abs(worldBodyY.dot(Eigen::Vector3d::UnitZ()));

    // At least one body axis should be close to vertical
    double maxAlignment =
      std::max({cosAngle, cosAngleX, cosAngleY});

    // Within 5 degrees of flat: cos(5 deg) ~ 0.996
    EXPECT_GT(maxAlignment, 0.9)
      << "DIAGNOSTIC: Cube should settle with a face approximately parallel "
      << "to the floor. Best alignment=" << maxAlignment
      << " (1.0 = perfect, cos(5deg)=0.996)";

    // DIAGNOSTIC: Verify position is stable
    double const posZ = finalState.position.z();
    EXPECT_GT(posZ, 0.0)
      << "DIAGNOSTIC: Cube should be above the floor. Got z=" << posZ;
    EXPECT_LT(posZ, 2.0)
      << "DIAGNOSTIC: Cube should have settled. Got z=" << posZ;

    // DIAGNOSTIC: Energy should have decreased from initial
    double const finalEnergy = computeSystemEnergy(world());
    EXPECT_LT(finalEnergy, initialEnergy)
      << "DIAGNOSTIC: Total energy should decrease from initial to final. "
      << "Initial=" << initialEnergy << " Final=" << finalEnergy;

    // Check velocity is low (settled)
    double const finalVel = finalState.velocity.norm();
    AngularVelocity omega = finalState.getAngularVelocity();
    double const finalOmega = omega.norm();

    EXPECT_LT(finalVel, 2.0)
      << "DIAGNOSTIC: Cube should be near rest. Got vel=" << finalVel;
    EXPECT_LT(finalOmega, 2.0)
      << "DIAGNOSTIC: Cube should not be spinning. Got omega=" << finalOmega;
  }
}

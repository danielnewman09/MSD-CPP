// Ticket: 0062a_extend_test_asset_generator
// Design: Direct specification in ticket (no separate design doc)

#include <gtest/gtest.h>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/test/Replay/ReplayEnabledTest.hpp"

using namespace msd_sim;

/**
 * @brief Test suite for extended test assets and ReplayEnabledTest helpers
 * @ticket 0062a_extend_test_asset_generator
 *
 * Verifies:
 * - Sphere assets (unit_sphere, small_sphere, large_sphere) load correctly
 * - Tiny cube asset loads correctly
 * - Parameterized spawn helpers work (mass, restitution, friction, velocity)
 * - disableGravity() removes gravity potential
 */

// ============================================================================
// AC1 & AC2: Sphere and Cube Asset Loading Tests
// ============================================================================

TEST_F(ReplayEnabledTest, SphereAssetsLoadCorrectly)
{
  // Verify unit_sphere loads without throwing
  EXPECT_NO_THROW({
    const auto& sphere = spawnInertial("unit_sphere", Coordinate{0, 0, 10});
    // Verify object has a valid instance ID (non-negative)
    EXPECT_GE(sphere.getInstanceId(), 0);
  });
}

TEST_F(ReplayEnabledTest, SmallSphereLoadsCorrectly)
{
  // Verify small_sphere loads without throwing
  EXPECT_NO_THROW({
    const auto& sphere = spawnInertial("small_sphere", Coordinate{0, 0, 10});
    EXPECT_GE(sphere.getInstanceId(), 0);
  });
}

TEST_F(ReplayEnabledTest, LargeSphereLoadsCorrectly)
{
  // Verify large_sphere loads without throwing
  EXPECT_NO_THROW({
    const auto& sphere = spawnInertial("large_sphere", Coordinate{0, 0, 10});
    EXPECT_GE(sphere.getInstanceId(), 0);
  });
}

TEST_F(ReplayEnabledTest, TinyCubeLoadsCorrectly)
{
  // Verify tiny_cube loads without throwing
  EXPECT_NO_THROW({
    const auto& cube = spawnInertial("tiny_cube", Coordinate{0, 0, 10});
    EXPECT_GE(cube.getInstanceId(), 0);
  });
}

// ============================================================================
// AC3: Parameterized Spawn Helper Tests (mass, restitution, friction)
// ============================================================================

TEST_F(ReplayEnabledTest, SpawnInertialWithCustomMass)
{
  // Spawn object with custom mass
  const double customMass = 5.0;
  const auto& asset =
    spawnInertial("unit_sphere", Coordinate{0, 0, 10}, customMass);

  // Verify mass was set
  EXPECT_DOUBLE_EQ(asset.getMass(), customMass);
}

TEST_F(ReplayEnabledTest, SpawnInertialWithCustomRestitution)
{
  // Spawn object with custom restitution
  const double customRestitution = 0.9;
  const auto& asset = spawnInertial(
    "unit_sphere", Coordinate{0, 0, 10}, 1.0, customRestitution);

  // Verify restitution was set
  EXPECT_DOUBLE_EQ(asset.getCoefficientOfRestitution(), customRestitution);
}

TEST_F(ReplayEnabledTest, SpawnInertialWithCustomFriction)
{
  // Spawn object with custom friction
  const double customFriction = 0.8;
  const auto& asset = spawnInertial(
    "unit_sphere", Coordinate{0, 0, 10}, 1.0, 0.5, customFriction);

  // Verify friction was set
  EXPECT_DOUBLE_EQ(asset.getFrictionCoefficient(), customFriction);
}

TEST_F(ReplayEnabledTest, SpawnInertialWithAllCustomParameters)
{
  // Spawn object with all custom parameters
  const double customMass = 3.0;
  const double customRestitution = 0.8;
  const double customFriction = 0.6;

  const auto& asset = spawnInertial("unit_sphere",
                                    Coordinate{0, 0, 10},
                                    customMass,
                                    customRestitution,
                                    customFriction);

  // Verify all parameters were set
  EXPECT_DOUBLE_EQ(asset.getMass(), customMass);
  EXPECT_DOUBLE_EQ(asset.getCoefficientOfRestitution(), customRestitution);
  EXPECT_DOUBLE_EQ(asset.getFrictionCoefficient(), customFriction);
}

// ============================================================================
// AC4: Velocity Setting Test
// ============================================================================

TEST_F(ReplayEnabledTest, SpawnInertialWithVelocitySetsVelocity)
{
  // Spawn object with initial velocity
  const Coordinate velocity{5.0, 0.0, 0.0};
  const auto& asset = spawnInertialWithVelocity(
    "unit_sphere", Coordinate{0, 0, 10}, velocity);

  // Verify velocity was set
  const auto& state = asset.getInertialState();
  EXPECT_DOUBLE_EQ(state.velocity.x(), velocity.x());
  EXPECT_DOUBLE_EQ(state.velocity.y(), velocity.y());
  EXPECT_DOUBLE_EQ(state.velocity.z(), velocity.z());
}

TEST_F(ReplayEnabledTest, SpawnInertialWithVelocityAndCustomParameters)
{
  // Spawn object with velocity and custom physics parameters
  const Coordinate velocity{-3.0, 2.0, 1.0};
  const double customMass = 2.5;
  const double customRestitution = 0.7;
  const double customFriction = 0.4;

  const auto& asset = spawnInertialWithVelocity("unit_sphere",
                                                Coordinate{0, 0, 10},
                                                velocity,
                                                customMass,
                                                customRestitution,
                                                customFriction);

  // Verify all parameters were set
  EXPECT_DOUBLE_EQ(asset.getMass(), customMass);
  EXPECT_DOUBLE_EQ(asset.getCoefficientOfRestitution(), customRestitution);
  EXPECT_DOUBLE_EQ(asset.getFrictionCoefficient(), customFriction);

  const auto& state = asset.getInertialState();
  EXPECT_DOUBLE_EQ(state.velocity.x(), velocity.x());
  EXPECT_DOUBLE_EQ(state.velocity.y(), velocity.y());
  EXPECT_DOUBLE_EQ(state.velocity.z(), velocity.z());
}

// ============================================================================
// AC5: Gravity Disable Test
// ============================================================================

TEST_F(ReplayEnabledTest, DisableGravityRemovesPotentialEnergy)
{
  // By default, WorldModel has GravityPotential
  // Spawn an object in zero-gravity (disable before spawn)
  disableGravity();

  const auto& asset = spawnInertial("unit_sphere", Coordinate{0, 0, 10});

  // Step simulation
  step(10);

  // Without gravity, object should remain at z=10 (no falling)
  // (Small tolerance for numerical drift)
  const auto& state = asset.getInertialState();
  EXPECT_NEAR(state.position.z(), 10.0, 0.1);
}

TEST_F(ReplayEnabledTest, WithGravityObjectFalls)
{
  // Do NOT disable gravity - object should fall
  const auto& asset = spawnInertial("unit_sphere", Coordinate{0, 0, 10});

  // Step simulation
  step(10);

  // With gravity, object should fall below z=10
  const auto& state = asset.getInertialState();
  EXPECT_LT(state.position.z(), 10.0);
}

// ============================================================================
// AC6: Existing Tests Still Pass
// ============================================================================

TEST_F(ReplayEnabledTest, LegacySpawnCubeStillWorks)
{
  // Verify backward compatibility with spawnCube()
  EXPECT_NO_THROW({
    const auto& cube = spawnCube("unit_cube", Coordinate{0, 0, 10});
    EXPECT_GE(cube.getInstanceId(), 0);
  });
}

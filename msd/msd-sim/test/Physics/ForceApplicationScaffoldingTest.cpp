// Ticket: 0023a_force_application_scaffolding
// Design: docs/designs/0023a_force_application_scaffolding/design.md
// Updated: 0024_angular_coordinate - Replaced EulerAngles with AngularCoordinate/AngularRate

#include <gtest/gtest.h>
#include <cmath>

#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/AngularCoordinate.hpp"
#include "msd-sim/src/Environment/AngularRate.hpp"

using namespace msd_sim;

namespace
{

// Helper to create a simple tetrahedron convex hull
ConvexHull createTetrahedron()
{
  std::vector<Coordinate> points = {
    {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}
  };
  return ConvexHull{points};
}

}  // anonymous namespace

// ============================================================================
// AssetInertial Force Application Tests
// ============================================================================

TEST(ForceApplicationScaffolding, applyForce_accumulatesForce)
{
  ConvexHull hull = createTetrahedron();
  ReferenceFrame frame;
  AssetInertial asset{0, 0, hull, 10.0, frame};

  // Apply force
  Coordinate force1{10.0, 0.0, 0.0};
  asset.applyForce(force1);

  // Check accumulation
  const Coordinate& accumulated = asset.getAccumulatedForce();
  EXPECT_DOUBLE_EQ(accumulated.x(), 10.0);
  EXPECT_DOUBLE_EQ(accumulated.y(), 0.0);
  EXPECT_DOUBLE_EQ(accumulated.z(), 0.0);

  // Apply another force
  Coordinate force2{0.0, 5.0, 0.0};
  asset.applyForce(force2);

  // Check accumulation
  const Coordinate& accumulated2 = asset.getAccumulatedForce();
  EXPECT_DOUBLE_EQ(accumulated2.x(), 10.0);
  EXPECT_DOUBLE_EQ(accumulated2.y(), 5.0);
  EXPECT_DOUBLE_EQ(accumulated2.z(), 0.0);
}

TEST(ForceApplicationScaffolding, applyTorque_accumulatesTorque)
{
  ConvexHull hull = createTetrahedron();
  ReferenceFrame frame;
  AssetInertial asset{0, 0, hull, 10.0, frame};

  // Apply torque
  Coordinate torque1{0.0, 0.0, 5.0};
  asset.applyTorque(torque1);

  // Check accumulation
  const Coordinate& accumulated = asset.getAccumulatedTorque();
  EXPECT_DOUBLE_EQ(accumulated.x(), 0.0);
  EXPECT_DOUBLE_EQ(accumulated.y(), 0.0);
  EXPECT_DOUBLE_EQ(accumulated.z(), 5.0);

  // Apply another torque
  Coordinate torque2{2.0, 0.0, 0.0};
  asset.applyTorque(torque2);

  // Check accumulation
  const Coordinate& accumulated2 = asset.getAccumulatedTorque();
  EXPECT_DOUBLE_EQ(accumulated2.x(), 2.0);
  EXPECT_DOUBLE_EQ(accumulated2.y(), 0.0);
  EXPECT_DOUBLE_EQ(accumulated2.z(), 5.0);
}

TEST(ForceApplicationScaffolding, clearForces_resetsAccumulators)
{
  ConvexHull hull = createTetrahedron();
  ReferenceFrame frame;
  AssetInertial asset{0, 0, hull, 10.0, frame};

  // Apply force and torque
  asset.applyForce(Coordinate{10.0, 5.0, 3.0});
  asset.applyTorque(Coordinate{1.0, 2.0, 3.0});

  // Verify they're set
  EXPECT_DOUBLE_EQ(asset.getAccumulatedForce().x(), 10.0);
  EXPECT_DOUBLE_EQ(asset.getAccumulatedTorque().z(), 3.0);

  // Clear forces
  asset.clearForces();

  // Verify they're zero
  const Coordinate& force = asset.getAccumulatedForce();
  const Coordinate& torque = asset.getAccumulatedTorque();
  EXPECT_DOUBLE_EQ(force.x(), 0.0);
  EXPECT_DOUBLE_EQ(force.y(), 0.0);
  EXPECT_DOUBLE_EQ(force.z(), 0.0);
  EXPECT_DOUBLE_EQ(torque.x(), 0.0);
  EXPECT_DOUBLE_EQ(torque.y(), 0.0);
  EXPECT_DOUBLE_EQ(torque.z(), 0.0);
}

TEST(ForceApplicationScaffolding, getAccumulatedForce_returnsAccumulatedValue)
{
  ConvexHull hull = createTetrahedron();
  ReferenceFrame frame;
  AssetInertial asset{0, 0, hull, 10.0, frame};

  // Initially zero
  const Coordinate& initial = asset.getAccumulatedForce();
  EXPECT_DOUBLE_EQ(initial.x(), 0.0);
  EXPECT_DOUBLE_EQ(initial.y(), 0.0);
  EXPECT_DOUBLE_EQ(initial.z(), 0.0);

  // After applying force
  asset.applyForce(Coordinate{7.0, 8.0, 9.0});
  const Coordinate& after = asset.getAccumulatedForce();
  EXPECT_DOUBLE_EQ(after.x(), 7.0);
  EXPECT_DOUBLE_EQ(after.y(), 8.0);
  EXPECT_DOUBLE_EQ(after.z(), 9.0);
}

TEST(ForceApplicationScaffolding, getAccumulatedTorque_returnsAccumulatedValue)
{
  ConvexHull hull = createTetrahedron();
  ReferenceFrame frame;
  AssetInertial asset{0, 0, hull, 10.0, frame};

  // Initially zero
  const Coordinate& initial = asset.getAccumulatedTorque();
  EXPECT_DOUBLE_EQ(initial.x(), 0.0);
  EXPECT_DOUBLE_EQ(initial.y(), 0.0);
  EXPECT_DOUBLE_EQ(initial.z(), 0.0);

  // After applying torque
  asset.applyTorque(Coordinate{3.0, 4.0, 5.0});
  const Coordinate& after = asset.getAccumulatedTorque();
  EXPECT_DOUBLE_EQ(after.x(), 3.0);
  EXPECT_DOUBLE_EQ(after.y(), 4.0);
  EXPECT_DOUBLE_EQ(after.z(), 5.0);
}

TEST(ForceApplicationScaffolding, applyForceAtPoint_accumulatesForce)
{
  ConvexHull hull = createTetrahedron();
  ReferenceFrame frame;
  AssetInertial asset{0, 0, hull, 10.0, frame};

  // Apply force at a point (torque calculation is TODO)
  Coordinate force{10.0, 0.0, 0.0};
  Coordinate worldPoint{0.0, 1.0, 0.0};
  asset.applyForceAtPoint(force, worldPoint);

  // Force should be accumulated
  const Coordinate& accumulatedForce = asset.getAccumulatedForce();
  EXPECT_DOUBLE_EQ(accumulatedForce.x(), 10.0);
  EXPECT_DOUBLE_EQ(accumulatedForce.y(), 0.0);
  EXPECT_DOUBLE_EQ(accumulatedForce.z(), 0.0);

  // Torque should still be zero (placeholder implementation)
  const Coordinate& accumulatedTorque = asset.getAccumulatedTorque();
  EXPECT_DOUBLE_EQ(accumulatedTorque.x(), 0.0);
  EXPECT_DOUBLE_EQ(accumulatedTorque.y(), 0.0);
  EXPECT_DOUBLE_EQ(accumulatedTorque.z(), 0.0);
}

// ============================================================================
// WorldModel Gravity Tests
// ============================================================================

TEST(ForceApplicationScaffolding, getGravity_returnsDefaultGravity)
{
  WorldModel world;

  const Coordinate& gravity = world.getGravity();
  EXPECT_DOUBLE_EQ(gravity.x(), 0.0);
  EXPECT_DOUBLE_EQ(gravity.y(), 0.0);
  EXPECT_DOUBLE_EQ(gravity.z(), -9.81);
}

TEST(ForceApplicationScaffolding, updatePhysics_callsClearForces)
{
  WorldModel world;

  // Create and spawn an asset
  ConvexHull hull = createTetrahedron();
  ReferenceFrame frame{Coordinate{0, 0, 0}};

  const AssetInertial& asset = world.spawnObject(0, hull, frame);
  uint32_t instanceId = asset.getInstanceId();

  // Apply forces to the asset
  AssetInertial& mutableAsset = world.getObject(instanceId);
  mutableAsset.applyForce(Coordinate{10.0, 5.0, 3.0});
  mutableAsset.applyTorque(Coordinate{1.0, 2.0, 3.0});

  // Verify forces are accumulated
  EXPECT_DOUBLE_EQ(mutableAsset.getAccumulatedForce().x(), 10.0);
  EXPECT_DOUBLE_EQ(mutableAsset.getAccumulatedTorque().z(), 3.0);

  // Update physics (should call clearForces)
  world.update(std::chrono::milliseconds{16});

  // Verify forces are cleared
  const Coordinate& force = mutableAsset.getAccumulatedForce();
  const Coordinate& torque = mutableAsset.getAccumulatedTorque();
  EXPECT_DOUBLE_EQ(force.x(), 0.0);
  EXPECT_DOUBLE_EQ(force.y(), 0.0);
  EXPECT_DOUBLE_EQ(force.z(), 0.0);
  EXPECT_DOUBLE_EQ(torque.x(), 0.0);
  EXPECT_DOUBLE_EQ(torque.y(), 0.0);
  EXPECT_DOUBLE_EQ(torque.z(), 0.0);
}

// ============================================================================
// InertialState Type Tests
// ============================================================================

TEST(ForceApplicationScaffolding, angularVelocity_isAngularRateType)
{
  InertialState state;

  // Set angular velocity as AngularRate
  state.angularVelocity = AngularRate{1.0, 2.0, 3.0};

  // Verify it's accessible as AngularRate with pitch/roll/yaw accessors
  EXPECT_DOUBLE_EQ(state.angularVelocity.pitch(), 1.0);
  EXPECT_DOUBLE_EQ(state.angularVelocity.roll(), 2.0);
  EXPECT_DOUBLE_EQ(state.angularVelocity.yaw(), 3.0);
}

TEST(ForceApplicationScaffolding, angularAcceleration_isAngularRateType)
{
  InertialState state;

  // Set angular acceleration as AngularRate
  state.angularAcceleration = AngularRate{0.5, 1.5, 2.5};

  // Verify it's accessible as AngularRate with pitch/roll/yaw accessors
  EXPECT_DOUBLE_EQ(state.angularAcceleration.pitch(), 0.5);
  EXPECT_DOUBLE_EQ(state.angularAcceleration.roll(), 1.5);
  EXPECT_DOUBLE_EQ(state.angularAcceleration.yaw(), 2.5);
}

TEST(ForceApplicationScaffolding, orientation_isAngularCoordinateType)
{
  InertialState state;

  // Set orientation as AngularCoordinate (radians)
  constexpr double pitchRad = 10.0 * M_PI / 180.0;
  constexpr double rollRad = 20.0 * M_PI / 180.0;
  constexpr double yawRad = 30.0 * M_PI / 180.0;

  state.orientation = AngularCoordinate{pitchRad, rollRad, yawRad};

  // Verify it's accessible as AngularCoordinate with degree accessors
  EXPECT_NEAR(state.orientation.pitchDeg(), 10.0, 1e-9);
  EXPECT_NEAR(state.orientation.rollDeg(), 20.0, 1e-9);
  EXPECT_NEAR(state.orientation.yawDeg(), 30.0, 1e-9);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(ForceApplicationScaffolding, gravityPersistsAcrossUpdates)
{
  WorldModel world;

  // Gravity should be constant
  const Coordinate& gravity1 = world.getGravity();
  EXPECT_DOUBLE_EQ(gravity1.z(), -9.81);

  // After updates, gravity should remain the same
  world.update(std::chrono::milliseconds{16});
  const Coordinate& gravity2 = world.getGravity();
  EXPECT_DOUBLE_EQ(gravity2.z(), -9.81);

  world.update(std::chrono::milliseconds{16});
  const Coordinate& gravity3 = world.getGravity();
  EXPECT_DOUBLE_EQ(gravity3.z(), -9.81);
}

TEST(ForceApplicationScaffolding, forceAccumulationAcrossMultipleFrames)
{
  WorldModel world;

  ConvexHull hull = createTetrahedron();
  ReferenceFrame frame{Coordinate{0, 0, 0}};

  const AssetInertial& asset = world.spawnObject(0, hull, frame);
  uint32_t instanceId = asset.getInstanceId();
  AssetInertial& mutableAsset = world.getObject(instanceId);

  // Frame 1: Apply forces and update
  mutableAsset.applyForce(Coordinate{5.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(mutableAsset.getAccumulatedForce().x(), 5.0);

  world.update(std::chrono::milliseconds{16});
  EXPECT_DOUBLE_EQ(mutableAsset.getAccumulatedForce().x(), 0.0);

  // Frame 2: Apply different forces
  mutableAsset.applyForce(Coordinate{0.0, 10.0, 0.0});
  mutableAsset.applyForce(Coordinate{0.0, 5.0, 0.0});
  EXPECT_DOUBLE_EQ(mutableAsset.getAccumulatedForce().y(), 15.0);

  world.update(std::chrono::milliseconds{16});
  EXPECT_DOUBLE_EQ(mutableAsset.getAccumulatedForce().y(), 0.0);
}

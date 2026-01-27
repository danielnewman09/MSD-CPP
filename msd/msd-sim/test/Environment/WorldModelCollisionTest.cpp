// Ticket: 0027_collision_response_system
// Design: docs/designs/0027_collision_response_system/design.md

#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <vector>
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

// Create a simple cube as a point cloud
std::vector<Coordinate> createCubePoints(double size)
{
  double half = size / 2.0;
  return {Coordinate(-half, -half, -half), Coordinate(half, -half, -half),
          Coordinate(half, half, -half),   Coordinate(-half, half, -half),
          Coordinate(-half, -half, half),  Coordinate(half, -half, half),
          Coordinate(half, half, half),    Coordinate(-half, half, half)};
}

}  // anonymous namespace

// ============================================================================
// WorldModel Collision Response Integration Tests
// ============================================================================

TEST(WorldModelCollisionTest, updateCollisions_NoObjects_NoError)
{
  // Empty world should handle updateCollisions gracefully
  WorldModel world;

  // Should not crash
  EXPECT_NO_THROW(world.update(std::chrono::milliseconds{16}));
}

TEST(WorldModelCollisionTest, updateCollisions_SingleObject_NoCollision)
{
  // Single object should not collide with itself
  WorldModel world;

  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  world.spawnObject(0, hull, frame);

  // Should not crash with single object
  EXPECT_NO_THROW(world.update(std::chrono::milliseconds{16}));
}

TEST(WorldModelCollisionTest, updateCollisions_TwoSeparatedObjects_NoInteraction)
{
  // Two separated objects should not interact
  WorldModel world;

  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{10.0, 0.0, 0.0}};  // Far apart

  world.spawnObject(0, hullA, frameA);
  world.spawnObject(1, hullB, frameB);

  // Get instance IDs (auto-assigned by WorldModel, starting from 1)
  uint32_t idA = 1;
  uint32_t idB = 2;

  // Set initial velocities
  world.getObject(idA).getInertialState().velocity = Coordinate{1.0, 0.0, 0.0};
  world.getObject(idB).getInertialState().velocity = Coordinate{-1.0, 0.0, 0.0};

  // Update simulation
  world.update(std::chrono::milliseconds{16});

  // Velocities should remain unchanged (no collision)
  EXPECT_DOUBLE_EQ(1.0, world.getObject(idA).getInertialState().velocity.x());
  EXPECT_DOUBLE_EQ(-1.0, world.getObject(idB).getInertialState().velocity.x());
}

TEST(WorldModelCollisionTest, updateCollisions_OverlappingObjects_ImpulseApplied)
{
  // Overlapping objects should have impulse applied
  WorldModel world;

  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  // Place objects overlapping along x-axis
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.8, 0.0, 0.0}};  // Overlapping

  world.spawnObject(0, hullA, frameA);
  world.spawnObject(1, hullB, frameB);

  // Get instance IDs (auto-assigned by WorldModel, starting from 1)
  uint32_t idA = 1;
  uint32_t idB = 2;

  // Set objects with high restitution for clear effect
  world.getObject(idA).setCoefficientOfRestitution(1.0);
  world.getObject(idB).setCoefficientOfRestitution(1.0);

  // Set initial velocities: approaching each other
  world.getObject(idA).getInertialState().velocity = Coordinate{1.0, 0.0, 0.0};
  world.getObject(idB).getInertialState().velocity = Coordinate{-1.0, 0.0, 0.0};

  // Store initial velocities
  double vAxInitial = world.getObject(idA).getInertialState().velocity.x();
  double vBxInitial = world.getObject(idB).getInertialState().velocity.x();

  // Update simulation (collision response happens)
  world.update(std::chrono::milliseconds{16});

  // Velocities should have changed due to collision
  double vAxFinal = world.getObject(idA).getInertialState().velocity.x();
  double vBxFinal = world.getObject(idB).getInertialState().velocity.x();

  // For elastic collision with equal mass, velocities should swap or at least change
  // We can't predict exact values without knowing penetration depth, but they should differ
  EXPECT_NE(vAxInitial, vAxFinal);
  EXPECT_NE(vBxInitial, vBxFinal);
}

TEST(WorldModelCollisionTest, updateCollisions_PositionCorrection_ObjectsSeparated)
{
  // Position correction should separate overlapping objects
  WorldModel world;

  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  // Place objects deeply overlapping
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.5, 0.0, 0.0}};  // Deep overlap

  world.spawnObject(0, hullA, frameA);
  world.spawnObject(1, hullB, frameB);

  // Get instance IDs (auto-assigned by WorldModel, starting from 1)
  uint32_t idA = 1;
  uint32_t idB = 2;

  // Objects at rest
  world.getObject(idA).getInertialState().velocity = Coordinate{0.0, 0.0, 0.0};
  world.getObject(idB).getInertialState().velocity = Coordinate{0.0, 0.0, 0.0};

  // Store initial distance
  double initialDistance = std::abs(
      world.getObject(idB).getInertialState().position.x() -
      world.getObject(idA).getInertialState().position.x());

  // Update simulation (position correction happens)
  world.update(std::chrono::milliseconds{16});

  // Final distance should be greater (objects pushed apart)
  double finalDistance = std::abs(
      world.getObject(idB).getInertialState().position.x() -
      world.getObject(idA).getInertialState().position.x());

  EXPECT_GT(finalDistance, initialDistance);
}

TEST(WorldModelCollisionTest, updateCollisions_InelasticCollision_VelocityReduced)
{
  // Inelastic collision (e=0) should absorb kinetic energy
  WorldModel world;

  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.9, 0.0, 0.0}};

  world.spawnObject(0, hullA, frameA);
  world.spawnObject(1, hullB, frameB);

  // Get instance IDs (auto-assigned by WorldModel, starting from 1)
  uint32_t idA = 1;
  uint32_t idB = 2;

  // Set fully inelastic restitution
  world.getObject(idA).setCoefficientOfRestitution(0.0);
  world.getObject(idB).setCoefficientOfRestitution(0.0);

  // Set initial velocities: approaching
  world.getObject(idA).getInertialState().velocity = Coordinate{2.0, 0.0, 0.0};
  world.getObject(idB).getInertialState().velocity = Coordinate{-2.0, 0.0, 0.0};

  // Store initial kinetic energy (approximation)
  double vAinitial = world.getObject(idA).getInertialState().velocity.norm();
  double vBinitial = world.getObject(idB).getInertialState().velocity.norm();
  double initialKE = 0.5 * 1.0 * (vAinitial * vAinitial + vBinitial * vBinitial);

  // Update simulation
  world.update(std::chrono::milliseconds{16});

  // Final velocities should be reduced (energy absorbed)
  double vAfinal = world.getObject(idA).getInertialState().velocity.norm();
  double vBfinal = world.getObject(idB).getInertialState().velocity.norm();
  double finalKE = 0.5 * 1.0 * (vAfinal * vAfinal + vBfinal * vBfinal);

  // For perfectly inelastic collision, final KE < initial KE
  EXPECT_LT(finalKE, initialKE);
}

TEST(WorldModelCollisionTest, updateCollisions_ElasticCollision_MomentumConserved)
{
  // Elastic collision should conserve total momentum
  // Note: Linear KE may decrease as energy goes into rotation
  WorldModel world;

  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.9, 0.0, 0.0}};

  world.spawnObject(0, hullA, frameA);
  world.spawnObject(1, hullB, frameB);

  // Get instance IDs (auto-assigned by WorldModel, starting from 1)
  uint32_t idA = 1;
  uint32_t idB = 2;

  // Set fully elastic restitution
  world.getObject(idA).setCoefficientOfRestitution(1.0);
  world.getObject(idB).setCoefficientOfRestitution(1.0);

  // Set initial velocities (approaching head-on)
  world.getObject(idA).getInertialState().velocity = Coordinate{2.0, 0.0, 0.0};
  world.getObject(idB).getInertialState().velocity = Coordinate{-2.0, 0.0, 0.0};

  // Store initial momentum (p = m * v)
  // Both masses are 1.0 (default in spawnObject)
  Coordinate initialMomentum =
      world.getObject(idA).getInertialState().velocity * 1.0 +
      world.getObject(idB).getInertialState().velocity * 1.0;
  // Initial momentum = (2, 0, 0) + (-2, 0, 0) = (0, 0, 0)

  // Update simulation
  world.update(std::chrono::milliseconds{16});

  // Final momentum (gravity adds z component, but x and y should be conserved)
  Coordinate finalMomentum =
      world.getObject(idA).getInertialState().velocity * 1.0 +
      world.getObject(idB).getInertialState().velocity * 1.0;

  // X and Y momentum should be conserved (collision impulse cancels)
  // Z momentum changes due to gravity: 2 * m * g * dt = 2 * 1 * 9.81 * 0.016 ≈ 0.31
  EXPECT_NEAR(initialMomentum.x(), finalMomentum.x(), 0.01);
  EXPECT_NEAR(initialMomentum.y(), finalMomentum.y(), 0.01);
}

// ============================================================================
// Inertial vs Environment (Dynamic-Static) Collision Tests
// ============================================================================

TEST(WorldModelStaticCollisionTest, spawnEnvironmentObject_CreatesStaticObject)
{
  WorldModel world;

  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  const AssetEnvironment& env = world.spawnEnvironmentObject(0, hull, frame);

  // Verify environment object was created
  EXPECT_EQ(1, world.getEnvironmentalObjects().size());
  EXPECT_EQ(0, env.getAssetId());
}

TEST(WorldModelStaticCollisionTest, inertialVsEnvironment_ImpulseApplied)
{
  WorldModel world;

  auto points = createCubePoints(1.0);
  ConvexHull hullDynamic{points};
  ConvexHull hullStatic{points};

  // Place dynamic object overlapping with static floor
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.8, 0.0, 0.0}};  // Overlapping

  world.spawnObject(0, hullDynamic, frameDynamic);
  world.spawnEnvironmentObject(1, hullStatic, frameStatic);

  uint32_t dynamicId = 1;

  // Set dynamic object moving toward static
  world.getObject(dynamicId).setCoefficientOfRestitution(1.0);
  world.getObject(dynamicId).getInertialState().velocity = Coordinate{2.0, 0.0, 0.0};

  double vxInitial = world.getObject(dynamicId).getInertialState().velocity.x();

  // Update simulation
  world.update(std::chrono::milliseconds{16});

  double vxFinal = world.getObject(dynamicId).getInertialState().velocity.x();

  // Velocity should have changed (bounced off static wall)
  EXPECT_NE(vxInitial, vxFinal);

  // For elastic collision with wall, velocity should reverse
  // (approximately, since there's also gravity affecting z)
  EXPECT_LT(vxFinal, 0.0);  // Should be moving in -x direction now
}

TEST(WorldModelStaticCollisionTest, inertialVsEnvironment_PositionCorrected)
{
  WorldModel world;

  auto points = createCubePoints(1.0);
  ConvexHull hullDynamic{points};
  ConvexHull hullStatic{points};

  // Place dynamic object deeply overlapping with static
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.5, 0.0, 0.0}};  // Deep overlap

  world.spawnObject(0, hullDynamic, frameDynamic);
  world.spawnEnvironmentObject(1, hullStatic, frameStatic);

  uint32_t dynamicId = 1;

  // Dynamic at rest
  world.getObject(dynamicId).getInertialState().velocity = Coordinate{0.0, 0.0, 0.0};

  double xInitial = world.getObject(dynamicId).getInertialState().position.x();

  // Update simulation
  world.update(std::chrono::milliseconds{16});

  double xFinal = world.getObject(dynamicId).getInertialState().position.x();

  // Dynamic object should have been pushed away from static (in -x direction)
  EXPECT_LT(xFinal, xInitial);
}

TEST(WorldModelStaticCollisionTest, inertialVsEnvironment_StaticUnchanged)
{
  WorldModel world;

  auto points = createCubePoints(1.0);
  ConvexHull hullDynamic{points};
  ConvexHull hullStatic{points};

  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.8, 0.0, 0.0}};

  world.spawnObject(0, hullDynamic, frameDynamic);
  world.spawnEnvironmentObject(1, hullStatic, frameStatic);

  uint32_t dynamicId = 1;

  // Dynamic object with high velocity toward static
  world.getObject(dynamicId).getInertialState().velocity = Coordinate{10.0, 0.0, 0.0};

  // Store static object's initial position
  Coordinate staticPosInitial = world.getEnvironmentalObjects()[0].getReferenceFrame().getOrigin();

  // Update simulation multiple times
  for (int i = 0; i < 10; ++i)
  {
    world.update(std::chrono::milliseconds{16});
  }

  // Static object position should be unchanged
  Coordinate staticPosFinal = world.getEnvironmentalObjects()[0].getReferenceFrame().getOrigin();

  EXPECT_DOUBLE_EQ(staticPosInitial.x(), staticPosFinal.x());
  EXPECT_DOUBLE_EQ(staticPosInitial.y(), staticPosFinal.y());
  EXPECT_DOUBLE_EQ(staticPosInitial.z(), staticPosFinal.z());
}

TEST(WorldModelStaticCollisionTest, inertialVsEnvironment_NoCollision_NoEffect)
{
  WorldModel world;

  auto points = createCubePoints(1.0);
  ConvexHull hullDynamic{points};
  ConvexHull hullStatic{points};

  // Place objects far apart (no collision)
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{10.0, 0.0, 0.0}};

  world.spawnObject(0, hullDynamic, frameDynamic);
  world.spawnEnvironmentObject(1, hullStatic, frameStatic);

  uint32_t dynamicId = 1;

  // Dynamic object moving away from static
  world.getObject(dynamicId).getInertialState().velocity = Coordinate{-1.0, 0.0, 0.0};

  double vxInitial = world.getObject(dynamicId).getInertialState().velocity.x();

  world.update(std::chrono::milliseconds{16});

  double vxFinal = world.getObject(dynamicId).getInertialState().velocity.x();

  // Velocity x-component should be unchanged (no collision)
  EXPECT_DOUBLE_EQ(vxInitial, vxFinal);
}

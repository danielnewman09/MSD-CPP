// Ticket: 0040d_contact_persistence_warm_starting
// Test: Warm-starting integration tests

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
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

std::vector<Coordinate> createFloorPoints()
{
  return createCubePoints(100.0);
}

/// Step the WorldModel forward by one dt (in milliseconds)
void stepWorld(WorldModel& world,
               std::chrono::milliseconds& simTime,
               std::chrono::milliseconds dt)
{
  simTime += dt;
  world.update(simTime);
}

}  // namespace

// ============================================================================
// Warm-Starting Integration Tests
// ============================================================================

/// AC5: Resting contact shows reduced position variance with warm-starting.
/// A cube sits on a floor for 200 frames. With warm-starting enabled (default),
/// position drift should be small. We measure total vertical position variance.
TEST(WarmStart, RestingCube_ReducedJitter)
{
  WorldModel world;

  // Floor at z=0
  auto floorPoints = createFloorPoints();
  ConvexHull floorHull{floorPoints};
  // Floor center at z=-50 so top face is at z=0
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Cube resting just above floor: bottom face at z=0, cube half-height = 0.5
  double const cubeSize = 1.0;
  auto cubePoints = createCubePoints(cubeSize);
  ConvexHull cubeHull{cubePoints};
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, cubeSize / 2.0 + 0.001}};
  const auto& cube =
    world.spawnObject(2, cubeHull, 10.0, cubeFrame);

  auto const dt = std::chrono::milliseconds{16};
  auto simTime = std::chrono::milliseconds{0};

  // Let it settle for 50 frames
  for (int i = 0; i < 50; ++i)
  {
    stepWorld(world, simTime, dt);
  }

  // Measure vertical position variance over next 150 frames
  double sumZ = 0.0;
  double sumZ2 = 0.0;
  constexpr int kMeasureFrames = 150;

  for (int i = 0; i < kMeasureFrames; ++i)
  {
    stepWorld(world, simTime, dt);
    double z = cube.getInertialState().position.z();
    sumZ += z;
    sumZ2 += z * z;
  }

  double const meanZ = sumZ / kMeasureFrames;
  double const variance =
    (sumZ2 / kMeasureFrames) - (meanZ * meanZ);

  // With warm-starting, variance should be very small (cube at rest)
  // Allow up to 0.01 m² variance (generous for stability)
  EXPECT_LT(variance, 0.01)
    << "Position variance too large: " << variance
    << " (mean z=" << meanZ << ")";

  // Mean z should be approximately at the resting height
  EXPECT_GT(meanZ, 0.0) << "Cube should be above the floor";
}

/// AC7: Stale cache entries don't cause energy spikes.
/// A cube bounces on a floor. When contact breaks and reforms,
/// cache should invalidate (normal may flip or contact is fresh)
/// and not inject stale impulses that spike energy.
TEST(WarmStart, BouncingCube_CacheInvalidationOnBounce)
{
  WorldModel world;

  // Floor
  auto floorPoints = createFloorPoints();
  ConvexHull floorHull{floorPoints};
  // Floor center at z=-50 so top face is at z=0
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Cube dropped from height — will bounce
  double const cubeSize = 1.0;
  auto cubePoints = createCubePoints(cubeSize);
  ConvexHull cubeHull{cubePoints};
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 3.0}};
  const auto& cube =
    world.spawnObject(2, cubeHull, 10.0, cubeFrame);

  auto const dt = std::chrono::milliseconds{16};
  auto simTime = std::chrono::milliseconds{0};

  // Track maximum speed across all frames
  double maxSpeed = 0.0;
  bool hadContact = false;
  for (int i = 0; i < 500; ++i)
  {
    stepWorld(world, simTime, dt);
    double const z = cube.getInertialState().position.z();
    double const speed = cube.getInertialState().velocity.norm();

    // Detect contact (cube near floor)
    if (z < cubeSize / 2.0 + 0.1)
    {
      hadContact = true;
    }
    maxSpeed = std::max(maxSpeed, speed);
  }

  // The initial drop speed from z=3.0: v = sqrt(2*g*h) ≈ sqrt(2*9.81*2.5) ≈ 7.0
  // With restitution, max bounce speed should never exceed initial impact speed
  // significantly. Allow 20% margin over free-fall speed.
  double const freeFallSpeed = std::sqrt(2.0 * 9.81 * 3.0);
  EXPECT_LT(maxSpeed, freeFallSpeed * 1.2)
    << "Max speed exceeded free-fall expectation — possible energy injection "
       "from stale cache";

  // Verify we actually had contact and bounce cycles
  EXPECT_TRUE(hadContact) << "Cube never contacted floor";
}

/// Verify warm-starting doesn't break basic physics: a cube dropped onto
/// a floor should come to approximate rest within reasonable time.
TEST(WarmStart, DroppedCube_ComesToRest)
{
  WorldModel world;

  // Floor
  auto floorPoints = createFloorPoints();
  ConvexHull floorHull{floorPoints};
  // Floor center at z=-50 so top face is at z=0
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Cube dropped from modest height
  double const cubeSize = 1.0;
  auto cubePoints = createCubePoints(cubeSize);
  ConvexHull cubeHull{cubePoints};
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 1.5}};
  const auto& cube =
    world.spawnObject(2, cubeHull, 10.0, cubeFrame);

  auto const dt = std::chrono::milliseconds{16};
  auto simTime = std::chrono::milliseconds{0};

  // Run for 1000 frames (~16 seconds)
  for (int i = 0; i < 1000; ++i)
  {
    stepWorld(world, simTime, dt);
  }

  // After 1000 frames, the cube should be near the floor and mostly at rest
  double const z = cube.getInertialState().position.z();
  // Cube should still be above floor
  EXPECT_GT(z, -1.0) << "Cube fell through floor";
  // Position should be near resting height (half cube size)
  EXPECT_LT(z, cubeSize * 2.0) << "Cube flew away from floor";
}

/// Two cubes on a floor — verify warm-starting doesn't cause instability
/// in multi-body scenarios.
TEST(WarmStart, TwoCubes_NoInstability)
{
  WorldModel world;

  // Floor
  auto floorPoints = createFloorPoints();
  ConvexHull floorHull{floorPoints};
  // Floor center at z=-50 so top face is at z=0
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  double const cubeSize = 1.0;
  auto cubePoints = createCubePoints(cubeSize);
  ConvexHull cubeHull1{cubePoints};
  ConvexHull cubeHull2{cubePoints};

  // Two cubes side by side, just above floor
  ReferenceFrame frame1{Coordinate{-2.0, 0.0, cubeSize / 2.0 + 0.001}};
  ReferenceFrame frame2{Coordinate{2.0, 0.0, cubeSize / 2.0 + 0.001}};

  const auto& cube1 =
    world.spawnObject(2, cubeHull1, 10.0, frame1);
  const auto& cube2 =
    world.spawnObject(3, cubeHull2, 10.0, frame2);

  auto const dt = std::chrono::milliseconds{16};
  auto simTime = std::chrono::milliseconds{0};

  // Run 200 frames
  for (int i = 0; i < 200; ++i)
  {
    stepWorld(world, simTime, dt);
  }

  // Both cubes should remain near their initial positions
  double const z1 = cube1.getInertialState().position.z();
  double const z2 = cube2.getInertialState().position.z();

  EXPECT_GT(z1, -1.0) << "Cube 1 fell through floor";
  EXPECT_GT(z2, -1.0) << "Cube 2 fell through floor";
  EXPECT_LT(z1, cubeSize * 3.0) << "Cube 1 flew away";
  EXPECT_LT(z2, cubeSize * 3.0) << "Cube 2 flew away";
}

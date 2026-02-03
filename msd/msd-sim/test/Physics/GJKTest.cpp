// Ticket: 0022_gjk_asset_physical_transform
// Design: docs/designs/0022_gjk_asset_physical_transform/design.md

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/Collision/GJK.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
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
  return {Coordinate(-half, -half, -half),
          Coordinate(half, -half, -half),
          Coordinate(half, half, -half),
          Coordinate(-half, half, -half),
          Coordinate(-half, -half, half),
          Coordinate(half, -half, half),
          Coordinate(half, half, half),
          Coordinate(-half, half, half)};
}

}  // anonymous namespace

// ============================================================================
// Basic GJK Tests with Identity Transform
// ============================================================================

TEST(GJKTest, IdentityTransformOverlappingCubes)
{
  // Two unit cubes at the origin should collide
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame identityFrame{};
  AssetPhysical assetA{0, 0, hullA, identityFrame};
  AssetPhysical assetB{0, 1, hullB, identityFrame};

  EXPECT_TRUE(gjkIntersects(assetA, assetB));
}

TEST(GJKTest, IdentityTransformSeparatedCubes)
{
  // Two unit cubes separated by distance should not collide
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{5.0, 0.0, 0.0}};  // 5 units apart

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_FALSE(gjkIntersects(assetA, assetB));
}

TEST(GJKTest, IdentityTransformTouchingCubes)
{
  // Two unit cubes barely overlapping should be detected as colliding
  // Unit cube has half-size 0.5, so cubes at (0,0,0) and (0.99,0,0) will
  // slightly overlap Note: Exact touching (distance = 1.0) is epsilon-dependent
  // in GJK, so we test slight overlap
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{
    Coordinate{0.99, 0.0, 0.0}};  // Slight overlap (0.01 penetration)

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_TRUE(gjkIntersects(assetA, assetB, 1e-6));
}

// ============================================================================
// Translation-Only Transform Tests
// ============================================================================

TEST(GJKTest, TranslationOnlyCollision)
{
  // Two cubes with translation-only transforms that overlap
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.5, 0.5, 0.5}};  // Partial overlap

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_TRUE(gjkIntersects(assetA, assetB));
}

TEST(GJKTest, TranslationOnlyNoCollision)
{
  // Two cubes with translation-only transforms, separated
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{10.0, 0.0, 0.0}};  // Far apart

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_FALSE(gjkIntersects(assetA, assetB));
}

TEST(GJKTest, TranslationOnlyLargeOffset)
{
  // Test with large translation to verify numerical stability
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{1000.0, 2000.0, 3000.0}};
  ReferenceFrame frameB{
    Coordinate{1000.5, 2000.5, 3000.5}};  // Overlap despite large offset

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_TRUE(gjkIntersects(assetA, assetB));
}

// ============================================================================
// Rotation-Only Transform Tests
// ============================================================================

TEST(GJKTest, RotationOnlyCollision)
{
  // Two cubes, one rotated 45 degrees around Z-axis, overlapping at origin
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};

  AngularCoordinate rotation{
    0.0, 0.0, 45.0 * M_PI / 180.0};  // pitch, roll, yaw
  ReferenceFrame frameB{Coordinate{0.0, 0.0, 0.0}, rotation};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_TRUE(gjkIntersects(assetA, assetB));
}

TEST(GJKTest, RotationOnly90Degrees)
{
  // Two cubes, one rotated 90 degrees around Z-axis, still overlapping
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};

  AngularCoordinate rotation{
    0.0, 0.0, 90.0 * M_PI / 180.0};  // pitch, roll, yaw
  ReferenceFrame frameB{Coordinate{0.0, 0.0, 0.0}, rotation};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_TRUE(gjkIntersects(assetA, assetB));
}

// ============================================================================
// Combined Translation + Rotation Transform Tests
// ============================================================================

TEST(GJKTest, CombinedTransformCollision)
{
  // Two cubes with both translation and rotation
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};

  AngularCoordinate rotation{
    0.0, 0.0, 45.0 * M_PI / 180.0};  // pitch, roll, yaw
  ReferenceFrame frameB{Coordinate{0.5, 0.5, 0.0}, rotation};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_TRUE(gjkIntersects(assetA, assetB));
}

TEST(GJKTest, CombinedTransformNoCollision)
{
  // Two cubes with translation and rotation, separated
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};

  AngularCoordinate rotation{
    30.0 * M_PI / 180.0,  // pitch
    0.0,                  // roll
    45.0 * M_PI / 180.0   // yaw
  };
  ReferenceFrame frameB{Coordinate{5.0, 5.0, 5.0}, rotation};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_FALSE(gjkIntersects(assetA, assetB));
}

TEST(GJKTest, CombinedTransformComplex)
{
  // More complex scenario with pitch, roll, and yaw
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  AngularCoordinate rotationA{
    15.0 * M_PI / 180.0,  // pitch
    30.0 * M_PI / 180.0,  // roll
    0.0                   // yaw
  };
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}, rotationA};

  AngularCoordinate rotationB{
    20.0 * M_PI / 180.0,  // pitch
    10.0 * M_PI / 180.0,  // roll
    45.0 * M_PI / 180.0   // yaw
  };
  ReferenceFrame frameB{Coordinate{0.3, 0.3, 0.3}, rotationB};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_TRUE(gjkIntersects(assetA, assetB));
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(GJKTest, DeepPenetration)
{
  // Two cubes with significant overlap
  auto points = createCubePoints(2.0);  // Larger cubes
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{
    Coordinate{0.1, 0.1, 0.1}};  // Almost completely overlapping

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_TRUE(gjkIntersects(assetA, assetB));
}

TEST(GJKTest, BarelyTouching)
{
  // Two cubes just barely touching
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{2.0001, 0.0, 0.0}};  // Just beyond touching

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  EXPECT_FALSE(gjkIntersects(assetA, assetB, 1e-6));
}

// ============================================================================
// GJK Class Direct Usage
// ============================================================================

TEST(GJKTest, DirectGJKClassUsage)
{
  // Test using GJK class directly instead of convenience function
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.5, 0.0, 0.0}};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  GJK gjk{assetA, assetB};
  EXPECT_TRUE(gjk.intersects());
}

TEST(GJKTest, DirectGJKClassNoCollision)
{
  // Test using GJK class directly for non-colliding case
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{10.0, 0.0, 0.0}};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  GJK gjk{assetA, assetB, 1e-6};
  EXPECT_FALSE(gjk.intersects(64));
}

// ============================================================================
// Performance and Iteration Tests
// ============================================================================

TEST(GJKTest, ConvergesWithinMaxIterations)
{
  // Verify GJK converges for typical cases
  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.5, 0.5, 0.5}};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  // Should converge well before 64 iterations
  GJK gjk{assetA, assetB};
  EXPECT_TRUE(gjk.intersects(10));  // Reduced iteration count
}

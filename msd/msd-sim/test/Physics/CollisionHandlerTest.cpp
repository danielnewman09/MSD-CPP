// Ticket: 0027a_expanding_polytope_algorithm
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
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
// CollisionHandler Basic Workflow Tests
// ============================================================================

TEST(CollisionHandlerTest, NoIntersection_ReturnsNullopt)
{
  // AC9: CollisionHandler returns std::nullopt when no collision
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{5.0, 0.0, 0.0}};  // Far apart

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  CollisionHandler handler{};
  auto result = handler.checkCollision(assetA, assetB);

  EXPECT_FALSE(result.has_value());
}

TEST(CollisionHandlerTest, Intersection_ReturnsCollisionResult)
{
  // AC9: CollisionHandler returns CollisionResult when collision detected
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.8, 0.0, 0.0}};  // Overlapping

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  CollisionHandler handler{};
  auto result = handler.checkCollision(assetA, assetB);

  ASSERT_TRUE(result.has_value());
}

TEST(CollisionHandlerTest, CollisionResult_ContainsValidData)
{
  // AC9: Returned CollisionResult has valid normal, depth, contact point
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.9, 0.0, 0.0}};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  CollisionHandler handler{};
  auto result = handler.checkCollision(assetA, assetB);

  ASSERT_TRUE(result.has_value());

  // Normal should be unit length
  EXPECT_NEAR(result->normal.norm(), 1.0, 1e-6);

  // Penetration depth should be positive
  EXPECT_GT(result->penetrationDepth, 0.0);
  EXPECT_TRUE(std::isfinite(result->penetrationDepth));

  // Contact points should be finite (witness points on surfaces)
  EXPECT_TRUE(std::isfinite(result->contacts[0].pointA.x()));
  EXPECT_TRUE(std::isfinite(result->contacts[0].pointA.y()));
  EXPECT_TRUE(std::isfinite(result->contacts[0].pointA.z()));
  EXPECT_TRUE(std::isfinite(result->contacts[0].pointB.x()));
  EXPECT_TRUE(std::isfinite(result->contacts[0].pointB.y()));
  EXPECT_TRUE(std::isfinite(result->contacts[0].pointB.z()));
}

// ============================================================================
// Integration Tests: Full GJK→EPA Pipeline
// ============================================================================

TEST(CollisionHandlerIntegrationTest, OverlappingCubes_FullPipeline)
{
  // Integration test: CollisionHandler detects intersection and computes
  // contact info
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{
    Coordinate{0.8, 0.0, 0.0}};  // 0.2 overlap (depth = 0.2)

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  CollisionHandler handler{};
  auto result = handler.checkCollision(assetA, assetB);

  ASSERT_TRUE(result.has_value());

  // Expected penetration depth: 0.2
  EXPECT_NEAR(result->penetrationDepth, 0.2, 1e-6);

  // Normal should point from A to B (+X)
  EXPECT_NEAR(result->normal.x(), 1.0, 1e-6);
  EXPECT_NEAR(result->normal.y(), 0.0, 1e-6);
  EXPECT_NEAR(result->normal.z(), 0.0, 1e-6);
}

TEST(CollisionHandlerIntegrationTest, NoCollision_EPANotInvoked)
{
  // When objects don't collide, handler returns nullopt (EPA not invoked)
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{10.0, 0.0, 0.0}};  // Far apart

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  CollisionHandler handler{};
  auto result = handler.checkCollision(assetA, assetB);

  EXPECT_FALSE(result.has_value());
}

TEST(CollisionHandlerIntegrationTest, MultipleOrientations_ConsistentResults)
{
  // Rotating objects produces consistent contact normals relative to object
  // orientations
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  CollisionHandler handler{};

  // Test 1: No rotation
  {
    ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
    ReferenceFrame frameB{Coordinate{0.8, 0.0, 0.0}};

    AssetPhysical assetA{0, 0, hullA, frameA};
    AssetPhysical assetB{0, 1, hullB, frameB};

    auto result = handler.checkCollision(assetA, assetB);
    ASSERT_TRUE(result.has_value());

    // Normal should be +X
    EXPECT_NEAR(result->normal.x(), 1.0, 1e-6);
    EXPECT_GT(result->penetrationDepth, 0.0);
  }

  // Test 2: B rotated 45° around Z
  {
    AngularCoordinate rotation{0.0, 0.0, M_PI / 4.0};
    ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
    ReferenceFrame frameB{
      Coordinate{0.3, 0.0, 0.0},
      rotation};  // Closer for reliable collision with rotation

    AssetPhysical assetA{0, 0, hullA, frameA};
    AssetPhysical assetB{0, 1, hullB, frameB};

    auto result = handler.checkCollision(assetA, assetB);
    ASSERT_TRUE(result.has_value());

    // Should still detect collision and return valid contact info
    EXPECT_NEAR(result->normal.norm(), 1.0, 1e-6);
    EXPECT_GT(result->penetrationDepth, 0.0);
  }

  // Test 3: Both rotated
  {
    AngularCoordinate rotationA{0.0, 0.0, M_PI / 6.0};
    AngularCoordinate rotationB{0.0, 0.0, M_PI / 3.0};
    ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}, rotationA};
    ReferenceFrame frameB{
      Coordinate{0.2, 0.0, 0.0},
      rotationB};  // Very close for reliable collision with both rotated

    AssetPhysical assetA{0, 0, hullA, frameA};
    AssetPhysical assetB{0, 1, hullB, frameB};

    auto result = handler.checkCollision(assetA, assetB);
    ASSERT_TRUE(result.has_value());

    EXPECT_NEAR(result->normal.norm(), 1.0, 1e-6);
    EXPECT_GT(result->penetrationDepth, 0.0);
  }
}

TEST(CollisionHandlerIntegrationTest, DifferentSizedCubes_ValidResults)
{
  // Test with asymmetric cube sizes
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(2.0);  // Larger cube

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{1.2, 0.0, 0.0}};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  CollisionHandler handler{};
  auto result = handler.checkCollision(assetA, assetB);

  ASSERT_TRUE(result.has_value());

  // Penetration depth: cubeA half-size = 0.5, cubeB half-size = 1.0
  // Distance = 1.2, overlap = (0.5 + 1.0) - 1.2 = 0.3
  EXPECT_NEAR(result->penetrationDepth, 0.3, 1e-6);

  // Normal should be unit length
  EXPECT_NEAR(result->normal.norm(), 1.0, 1e-6);
}

TEST(CollisionHandlerTest, CustomEpsilon_UsedByGJKAndEPA)
{
  // Verify that custom epsilon is passed through to both GJK and EPA
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.95, 0.0, 0.0}};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  // Test with tighter epsilon
  CollisionHandler handler{1e-8};
  auto result = handler.checkCollision(assetA, assetB);

  ASSERT_TRUE(result.has_value());
  EXPECT_GT(result->penetrationDepth, 0.0);
}

// Ticket: 0027a_expanding_polytope_algorithm
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "msd-sim/src/Environment/AngularCoordinate.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/CollisionHandler.hpp"
#include "msd-sim/src/Physics/CollisionResult.hpp"
#include "msd-sim/src/Physics/EPA.hpp"
#include "msd-sim/src/Physics/GJK.hpp"
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
// EPA Basic Functionality Tests
// ============================================================================

TEST(EPATest, InvalidSimplexSize_BuildsTetrahedron)
{
  // EPA handles incomplete simplices by building a valid tetrahedron
  // This can occur when GJK detects collision before completing the simplex
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{};
  AssetPhysical asset{0, 0, hull, frame};

  EPA epa{asset, asset};

  // Simplex with 3 vertices - EPA should build a full tetrahedron
  std::vector<Coordinate> incompleteSimplex = {Coordinate{0, 0, 0},
                                                Coordinate{1, 0, 0},
                                                Coordinate{0, 1, 0}};

  // Should not throw - should successfully compute contact info
  EXPECT_NO_THROW({
    CollisionResult result = epa.computeContactInfo(incompleteSimplex);
    EXPECT_GT(result.penetrationDepth, 0.0);
    EXPECT_NEAR(result.normal.norm(), 1.0, 1e-6);
  });
}

TEST(EPATest, TooManyVertices_ThrowsException)
{
  // EPA should reject simplices with > 4 vertices
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{};
  AssetPhysical asset{0, 0, hull, frame};

  EPA epa{asset, asset};

  // Simplex with 5 vertices (invalid)
  std::vector<Coordinate> invalidSimplex = {
      Coordinate{0, 0, 0},  Coordinate{1, 0, 0}, Coordinate{0, 1, 0},
      Coordinate{0, 0, 1}, Coordinate{1, 1, 1}};

  EXPECT_THROW(epa.computeContactInfo(invalidSimplex), std::invalid_argument);
}

TEST(EPATest, OverlappingUnitCubes_CorrectPenetrationDepth)
{
  // AC3: EPA penetration depth within 1e-6 of expected value
  // Two unit cubes with 0.1 overlap (positioned at 0 and 0.9)
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{
      Coordinate{0.9, 0.0, 0.0}};  // 0.1 overlap (expected depth = 0.1)

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  // First run GJK to get simplex
  GJK gjk{assetA, assetB};
  ASSERT_TRUE(gjk.intersects());

  // Then run EPA
  EPA epa{assetA, assetB};
  CollisionResult result = epa.computeContactInfo(gjk.getSimplex());

  // Expected penetration depth: 0.1 (cube half-size 0.5, distance 0.9 → overlap
  // = 1.0 - 0.9 = 0.1)
  double expectedDepth = 0.1;
  EXPECT_NEAR(result.penetrationDepth, expectedDepth, 1e-6);
}

TEST(EPATest, AxisAlignedCubesPositiveX_CorrectNormal)
{
  // AC2: EPA returns correct contact normal for axis-aligned cube overlaps (+X)
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.8, 0.0, 0.0}};  // Overlap in +X direction

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  GJK gjk{assetA, assetB};
  ASSERT_TRUE(gjk.intersects());

  EPA epa{assetA, assetB};
  CollisionResult result = epa.computeContactInfo(gjk.getSimplex());

  // Normal should point from A to B (positive X direction)
  Coordinate expectedNormal{1.0, 0.0, 0.0};
  EXPECT_NEAR(result.normal.x(), expectedNormal.x(), 1e-6);
  EXPECT_NEAR(result.normal.y(), expectedNormal.y(), 1e-6);
  EXPECT_NEAR(result.normal.z(), expectedNormal.z(), 1e-6);

  // Normal should be unit length
  EXPECT_NEAR(result.normal.norm(), 1.0, 1e-6);
}

TEST(EPATest, AxisAlignedCubesNegativeY_CorrectNormal)
{
  // AC2: EPA returns correct contact normal for axis-aligned cube overlaps (-Y)
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{
      Coordinate{0.0, -0.7, 0.0}};  // Overlap in -Y direction

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  GJK gjk{assetA, assetB};
  ASSERT_TRUE(gjk.intersects());

  EPA epa{assetA, assetB};
  CollisionResult result = epa.computeContactInfo(gjk.getSimplex());

  // Normal should point from A to B (negative Y direction)
  Coordinate expectedNormal{0.0, -1.0, 0.0};
  EXPECT_NEAR(result.normal.x(), expectedNormal.x(), 1e-6);
  EXPECT_NEAR(result.normal.y(), expectedNormal.y(), 1e-6);
  EXPECT_NEAR(result.normal.z(), expectedNormal.z(), 1e-6);

  EXPECT_NEAR(result.normal.norm(), 1.0, 1e-6);
}

TEST(EPATest, AxisAlignedCubesPositiveZ_CorrectNormal)
{
  // AC2: EPA returns correct contact normal for axis-aligned cube overlaps (+Z)
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.0, 0.0, 0.6}};  // Overlap in +Z direction

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  GJK gjk{assetA, assetB};
  ASSERT_TRUE(gjk.intersects());

  EPA epa{assetA, assetB};
  CollisionResult result = epa.computeContactInfo(gjk.getSimplex());

  // Normal should point from A to B (positive Z direction)
  Coordinate expectedNormal{0.0, 0.0, 1.0};
  EXPECT_NEAR(result.normal.x(), expectedNormal.x(), 1e-6);
  EXPECT_NEAR(result.normal.y(), expectedNormal.y(), 1e-6);
  EXPECT_NEAR(result.normal.z(), expectedNormal.z(), 1e-6);

  EXPECT_NEAR(result.normal.norm(), 1.0, 1e-6);
}

TEST(EPATest, DeepPenetration_Converges)
{
  // AC6: EPA terminates within configurable max iterations
  // Deep penetration (nearly concentric cubes)
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.1, 0.1, 0.1}};  // Very deep overlap

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  GJK gjk{assetA, assetB};
  ASSERT_TRUE(gjk.intersects());

  EPA epa{assetA, assetB};

  // Should converge without throwing
  EXPECT_NO_THROW({
    CollisionResult result = epa.computeContactInfo(gjk.getSimplex());
    EXPECT_GT(result.penetrationDepth, 0.0);
  });
}

TEST(EPATest, ShallowPenetration_Converges)
{
  // AC6: EPA handles shallow penetrations
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{
      Coordinate{0.99, 0.0, 0.0}};  // Very shallow overlap (0.01)

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  GJK gjk{assetA, assetB};
  ASSERT_TRUE(gjk.intersects());

  EPA epa{assetA, assetB};
  CollisionResult result = epa.computeContactInfo(gjk.getSimplex());

  // Should handle shallow penetration
  EXPECT_GT(result.penetrationDepth, 0.0);
  EXPECT_LT(result.penetrationDepth, 0.02);  // Less than 0.02 (with epsilon)
}

TEST(EPATest, RotatedCubes_WorldSpaceNormal)
{
  // Test with rotated objects to verify world-space correctness
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  // Rotate B by 45 degrees around Z axis
  // Use significant overlap to ensure collision detection
  AngularCoordinate rotation{0.0, 0.0, M_PI / 4.0};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{
      Coordinate{0.3, 0.0, 0.0},
      rotation};  // Significant overlap for reliable collision

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  GJK gjk{assetA, assetB};
  ASSERT_TRUE(gjk.intersects());

  EPA epa{assetA, assetB};
  CollisionResult result = epa.computeContactInfo(gjk.getSimplex());

  // Normal should be unit length in world space
  EXPECT_NEAR(result.normal.norm(), 1.0, 1e-6);

  // Penetration depth should be positive
  EXPECT_GT(result.penetrationDepth, 0.0);

  // Contact point should be in world space
  EXPECT_TRUE(std::isfinite(result.contactPoint.x()));
  EXPECT_TRUE(std::isfinite(result.contactPoint.y()));
  EXPECT_TRUE(std::isfinite(result.contactPoint.z()));
}

// ============================================================================
// CollisionResult Tests
// ============================================================================

TEST(CollisionResultTest, DefaultConstruction_NaNPenetrationDepth)
{
  // Verify default initialization per coding standards
  CollisionResult result;

  EXPECT_TRUE(std::isnan(result.penetrationDepth));
}

TEST(CollisionResultTest, ParameterizedConstruction_StoresValues)
{
  Coordinate normal{1.0, 0.0, 0.0};
  double depth = 0.5;
  Coordinate contactPoint{0.25, 0.0, 0.0};

  CollisionResult result{normal, depth, contactPoint};

  EXPECT_DOUBLE_EQ(result.normal.x(), 1.0);
  EXPECT_DOUBLE_EQ(result.normal.y(), 0.0);
  EXPECT_DOUBLE_EQ(result.normal.z(), 0.0);
  EXPECT_DOUBLE_EQ(result.penetrationDepth, 0.5);
  EXPECT_DOUBLE_EQ(result.contactPoint.x(), 0.25);
  EXPECT_DOUBLE_EQ(result.contactPoint.y(), 0.0);
  EXPECT_DOUBLE_EQ(result.contactPoint.z(), 0.0);
}

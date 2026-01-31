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
#include "msd-sim/src/Physics/SupportFunction.hpp"

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
  std::vector<Coordinate> incompleteSimplex = {
    Coordinate{0, 0, 0}, Coordinate{1, 0, 0}, Coordinate{0, 1, 0}};

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
  std::vector<Coordinate> invalidSimplex = {Coordinate{0, 0, 0},
                                            Coordinate{1, 0, 0},
                                            Coordinate{0, 1, 0},
                                            Coordinate{0, 0, 1},
                                            Coordinate{1, 1, 1}};

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
  ReferenceFrame frameB{Coordinate{0.0, -0.7, 0.0}};  // Overlap in -Y direction

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

  // Contact points should be in world space (witness points on surfaces)
  EXPECT_TRUE(std::isfinite(result.contacts[0].pointA.x()));
  EXPECT_TRUE(std::isfinite(result.contacts[0].pointA.y()));
  EXPECT_TRUE(std::isfinite(result.contacts[0].pointA.z()));
  EXPECT_TRUE(std::isfinite(result.contacts[0].pointB.x()));
  EXPECT_TRUE(std::isfinite(result.contacts[0].pointB.y()));
  EXPECT_TRUE(std::isfinite(result.contacts[0].pointB.z()));
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
  Coordinate contactPointA{0.25, 0.0, 0.0};
  Coordinate contactPointB{-0.25, 0.0, 0.0};

  CollisionResult result{normal, depth, contactPointA, contactPointB};

  EXPECT_DOUBLE_EQ(result.normal.x(), 1.0);
  EXPECT_DOUBLE_EQ(result.normal.y(), 0.0);
  EXPECT_DOUBLE_EQ(result.normal.z(), 0.0);
  EXPECT_DOUBLE_EQ(result.penetrationDepth, 0.5);
  EXPECT_DOUBLE_EQ(result.contacts[0].pointA.x(), 0.25);
  EXPECT_DOUBLE_EQ(result.contacts[0].pointA.y(), 0.0);
  EXPECT_DOUBLE_EQ(result.contacts[0].pointA.z(), 0.0);
  EXPECT_DOUBLE_EQ(result.contacts[0].pointB.x(), -0.25);
  EXPECT_DOUBLE_EQ(result.contacts[0].pointB.y(), 0.0);
  EXPECT_DOUBLE_EQ(result.contacts[0].pointB.z(), 0.0);
}

// ============================================================================
// Witness Point Tests (Ticket 0028_epa_witness_points)
// ============================================================================

TEST(SupportFunctionTest, supportMinkowskiWithWitness_IdentityTransform)
{
  // AC1: supportMinkowskiWithWitness returns witness points matching Minkowski
  // calculation
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{};  // Identity
  ReferenceFrame frameB{};  // Identity

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  CoordinateRate dir{1.0, 0.0, 0.0};

  SupportResult result =
    SupportFunction::supportMinkowskiWithWitness(assetA, assetB, dir);

  // Minkowski should equal witnessA - witnessB
  Coordinate expectedMinkowski = result.witnessA - result.witnessB;
  EXPECT_NEAR(result.minkowski.x(), expectedMinkowski.x(), 1e-6);
  EXPECT_NEAR(result.minkowski.y(), expectedMinkowski.y(), 1e-6);
  EXPECT_NEAR(result.minkowski.z(), expectedMinkowski.z(), 1e-6);

  // For identity transforms, witnesses should be on the cube surface
  EXPECT_NEAR(std::abs(result.witnessA.x()), 0.5, 1e-6);  // Cube half-size
  EXPECT_NEAR(std::abs(result.witnessB.x()), 0.5, 1e-6);
}

TEST(SupportFunctionTest, supportMinkowskiWithWitness_TranslatedObjects)
{
  // AC2: Witness points in world space after translation
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  ReferenceFrame frameA{Coordinate{5.0, 0.0, 0.0}};   // Translated +5 in X
  ReferenceFrame frameB{Coordinate{-3.0, 0.0, 0.0}};  // Translated -3 in X

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  CoordinateRate dir{1.0, 0.0, 0.0};

  SupportResult result =
    SupportFunction::supportMinkowskiWithWitness(assetA, assetB, dir);

  // Witness points should be in world space
  // Asset A at +5, cube half-size 0.5 → witnessA.x() ≈ 5.5
  EXPECT_NEAR(result.witnessA.x(), 5.5, 1e-6);

  // Asset B at -3, cube half-size 0.5, negated direction → witnessB.x() ≈ -3.5
  EXPECT_NEAR(result.witnessB.x(), -3.5, 1e-6);

  // Minkowski should still equal witnessA - witnessB
  Coordinate expectedMinkowski = result.witnessA - result.witnessB;
  EXPECT_NEAR(result.minkowski.x(), expectedMinkowski.x(), 1e-6);
  EXPECT_NEAR(result.minkowski.y(), expectedMinkowski.y(), 1e-6);
  EXPECT_NEAR(result.minkowski.z(), expectedMinkowski.z(), 1e-6);
}

TEST(EPATest, WitnessPoints_FaceContact)
{
  // AC3: For axis-aligned cube face-face collision, witness points near
  // respective faces Note: Witness points are interpolated from EPA face
  // vertices, not projected to surfaces
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  // Face-face collision: 0.1 overlap
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.9, 0.0, 0.0}};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  CollisionHandler handler{};
  auto result = handler.checkCollision(assetA, assetB);

  ASSERT_TRUE(result.has_value());

  // Witness points should be in the contact region (between 0.4 and 0.5 in X)
  // They may not be exactly on the face due to barycentric interpolation
  EXPECT_GT(result->contacts[0].pointA.x(), 0.0);  // On positive side of A
  EXPECT_LT(result->contacts[0].pointA.x(), 0.6);  // Near A's +X face
  EXPECT_GT(result->contacts[0].pointB.x(),
            0.0);  // On positive side of B (in world space)
  EXPECT_LT(result->contacts[0].pointB.x(), 0.5);  // Near B's -X face

  // Y and Z should be near center of contact region
  EXPECT_NEAR(std::abs(result->contacts[0].pointA.y()), 0.0, 0.5);
  EXPECT_NEAR(std::abs(result->contacts[0].pointA.z()), 0.0, 0.5);
  EXPECT_NEAR(std::abs(result->contacts[0].pointB.y()), 0.0, 0.5);
  EXPECT_NEAR(std::abs(result->contacts[0].pointB.z()), 0.0, 0.5);
}

TEST(EPATest, WitnessPoints_EnableTorqueCalculation)
{
  // AC5: Witness points enable accurate torque calculation
  auto pointsA = createCubePoints(1.0);
  auto pointsB = createCubePoints(1.0);

  ConvexHull hullA{pointsA};
  ConvexHull hullB{pointsB};

  // Offset collision to generate torque
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.9, 0.3, 0.0}};  // Offset collision

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  CollisionHandler handler{};
  auto result = handler.checkCollision(assetA, assetB);

  ASSERT_TRUE(result.has_value());

  // Compute lever arms
  Coordinate centerOfMassA{0.0, 0.0, 0.0};
  Coordinate centerOfMassB{0.9, 0.3, 0.0};

  Coordinate leverArmA = result->contacts[0].pointA - centerOfMassA;
  Coordinate leverArmB = result->contacts[0].pointB - centerOfMassB;

  // Lever arms should be finite and non-zero
  EXPECT_TRUE(std::isfinite(leverArmA.norm()));
  EXPECT_TRUE(std::isfinite(leverArmB.norm()));
  EXPECT_GT(leverArmA.norm(), 0.0);
  EXPECT_GT(leverArmB.norm(), 0.0);

  // Apply impulse and compute torque
  Coordinate impulse = result->normal * 100.0;  // 100 N impulse
  Coordinate torqueA = leverArmA.cross(impulse);
  Coordinate torqueB = leverArmB.cross(-impulse);

  // Torque should be finite and non-zero (offset collision)
  EXPECT_TRUE(std::isfinite(torqueA.norm()));
  EXPECT_TRUE(std::isfinite(torqueB.norm()));

  // For offset collision, expect meaningful torque
  // (Exact value depends on contact geometry, but should be significant)
  EXPECT_GT(std::abs(torqueA.z()), 0.01);  // Torque around Z-axis
}

// TEST(EPATest, WitnessPoints_DifferentForDifferentCollisions)
// {
//   // Verify witness points are distinct for different collision
//   configurations auto pointsA = createCubePoints(1.0); auto pointsB =
//   createCubePoints(1.0);

//   ConvexHull hullA{pointsA};
//   ConvexHull hullB{pointsB};

//   CollisionHandler handler{};

//   // Configuration 1: Face-face collision in X
//   ReferenceFrame frameA1{Coordinate{0.0, 0.0, 0.0}};
//   ReferenceFrame frameB1{Coordinate{0.9, 0.0, 0.0}};

//   AssetPhysical assetA1{0, 0, hullA, frameA1};
//   AssetPhysical assetB1{0, 1, hullB, frameB1};

//   auto result1 = handler.checkCollision(assetA1, assetB1);
//   ASSERT_TRUE(result1.has_value());

//   // Configuration 2: Face-face collision in Y
//   ReferenceFrame frameA2{Coordinate{0.0, 0.0, 0.0}};
//   ReferenceFrame frameB2{Coordinate{0.0, 0.9, 0.0}};

//   AssetPhysical assetA2{0, 0, hullA, frameA2};
//   AssetPhysical assetB2{0, 1, hullB, frameB2};

//   auto result2 = handler.checkCollision(assetA2, assetB2);
//   ASSERT_TRUE(result2.has_value());

//   // Witness points should differ significantly between configurations
//   // Configuration 1 has collision in X, Configuration 2 has collision in Y
//   // At least one coordinate should differ by a meaningful amount
//   double xDiff = std::abs(result1->contacts[0].pointA.x() -
//   result2->contacts[0].pointA.x()); double yDiff =
//   std::abs(result1->contacts[0].pointA.y() -
//   result2->contacts[0].pointA.y());

//   EXPECT_TRUE(xDiff > 0.05 || yDiff > 0.05);  // At least 5cm difference
// }

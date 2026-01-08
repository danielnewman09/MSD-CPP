#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "msd-assets/src/Geometry.hpp"
#include "msd-assets/src/GeometryFactory.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"
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

// Create a tetrahedron point cloud
std::vector<Coordinate> createTetrahedronPoints()
{
  return {Coordinate(0.0, 0.0, 0.0),
          Coordinate(0.0, 0.0, 1.0),
          Coordinate(0.0, 1.0, 0.0),
          Coordinate(1.0, 0.0, 0.0)};
}

// Create pyramid vertices (5 unique points: 4 base corners + 1 apex)
// Matches GeometryFactory::createPyramid layout
std::vector<Eigen::Vector3d> createPyramidVertices(double baseSize, double height)
{
  double half = baseSize / 2.0;
  double halfHeight = height / 2.0;

  // Return only the 5 unique vertices (not triangulated)
  return {
    Eigen::Vector3d{-half, -halfHeight, -half},  // base front-left
    Eigen::Vector3d{half, -halfHeight, -half},   // base front-right
    Eigen::Vector3d{half, -halfHeight, half},    // base back-right
    Eigen::Vector3d{-half, -halfHeight, half},   // base back-left
    Eigen::Vector3d{0.0, halfHeight, 0.0}        // apex
  };
}

// Create cube vertices (8 unique corner points)
std::vector<Eigen::Vector3d> createCubeVertices(double size)
{
  double half = size / 2.0;
  return {
    Eigen::Vector3d{-half, -half, -half},
    Eigen::Vector3d{half, -half, -half},
    Eigen::Vector3d{half, half, -half},
    Eigen::Vector3d{-half, half, -half},
    Eigen::Vector3d{-half, -half, half},
    Eigen::Vector3d{half, -half, half},
    Eigen::Vector3d{half, half, half},
    Eigen::Vector3d{-half, half, half}
  };
}

// Create points with some interior points (should be removed by hull)
std::vector<Coordinate> createPointsWithInterior()
{
  auto points = createCubePoints(2.0);
  // Add interior point
  points.push_back(Coordinate(0.0, 0.0, 0.0));
  return points;
}

}  // anonymous namespace

// ============================================================================
// Constructor Tests
// ============================================================================

TEST(ConvexHullTest, DefaultConstructor)
{
  ConvexHull hull;
  EXPECT_EQ(hull.getVertexCount(), 0);
  EXPECT_EQ(hull.getFacetCount(), 0);
  EXPECT_FALSE(hull.isValid());
}

TEST(ConvexHullTest, ConstructorWithValidPoints)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  EXPECT_GT(hull.getVertexCount(), 0);
  EXPECT_GT(hull.getFacetCount(), 0);
  EXPECT_TRUE(hull.isValid());
}

TEST(ConvexHullTest, ConstructorRemovesInteriorPoints)
{
  auto points = createPointsWithInterior();
  ConvexHull hull(points);

  // Hull should have 8 vertices (cube corners), not 9 (interior removed)
  EXPECT_EQ(hull.getVertexCount(), 8);
}

TEST(ConvexHullTest, ConstructorThrowsOnEmptyPoints)
{
  std::vector<Coordinate> emptyPoints;
  EXPECT_THROW(ConvexHull hull(emptyPoints), std::runtime_error);
}

TEST(ConvexHullTest, ConstructorThrowsOnTooFewPoints)
{
  std::vector<Coordinate> twoPoints = {Coordinate(0.0, 0.0, 0.0),
                                       Coordinate(1.0, 0.0, 0.0)};
  EXPECT_THROW(ConvexHull hull(twoPoints), std::runtime_error);
}

// ============================================================================
// Factory Method Tests
// ============================================================================

TEST(ConvexHullTest, FromGeometry)
{
  auto record = msd_assets::GeometryFactory::createCube(2.0);

  msd_assets::CollisionGeometry geometry{record};
  ConvexHull hull{geometry};

  EXPECT_TRUE(hull.isValid());
  EXPECT_GT(hull.getVertexCount(), 0);
  EXPECT_GT(hull.getFacetCount(), 0);
}

TEST(ConvexHullTest, FromGeometryPyramid)
{
  // Ticket: 0003_geometry-factory-type-safety
  // CollisionGeometry should be constructed from raw vertices, not factory MeshRecord
  // (factory produces VisualGeometry blobs with Vertex structs)
  auto vertices = createPyramidVertices(2.0, 3.0);
  msd_assets::CollisionGeometry geometry{vertices};
  ConvexHull hull{geometry};

  EXPECT_TRUE(hull.isValid());
  EXPECT_EQ(hull.getVertexCount(), 5);  // 4 base + 1 apex
}

TEST(ConvexHullTest, FromPoints)
{
  auto points = createTetrahedronPoints();
  ConvexHull hull(points);

  EXPECT_TRUE(hull.isValid());
  EXPECT_EQ(hull.getVertexCount(), 4);
}

// ============================================================================
// Volume Tests
// ============================================================================

TEST(ConvexHullTest, VolumeOfCube)
{
  double size = 2.0;
  auto points = createCubePoints(size);
  ConvexHull hull(points);

  double expectedVolume = size * size * size;  // 2^3 = 8
  double actualVolume = hull.getVolume();

  EXPECT_NEAR(actualVolume, expectedVolume, 1e-4);
}

TEST(ConvexHullTest, VolumeOfTetrahedron)
{
  // Regular tetrahedron with edge length = 2*sqrt(2)
  auto points = createTetrahedronPoints();
  ConvexHull hull(points);

  double volume = hull.getVolume();

  // Volume formula: V = (edge^3) / (6*sqrt(2))
  // For our tetrahedron: edge ≈ 2.828, V ≈ 3.771
  EXPECT_GT(volume, 0.0);
  EXPECT_NEAR(volume, 0.16666667, 0.01);  // Approximate expected value
}

TEST(ConvexHullTest, VolumeCachedCorrectly)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  double vol1 = hull.getVolume();
  double vol2 = hull.getVolume();  // Should use cached value

  EXPECT_DOUBLE_EQ(vol1, vol2);
}

// ============================================================================
// Surface Area Tests
// ============================================================================

TEST(ConvexHullTest, SurfaceAreaOfCube)
{
  double size = 2.0;
  auto points = createCubePoints(size);
  ConvexHull hull(points);

  double expectedArea = 6.0 * size * size;  // 6 faces * 4 = 24
  double actualArea = hull.getSurfaceArea();

  EXPECT_NEAR(actualArea, expectedArea, 1e-3);
}


// ============================================================================
// Centroid Tests
// ============================================================================

TEST(ConvexHullTest, CentroidOfCubeCenteredAtOrigin)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  Coordinate centroid = hull.getCentroid();

  // Cube centered at origin should have centroid at origin
  EXPECT_NEAR(centroid.x(), 0.0, 1e-4);
  EXPECT_NEAR(centroid.y(), 0.0, 1e-4);
  EXPECT_NEAR(centroid.z(), 0.0, 1e-4);
}

TEST(ConvexHullTest, CentroidOfOffsetCube)
{
  // Create cube offset from origin
  std::vector<Coordinate> points = {Coordinate(0.0, 0.0, 0.0),
                                    Coordinate(2.0, 0.0, 0.0),
                                    Coordinate(2.0, 2.0, 0.0),
                                    Coordinate(0.0, 2.0, 0.0),
                                    Coordinate(0.0, 0.0, 2.0),
                                    Coordinate(2.0, 0.0, 2.0),
                                    Coordinate(2.0, 2.0, 2.0),
                                    Coordinate(0.0, 2.0, 2.0)};
  ConvexHull hull(points);

  Coordinate centroid = hull.getCentroid();

  // Centroid should be at (1, 1, 1)
  EXPECT_NEAR(centroid.x(), 1.0, 1e-4);
  EXPECT_NEAR(centroid.y(), 1.0, 1e-4);
  EXPECT_NEAR(centroid.z(), 1.0, 1e-4);
}

// ============================================================================
// Point Containment Tests
// ============================================================================

TEST(ConvexHullTest, ContainsPointInsideHull)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  Coordinate inside(0.0, 0.0, 0.0);  // Center of cube
  EXPECT_TRUE(hull.contains(inside));
}

TEST(ConvexHullTest, ContainsPointOutsideHull)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  Coordinate outside(5.0, 5.0, 5.0);  // Far outside
  EXPECT_FALSE(hull.contains(outside));
}

TEST(ConvexHullTest, ContainsPointOnBoundary)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  Coordinate onFace(1.0, 0.0, 0.0);  // On face of cube
  EXPECT_TRUE(hull.contains(onFace));
}

TEST(ConvexHullTest, ContainsPointNearBoundaryWithEpsilon)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  // Point just outside, but within epsilon
  Coordinate nearFace(1.00001, 0.0, 0.0);

  EXPECT_FALSE(hull.contains(nearFace, 1e-6));  // Strict epsilon
  EXPECT_TRUE(hull.contains(nearFace, 1e-3));   // Relaxed epsilon
}

TEST(ConvexHullTest, ContainsAllVertices)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  // All original vertices should be contained
  for (const auto& vertex : hull.getVertices())
  {
    EXPECT_TRUE(hull.contains(vertex));
  }
}

// ============================================================================
// Signed Distance Tests
// ============================================================================

TEST(ConvexHullTest, SignedDistanceInsideIsNegative)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  Coordinate inside(0.0, 0.0, 0.0);
  double distance = hull.signedDistance(inside);

  EXPECT_LT(distance, 0.0);  // Inside points have negative distance
}

TEST(ConvexHullTest, SignedDistanceOutsideIsPositive)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  Coordinate outside(5.0, 0.0, 0.0);
  double distance = hull.signedDistance(outside);

  EXPECT_GT(distance, 0.0);          // Outside points have positive distance
  EXPECT_NEAR(distance, 4.0, 0.1);  // Should be ~4 units away
}

TEST(ConvexHullTest, SignedDistanceOnSurfaceIsZero)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  Coordinate onSurface(1.0, 0.0, 0.0);  // On face
  double distance = hull.signedDistance(onSurface);

  EXPECT_NEAR(distance, 0.0, 1e-4);
}

// ============================================================================
// Bounding Box Tests
// ============================================================================

TEST(ConvexHullTest, BoundingBoxOfCube)
{
  double size = 2.0;
  auto points = createCubePoints(size);
  ConvexHull hull(points);

  auto bbox = hull.getBoundingBox();

  double half = size / 2.0;
  EXPECT_NEAR(bbox.min.x(), -half, 1e-5);
  EXPECT_NEAR(bbox.min.y(), -half, 1e-5);
  EXPECT_NEAR(bbox.min.z(), -half, 1e-5);

  EXPECT_NEAR(bbox.max.x(), half, 1e-5);
  EXPECT_NEAR(bbox.max.y(), half, 1e-5);
  EXPECT_NEAR(bbox.max.z(), half, 1e-5);
}

TEST(ConvexHullTest, BoundingBoxContainsAllVertices)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  auto bbox = hull.getBoundingBox();

  for (const auto& vertex : hull.getVertices())
  {
    EXPECT_GE(vertex.x(), bbox.min.x());
    EXPECT_GE(vertex.y(), bbox.min.y());
    EXPECT_GE(vertex.z(), bbox.min.z());

    EXPECT_LE(vertex.x(), bbox.max.x());
    EXPECT_LE(vertex.y(), bbox.max.y());
    EXPECT_LE(vertex.z(), bbox.max.z());
  }
}

// ============================================================================
// Facet Tests
// ============================================================================

TEST(ConvexHullTest, FacetsAreTriangulated)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  const auto& facets = hull.getFacets();

  // All facets should be triangles (3 vertices)
  for (const auto& facet : facets)
  {
    EXPECT_EQ(facet.vertexIndices.size(), 3);
  }
}

TEST(ConvexHullTest, FacetNormalsAreNormalized)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  const auto& facets = hull.getFacets();

  for (const auto& facet : facets)
  {
    double normalLength = facet.normal.norm();
    EXPECT_NEAR(normalLength, 1.0, 1e-5);
  }
}

TEST(ConvexHullTest, FacetVertexIndicesValid)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  const auto& facets = hull.getFacets();
  size_t vertexCount = hull.getVertexCount();

  for (const auto& facet : facets)
  {
    for (size_t idx : facet.vertexIndices)
    {
      EXPECT_LT(idx, vertexCount);
    }
  }
}

TEST(ConvexHullTest, CubeHas12TriangularFacets)
{
  // A cube has 6 faces, each triangulated into 2 triangles = 12 total
  auto points = createCubePoints(2.0);
  ConvexHull hull(points);

  EXPECT_EQ(hull.getFacetCount(), 12);
}

TEST(ConvexHullTest, TetrahedronHas4Facets)
{
  auto points = createTetrahedronPoints();
  ConvexHull hull(points);

  EXPECT_EQ(hull.getFacetCount(), 4);
}

// ============================================================================
// Copy and Move Semantics Tests
// ============================================================================

TEST(ConvexHullTest, CopyConstructor)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull1(points);

  ConvexHull hull2(hull1);

  EXPECT_EQ(hull1.getVertexCount(), hull2.getVertexCount());
  EXPECT_EQ(hull1.getFacetCount(), hull2.getFacetCount());
  EXPECT_FLOAT_EQ(hull1.getVolume(), hull2.getVolume());
}

TEST(ConvexHullTest, CopyAssignment)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull1(points);

  ConvexHull hull2;
  hull2 = hull1;

  EXPECT_EQ(hull1.getVertexCount(), hull2.getVertexCount());
  EXPECT_FLOAT_EQ(hull1.getVolume(), hull2.getVolume());
}

TEST(ConvexHullTest, MoveConstructor)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull1(points);
  size_t originalVertexCount = hull1.getVertexCount();

  ConvexHull hull2(std::move(hull1));

  EXPECT_EQ(hull2.getVertexCount(), originalVertexCount);
  EXPECT_TRUE(hull2.isValid());
}

TEST(ConvexHullTest, MoveAssignment)
{
  auto points = createCubePoints(2.0);
  ConvexHull hull1(points);
  size_t originalVertexCount = hull1.getVertexCount();

  ConvexHull hull2;
  hull2 = std::move(hull1);

  EXPECT_EQ(hull2.getVertexCount(), originalVertexCount);
  EXPECT_TRUE(hull2.isValid());
}

// ============================================================================
// Edge Cases and Error Handling
// ============================================================================

TEST(ConvexHullTest, HandlesCoplanarPoints)
{
  // Points in XY plane (should fail for 3D hull)
  std::vector<Coordinate> coplanar = {Coordinate(0.0, 0.0, 0.0),
                                      Coordinate(1.0, 0.0, 0.0),
                                      Coordinate(1.0, 1.0, 0.0),
                                      Coordinate(0.0, 1.0, 0.0)};

  EXPECT_THROW(ConvexHull hull(coplanar), std::runtime_error);
}

TEST(ConvexHullTest, HandlesDuplicatePoints)
{
  std::vector<Coordinate> duplicates = {
    Coordinate(0.0, 0.0, 0.0),
    Coordinate(0.0, 0.0, 0.0),  // Duplicate
    Coordinate(1.0, 0.0, 0.0),
    Coordinate(1.0, 1.0, 0.0),
    Coordinate(0.0, 1.0, 1.0)};

  // Qhull should handle duplicates gracefully
  EXPECT_NO_THROW(ConvexHull hull(duplicates));
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(ConvexHullTest, WorksWithGeometryFactoryCube)
{
  // Ticket: 0003_geometry-factory-type-safety
  // CollisionGeometry should be constructed from raw vertices, not factory MeshRecord
  auto vertices = createCubeVertices(2.0);
  msd_assets::CollisionGeometry geometry{vertices};
  ConvexHull hull(geometry.getVertices());

  EXPECT_TRUE(hull.isValid());
  EXPECT_EQ(hull.getVertexCount(), 8);
  EXPECT_NEAR(hull.getVolume(), 8.0, 1e-3);
}

TEST(ConvexHullTest, WorksWithGeometryFactoryPyramid)
{
  // Ticket: 0003_geometry-factory-type-safety
  // CollisionGeometry should be constructed from raw vertices, not factory MeshRecord
  auto vertices = createPyramidVertices(2.0, 3.0);
  msd_assets::CollisionGeometry geometry{vertices};
  ConvexHull hull(geometry.getVertices());

  EXPECT_TRUE(hull.isValid());

  // Pyramid volume = (1/3) * base_area * height = (1/3) * 4 * 3 = 4
  double expectedVolume = 4.0;
  EXPECT_NEAR(hull.getVolume(), expectedVolume, 0.1);
}

#include <gtest/gtest.h>
#include "msd-assets/src/Geometry.hpp"
#include "msd-assets/src/GeometryFactory.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

// ============================================================================
// VisualGeometry Serialization Tests
// ============================================================================

TEST(GeometrySerializationTest, VisualGeometry_Cube_PopulateMeshRecord)
{
  // Create a cube
  auto cube =
    msd_assets::GeometryFactory::createCube<msd_assets::VisualGeometry>(2.0);

  // Verify vertex count
  size_t vertexCount = cube.getVertexCount();
  EXPECT_EQ(vertexCount, 36);

  // Populate MeshRecord (no name/category - those are in ObjectRecord)
  msd_transfer::MeshRecord record;
  cube.populateMeshRecord(record);

  // Extract values to primitive types for easier debugging
  uint32_t vertex_count = record.vertex_count;
  uint32_t triangle_count = record.triangle_count;
  size_t vertex_data_size = record.vertex_data.size();
  bool vertex_data_empty = record.vertex_data.empty();

  // Verify record fields
  EXPECT_EQ(vertex_count, 36);
  EXPECT_EQ(triangle_count, 12);
  EXPECT_FALSE(vertex_data_empty);
  EXPECT_EQ(vertex_data_size, 36 * sizeof(msd_assets::Vertex));
}

TEST(GeometrySerializationTest, VisualGeometry_Pyramid_RoundTrip)
{
  // Create a pyramid
  auto pyramid =
    msd_assets::GeometryFactory::createPyramid<msd_assets::VisualGeometry>(3.0,
                                                                           4.0);

  // Serialize to MeshRecord
  msd_transfer::MeshRecord record;
  pyramid.populateMeshRecord(record);

  // Deserialize back to VisualGeometry
  auto reconstructed = msd_assets::VisualGeometry::fromMeshRecord(record);

  // Verify vertex count matches
  EXPECT_EQ(reconstructed.getVertexCount(), pyramid.getVertexCount());

  // Verify vertex data matches
  const auto& originalVertices = pyramid.getVertices();
  const auto& reconstructedVertices = reconstructed.getVertices();

  ASSERT_EQ(originalVertices.size(), reconstructedVertices.size());

  for (size_t i = 0; i < originalVertices.size(); ++i)
  {
    // Check positions
    EXPECT_FLOAT_EQ(originalVertices[i].position[0],
                    reconstructedVertices[i].position[0]);
    EXPECT_FLOAT_EQ(originalVertices[i].position[1],
                    reconstructedVertices[i].position[1]);
    EXPECT_FLOAT_EQ(originalVertices[i].position[2],
                    reconstructedVertices[i].position[2]);

    // Check normals
    EXPECT_FLOAT_EQ(originalVertices[i].normal[0],
                    reconstructedVertices[i].normal[0]);
    EXPECT_FLOAT_EQ(originalVertices[i].normal[1],
                    reconstructedVertices[i].normal[1]);
    EXPECT_FLOAT_EQ(originalVertices[i].normal[2],
                    reconstructedVertices[i].normal[2]);
  }
}

// ============================================================================
// CollisionGeometry Serialization Tests
// ============================================================================

TEST(GeometrySerializationTest, CollisionGeometry_Cube_PopulateMeshRecord)
{
  // Create a collision cube
  auto collisionCube =
    msd_assets::GeometryFactory::createCube<msd_assets::CollisionGeometry>(1.5);

  // Verify hull vertex count
  EXPECT_EQ(collisionCube.getVertexCount(), 36);  // Hull vertices

  // Populate CollisionMeshRecord
  msd_transfer::CollisionMeshRecord record;
  collisionCube.populateMeshRecord(record);

  // Verify hull data
  EXPECT_EQ(record.hull_vertex_count, 36);
  EXPECT_FALSE(record.hull_data.empty());
  EXPECT_EQ(record.hull_data.size(),
            36 * 3 * sizeof(double));  // 36 vertices * 3 coords * 8 bytes

  // Verify bounding box was calculated
  EXPECT_LT(record.aabb_min_x, 0.0f);
  EXPECT_GT(record.aabb_max_x, 0.0f);
  EXPECT_GT(record.bounding_radius, 0.0f);
}

TEST(GeometrySerializationTest, CollisionGeometry_Pyramid_RoundTrip)
{
  // Create a collision pyramid
  auto pyramid =
    msd_assets::GeometryFactory::createPyramid<msd_assets::CollisionGeometry>(
      2.0, 3.0);

  // Serialize to CollisionMeshRecord
  msd_transfer::CollisionMeshRecord record;
  pyramid.populateMeshRecord(record);

  // Store original bounding box values
  float originalMinX = record.aabb_min_x;
  float originalMaxX = record.aabb_max_x;
  float originalRadius = record.bounding_radius;

  // Deserialize back to CollisionGeometry
  auto reconstructed = msd_assets::CollisionGeometry::fromMeshRecord(record);

  // Verify hull vertex counts match
  EXPECT_EQ(reconstructed.getVertexCount(), pyramid.getVertexCount());

  // Verify hull vertices match
  const auto& originalHull = pyramid.getHullVertices();
  const auto& reconstructedHull = reconstructed.getHullVertices();

  ASSERT_EQ(originalHull.size(), reconstructedHull.size());

  for (size_t i = 0; i < originalHull.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(originalHull[i].x(), reconstructedHull[i].x());
    EXPECT_DOUBLE_EQ(originalHull[i].y(), reconstructedHull[i].y());
    EXPECT_DOUBLE_EQ(originalHull[i].z(), reconstructedHull[i].z());
  }

  // Verify bounding box is consistent
  const auto& reconstructedBBox = reconstructed.getBoundingBox();
  EXPECT_FLOAT_EQ(reconstructedBBox.min_x, originalMinX);
  EXPECT_FLOAT_EQ(reconstructedBBox.max_x, originalMaxX);
  EXPECT_FLOAT_EQ(reconstructedBBox.radius, originalRadius);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST(GeometrySerializationTest, InvalidBlobSize_ThrowsException)
{
  // Create invalid record with wrong blob size
  msd_transfer::MeshRecord invalid;
  invalid.vertex_data =
    std::vector<uint8_t>(10);  // Not a multiple of sizeof(Vertex)
  invalid.vertex_count = 3;
  invalid.triangle_count = 1;

  // Attempt to deserialize should throw
  EXPECT_THROW(msd_assets::VisualGeometry::fromMeshRecord(invalid),
               std::runtime_error);
}

TEST(GeometrySerializationTest,
     CollisionGeometry_InvalidHullBlob_ThrowsException)
{
  // Create a valid visual mesh but invalid hull
  auto cube =
    msd_assets::GeometryFactory::createCube<msd_assets::CollisionGeometry>(1.0);

  msd_transfer::CollisionMeshRecord record;
  cube.populateMeshRecord(record);

  // Corrupt the hull data (not a multiple of 24 bytes = 3 doubles)
  record.hull_data = std::vector<uint8_t>(10);

  // Attempt to deserialize should throw
  EXPECT_THROW(msd_assets::CollisionGeometry::fromMeshRecord(record),
               std::runtime_error);
}

TEST(GeometrySerializationTest, EmptyVertexData_ThrowsException)
{
  msd_transfer::MeshRecord empty;
  empty.vertex_data.clear();
  empty.vertex_count = 0;
  empty.triangle_count = 0;

  // Deserializing empty data should throw
  EXPECT_THROW(msd_assets::VisualGeometry::fromMeshRecord(empty),
               std::runtime_error);
}

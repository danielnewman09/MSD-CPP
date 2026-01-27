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
  auto record = msd_assets::GeometryFactory::createCube(2.0);

  // Extract values to primitive types for easier debugging
  uint32_t vertex_count = record.vertex_count;
  size_t vertex_data_size = record.vertex_data.size();
  bool vertex_data_empty = record.vertex_data.empty();

  // Verify record fields
  EXPECT_EQ(vertex_count, 36);
  EXPECT_FALSE(vertex_data_empty);
  EXPECT_EQ(vertex_data_size, 36 * sizeof(Eigen::Vector3d));
}

TEST(GeometrySerializationTest, CreateCube)
{
  // Create CollisionMeshRecord directly from factory
  auto collisionRecord = msd_assets::GeometryFactory::createCube(1.0);

  msd_assets::CollisionGeometry collisionGeometry{collisionRecord, 1};

  msd_assets::VisualGeometry visualGeometry{collisionRecord, 1};


  ASSERT_TRUE(true);
}
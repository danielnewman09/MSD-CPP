#include <gtest/gtest.h>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <filesystem>
#include <memory>
#include "msd-assets/src/Geometry.hpp"
#include "msd-assets/src/GeometryFactory.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

namespace fs = std::filesystem;

// ============================================================================
// Test Fixture for Database Operations
// ============================================================================

class GeometryDatabaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create temporary database file
    testDbPath_ = "test_geometry.db";

    // Remove if exists from previous test
    if (fs::exists(testDbPath_))
    {
      fs::remove(testDbPath_);
    }

    // Get logger instance
    auto& logger = cpp_sqlite::Logger::getInstance();


    // Create database (allowWrite = true)
    db_ = std::make_unique<cpp_sqlite::Database>(
      testDbPath_, true, logger.getLogger());
  }


  void TearDown() override
  {
    // Close database
    db_.reset();

    // // Clean up test database file
    // if (fs::exists(testDbPath_))
    // {
    //   fs::remove(testDbPath_);
    // }
  }

  std::string testDbPath_;
  std::unique_ptr<cpp_sqlite::Database> db_;
};

// ============================================================================
// msd_assets::BaseGeometry Database Tests
// ============================================================================

TEST_F(GeometryDatabaseTest, DISABLED_BaseGeometry_CreateAndStore_Cube)
{
  // Create a cube using the factory
  auto cube =
    msd_assets::GeometryFactory::createCube<msd_assets::BaseGeometry>(2.0);

  // Verify vertex count (12 triangles × 3 vertices = 36 vertices)
  EXPECT_EQ(cube.getVertexCount(), 36);

  // Create a msd_transfer::MeshRecord and populate it
  msd_transfer::MeshRecord record;
  record.name = "test_cube";
  record.category = "primitives";
  cube.populateMeshRecord(record);

  // Verify record fields
  EXPECT_EQ(record.vertex_count, 36);
  EXPECT_EQ(record.triangle_count, 12);
  EXPECT_FALSE(record.vertex_data.empty());
  EXPECT_EQ(record.vertex_data.size(), 36 * sizeof(msd_assets::Vertex));

  auto& meshDAO = db_->getDAO<msd_transfer::MeshRecord>();

  // Insert into database using DAO
  meshDAO.insert(record);

  // Verify insertion
  EXPECT_GT(record.id, 0);

  // Query the record back using DAO
  auto retrieved = meshDAO.selectById(record.id);

  ASSERT_TRUE(retrieved.has_value());

  // Verify retrieved data matches
  EXPECT_EQ(retrieved->name, record.name);
  EXPECT_EQ(retrieved->category, record.category);
  EXPECT_EQ(retrieved->vertex_count, record.vertex_count);
  EXPECT_EQ(retrieved->triangle_count, record.triangle_count);
  EXPECT_EQ(retrieved->vertex_data.size(), record.vertex_data.size());
  EXPECT_EQ(retrieved->vertex_data, record.vertex_data);
}

// Minimal test to isolate debugger issue
TEST_F(GeometryDatabaseTest, MinimalDatabaseTest)
{
  std::string testPath{"test_db.db"};
  cpp_sqlite::Database myDb{testPath, true};
  EXPECT_TRUE(fs::exists(testPath));
}

TEST_F(GeometryDatabaseTest, BaseGeometry_RoundTrip_Pyramid)
{
  // Create a pyramid
  auto pyramid =
    msd_assets::GeometryFactory::createPyramid<msd_assets::BaseGeometry>(3.0,
                                                                         4.0);

  // Verify vertex count (6 triangles × 3 vertices = 18 vertices)
  EXPECT_EQ(pyramid.getVertexCount(), 18);

  // Serialize to msd_transfer::MeshRecord
  msd_transfer::MeshRecord record;
  record.name = "test_pyramid";
  record.category = "primitives";
  pyramid.populateMeshRecord(record);

  auto& meshDAO = db_->getDAO<msd_transfer::MeshRecord>();

  ASSERT_TRUE(meshDAO.isInitialized());

  // Store in database
  meshDAO.insert(record);
  EXPECT_GT(record.id, 0);

  // Retrieve from database
  auto retrieved = meshDAO.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  // Reconstruct geometry from retrieved record
  auto reconstructed = msd_assets::BaseGeometry::fromMeshRecord(*retrieved);

  // Verify reconstructed geometry matches original
  EXPECT_EQ(reconstructed.getVertexCount(), pyramid.getVertexCount());

  // Verify vertex data matches by comparing cached vertices
  const auto& originalVertices = pyramid.getVertices();
  const auto& reconstructedVertices = reconstructed.getVertices();

  ASSERT_EQ(originalVertices.size(), reconstructedVertices.size());

  for (size_t i = 0; i < originalVertices.size(); ++i)
  {
    EXPECT_FLOAT_EQ(originalVertices[i].position[0],
                    reconstructedVertices[i].position[0]);
    EXPECT_FLOAT_EQ(originalVertices[i].position[1],
                    reconstructedVertices[i].position[1]);
    EXPECT_FLOAT_EQ(originalVertices[i].position[2],
                    reconstructedVertices[i].position[2]);

    // Normals should match
    EXPECT_FLOAT_EQ(originalVertices[i].normal[0],
                    reconstructedVertices[i].normal[0]);
    EXPECT_FLOAT_EQ(originalVertices[i].normal[1],
                    reconstructedVertices[i].normal[1]);
    EXPECT_FLOAT_EQ(originalVertices[i].normal[2],
                    reconstructedVertices[i].normal[2]);
  }
}

// ============================================================================
// msd_assets::CollisionGeometry Database Tests
// ============================================================================

TEST_F(GeometryDatabaseTest, CollisionGeometry_CreateAndStore_Cube)
{
  // Create a collision cube
  auto collisionCube =
    msd_assets::GeometryFactory::createCube<msd_assets::CollisionGeometry>(1.5);

  auto& collisionDAO = db_->getDAO<msd_transfer::CollisionMeshRecord>();

  // Verify vertex count (visual mesh: 36, hull vertices: 36)
  EXPECT_EQ(collisionCube.msd_assets::BaseGeometry::getVertexCount(), 36);
  EXPECT_EQ(collisionCube.getVertexCount(), 36);  // Hull vertices

  // Create a msd_transfer::CollisionMeshRecord and populate it
  msd_transfer::CollisionMeshRecord record;
  record.name = "collision_cube";
  record.category = "collision";
  collisionCube.populateMeshRecord(record);

  // Verify base record fields
  EXPECT_EQ(record.vertex_count, 36);
  EXPECT_EQ(record.triangle_count, 12);
  EXPECT_FALSE(record.vertex_data.empty());

  // Verify hull data
  EXPECT_EQ(record.hull_vertex_count, 36);
  EXPECT_FALSE(record.hull_data.empty());

  // Verify bounding box was calculated
  EXPECT_LT(record.aabb_min_x, 0.0f);
  EXPECT_GT(record.aabb_max_x, 0.0f);
  EXPECT_GT(record.bounding_radius, 0.0f);

  // Insert into database using DAO
  EXPECT_TRUE(collisionDAO.insert(record));

  // Verify insertion
  EXPECT_GT(record.id, 0);
}

TEST_F(GeometryDatabaseTest, CollisionGeometry_RoundTrip_CompleteData)
{
  // Create a collision pyramid
  auto pyramid =
    msd_assets::GeometryFactory::createPyramid<msd_assets::CollisionGeometry>(
      2.0, 3.0);


  auto& collisionDAO = db_->getDAO<msd_transfer::CollisionMeshRecord>();

  // Serialize to msd_transfer::CollisionMeshRecord
  msd_transfer::CollisionMeshRecord record;
  record.name = "collision_pyramid";
  record.category = "collision";
  pyramid.populateMeshRecord(record);

  // Store in database
  collisionDAO.insert(record);
  EXPECT_GT(record.id, 0);

  // Retrieve from database
  auto retrieved = collisionDAO.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  // Verify all fields match
  EXPECT_EQ(retrieved->name, record.name);
  EXPECT_EQ(retrieved->category, record.category);
  EXPECT_EQ(retrieved->vertex_count, record.vertex_count);
  EXPECT_EQ(retrieved->triangle_count, record.triangle_count);
  EXPECT_EQ(retrieved->hull_vertex_count, record.hull_vertex_count);
  EXPECT_EQ(retrieved->vertex_data, record.vertex_data);
  EXPECT_EQ(retrieved->hull_data, record.hull_data);

  // Verify bounding box data
  EXPECT_FLOAT_EQ(retrieved->aabb_min_x, record.aabb_min_x);
  EXPECT_FLOAT_EQ(retrieved->aabb_min_y, record.aabb_min_y);
  EXPECT_FLOAT_EQ(retrieved->aabb_min_z, record.aabb_min_z);
  EXPECT_FLOAT_EQ(retrieved->aabb_max_x, record.aabb_max_x);
  EXPECT_FLOAT_EQ(retrieved->aabb_max_y, record.aabb_max_y);
  EXPECT_FLOAT_EQ(retrieved->aabb_max_z, record.aabb_max_z);
  EXPECT_FLOAT_EQ(retrieved->bounding_radius, record.bounding_radius);

  // Reconstruct geometry
  auto reconstructed =
    msd_assets::CollisionGeometry::fromMeshRecord(*retrieved);

  // Verify geometry matches
  EXPECT_EQ(reconstructed.msd_assets::BaseGeometry::getVertexCount(),
            pyramid.msd_assets::BaseGeometry::getVertexCount());
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
}

// ============================================================================
// Query and Filter Tests
// ============================================================================

TEST_F(GeometryDatabaseTest, QueryByName_FindsCorrectRecord)
{
  // Create and store multiple geometries
  auto cube =
    msd_assets::GeometryFactory::createCube<msd_assets::BaseGeometry>(1.0);
  auto pyramid =
    msd_assets::GeometryFactory::createPyramid<msd_assets::BaseGeometry>(1.5,
                                                                         2.0);

  msd_transfer::MeshRecord cubeRecord;
  cubeRecord.name = "my_cube";
  cubeRecord.category = "primitives";
  cube.populateMeshRecord(cubeRecord);

  msd_transfer::MeshRecord pyramidRecord;
  pyramidRecord.name = "my_pyramid";
  pyramidRecord.category = "primitives";
  pyramid.populateMeshRecord(pyramidRecord);

  auto& meshDAO = db_->getDAO<msd_transfer::MeshRecord>();

  meshDAO.insert(cubeRecord);
  meshDAO.insert(pyramidRecord);

  // Query all and filter by name
  auto allResults = meshDAO.selectAll();
  std::vector<msd_transfer::MeshRecord> results;
  for (const auto& r : allResults)
  {
    if (r.name == "my_pyramid")
    {
      results.push_back(r);
    }
  }

  ASSERT_EQ(results.size(), 1);
  EXPECT_EQ(results[0].name, "my_pyramid");
  EXPECT_EQ(results[0].vertex_count, pyramidRecord.vertex_count);
}

TEST_F(GeometryDatabaseTest, QueryByCategory_FindsMultipleRecords)
{
  auto& collisionDAO = db_->getDAO<msd_transfer::CollisionMeshRecord>();

  // Create and store multiple geometries in same category
  auto cube1 =
    msd_assets::GeometryFactory::createCube<msd_assets::CollisionGeometry>(1.0);
  auto cube2 =
    msd_assets::GeometryFactory::createCube<msd_assets::CollisionGeometry>(2.0);
  auto pyramid =
    msd_assets::GeometryFactory::createPyramid<msd_assets::CollisionGeometry>(
      1.5, 2.0);

  msd_transfer::CollisionMeshRecord record1, record2, record3;

  record1.name = "cube_1";
  record1.category = "collision";
  cube1.populateMeshRecord(record1);

  record2.name = "cube_2";
  record2.category = "collision";
  cube2.populateMeshRecord(record2);

  record3.name = "pyramid_1";
  record3.category = "special";
  pyramid.populateMeshRecord(record3);

  collisionDAO.insert(record1);
  collisionDAO.insert(record2);
  collisionDAO.insert(record3);

  // Query all and filter by category
  auto allResults = collisionDAO.selectAll();
  std::vector<msd_transfer::CollisionMeshRecord> collisionResults;
  for (const auto& r : allResults)
  {
    if (r.category == "collision")
    {
      collisionResults.push_back(r);
    }
  }

  EXPECT_EQ(collisionResults.size(), 2);

  // Verify both cubes are returned
  std::vector<std::string> names;
  for (const auto& result : collisionResults)
  {
    names.push_back(result.name);
  }

  EXPECT_TRUE(std::find(names.begin(), names.end(), "cube_1") != names.end());
  EXPECT_TRUE(std::find(names.begin(), names.end(), "cube_2") != names.end());
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(GeometryDatabaseTest, InvalidBlobSize_ThrowsException)
{
  // Create invalid record with wrong blob size
  msd_transfer::MeshRecord invalid;
  invalid.name = "invalid";
  invalid.category = "test";
  invalid.vertex_data = std::vector<uint8_t>(10);  // Not a multiple of 36
  invalid.vertex_count = 3;
  invalid.triangle_count = 1;

  // Attempt to deserialize should throw
  EXPECT_THROW(msd_assets::BaseGeometry::fromMeshRecord(invalid),
               std::runtime_error);
}

TEST_F(GeometryDatabaseTest, CollisionGeometry_InvalidHullBlob_ThrowsException)
{
  // Create a valid visual mesh but invalid hull
  auto cube =
    msd_assets::GeometryFactory::createCube<msd_assets::CollisionGeometry>(1.0);

  msd_transfer::CollisionMeshRecord record;
  record.name = "invalid_hull";
  record.category = "test";
  cube.populateMeshRecord(record);

  // Corrupt the hull data (not a multiple of 24 bytes)
  record.hull_data = std::vector<uint8_t>(10);

  // Attempt to deserialize should throw
  EXPECT_THROW(msd_assets::CollisionGeometry::fromMeshRecord(record),
               std::runtime_error);
}

TEST_F(GeometryDatabaseTest, GetNonexistentRecord_ReturnsNullopt)
{
  auto& meshDAO = db_->getDAO<msd_transfer::MeshRecord>();

  // Try to get a record that doesn't exist
  auto result = meshDAO.selectById(99999);

  EXPECT_FALSE(result.has_value());
}

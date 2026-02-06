#include <filesystem>
#include <memory>

#include <gtest/gtest.h>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>

#include "msd-assets/src/Geometry.hpp"
#include "msd-assets/src/GeometryFactory.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

namespace fs = std::filesystem;

// ============================================================================
// Helper Functions for Generating Raw Vertices
// ============================================================================
// Ticket: 0003_geometry-factory-type-safety
// These helpers generate raw vertex coordinates for CollisionGeometry.
// VisualGeometry should use GeometryFactory, which produces Vertex blobs.

std::vector<msd_sim::Vector3D> generateCubeVertices(double size)
{
  double half = size / 2.0;

  // Define 8 corners
  std::array<msd_sim::Vector3D, 8> corners = {
    msd_sim::Vector3D{-half, -half, -half},  // 0: FTL
    msd_sim::Vector3D{half, -half, -half},   // 1: FTR
    msd_sim::Vector3D{half, half, -half},    // 2: FBR
    msd_sim::Vector3D{-half, half, -half},   // 3: FBL
    msd_sim::Vector3D{-half, -half, half},   // 4: BTL
    msd_sim::Vector3D{half, -half, half},    // 5: BTR
    msd_sim::Vector3D{half, half, half},     // 6: BBR
    msd_sim::Vector3D{-half, half, half}     // 7: BBL
  };

  std::vector<msd_sim::Vector3D> vertices;
  vertices.reserve(36);

  // Front face (z = -half)
  vertices.push_back(corners[0]);
  vertices.push_back(corners[1]);
  vertices.push_back(corners[2]);
  vertices.push_back(corners[0]);
  vertices.push_back(corners[2]);
  vertices.push_back(corners[3]);

  // Back face (z = +half)
  vertices.push_back(corners[5]);
  vertices.push_back(corners[4]);
  vertices.push_back(corners[7]);
  vertices.push_back(corners[5]);
  vertices.push_back(corners[7]);
  vertices.push_back(corners[6]);

  // Left face (x = -half)
  vertices.push_back(corners[4]);
  vertices.push_back(corners[0]);
  vertices.push_back(corners[3]);
  vertices.push_back(corners[4]);
  vertices.push_back(corners[3]);
  vertices.push_back(corners[7]);

  // Right face (x = +half)
  vertices.push_back(corners[1]);
  vertices.push_back(corners[5]);
  vertices.push_back(corners[6]);
  vertices.push_back(corners[1]);
  vertices.push_back(corners[6]);
  vertices.push_back(corners[2]);

  // Top face (y = -half)
  vertices.push_back(corners[4]);
  vertices.push_back(corners[5]);
  vertices.push_back(corners[1]);
  vertices.push_back(corners[4]);
  vertices.push_back(corners[1]);
  vertices.push_back(corners[0]);

  // Bottom face (y = +half)
  vertices.push_back(corners[3]);
  vertices.push_back(corners[2]);
  vertices.push_back(corners[6]);
  vertices.push_back(corners[3]);
  vertices.push_back(corners[6]);
  vertices.push_back(corners[7]);

  return vertices;
}

std::vector<msd_sim::Vector3D> generatePyramidVertices(double baseSize,
                                                       double height)
{
  double half = baseSize / 2.0;
  double halfHeight = height / 2.0;

  std::vector<msd_sim::Vector3D> vertices;
  vertices.reserve(18);

  // Base corners (y = -height/2)
  msd_sim::Vector3D base_fl{-half, -halfHeight, -half};
  msd_sim::Vector3D base_fr{half, -halfHeight, -half};
  msd_sim::Vector3D base_br{half, -halfHeight, half};
  msd_sim::Vector3D base_bl{-half, -halfHeight, half};

  // Apex
  msd_sim::Vector3D apex{0.0, halfHeight, 0.0};

  // Front face
  vertices.push_back(base_fl);
  vertices.push_back(base_fr);
  vertices.push_back(apex);

  // Right face
  vertices.push_back(base_fr);
  vertices.push_back(base_br);
  vertices.push_back(apex);

  // Back face
  vertices.push_back(base_br);
  vertices.push_back(base_bl);
  vertices.push_back(apex);

  // Left face
  vertices.push_back(base_bl);
  vertices.push_back(base_fl);
  vertices.push_back(apex);

  // Base bottom (2 triangles)
  vertices.push_back(base_fl);
  vertices.push_back(base_br);
  vertices.push_back(base_fr);

  vertices.push_back(base_fl);
  vertices.push_back(base_bl);
  vertices.push_back(base_br);

  return vertices;
}

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
// msd_assets::VisualGeometry Database Tests
// ============================================================================

TEST_F(GeometryDatabaseTest, VisualGeometry_CreateAndStore_Cube)
{
  // Create MeshRecord directly from factory
  auto meshRecord = msd_assets::GeometryFactory::createCube(2.0);

  // Verify record fields
  EXPECT_EQ(meshRecord.vertex_count, 36);
  EXPECT_FALSE(meshRecord.vertex_data.empty());
  EXPECT_EQ(meshRecord.vertex_data.size(), 36 * sizeof(msd_sim::Vector3D));

  auto& meshDAO = db_->getDAO<msd_transfer::MeshRecord>();

  // Insert mesh into database
  meshDAO.insert(meshRecord);
  EXPECT_GT(meshRecord.id, 0);

  // Reconstruct VisualGeometry from the record
  msd_assets::VisualGeometry cube{meshRecord, 1};

  // Verify vertex count (12 triangles × 3 vertices = 36 vertices)
  EXPECT_EQ(cube.getVertexCount(), 36);

  // Create ObjectRecord that references the mesh
  msd_transfer::ObjectRecord objectRecord;
  objectRecord.name = "test_cube";
  objectRecord.category = "primitives";
  objectRecord.meshRecord.id = meshRecord.id;
  // No collision mesh for this object

  auto& objectDAO = db_->getDAO<msd_transfer::ObjectRecord>();
  objectDAO.insert(objectRecord);
  EXPECT_GT(objectRecord.id, 0);

  // Query the mesh record back
  auto retrievedMesh = meshDAO.selectById(meshRecord.id);
  ASSERT_TRUE(retrievedMesh.has_value());

  // Verify retrieved mesh data matches
  EXPECT_EQ(retrievedMesh->vertex_count, meshRecord.vertex_count);
  EXPECT_EQ(retrievedMesh->vertex_data.size(), meshRecord.vertex_data.size());
  EXPECT_EQ(retrievedMesh->vertex_data, meshRecord.vertex_data);

  // Query the object record back
  auto retrievedObject = objectDAO.selectById(objectRecord.id);
  ASSERT_TRUE(retrievedObject.has_value());
  EXPECT_EQ(retrievedObject->name, "test_cube");
  EXPECT_EQ(retrievedObject->category, "primitives");
  EXPECT_EQ(retrievedObject->meshRecord.id, meshRecord.id);
}

// Minimal test to isolate debugger issue
TEST_F(GeometryDatabaseTest, MinimalDatabaseTest)
{
  std::string testPath{"test_db.db"};
  cpp_sqlite::Database myDb{testPath, true};
  EXPECT_TRUE(fs::exists(testPath));
}

TEST_F(GeometryDatabaseTest, VisualGeometry_RoundTrip_Pyramid)
{
  // Create MeshRecord directly from factory
  auto record = msd_assets::GeometryFactory::createPyramid(3.0, 4.0);

  // Verify record data
  EXPECT_EQ(record.vertex_count, 18);  // 6 triangles × 3 vertices
  auto& meshDAO = db_->getDAO<msd_transfer::MeshRecord>();
  ASSERT_TRUE(meshDAO.isInitialized());

  // Store in database
  meshDAO.insert(record);
  EXPECT_GT(record.id, 0);

  // Reconstruct original geometry from the record before storing
  msd_assets::VisualGeometry pyramid{record, 1};
  EXPECT_EQ(pyramid.getVertexCount(), 18);

  // Retrieve from database
  auto retrieved = meshDAO.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  // Reconstruct geometry from retrieved record
  msd_assets::VisualGeometry reconstructed{*retrieved, 1};

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
  // Ticket: 0003_geometry-factory-type-safety
  // Create CollisionGeometry from raw vertices (not factory)
  auto cubeVertices = generateCubeVertices(1.5);
  msd_assets::CollisionGeometry collisionCube{cubeVertices, 1};
  EXPECT_EQ(collisionCube.getVertexCount(), 36);

  // Serialize to MeshRecord for database storage
  auto collisionRecord = collisionCube.populateMeshRecord();

  // Verify record data (Vector3d is 24 bytes each)
  EXPECT_EQ(collisionRecord.vertex_count, 36);
  EXPECT_FALSE(collisionRecord.vertex_data.empty());
  EXPECT_EQ(collisionRecord.vertex_data.size(), 36 * sizeof(msd_sim::Vector3D));

  // Insert collision mesh into database
  auto& collisionDAO = db_->getDAO<msd_transfer::MeshRecord>();
  EXPECT_TRUE(collisionDAO.insert(collisionRecord));
  EXPECT_GT(collisionRecord.id, 0);

  // Create ObjectRecord that references the collision mesh
  msd_transfer::ObjectRecord objectRecord;
  objectRecord.name = "collision_cube";
  objectRecord.category = "collision";
  objectRecord.collisionMeshRecord.id = collisionRecord.id;
  // No visual mesh for this collision-only object

  auto& objectDAO = db_->getDAO<msd_transfer::ObjectRecord>();
  objectDAO.insert(objectRecord);
  EXPECT_GT(objectRecord.id, 0);
}

TEST_F(GeometryDatabaseTest, CollisionGeometry_RoundTrip_CompleteData)
{
  // Ticket: 0003_geometry-factory-type-safety
  // Create CollisionGeometry from raw vertices (not factory)
  auto pyramidVertices = generatePyramidVertices(2.0, 3.0);
  msd_assets::CollisionGeometry pyramid{pyramidVertices, 1};

  // Serialize to MeshRecord
  auto record = pyramid.populateMeshRecord();

  auto& collisionDAO = db_->getDAO<msd_transfer::MeshRecord>();

  // Store in database
  collisionDAO.insert(record);
  EXPECT_GT(record.id, 0);

  // Retrieve from database
  auto retrieved = collisionDAO.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  // Verify all fields match
  EXPECT_EQ(retrieved->vertex_count, record.vertex_count);
  EXPECT_EQ(retrieved->vertex_data, record.vertex_data);

  // Reconstruct geometry from retrieved record
  msd_assets::CollisionGeometry reconstructed{*retrieved, 1};

  // Verify geometry matches
  EXPECT_EQ(reconstructed.getVertexCount(), pyramid.getVertexCount());

  // Verify hull vertices match
  const auto& originalHull = pyramid.getVertices();
  const auto& reconstructedHull = reconstructed.getVertices();

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
  // Create MeshRecords directly from factory
  auto cubeRecord = msd_assets::GeometryFactory::createCube(1.0);
  auto pyramidRecord = msd_assets::GeometryFactory::createPyramid(1.5, 2.0);

  // Store mesh data
  auto& meshDAO = db_->getDAO<msd_transfer::MeshRecord>();
  meshDAO.insert(cubeRecord);
  meshDAO.insert(pyramidRecord);

  // Create object records with names
  msd_transfer::ObjectRecord cubeObject;
  cubeObject.name = "my_cube";
  cubeObject.category = "primitives";
  cubeObject.meshRecord.id = cubeRecord.id;

  msd_transfer::ObjectRecord pyramidObject;
  pyramidObject.name = "my_pyramid";
  pyramidObject.category = "primitives";
  pyramidObject.meshRecord.id = pyramidRecord.id;

  auto& objectDAO = db_->getDAO<msd_transfer::ObjectRecord>();
  objectDAO.insert(cubeObject);
  objectDAO.insert(pyramidObject);

  // Query all objects and filter by name
  auto allResults = objectDAO.selectAll();
  std::vector<msd_transfer::ObjectRecord> results;
  for (const auto& r : allResults)
  {
    if (r.name == "my_pyramid")
    {
      results.push_back(r);
    }
  }

  ASSERT_EQ(results.size(), 1);
  EXPECT_EQ(results[0].name, "my_pyramid");
  EXPECT_EQ(results[0].meshRecord.id, pyramidRecord.id);
}

TEST_F(GeometryDatabaseTest, QueryByCategory_FindsMultipleRecords)
{
  // Create CollisionMeshRecords directly from factory
  auto record1 = msd_assets::GeometryFactory::createCube(1.0);
  auto record2 = msd_assets::GeometryFactory::createCube(2.0);
  auto record3 = msd_assets::GeometryFactory::createPyramid(1.5, 2.0);

  // Store collision mesh data
  auto& collisionDAO = db_->getDAO<msd_transfer::MeshRecord>();
  collisionDAO.insert(record1);
  collisionDAO.insert(record2);
  collisionDAO.insert(record3);

  // Create object records with categories
  msd_transfer::ObjectRecord object1, object2, object3;

  object1.name = "cube_1";
  object1.category = "collision";
  object1.collisionMeshRecord.id = record1.id;

  object2.name = "cube_2";
  object2.category = "collision";
  object2.collisionMeshRecord.id = record2.id;

  object3.name = "pyramid_1";
  object3.category = "special";
  object3.collisionMeshRecord.id = record3.id;

  auto& objectDAO = db_->getDAO<msd_transfer::ObjectRecord>();
  objectDAO.insert(object1);
  objectDAO.insert(object2);
  objectDAO.insert(object3);

  // Query all objects and filter by category
  auto allResults = objectDAO.selectAll();
  std::vector<msd_transfer::ObjectRecord> collisionResults;
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
  invalid.vertex_data = std::vector<uint8_t>(10);  // Not a multiple of 36
  invalid.vertex_count = 3;

  // Attempt to deserialize should throw
  EXPECT_THROW(msd_assets::VisualGeometry(invalid, 1), std::runtime_error);
}

TEST_F(GeometryDatabaseTest, CollisionGeometry_InvalidHullBlob_ThrowsException)
{
  // Ticket: 0003_geometry-factory-type-safety
  // Create a valid CollisionGeometry record then corrupt it
  auto cubeVertices = generateCubeVertices(1.0);
  msd_assets::CollisionGeometry cube{cubeVertices, 1};
  auto record = cube.populateMeshRecord();

  // Corrupt the vertex data (not a multiple of 24 bytes for Vector3d)
  record.vertex_data = std::vector<uint8_t>(10);

  // Attempt to deserialize should throw
  EXPECT_THROW(msd_assets::CollisionGeometry(record, 1), std::runtime_error);
}

TEST_F(GeometryDatabaseTest, GetNonexistentRecord_ReturnsNullopt)
{
  auto& meshDAO = db_->getDAO<msd_transfer::MeshRecord>();

  // Try to get a record that doesn't exist
  auto result = meshDAO.selectById(99999);

  EXPECT_FALSE(result.has_value());
}

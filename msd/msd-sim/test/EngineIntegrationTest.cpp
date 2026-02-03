// Ticket: Engine integration test for collision simulation
// Purpose: Holistic testing of Engine simulation loop with collision detection

#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <filesystem>
#include <vector>
#include "msd-assets/src/GeometryFactory.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Engine.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

namespace fs = std::filesystem;
using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

// Create cube vertices as raw Eigen::Vector3d for collision geometry
std::vector<Eigen::Vector3d> createCubeCollisionVertices(double size)
{
  double half = size / 2.0;
  return {Eigen::Vector3d{-half, -half, -half},
          Eigen::Vector3d{half, -half, -half},
          Eigen::Vector3d{half, half, -half},
          Eigen::Vector3d{-half, half, -half},
          Eigen::Vector3d{-half, -half, half},
          Eigen::Vector3d{half, -half, half},
          Eigen::Vector3d{half, half, half},
          Eigen::Vector3d{-half, half, half}};
}

// Serialize collision vertices to BLOB format
std::vector<uint8_t> serializeCollisionVertices(
  const std::vector<Eigen::Vector3d>& vertices)
{
  std::vector<uint8_t> blob(vertices.size() * sizeof(Eigen::Vector3d));
  std::memcpy(
    blob.data(), vertices.data(), vertices.size() * sizeof(Eigen::Vector3d));
  return blob;
}

}  // anonymous namespace

// ============================================================================
// Test Fixture
// ============================================================================

class EngineIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create temporary database file
    testDbPath_ = "test_engine_integration.db";

    // Remove if exists from previous test
    if (fs::exists(testDbPath_))
    {
      fs::remove(testDbPath_);
    }

    // Create database (allowWrite = true)
    db_ = std::make_unique<cpp_sqlite::Database>(testDbPath_, true);

    // Create test assets in database
    createTestAssets();
  }

  void TearDown() override
  {
    // Close database
    db_.reset();

    // Clean up test database file
    if (fs::exists(testDbPath_))
    {
      fs::remove(testDbPath_);
    }
  }

  void createTestAssets()
  {
    auto& meshDAO = db_->getDAO<msd_transfer::MeshRecord>();
    auto& objectDAO = db_->getDAO<msd_transfer::ObjectRecord>();

    // Create a 1x1x1 cube collision mesh
    auto cubeVertices = createCubeCollisionVertices(1.0);
    msd_transfer::MeshRecord cubeMesh;
    cubeMesh.vertex_data = serializeCollisionVertices(cubeVertices);
    cubeMesh.vertex_count = static_cast<uint32_t>(cubeVertices.size());
    meshDAO.insert(cubeMesh);

    // Create "test_cube" object with collision geometry
    msd_transfer::ObjectRecord cubeObject;
    cubeObject.name = "test_cube";
    cubeObject.category = "test";
    cubeObject.collisionMeshRecord.id = cubeMesh.id;
    objectDAO.insert(cubeObject);

    // Create a 2x2x2 cube collision mesh for larger object
    auto largeCubeVertices = createCubeCollisionVertices(2.0);
    msd_transfer::MeshRecord largeCubeMesh;
    largeCubeMesh.vertex_data = serializeCollisionVertices(largeCubeVertices);
    largeCubeMesh.vertex_count =
      static_cast<uint32_t>(largeCubeVertices.size());
    meshDAO.insert(largeCubeMesh);

    // Create "test_large_cube" object
    msd_transfer::ObjectRecord largeCubeObject;
    largeCubeObject.name = "test_large_cube";
    largeCubeObject.category = "test";
    largeCubeObject.collisionMeshRecord.id = largeCubeMesh.id;
    objectDAO.insert(largeCubeObject);
  }

  std::string testDbPath_;
  std::unique_ptr<cpp_sqlite::Database> db_;
};

// ============================================================================
// Basic Engine Initialization Tests
// ============================================================================

TEST_F(EngineIntegrationTest, Engine_ConstructsWithValidDatabase)
{
  // Should not throw with valid database path
  EXPECT_NO_THROW({ Engine engine{testDbPath_}; });
}

TEST_F(EngineIntegrationTest, Engine_SpawnsInertialObject)
{
  Engine engine{testDbPath_};

  // Spawn a test cube
  EXPECT_NO_THROW({
    engine.spawnInertialObject(
      "test_cube", Coordinate{0.0, 0.0, 0.0}, AngularCoordinate{});
  });

  // Verify object was added to world model
  EXPECT_EQ(1, engine.getWorldModel().getObjectCount());
}

TEST_F(EngineIntegrationTest, Engine_SpawnsEnvironmentObject)
{
  Engine engine{testDbPath_};

  // Engine constructor already creates a floor environment object
  size_t initialEnvCount =
    engine.getWorldModel().getEnvironmentalObjects().size();

  // Spawn a static environment object
  EXPECT_NO_THROW({
    engine.spawnEnvironmentObject(
      "test_cube", Coordinate{0.0, 0.0, -5.0}, AngularCoordinate{});
  });

  // Verify environment object was added (one more than initial)
  EXPECT_EQ(initialEnvCount + 1,
            engine.getWorldModel().getEnvironmentalObjects().size());
}

TEST_F(EngineIntegrationTest, Engine_ThrowsOnInvalidAssetName)
{
  Engine engine{testDbPath_};

  // Should throw for non-existent asset
  EXPECT_THROW(
    {
      engine.spawnInertialObject(
        "nonexistent_asset", Coordinate{0.0, 0.0, 0.0}, AngularCoordinate{});
    },
    std::runtime_error);
}

// ============================================================================
// Simulation Update Tests
// ============================================================================

TEST_F(EngineIntegrationTest, Engine_UpdateAdvancesSimulationTime)
{
  Engine engine{testDbPath_};

  auto initialTime = engine.getWorldModel().getTime();

  engine.update(std::chrono::milliseconds{16});

  auto finalTime = engine.getWorldModel().getTime();
  EXPECT_GT(finalTime, initialTime);
  EXPECT_EQ(std::chrono::milliseconds{16}, finalTime - initialTime);
}

TEST_F(EngineIntegrationTest, Engine_GravityAffectsInertialObjects)
{
  Engine engine{testDbPath_};

  // Spawn object above origin - capture instance ID immediately
  uint32_t objId =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.0, 0.0, 10.0}, AngularCoordinate{})
      .getInstanceId();

  double initialZ =
    engine.getWorldModel().getObject(objId).getInertialState().position.z();

  // Step simulation multiple times
  for (int i = 0; i < 10; ++i)
  {
    engine.update(std::chrono::milliseconds{16});
  }

  // Object should have fallen due to gravity
  double finalZ =
    engine.getWorldModel().getObject(objId).getInertialState().position.z();
  EXPECT_LT(finalZ, initialZ);
}

// ============================================================================
// Collision Detection Tests
// ============================================================================

TEST_F(EngineIntegrationTest, Engine_TwoSeparatedObjects_NoCollision)
{
  Engine engine{testDbPath_};

  // Spawn two well-separated objects high above floor (z=50)
  // Capture instance IDs immediately - references may be invalidated on realloc
  uint32_t idA =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.0, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();
  uint32_t idB =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{10.0, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();

  // Set velocities moving apart in X (not Z, which is affected by gravity)
  engine.getWorldModel().getObject(idA).getInertialState().velocity =
    Coordinate{-1.0, 0.0, 0.0};
  engine.getWorldModel().getObject(idB).getInertialState().velocity =
    Coordinate{1.0, 0.0, 0.0};

  double vAxInitial =
    engine.getWorldModel().getObject(idA).getInertialState().velocity.x();
  double vBxInitial =
    engine.getWorldModel().getObject(idB).getInertialState().velocity.x();

  // Update simulation
  engine.update(std::chrono::milliseconds{16});

  // X velocities should remain unchanged (no collision - gravity only affects
  // Z)
  double vAxFinal =
    engine.getWorldModel().getObject(idA).getInertialState().velocity.x();
  double vBxFinal =
    engine.getWorldModel().getObject(idB).getInertialState().velocity.x();

  EXPECT_DOUBLE_EQ(vAxInitial, vAxFinal);
  EXPECT_DOUBLE_EQ(vBxInitial, vBxFinal);
}

TEST_F(EngineIntegrationTest, Engine_OverlappingObjects_VelocitiesChange)
{
  Engine engine{testDbPath_};

  // Spawn two overlapping objects high above floor
  // Capture instance IDs immediately - references may be invalidated on realloc
  uint32_t idA =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.0, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();
  uint32_t idB =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.8, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();

  // Set elastic collision
  engine.getWorldModel().getObject(idA).setCoefficientOfRestitution(1.0);
  engine.getWorldModel().getObject(idB).setCoefficientOfRestitution(1.0);

  // Set approaching velocities in X direction
  engine.getWorldModel().getObject(idA).getInertialState().velocity =
    Coordinate{1.0, 0.0, 0.0};
  engine.getWorldModel().getObject(idB).getInertialState().velocity =
    Coordinate{-1.0, 0.0, 0.0};

  double vAxInitial =
    engine.getWorldModel().getObject(idA).getInertialState().velocity.x();
  double vBxInitial =
    engine.getWorldModel().getObject(idB).getInertialState().velocity.x();

  // Update simulation
  engine.update(std::chrono::milliseconds{16});

  // X velocities should have changed due to collision
  double vAxFinal =
    engine.getWorldModel().getObject(idA).getInertialState().velocity.x();
  double vBxFinal =
    engine.getWorldModel().getObject(idB).getInertialState().velocity.x();

  EXPECT_NE(vAxInitial, vAxFinal);
  EXPECT_NE(vBxInitial, vBxFinal);
}

TEST_F(EngineIntegrationTest, Engine_ObjectCollidesWithFloor)
{
  Engine engine{testDbPath_};

  // Engine constructor creates a floor at z=-60
  // Spawn object above the floor - capture instance ID immediately
  uint32_t objId =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.0, 0.0, -8.0}, AngularCoordinate{})
      .getInstanceId();

  // Set elastic collision
  engine.getWorldModel().getObject(objId).setCoefficientOfRestitution(1.0);

  // Give it downward velocity toward floor
  engine.getWorldModel().getObject(objId).getInertialState().velocity =
    Coordinate{0.0, 0.0, -10.0};

  // Update many times (object should fall and eventually hit the floor)
  for (int i = 0; i < 100; ++i)
  {
    engine.update(std::chrono::milliseconds{16});
  }

  // Object should not have fallen through the floor (z > -60)
  double finalZ =
    engine.getWorldModel().getObject(objId).getInertialState().position.z();

  // The floor is at z=-60 with size 100 (extends from z=-110 to z=-10)
  // Object should bounce and not penetrate significantly below the floor
  // surface
  EXPECT_GT(finalZ, -60.0);
}


TEST_F(EngineIntegrationTest, ObjectRestsOnfloor)
{
  Engine engine{testDbPath_};

  // Engine constructor creates a floor at z=-60
  // Spawn object above the floor - capture instance ID immediately
  uint32_t objId =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.0, 0.0, -9.51}, AngularCoordinate{})
      .getInstanceId();

  // Set elastic collision
  engine.getWorldModel().getObject(objId).setCoefficientOfRestitution(1.0);

  // Give it downward velocity toward floor
  engine.getWorldModel().getObject(objId).getInertialState().velocity =
    Coordinate{0.0, 0.0, 0.0};

  // Update many times (object should fall and eventually hit the floor)
  for (int i = 0; i < 100; ++i)
  {
    engine.update(std::chrono::milliseconds{16});
  }

  // Object should not have fallen through the floor (z > -60)
  double finalZ =
    engine.getWorldModel().getObject(objId).getInertialState().position.z();

  // The floor is at z=-60 with size 100 (extends from z=-110 to z=-10)
  // Object should bounce and not penetrate significantly below the floor
  // surface
  EXPECT_GT(finalZ, -60.0);
}

// ============================================================================
// Multi-step Simulation Tests
// ============================================================================

TEST_F(EngineIntegrationTest, Engine_MomentumConserved_ElasticCollision)
{
  Engine engine{testDbPath_};

  // Spawn two overlapping objects
  // Capture instance IDs immediately - references may be invalidated on realloc
  uint32_t idA =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.0, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();
  uint32_t idB =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.9, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();

  // Set elastic collision
  engine.getWorldModel().getObject(idA).setCoefficientOfRestitution(1.0);
  engine.getWorldModel().getObject(idB).setCoefficientOfRestitution(1.0);

  // Set head-on velocities (total momentum = 0)
  engine.getWorldModel().getObject(idA).getInertialState().velocity =
    Coordinate{2.0, 0.0, 0.0};
  engine.getWorldModel().getObject(idB).getInertialState().velocity =
    Coordinate{-2.0, 0.0, 0.0};

  // Calculate initial x-momentum (masses are both 1.0)
  double initialMomentumX =
    engine.getWorldModel().getObject(idA).getInertialState().velocity.x() +
    engine.getWorldModel().getObject(idB).getInertialState().velocity.x();

  // Update simulation
  engine.update(std::chrono::milliseconds{16});

  // Calculate final x-momentum
  double finalMomentumX =
    engine.getWorldModel().getObject(idA).getInertialState().velocity.x() +
    engine.getWorldModel().getObject(idB).getInertialState().velocity.x();

  // X momentum should be conserved (approximately)
  EXPECT_NEAR(initialMomentumX, finalMomentumX, 0.01);
}

TEST_F(EngineIntegrationTest, Engine_InelasticCollision_EnergyLost)
{
  Engine engine{testDbPath_};

  // Spawn two overlapping objects high enough to avoid floor interaction
  // Capture instance IDs immediately - references may be invalidated on realloc
  uint32_t idA =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.0, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();
  uint32_t idB =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.9, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();

  // Set fully inelastic collision
  engine.getWorldModel().getObject(idA).setCoefficientOfRestitution(0.0);
  engine.getWorldModel().getObject(idB).setCoefficientOfRestitution(0.0);

  // Set velocities
  engine.getWorldModel().getObject(idA).getInertialState().velocity =
    Coordinate{2.0, 0.0, 0.0};
  engine.getWorldModel().getObject(idB).getInertialState().velocity =
    Coordinate{-2.0, 0.0, 0.0};

  // Calculate initial kinetic energy (m=1, KE = 0.5 * m * v^2)
  double vAi =
    engine.getWorldModel().getObject(idA).getInertialState().velocity.norm();
  double vBi =
    engine.getWorldModel().getObject(idB).getInertialState().velocity.norm();
  double initialKE = 0.5 * 1.0 * (vAi * vAi + vBi * vBi);

  // Update simulation
  engine.update(std::chrono::milliseconds{16});

  // Calculate final kinetic energy
  double vAf =
    engine.getWorldModel().getObject(idA).getInertialState().velocity.norm();
  double vBf =
    engine.getWorldModel().getObject(idB).getInertialState().velocity.norm();
  double finalKE = 0.5 * 1.0 * (vAf * vAf + vBf * vBf);

  // Energy should be lost in inelastic collision
  EXPECT_LT(finalKE, initialKE);
}

TEST_F(EngineIntegrationTest, Engine_PositionCorrection_ObjectsSeparated)
{
  Engine engine{testDbPath_};

  // Spawn two deeply overlapping objects
  // Capture instance IDs immediately - references may be invalidated on realloc
  uint32_t idA =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.0, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();
  uint32_t idB =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.5, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();

  // Both at rest
  engine.getWorldModel().getObject(idA).getInertialState().velocity =
    Coordinate{0.0, 0.0, 0.0};
  engine.getWorldModel().getObject(idB).getInertialState().velocity =
    Coordinate{0.0, 0.0, 0.0};

  double initialDistance = std::abs(
    engine.getWorldModel().getObject(idB).getInertialState().position.x() -
    engine.getWorldModel().getObject(idA).getInertialState().position.x());

  // Update simulation
  engine.update(std::chrono::milliseconds{16});

  // Objects should be pushed apart
  double finalDistance = std::abs(
    engine.getWorldModel().getObject(idB).getInertialState().position.x() -
    engine.getWorldModel().getObject(idA).getInertialState().position.x());

  EXPECT_GT(finalDistance, initialDistance);
}

// ============================================================================
// Environment Object (Static) Collision Tests
// ============================================================================

TEST_F(EngineIntegrationTest, Engine_InertialBouncesOffStaticEnvironment)
{
  Engine engine{testDbPath_};

  // Spawn static wall at x=5
  engine.spawnEnvironmentObject(
    "test_large_cube", Coordinate{5.0, 0.0, 50.0}, AngularCoordinate{});

  // Spawn dynamic object overlapping with wall (starts inside collision zone)
  // test_large_cube is 2x2x2, so its edge is at x=4
  // test_cube is 1x1x1, so placing at x=3.5 means edge at x=4, overlapping
  // Capture instance ID immediately
  uint32_t objId =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{3.5, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();

  // Set elastic restitution
  engine.getWorldModel().getObject(objId).setCoefficientOfRestitution(1.0);

  // Moving toward wall
  engine.getWorldModel().getObject(objId).getInertialState().velocity =
    Coordinate{5.0, 0.0, 0.0};

  double vxInitial =
    engine.getWorldModel().getObject(objId).getInertialState().velocity.x();
  EXPECT_GT(vxInitial, 0.0);  // Moving in +x direction

  // Run simulation - collision response should happen
  engine.update(std::chrono::milliseconds{16});

  // After collision with wall, x-velocity should have changed
  double vxFinal =
    engine.getWorldModel().getObject(objId).getInertialState().velocity.x();

  // Velocity should have changed due to collision with static object
  EXPECT_NE(vxInitial, vxFinal);
}

TEST_F(EngineIntegrationTest, Engine_StaticEnvironmentUnaffectedByCollision)
{
  Engine engine{testDbPath_};

  // Spawn static wall
  const auto& wall = engine.spawnEnvironmentObject(
    "test_large_cube", Coordinate{5.0, 0.0, 50.0}, AngularCoordinate{});

  // Store initial wall position
  Coordinate wallInitialPos = wall.getReferenceFrame().getOrigin();

  // Spawn fast-moving dynamic object toward wall
  // Capture instance ID immediately
  uint32_t objId =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{3.5, 0.0, 50.0}, AngularCoordinate{})
      .getInstanceId();

  engine.getWorldModel().getObject(objId).getInertialState().velocity =
    Coordinate{10.0, 0.0, 0.0};

  // Run simulation
  for (int i = 0; i < 50; ++i)
  {
    engine.update(std::chrono::milliseconds{16});
  }

  // Wall position should be unchanged
  Coordinate wallFinalPos = wall.getReferenceFrame().getOrigin();

  EXPECT_DOUBLE_EQ(wallInitialPos.x(), wallFinalPos.x());
  EXPECT_DOUBLE_EQ(wallInitialPos.y(), wallFinalPos.y());
  EXPECT_DOUBLE_EQ(wallInitialPos.z(), wallFinalPos.z());
}

// ============================================================================
// Long-running Simulation Tests
// ============================================================================

TEST_F(EngineIntegrationTest, Engine_LongSimulation_StablePhysics)
{
  Engine engine{testDbPath_};

  // Spawn object - capture instance ID immediately
  uint32_t objId =
    engine
      .spawnInertialObject(
        "test_cube", Coordinate{0.0, 0.0, 0.0}, AngularCoordinate{})
      .getInstanceId();

  // Run simulation for many frames (simulating ~10 seconds at 60fps)
  for (int i = 0; i < 600; ++i)
  {
    EXPECT_NO_THROW({ engine.update(std::chrono::milliseconds{16}); });
  }

  // Verify state is still valid (not NaN)
  const auto& state =
    engine.getWorldModel().getObject(objId).getInertialState();
  EXPECT_FALSE(std::isnan(state.position.x()));
  EXPECT_FALSE(std::isnan(state.position.y()));
  EXPECT_FALSE(std::isnan(state.position.z()));
  EXPECT_FALSE(std::isnan(state.velocity.x()));
  EXPECT_FALSE(std::isnan(state.velocity.y()));
  EXPECT_FALSE(std::isnan(state.velocity.z()));
}

TEST_F(EngineIntegrationTest, Engine_MultipleObjectsSimulation_NoErrors)
{
  Engine engine{testDbPath_};

  // Spawn multiple objects
  for (int i = 0; i < 5; ++i)
  {
    engine.spawnInertialObject(
      "test_cube",
      Coordinate{static_cast<double>(i * 3), 0.0, 50.0},
      AngularCoordinate{});
  }

  EXPECT_EQ(5, engine.getWorldModel().getObjectCount());

  // Run simulation
  for (int i = 0; i < 100; ++i)
  {
    EXPECT_NO_THROW({ engine.update(std::chrono::milliseconds{16}); });
  }
}

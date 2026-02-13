// Ticket: 0060a_replay_enabled_test_fixture
// Tests for ReplayEnabledTest fixture

#include "ReplayEnabledTest.hpp"

#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <cpp_sqlite/src/utils/Logger.hpp>
#include <gtest/gtest.h>

#include "msd-transfer/src/MeshRecord.hpp"
#include "msd-transfer/src/SimulationFrameRecord.hpp"

using namespace msd_sim;

// Test fixture setup creates database [0060a_replay_enabled_test_fixture]
TEST_F(ReplayEnabledTest, SetUp_CreatesDatabase)
{
  // Verify recording database exists
  EXPECT_TRUE(std::filesystem::exists(recordingPath()));
}

// Test database contains geometry from pre-built asset DB
// [0060a_replay_enabled_test_fixture]
TEST_F(ReplayEnabledTest, SetUp_DatabaseContainsGeometry)
{
  // Open database independently
  auto& logger = cpp_sqlite::Logger::getInstance();
  cpp_sqlite::Database db{recordingPath().string(), false, logger.getLogger()};

  // Verify ObjectRecord table exists and has entries
  auto& objectDAO = db.getDAO<msd_transfer::ObjectRecord>();
  const auto objects = objectDAO.selectAll();

  ASSERT_GE(objects.size(), 3);  // At least unit_cube, large_cube, floor_slab

  // Verify expected asset names exist
  std::set<std::string> assetNames;
  for (const auto& obj : objects)
  {
    assetNames.insert(obj.name);
  }

  EXPECT_TRUE(assetNames.count("unit_cube") > 0);
  EXPECT_TRUE(assetNames.count("large_cube") > 0);
  EXPECT_TRUE(assetNames.count("floor_slab") > 0);

  // Verify MeshRecord table exists and has entries
  auto& meshDAO = db.getDAO<msd_transfer::MeshRecord>();
  const auto meshes = meshDAO.selectAll();

  EXPECT_GE(meshes.size(),
            6);  // At least 2 meshes per asset (visual + collision)
}

// Test spawning a cube creates inertial asset
// [0060a_replay_enabled_test_fixture]
TEST_F(ReplayEnabledTest, SpawnCube_CreatesInertialAsset)
{
  const auto& cube = spawnCube("unit_cube", Coordinate{0, 0, 10});

  // Verify asset was created at expected position
  EXPECT_EQ(cube.getInertialState().position.z(), 10.0);
}

// Test stepping advances simulation time
// [0060a_replay_enabled_test_fixture]
TEST_F(ReplayEnabledTest, Step_AdvancesSimulationTime)
{
  const auto initialTime = world().getTime().count() / 1000.0;

  step(5, std::chrono::milliseconds{16});

  const auto expectedTimeDelta = (5 * 16) / 1000.0;
  const auto expectedTime = initialTime + expectedTimeDelta;
  const auto actualTime = world().getTime().count() / 1000.0;
  EXPECT_DOUBLE_EQ(actualTime, expectedTime);
}

// Test TearDown produces recording with frames
// [0060a_replay_enabled_test_fixture]
TEST_F(ReplayEnabledTest, TearDown_ProducesRecordingWithFrames)
{
  // Spawn object and run simulation
  spawnCube("unit_cube", Coordinate{0, 0, 10});
  step(10);

  // Store path before TearDown
  const auto dbPath = recordingPath();

  // Manually trigger TearDown (will be called again by framework, but that's
  // OK)
  TearDown();

  // Verify database exists (default MSD_KEEP_RECORDINGS policy should preserve
  // it)
  ASSERT_TRUE(std::filesystem::exists(dbPath));

  // Open database and verify SimulationFrameRecord entries
  auto& logger = cpp_sqlite::Logger::getInstance();
  cpp_sqlite::Database db{dbPath.string(), false, logger.getLogger()};

  auto& frameDAO = db.getDAO<msd_transfer::SimulationFrameRecord>();
  const auto frames = frameDAO.selectAll();

  EXPECT_GT(frames.size(), 0);  // Should have recorded frames
}

// Test environment object spawning
// [0060a_replay_enabled_test_fixture]
TEST_F(ReplayEnabledTest, SpawnEnvironment_CreatesEnvironmentAsset)
{
  const auto& floor = spawnEnvironment("floor_slab", Coordinate{0, 0, -50});

  // Verify asset was created at expected position
  EXPECT_EQ(floor.getReferenceFrame().getOrigin().z(), -50.0);
}

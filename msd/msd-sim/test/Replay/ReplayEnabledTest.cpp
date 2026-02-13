// Ticket: 0060a_replay_enabled_test_fixture
// Design: Direct specification in ticket (no separate design doc)

#include "ReplayEnabledTest.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"

namespace msd_sim
{

void ReplayEnabledTest::SetUp()
{
  // Get test name from GTest
  const auto* testInfo =
    ::testing::UnitTest::GetInstance()->current_test_info();
  const std::string testSuite = testInfo->test_suite_name();
  const std::string testName = testInfo->name();

  // Build per-test recording path: {MSD_RECORDINGS_DIR}/{Suite}_{Test}.db
  const std::filesystem::path recordingsDir{MSD_RECORDINGS_DIR};
  const std::string dbFilename = testSuite + "_" + testName + ".db";
  dbPath_ = recordingsDir / dbFilename;

  // Create recordings directory if it doesn't exist
  std::filesystem::create_directories(recordingsDir);

  // Copy pre-built test asset database to per-test recording path
  const std::filesystem::path sourceDb{MSD_TEST_ASSETS_DB};
  std::filesystem::copy_file(
    sourceDb, dbPath_, std::filesystem::copy_options::overwrite_existing);

  // Create Engine (reads geometry via AssetRegistry)
  engine_ = std::make_unique<Engine>(dbPath_.string());

  // Enable recording (DataRecorder opens same DB read-write, adds state
  // tables)
  engine_->getWorldModel().enableRecording(dbPath_.string());
}

void ReplayEnabledTest::TearDown()
{
  // Destroy Engine — RAII triggers DataRecorder flush and thread join
  engine_.reset();

  // Check MSD_KEEP_RECORDINGS env var for cleanup policy
  const char* keepRecordings = std::getenv("MSD_KEEP_RECORDINGS");
  const bool shouldKeep = (!keepRecordings || std::string(keepRecordings) != "0");

  if (!shouldKeep)
  {
    // Remove recording file
    std::filesystem::remove(dbPath_);
  }
  else
  {
    // Preserve recording for replay viewer
    std::cout << "Recording saved: " << dbPath_ << "\n";
  }
}

const AssetInertial& ReplayEnabledTest::spawnCube(const std::string& assetName,
                                                   const Coordinate& position)
{
  // Default orientation (identity quaternion)
  return engine_->spawnInertialObject(assetName, position, AngularCoordinate{});
}

const AssetEnvironment&
  ReplayEnabledTest::spawnEnvironment(const std::string& assetName,
                                      const Coordinate& position)
{
  // Default orientation (identity quaternion)
  return engine_->spawnEnvironmentObject(
    assetName, position, AngularCoordinate{});
}

void ReplayEnabledTest::step(int frames, std::chrono::milliseconds dt)
{
  for (int i = 0; i < frames; ++i)
  {
    currentTime_ += dt;
    engine_->update(currentTime_);
  }
}

}  // namespace msd_sim

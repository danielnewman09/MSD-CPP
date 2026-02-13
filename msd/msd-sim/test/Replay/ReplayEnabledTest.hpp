// Ticket: 0060a_replay_enabled_test_fixture
// Design: Direct specification in ticket (no separate design doc)

#ifndef MSD_SIM_TEST_REPLAY_ENABLED_TEST_HPP
#define MSD_SIM_TEST_REPLAY_ENABLED_TEST_HPP

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "msd-sim/src/Engine.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"

namespace msd_sim
{

/**
 * @brief GTest fixture for replay-enabled simulation tests
 * @ticket 0060a_replay_enabled_test_fixture
 *
 * Provides a test fixture that:
 * - Copies pre-built test asset database to per-test recording path
 * - Initializes Engine with real asset-registry-backed geometry
 * - Enables DataRecorder for automatic state recording
 * - Produces self-contained .db files viewable in the replay viewer
 * - Cleans up recordings based on MSD_KEEP_RECORDINGS env var
 *
 * Usage:
 * ```cpp
 * TEST_F(ReplayEnabledTest, MyPhysicsTest) {
 *   const auto& cube = spawnCube("unit_cube", Coordinate{0, 0, 10});
 *   step(100);  // Simulate 100 frames
 *   EXPECT_GT(cube.inertialState().position().z(), 0.0);
 * }
 * ```
 *
 * Output: `replay/recordings/{TestSuite}_{TestName}.db`
 */
class ReplayEnabledTest : public ::testing::Test
{
protected:
  void SetUp() override;
  void TearDown() override;

  /**
   * @brief Spawn a cube from the test asset registry
   * @param assetName Name of asset ("unit_cube", "large_cube", or
   * "floor_slab")
   * @param position World position for spawned cube
   * @return Reference to spawned inertial asset
   *
   * Uses default orientation (identity quaternion).
   */
  const AssetInertial& spawnCube(const std::string& assetName,
                                 const Coordinate& position);

  /**
   * @brief Spawn an environment object from the test asset registry
   * @param assetName Name of asset ("unit_cube", "large_cube", or
   * "floor_slab")
   * @param position World position for spawned object
   * @return Reference to spawned environment asset
   *
   * Environment objects are static (immovable) and participate in collision
   * detection. Uses default orientation (identity quaternion).
   */
  const AssetEnvironment& spawnEnvironment(const std::string& assetName,
                                           const Coordinate& position);

  /**
   * @brief Advance simulation by specified number of frames
   * @param frames Number of frames to simulate (default: 1)
   * @param dt Time step per frame (default: 16ms for ~60 FPS)
   *
   * Tracks cumulative simulation time internally.
   */
  void step(int frames = 1,
            std::chrono::milliseconds dt = std::chrono::milliseconds{16});

  /**
   * @brief Get access to the simulation engine
   * @return Reference to Engine instance
   */
  Engine& engine()
  {
    return *engine_;
  }

  /**
   * @brief Get access to the world model
   * @return Reference to WorldModel instance
   */
  WorldModel& world()
  {
    return engine_->getWorldModel();
  }

  /**
   * @brief Get the path to the recording database for this test
   * @return Filesystem path to the .db file
   */
  std::filesystem::path recordingPath() const
  {
    return dbPath_;
  }

private:
  std::filesystem::path dbPath_;
  std::unique_ptr<Engine> engine_;
  std::chrono::milliseconds currentTime_{0};
};

}  // namespace msd_sim

#endif  // MSD_SIM_TEST_REPLAY_ENABLED_TEST_HPP

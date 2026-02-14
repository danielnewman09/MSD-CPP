// Ticket: 0060d_example_replay_tests
// Example test demonstrating ReplayEnabledTest fixture with cube drop scenario

#include "ReplayEnabledTest.hpp"

#include <gtest/gtest.h>

#include "msd-sim/src/DataTypes/Coordinate.hpp"

using namespace msd_sim;

/**
 * @brief Example test suite demonstrating replay-enabled cube drop
 * @ticket 0060d_example_replay_tests
 *
 * Shows how to:
 * - Inherit from ReplayEnabledTest
 * - Spawn objects using real asset database
 * - Run multi-frame simulations
 * - Use traditional in-memory assertions
 * - Produce .db files viewable in browser
 */
class ReplayDropTest : public ReplayEnabledTest
{
};

/**
 * @brief Example test: Cube drops from height and settles
 * @ticket 0060d_example_replay_tests
 *
 * **Scenario**: Single cube dropped from height onto Engine's default floor.
 *
 * **Physics tested**:
 * - Gravity
 * - Resting contact
 * - Energy dissipation
 * - Settling behavior
 *
 * **Recording output**: `replay/recordings/ReplayDropTest_CubeDropsAndSettles.db`
 *
 * **Python validation**: See `replay/tests/test_drop_recording.py`
 */
TEST_F(ReplayDropTest, CubeDropsAndSettles)
{
  // Spawn cube at z=5.0 meters
  const auto& cube = spawnCube("unit_cube", Coordinate{0.0, 0.0, 5.0});
  uint32_t cubeId = cube.getInstanceId();

  // Simulate ~5 seconds at 60 FPS
  step(300);

  // Traditional in-memory assertion: cube should not penetrate too far below floor
  // Floor top is at z=-10.0, allow some penetration due to current physics limitations
  // Python test validates cube never goes below z=-11.0
  EXPECT_GT(world().getObject(cubeId).getInertialState().position.z(), -11.0);

  // Recording database automatically saved by fixture TearDown
  // Python tests will validate:
  // - Frame count > 290
  // - Never penetrates below z=-11.0
  // - Comes to rest (speed < 0.5 m/s)
}

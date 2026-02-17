// Ticket: 0066_friction_cone_solver_saturation_bug
// Test: Targeted friction cone solver diagnostic test
//
// PURPOSE: Place a cube on a floor with horizontal velocity to create a
// sliding contact. With replay enabled, we can inspect the recording to
// verify that the friction solver produces forces at the Coulomb cone
// boundary for sliding contacts.
//
// PHYSICS (manual verification):
//   - Cube: 1×1×1 m, mass = 1 kg, mu = 0.5, e = 0.0 (perfectly inelastic)
//   - Floor: static, at z = 0 (surface)
//   - Gravity: g = 9.81 m/s² downward
//   - Initial velocity: (2, 0, 0) m/s horizontal
//
//   Normal force:   N = m * g = 1.0 * 9.81 = 9.81 N
//   Friction limit: f_max = mu * N = 0.5 * 9.81 = 4.905 N
//   Force to arrest sliding in 1 frame (dt=16ms):
//     f_arrest = m * v / dt = 1.0 * 2.0 / 0.016 = 125 N
//   Since f_arrest >> f_max, this is a SLIDING contact.
//   Coulomb model requires: ||f_t|| = mu * N = 4.905 N (saturated)
//
//   Expected deceleration: a = mu * g = 0.5 * 9.81 = 4.905 m/s²
//   Expected velocity after 1 frame: v = 2.0 - 4.905 * 0.016 = 1.9215 m/s
//   Time to stop: t = v0 / (mu * g) = 2.0 / 4.905 = 0.408 s (~25 frames)

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/test/Replay/ReplayEnabledTest.hpp"

using namespace msd_sim;

// ============================================================================
// Friction Cone Solver Diagnostic Tests
// ============================================================================

class FrictionConeSolverTest : public ReplayEnabledTest
{
};

// ---------------------------------------------------------------------------
// F1: Sliding cube on floor — friction should saturate at cone boundary
// ---------------------------------------------------------------------------
TEST_F(FrictionConeSolverTest, SlidingCubeOnFloor_FrictionSaturatesAtConeLimit)
{
  // Setup: floor at z=-50 (surface at z=0), cube resting on floor
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube at z=0.5 (bottom face at z=0, touching floor)
  // Horizontal velocity of 2 m/s in x-direction
  // e=0.0 to avoid any bounce complications
  // mu=0.5 for clear friction behavior
  constexpr double mass = 1.0;
  constexpr double restitution = 0.0;
  constexpr double friction = 0.5;
  constexpr double initialVelocityX = 2.0;

  const auto& cube =
    spawnInertialWithVelocity("unit_cube",
                              Coordinate{0.0, 0.0, 0.5},
                              Coordinate{initialVelocityX, 0.0, 0.0},
                              mass,
                              restitution,
                              friction);

  // Physics constants
  constexpr double g = 9.81;
  constexpr double dt = 0.016;
  constexpr double muG = friction * g;  // 4.905 m/s²

  // Let contact settle for 10 frames. The coupled QP has a transient in the
  // first few frames where the cube wobbles (angular velocity from corner
  // contacts, vertical bounce despite e=0). By frame 10 the cube is in
  // clean sliding contact with stable deceleration.
  constexpr int warmupFrames = 10;
  step(warmupFrames);

  const auto& stateWarmup = cube.getInertialState();
  const double vxAfterWarmup = stateWarmup.velocity.x();

  // Measure steady-state friction over 15 frames (frames 10-25)
  constexpr int measureFrames = 15;
  step(measureFrames);

  const auto& stateMeasure = cube.getInertialState();
  const double vx = stateMeasure.velocity.x();

  std::cout << "\n=== Friction Cone Solver Diagnostic ===\n";
  std::cout << "After warmup (" << warmupFrames << " frames): vx = "
            << vxAfterWarmup << " m/s\n";
  std::cout << "After measure (" << measureFrames << " frames): vx = "
            << vx << " m/s\n";

  // The cube should be decelerating
  EXPECT_GT(vxAfterWarmup, 0.0) << "Cube should still be moving after warmup";
  EXPECT_LT(vx, vxAfterWarmup)
    << "Cube should have decelerated from friction";

  // Measure steady-state deceleration over the measurement window
  const double actualDecel =
    (vxAfterWarmup - vx) / (measureFrames * dt);
  std::cout << "Expected deceleration: " << muG << " m/s²\n";
  std::cout << "Actual deceleration:   " << actualDecel << " m/s²\n";
  std::cout << "Ratio (actual/expected): " << (actualDecel / muG) << "\n";

  // Steady-state deceleration should be close to mu*g (within 10%)
  EXPECT_NEAR(actualDecel, muG, muG * 0.10)
    << "Friction deceleration should be mu*g = " << muG << " m/s².\n"
    << "If significantly less, the friction cone solver is undersaturating.";

  // Continue to frame 30 to observe full sliding behavior
  step(30 - warmupFrames - measureFrames);  // Total: 30 frames

  const auto& state30 = cube.getInertialState();
  const double vxFinal = state30.velocity.x();
  const double expectedVxFinal = initialVelocityX - muG * 30.0 * dt;

  std::cout << "\n=== Frame 30 ===\n";
  std::cout << "Velocity x: " << vxFinal << " m/s\n";
  std::cout << "Expected:   " << std::max(0.0, expectedVxFinal) << " m/s\n";
  std::cout << "Position:   (" << state30.position.x() << ", "
            << state30.position.y() << ", " << state30.position.z() << ")\n";

  // Y and Z velocity should remain near zero
  EXPECT_NEAR(state30.velocity.y(), 0.0, 0.1) << "No lateral velocity expected";
  EXPECT_NEAR(state30.velocity.z(), 0.0, 0.2)
    << "No vertical velocity expected (cube should stay on floor)";

  // Z position should remain near 0.5 (resting on floor)
  // Allow some tolerance for penetration correction
  EXPECT_NEAR(state30.position.z(), 0.5, 0.1)
    << "Cube should remain resting on floor";

  std::cout << "\nRecording saved to: " << recordingPath() << "\n";
  std::cout << "Analyze with: load_recording(\""
            << recordingPath().filename().string() << "\")\n";
  std::cout << "Then: check_friction_cone(<frame>)\n";
  std::cout << "      get_contacts_for_body(1, <frame>)\n";
}

TEST_F(FrictionConeSolverTest, test)
{
  // Setup: floor at z=-50 (surface at z=0), cube resting on floor
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube at z=0.5 (bottom face at z=0, touching floor)
  // Horizontal velocity of 2 m/s in x-direction
  // e=0.0 to avoid any bounce complications
  // mu=0.5 for clear friction behavior
  constexpr double mass = 1.0;
  constexpr double restitution = 0.0;
  constexpr double friction = 0.5;
  constexpr double initialVelocityX = 5.0;

  const auto& cube =
    spawnInertialWithVelocity("unit_cube",
                              Coordinate{0.0, 0.0, 0.64},
                              Coordinate{initialVelocityX, 0.0, 0.0},
                              mass,
                              restitution,
                              friction);

  // Physics constants
  constexpr double g = 9.81;
  constexpr double dt = 0.016;
  constexpr double muG = friction * g;  // 4.905 m/s²

  // Continue to frame 30 to observe full sliding behavior
  step(250);  // Total: 30 frames
}

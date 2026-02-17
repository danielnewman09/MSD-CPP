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

  // Step 5 frames to let contact establish and observe friction behavior
  step(5);

  // After 5 frames, cube should have decelerated due to friction
  const auto& state5 = cube.getInertialState();
  const double vx = state5.velocity.x();
  const double vy = state5.velocity.y();
  const double vz = state5.velocity.z();

  std::cout << "\n=== Friction Cone Solver Diagnostic (Frame 5) ===\n";
  std::cout << "Velocity: (" << vx << ", " << vy << ", " << vz << ") m/s\n";
  std::cout << "Speed (x): " << vx << " m/s\n";

  // Expected velocity after 5 frames of mu*g deceleration:
  // v = v0 - mu*g*t = 2.0 - 4.905 * 5 * 0.016 = 2.0 - 0.3924 = 1.6076 m/s
  const double expectedVx = initialVelocityX - muG * 5.0 * dt;
  std::cout << "Expected vx: " << expectedVx << " m/s\n";
  std::cout << "Actual vx:   " << vx << " m/s\n";
  std::cout << "Difference:  " << (vx - expectedVx) << " m/s\n";

  // The cube should be decelerating. If friction is undersaturated,
  // the cube decelerates less than expected.
  EXPECT_GT(vx, 0.0) << "Cube should still be moving forward after 5 frames";
  EXPECT_LT(vx, initialVelocityX)
    << "Cube should have decelerated from friction";

  // Check that deceleration is close to mu*g (within 20% tolerance for now)
  // If the friction solver undersaturates, we'll see significantly less
  // deceleration
  const double actualDecel = (initialVelocityX - vx) / (5.0 * dt);
  std::cout << "Expected deceleration: " << muG << " m/s²\n";
  std::cout << "Actual deceleration:   " << actualDecel << " m/s²\n";
  std::cout << "Ratio (actual/expected): " << (actualDecel / muG) << "\n";

  // This is the key assertion: deceleration should be close to mu*g
  // If the friction cone solver undersaturates, this ratio will be < 1.0
  EXPECT_NEAR(actualDecel, muG, muG * 0.20)
    << "Friction deceleration should be mu*g = " << muG << " m/s².\n"
    << "If significantly less, the friction cone solver is undersaturating.";

  // Continue to frame 30 to observe full sliding behavior
  step(25);  // Total: 30 frames

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

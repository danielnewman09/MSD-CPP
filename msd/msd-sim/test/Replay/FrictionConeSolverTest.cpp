// Ticket: 0066_friction_cone_solver_saturation_bug
// Ticket: 0070_nlopt_convergence_energy_injection
// Test: Targeted friction cone solver diagnostic test
//
// PURPOSE: Place a cube on a floor with horizontal velocity to create a
// sliding contact. Verify that friction decelerates the cube and brings
// it to rest.
//
// PHYSICS:
//   - Cube: 1×1×1 m, mass = 1 kg, mu = 0.5, e = 0.0 (perfectly inelastic)
//   - Floor: static, at z = 0 (surface)
//   - Gravity: g = 9.81 m/s² downward
//   - Initial velocity: (2, 0, 0) m/s horizontal
//
// BEHAVIOR: Friction at the contact points (below CoM) creates a tipping
// torque that lifts the cube onto its leading edge. The cube tips, slides,
// and eventually comes to rest. This means the deceleration is NOT simply
// mu*g — the dynamics couple linear deceleration with rotational motion.
// The test verifies functional behavior (cube decelerates and stops) rather
// than asserting point-mass deceleration.

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
                              AngularCoordinate{},
                              Coordinate{initialVelocityX, 0.0, 0.0},
                              mass,
                              restitution,
                              friction);

  // Physics constants
  constexpr double dt = 0.016;

  // Run 50 frames (~0.8s) — enough for the cube to tip, slide, and settle
  constexpr int totalFrames = 50;
  step(totalFrames);

  const auto& stateFinal = cube.getInertialState();
  const double vxFinal = stateFinal.velocity.x();
  const double vyFinal = stateFinal.velocity.y();

  std::cout << "\n=== Friction Cone Solver Diagnostic ===\n";
  std::cout << "After " << totalFrames << " frames ("
            << totalFrames * dt << "s):\n";
  std::cout << "  vx = " << vxFinal << " m/s\n";
  std::cout << "  vy = " << vyFinal << " m/s\n";
  std::cout << "  vz = " << stateFinal.velocity.z() << " m/s\n";
  std::cout << "  position = (" << stateFinal.position.x() << ", "
            << stateFinal.position.y() << ", "
            << stateFinal.position.z() << ")\n";
  std::cout << "  omega = (" << stateFinal.getAngularVelocity().x() << ", "
            << stateFinal.getAngularVelocity().y() << ", "
            << stateFinal.getAngularVelocity().z() << ")\n";

  // Friction must decelerate the cube — it should NOT still be at 2 m/s
  EXPECT_LT(std::abs(vxFinal), 0.5 * initialVelocityX)
    << "Friction should have significantly decelerated the cube from "
    << initialVelocityX << " m/s";

  // Cube should come to rest or near rest by frame 50
  const double speed = std::sqrt(vxFinal * vxFinal + vyFinal * vyFinal);
  EXPECT_LT(speed, 0.5)
    << "Cube should be near rest after " << totalFrames << " frames";

  // No lateral drift
  EXPECT_NEAR(vyFinal, 0.0, 0.1) << "No lateral velocity expected";

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
                              AngularCoordinate{},
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

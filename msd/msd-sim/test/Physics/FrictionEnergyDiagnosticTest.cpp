// DEBUG TICKET: DEBUG_0035d_friction_energy_injection
// Phase 1: Reproduce and Characterize — Per-step instrumentation

#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

namespace
{

std::vector<Coordinate> createCubePoints(double size)
{
  double half = size / 2.0;
  return {Coordinate{-half, -half, -half},
          Coordinate{half, -half, -half},
          Coordinate{half, half, -half},
          Coordinate{-half, half, -half},
          Coordinate{-half, -half, half},
          Coordinate{half, -half, half},
          Coordinate{half, half, half},
          Coordinate{-half, half, half}};
}

}  // anonymous namespace

// ============================================================================
// Phase 1 Diagnostic: Full per-step state trace for sliding block
// ============================================================================

TEST(FrictionEnergyDiagnostic, PerStepStateTrace)
{
  WorldModel world;

  auto floorPoints = createCubePoints(10.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -5.0}};
  world.spawnEnvironmentObject(0, floorHull, floorFrame);

  auto blockPoints = createCubePoints(1.0);
  ConvexHull blockHull{blockPoints};
  ReferenceFrame blockFrame{Coordinate{0.0, 0.0, 0.49}};
  world.spawnObject(1, blockHull, blockFrame);

  uint32_t blockId = 1;
  AssetInertial& block = world.getObject(blockId);

  block.setFrictionCoefficient(0.5);
  block.setCoefficientOfRestitution(0.0);
  block.getInertialState().velocity = Coordinate{10.0, 0.0, 0.0};

  double mass = block.getMass();

  std::cerr << std::fixed << std::setprecision(6);
  std::cerr << "\n=== FRICTION ENERGY DIAGNOSTIC: Per-Step State Trace ===\n";
  std::cerr << "Initial: pos=(" << block.getInertialState().position.x()
            << ", " << block.getInertialState().position.y()
            << ", " << block.getInertialState().position.z()
            << ") vel=(" << block.getInertialState().velocity.x()
            << ", " << block.getInertialState().velocity.y()
            << ", " << block.getInertialState().velocity.z()
            << ") mass=" << mass << "\n\n";

  std::cerr << "Step | vx       | vy       | vz       | pos_z    | E_lat    | E_total  | lateral_spd\n";
  std::cerr << "-----|----------|----------|----------|----------|----------|----------|----------\n";

  const int numSteps = 20;
  double prevELateral = 0.5 * mass * 10.0 * 10.0;  // Initial lateral KE

  for (int i = 0; i < numSteps; ++i)
  {
    world.update(std::chrono::milliseconds{16 * (i + 1)});

    const auto& vel = block.getInertialState().velocity;
    const auto& pos = block.getInertialState().position;

    double lateralSpeed = std::sqrt(vel.x() * vel.x() + vel.y() * vel.y());
    double E_lateral = 0.5 * mass * lateralSpeed * lateralSpeed;
    double E_total = 0.5 * mass * vel.squaredNorm();

    std::cerr << std::setw(4) << (i + 1) << " | "
              << std::setw(8) << vel.x() << " | "
              << std::setw(8) << vel.y() << " | "
              << std::setw(8) << vel.z() << " | "
              << std::setw(8) << pos.z() << " | "
              << std::setw(8) << E_lateral << " | "
              << std::setw(8) << E_total << " | "
              << std::setw(8) << lateralSpeed;

    if (E_lateral > prevELateral + 1e-6)
    {
      std::cerr << " ** ENERGY INCREASE: +" << (E_lateral - prevELateral);
    }

    std::cerr << "\n";
    prevELateral = E_lateral;
  }

  std::cerr << "\n=== END DIAGNOSTIC ===\n\n";

  // No assertions — this test is purely for data collection
  SUCCEED();
}

// ============================================================================
// Phase 2 Diagnostic: Gravity-off isolation (H2/H3 test)
// ============================================================================

TEST(FrictionEnergyDiagnostic, GravityOffIsolation)
{
  WorldModel world;
  world.clearPotentialEnergies();  // Remove gravity

  auto floorPoints = createCubePoints(10.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -5.0}};
  world.spawnEnvironmentObject(0, floorHull, floorFrame);

  auto blockPoints = createCubePoints(1.0);
  ConvexHull blockHull{blockPoints};
  ReferenceFrame blockFrame{Coordinate{0.0, 0.0, 0.49}};
  world.spawnObject(1, blockHull, blockFrame);

  uint32_t blockId = 1;
  AssetInertial& block = world.getObject(blockId);

  block.setFrictionCoefficient(0.5);
  block.setCoefficientOfRestitution(0.0);
  block.getInertialState().velocity = Coordinate{10.0, 0.0, 0.0};

  double mass = block.getMass();

  std::cerr << "\n=== GRAVITY-OFF ISOLATION ===\n";
  std::cerr << "Step | vx       | vy       | vz       | pos_z    | E_lat    | lateral_spd\n";
  std::cerr << "-----|----------|----------|----------|----------|----------|----------\n";

  const int numSteps = 20;

  for (int i = 0; i < numSteps; ++i)
  {
    world.update(std::chrono::milliseconds{16 * (i + 1)});

    const auto& vel = block.getInertialState().velocity;
    const auto& pos = block.getInertialState().position;

    double lateralSpeed = std::sqrt(vel.x() * vel.x() + vel.y() * vel.y());
    double E_lateral = 0.5 * mass * lateralSpeed * lateralSpeed;

    std::cerr << std::setw(4) << (i + 1) << " | "
              << std::setw(8) << vel.x() << " | "
              << std::setw(8) << vel.y() << " | "
              << std::setw(8) << vel.z() << " | "
              << std::setw(8) << pos.z() << " | "
              << std::setw(8) << E_lateral << " | "
              << std::setw(8) << lateralSpeed << "\n";
  }

  std::cerr << "\n=== END GRAVITY-OFF ===\n\n";
  SUCCEED();
}

// ============================================================================
// Phase 2 Diagnostic: Friction-off isolation (H1/H3 test)
// ============================================================================

TEST(FrictionEnergyDiagnostic, FrictionOffIsolation)
{
  WorldModel world;

  auto floorPoints = createCubePoints(10.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -5.0}};
  world.spawnEnvironmentObject(0, floorHull, floorFrame);

  auto blockPoints = createCubePoints(1.0);
  ConvexHull blockHull{blockPoints};
  ReferenceFrame blockFrame{Coordinate{0.0, 0.0, 0.49}};
  world.spawnObject(1, blockHull, blockFrame);

  uint32_t blockId = 1;
  AssetInertial& block = world.getObject(blockId);

  block.setFrictionCoefficient(0.0);  // No friction
  block.setCoefficientOfRestitution(0.0);
  block.getInertialState().velocity = Coordinate{10.0, 0.0, 0.0};

  double mass = block.getMass();

  std::cerr << "\n=== FRICTION-OFF ISOLATION ===\n";
  std::cerr << "Step | vx       | vy       | vz       | pos_z    | E_lat    | lateral_spd\n";
  std::cerr << "-----|----------|----------|----------|----------|----------|----------\n";

  const int numSteps = 20;

  for (int i = 0; i < numSteps; ++i)
  {
    world.update(std::chrono::milliseconds{16 * (i + 1)});

    const auto& vel = block.getInertialState().velocity;
    const auto& pos = block.getInertialState().position;

    double lateralSpeed = std::sqrt(vel.x() * vel.x() + vel.y() * vel.y());
    double E_lateral = 0.5 * mass * lateralSpeed * lateralSpeed;

    std::cerr << std::setw(4) << (i + 1) << " | "
              << std::setw(8) << vel.x() << " | "
              << std::setw(8) << vel.y() << " | "
              << std::setw(8) << vel.z() << " | "
              << std::setw(8) << pos.z() << " | "
              << std::setw(8) << E_lateral << " | "
              << std::setw(8) << lateralSpeed << "\n";
  }

  std::cerr << "\n=== END FRICTION-OFF ===\n\n";
  SUCCEED();
}

// ============================================================================
// Phase 2 Diagnostic: No-gravity + No-friction baseline
// ============================================================================

TEST(FrictionEnergyDiagnostic, NoGravityNoFrictionBaseline)
{
  WorldModel world;
  world.clearPotentialEnergies();  // Remove gravity

  auto floorPoints = createCubePoints(10.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -5.0}};
  world.spawnEnvironmentObject(0, floorHull, floorFrame);

  auto blockPoints = createCubePoints(1.0);
  ConvexHull blockHull{blockPoints};
  ReferenceFrame blockFrame{Coordinate{0.0, 0.0, 0.49}};
  world.spawnObject(1, blockHull, blockFrame);

  uint32_t blockId = 1;
  AssetInertial& block = world.getObject(blockId);

  block.setFrictionCoefficient(0.0);  // No friction
  block.setCoefficientOfRestitution(0.0);
  block.getInertialState().velocity = Coordinate{10.0, 0.0, 0.0};

  double mass = block.getMass();

  std::cerr << "\n=== NO GRAVITY + NO FRICTION BASELINE ===\n";
  std::cerr << "Step | vx       | vy       | vz       | pos_z    | E_lat    | lateral_spd\n";
  std::cerr << "-----|----------|----------|----------|----------|----------|----------\n";

  const int numSteps = 20;

  for (int i = 0; i < numSteps; ++i)
  {
    world.update(std::chrono::milliseconds{16 * (i + 1)});

    const auto& vel = block.getInertialState().velocity;
    const auto& pos = block.getInertialState().position;

    double lateralSpeed = std::sqrt(vel.x() * vel.x() + vel.y() * vel.y());
    double E_lateral = 0.5 * mass * lateralSpeed * lateralSpeed;

    std::cerr << std::setw(4) << (i + 1) << " | "
              << std::setw(8) << vel.x() << " | "
              << std::setw(8) << vel.y() << " | "
              << std::setw(8) << vel.z() << " | "
              << std::setw(8) << pos.z() << " | "
              << std::setw(8) << E_lateral << " | "
              << std::setw(8) << lateralSpeed << "\n";
  }

  std::cerr << "\n=== END BASELINE ===\n\n";
  SUCCEED();
}

// ============================================================================
// Phase 2 Diagnostic: Single-step energy audit
// ============================================================================

TEST(FrictionEnergyDiagnostic, SingleStepEnergyAudit)
{
  WorldModel world;

  auto floorPoints = createCubePoints(10.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -5.0}};
  world.spawnEnvironmentObject(0, floorHull, floorFrame);

  auto blockPoints = createCubePoints(1.0);
  ConvexHull blockHull{blockPoints};
  ReferenceFrame blockFrame{Coordinate{0.0, 0.0, 0.49}};
  world.spawnObject(1, blockHull, blockFrame);

  uint32_t blockId = 1;
  AssetInertial& block = world.getObject(blockId);

  block.setFrictionCoefficient(0.5);
  block.setCoefficientOfRestitution(0.0);
  block.getInertialState().velocity = Coordinate{10.0, 0.0, 0.0};

  double mass = block.getMass();

  std::cerr << "\n=== SINGLE STEP ENERGY AUDIT ===\n";

  // Record pre-step state
  auto velBefore = block.getInertialState().velocity;
  auto posBefore = block.getInertialState().position;
  double E_before = 0.5 * mass * velBefore.squaredNorm();
  double E_lat_before = 0.5 * mass * (velBefore.x() * velBefore.x() + velBefore.y() * velBefore.y());

  std::cerr << "Before step 1:\n";
  std::cerr << "  pos = (" << posBefore.x() << ", " << posBefore.y() << ", " << posBefore.z() << ")\n";
  std::cerr << "  vel = (" << velBefore.x() << ", " << velBefore.y() << ", " << velBefore.z() << ")\n";
  std::cerr << "  E_total = " << E_before << " J\n";
  std::cerr << "  E_lateral = " << E_lat_before << " J\n\n";

  // Execute one step
  world.update(std::chrono::milliseconds{16});

  // Record post-step state
  auto velAfter = block.getInertialState().velocity;
  auto posAfter = block.getInertialState().position;
  double E_after = 0.5 * mass * velAfter.squaredNorm();
  double E_lat_after = 0.5 * mass * (velAfter.x() * velAfter.x() + velAfter.y() * velAfter.y());

  std::cerr << "After step 1:\n";
  std::cerr << "  pos = (" << posAfter.x() << ", " << posAfter.y() << ", " << posAfter.z() << ")\n";
  std::cerr << "  vel = (" << velAfter.x() << ", " << velAfter.y() << ", " << velAfter.z() << ")\n";
  std::cerr << "  E_total = " << E_after << " J\n";
  std::cerr << "  E_lateral = " << E_lat_after << " J\n\n";

  std::cerr << "Delta:\n";
  std::cerr << "  dE_total = " << (E_after - E_before) << " J\n";
  std::cerr << "  dE_lateral = " << (E_lat_after - E_lat_before) << " J\n";
  std::cerr << "  dv = (" << (velAfter.x() - velBefore.x())
            << ", " << (velAfter.y() - velBefore.y())
            << ", " << (velAfter.z() - velBefore.z()) << ")\n";
  std::cerr << "  dx = (" << (posAfter.x() - posBefore.x())
            << ", " << (posAfter.y() - posBefore.y())
            << ", " << (posAfter.z() - posBefore.z()) << ")\n";

  // Expected friction deceleration: a = -mu * g = -0.5 * 9.81 = -4.905 m/s^2
  // Expected dv_x = a * dt = -4.905 * 0.016 = -0.07848 m/s
  double expectedDvx = -0.5 * 9.81 * 0.016;
  std::cerr << "\nExpected (analytical):\n";
  std::cerr << "  dv_x = " << expectedDvx << " m/s (from mu*g*dt)\n";
  std::cerr << "  actual dv_x = " << (velAfter.x() - velBefore.x()) << " m/s\n";

  std::cerr << "\n=== END SINGLE STEP AUDIT ===\n\n";
  SUCCEED();
}

// Ticket: 0075-unified-contact-constraint
//
// Shared multi-body collision scenario setup used by both benchmarks and
// recording tools. Single source of truth — avoids scenario divergence.

#pragma once

#include <chrono>
#include <cmath>
#include <random>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

namespace msd_sim::bench
{

// ============================================================================
// Constants
// ============================================================================

constexpr double kCubeHalfSize = 0.5;         // Unit cube half-extent [m]
constexpr double kFloorHalfSize = 100.0;      // Floor half-extent [m]
constexpr double kMass = 10.0;                // Default mass [kg]
constexpr double kRestitution = 0.5;          // Default restitution
constexpr double kFriction = 0.5;             // Default friction
constexpr unsigned int kRandomSeed = 42;      // Fixed seed for reproducibility

// ============================================================================
// Setup struct
// ============================================================================

// Owns all hulls and WorldModel for a scenario.
// ConvexHulls must outlive assets that reference them.
struct MultiBodySetup
{
  std::vector<ConvexHull> hulls;
  WorldModel world;
  std::chrono::milliseconds simTime{0};

  void stepFrames(int frames)
  {
    for (int f = 0; f < frames; ++f)
    {
      simTime += std::chrono::milliseconds{16};
      world.update(simTime);
    }
  }
};

// ============================================================================
// Helpers
// ============================================================================

inline std::vector<Coordinate> createCubePoints(double halfSize)
{
  return {Coordinate{-halfSize, -halfSize, -halfSize},
          Coordinate{halfSize, -halfSize, -halfSize},
          Coordinate{halfSize, halfSize, -halfSize},
          Coordinate{-halfSize, halfSize, -halfSize},
          Coordinate{-halfSize, -halfSize, halfSize},
          Coordinate{halfSize, -halfSize, halfSize},
          Coordinate{halfSize, halfSize, halfSize},
          Coordinate{-halfSize, halfSize, halfSize}};
}

// Spawn a floor at z=0 (hull centered at z=-100, top face at z=0)
inline void spawnFloor(MultiBodySetup& setup)
{
  setup.hulls.emplace_back(createCubePoints(kFloorHalfSize));
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -kFloorHalfSize}};
  setup.world.spawnEnvironmentObject(0, setup.hulls.back(), floorFrame);
}

// ============================================================================
// Scenario setup functions
// ============================================================================

// N cubes in a vertical stack with slight random offsets making it unstable.
// Stack collapses and bodies settle — sustained multi-contact resting + tumbling.
inline void setupStackCollapse(MultiBodySetup& setup, int numBodies)
{
  setup.hulls.reserve(static_cast<size_t>(numBodies) + 1);
  spawnFloor(setup);

  std::mt19937 rng{kRandomSeed};
  std::uniform_real_distribution<double> offsetDist{-0.15, 0.15};

  for (int i = 0; i < numBodies; ++i)
  {
    setup.hulls.emplace_back(createCubePoints(kCubeHalfSize));
    double const stackZ =
      kCubeHalfSize + static_cast<double>(i) * 2.0 * kCubeHalfSize;
    ReferenceFrame frame{
      Coordinate{offsetDist(rng), offsetDist(rng), stackZ}};

    setup.world.spawnObject(
      0, setup.hulls.back(), kMass, frame, kRestitution, kFriction);
  }
}

// N cubes spawned in a tight random cluster above the floor.
// Each has a small random velocity. Creates dense body-body + body-floor contacts.
inline void setupClusterDrop(MultiBodySetup& setup, int numBodies)
{
  setup.hulls.reserve(static_cast<size_t>(numBodies) + 1);
  spawnFloor(setup);

  double const clusterRadius = kCubeHalfSize * 2.0 *
                               std::cbrt(static_cast<double>(numBodies));

  std::mt19937 rng{kRandomSeed};
  std::uniform_real_distribution<double> posDist{-clusterRadius, clusterRadius};
  std::uniform_real_distribution<double> velDist{-1.0, 1.0};

  for (int i = 0; i < numBodies; ++i)
  {
    setup.hulls.emplace_back(createCubePoints(kCubeHalfSize));
    double const dropHeight = 3.0 + kCubeHalfSize;
    ReferenceFrame frame{
      Coordinate{posDist(rng), posDist(rng), dropHeight + std::abs(posDist(rng))}};

    const auto& asset = setup.world.spawnObject(
      0, setup.hulls.back(), kMass, frame, kRestitution, kFriction);
    uint32_t id = asset.getInstanceId();
    setup.world.getObject(id).getInertialState().velocity =
      Velocity{velDist(rng), velDist(rng), velDist(rng)};
  }
}

// N cubes in a grid pattern settling onto the floor.
// Many parallel body-floor contacts, minimal body-body interaction.
inline void setupGridSettle(MultiBodySetup& setup, int numBodies)
{
  setup.hulls.reserve(static_cast<size_t>(numBodies) + 1);
  spawnFloor(setup);

  int const gridSide = static_cast<int>(std::ceil(std::sqrt(numBodies)));
  double const spacing = 3.0 * kCubeHalfSize * 2.0;
  double const gridOffset =
    -static_cast<double>(gridSide - 1) * spacing / 2.0;

  int spawned = 0;
  for (int row = 0; row < gridSide && spawned < numBodies; ++row)
  {
    for (int col = 0; col < gridSide && spawned < numBodies; ++col)
    {
      setup.hulls.emplace_back(createCubePoints(kCubeHalfSize));
      double const dropHeight = 2.0 + kCubeHalfSize;
      ReferenceFrame frame{Coordinate{
        gridOffset + static_cast<double>(col) * spacing,
        gridOffset + static_cast<double>(row) * spacing,
        dropHeight}};

      setup.world.spawnObject(
        0, setup.hulls.back(), kMass, frame, kRestitution, kFriction);
      ++spawned;
    }
  }
}

}  // namespace msd_sim::bench

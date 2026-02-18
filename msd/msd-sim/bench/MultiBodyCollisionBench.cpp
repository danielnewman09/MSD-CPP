// Ticket: 0071_collision_pipeline_profiling
//
// Multi-body collision benchmarks exercising the full WorldModel simulation loop
// with dense body-body interactions. Uses gravity, integration, collision detection,
// constraint solving, and position correction — no DataRecorder overhead.

#include <benchmark/benchmark.h>

#include <chrono>
#include <random>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

// ============================================================================
// Helpers
// ============================================================================

namespace
{

constexpr int kFramesPerIteration = 20;       // Frames to step per benchmark iteration
constexpr double kCubeHalfSize = 0.5;         // Unit cube half-extent [m]
constexpr double kFloorHalfSize = 100.0;      // Floor half-extent [m]
constexpr double kMass = 10.0;                // Default mass [kg]
constexpr double kRestitution = 0.5;          // Default restitution
constexpr double kFriction = 0.5;             // Default friction
constexpr unsigned int kRandomSeed = 42;      // Fixed seed for reproducibility

std::vector<Coordinate> createCubePoints(double halfSize)
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

// Owns all hulls and WorldModel for a benchmark scenario.
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

// Spawn a floor at z=0 (hull centered at z=-100, top face at z=0)
void spawnFloor(MultiBodySetup& setup)
{
  setup.hulls.emplace_back(createCubePoints(kFloorHalfSize));
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -kFloorHalfSize}};
  setup.world.spawnEnvironmentObject(0, setup.hulls.back(), floorFrame);
}

}  // namespace

// ============================================================================
// Benchmark: ClusterDrop
// ============================================================================

// N cubes spawned in a tight random cluster above the floor.
// Each has a small random velocity. Creates dense body-body + body-floor contacts.
static void BM_MultiBody_ClusterDrop(benchmark::State& state)
{
  int const numBodies = static_cast<int>(state.range(0));

  for (auto _ : state)
  {
    state.PauseTiming();

    MultiBodySetup setup;
    // Reserve: N body hulls + 1 floor hull
    setup.hulls.reserve(static_cast<size_t>(numBodies) + 1);
    spawnFloor(setup);

    // Cluster radius scales with body count to maintain density
    double const clusterRadius = kCubeHalfSize * 2.0 *
                                 std::cbrt(static_cast<double>(numBodies));

    std::mt19937 rng{kRandomSeed};
    std::uniform_real_distribution<double> posDist{-clusterRadius, clusterRadius};
    std::uniform_real_distribution<double> velDist{-1.0, 1.0};

    // Spawn cubes above the floor in a random cluster
    for (int i = 0; i < numBodies; ++i)
    {
      setup.hulls.emplace_back(createCubePoints(kCubeHalfSize));
      double const dropHeight = 3.0 + kCubeHalfSize;
      ReferenceFrame frame{
        Coordinate{posDist(rng), posDist(rng), dropHeight + std::abs(posDist(rng))}};

      const auto& asset = setup.world.spawnObject(
        0, setup.hulls.back(), kMass, frame, kRestitution, kFriction);
      // Small random velocity to create body-body interactions
      uint32_t id = asset.getInstanceId();
      setup.world.getObject(id).getInertialState().velocity =
        Velocity{velDist(rng), velDist(rng), velDist(rng)};
    }

    state.ResumeTiming();

    setup.stepFrames(kFramesPerIteration);
    benchmark::DoNotOptimize(setup.world.getTime());
  }

  state.SetItemsProcessed(
    state.iterations() * kFramesPerIteration * numBodies);
}
BENCHMARK(BM_MultiBody_ClusterDrop)
  ->Arg(4)
  ->Arg(8)
  ->Arg(16)
  ->Arg(32)
  ->Unit(benchmark::kMillisecond);

// ============================================================================
// Benchmark: StackCollapse
// ============================================================================

// N cubes in a vertical stack with slight random offsets making it unstable.
// Stack collapses and bodies settle — sustained multi-contact resting + tumbling.
static void BM_MultiBody_StackCollapse(benchmark::State& state)
{
  int const numBodies = static_cast<int>(state.range(0));

  for (auto _ : state)
  {
    state.PauseTiming();

    MultiBodySetup setup;
    setup.hulls.reserve(static_cast<size_t>(numBodies) + 1);
    spawnFloor(setup);

    std::mt19937 rng{kRandomSeed};
    std::uniform_real_distribution<double> offsetDist{-0.15, 0.15};

    // Stack cubes vertically with slight lateral offsets
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

    state.ResumeTiming();

    setup.stepFrames(kFramesPerIteration);
    benchmark::DoNotOptimize(setup.world.getTime());
  }

  state.SetItemsProcessed(
    state.iterations() * kFramesPerIteration * numBodies);
}
BENCHMARK(BM_MultiBody_StackCollapse)
  ->Arg(4)
  ->Arg(8)
  ->Arg(16)
  ->Unit(benchmark::kMillisecond);

// ============================================================================
// Benchmark: GridSettle
// ============================================================================

// N cubes in a grid pattern settling onto the floor.
// Many parallel body-floor contacts, minimal body-body interaction.
// Baseline for comparing against dense body-body scenarios.
static void BM_MultiBody_GridSettle(benchmark::State& state)
{
  int const numBodies = static_cast<int>(state.range(0));

  for (auto _ : state)
  {
    state.PauseTiming();

    MultiBodySetup setup;
    setup.hulls.reserve(static_cast<size_t>(numBodies) + 1);
    spawnFloor(setup);

    // Arrange in a square grid with 3x cube-width spacing (no overlap)
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

    state.ResumeTiming();

    setup.stepFrames(kFramesPerIteration);
    benchmark::DoNotOptimize(setup.world.getTime());
  }

  state.SetItemsProcessed(
    state.iterations() * kFramesPerIteration * numBodies);
}
BENCHMARK(BM_MultiBody_GridSettle)
  ->Arg(4)
  ->Arg(9)
  ->Arg(16)
  ->Arg(25)
  ->Unit(benchmark::kMillisecond);

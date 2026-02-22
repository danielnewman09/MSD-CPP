// Ticket: 0071_collision_pipeline_profiling
//
// Multi-body collision benchmarks exercising the full WorldModel simulation loop
// with dense body-body interactions. Uses gravity, integration, collision detection,
// constraint solving, and position correction — no DataRecorder overhead.

#include <benchmark/benchmark.h>

#include "MultiBodyScenarios.hpp"

using namespace msd_sim::bench;

constexpr int kFramesPerIteration = 20;  // Frames to step per benchmark iteration

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
    setupClusterDrop(setup, numBodies);

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
    setupStackCollapse(setup, numBodies);

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
    setupGridSettle(setup, numBodies);

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

// Ticket: 0053_collision_pipeline_performance
//
// Benchmarks for the full collision response pipeline including friction solver,
// position correction, and warm-starting. Exercises code paths optimized in 0053.

#include <benchmark/benchmark.h>

#include <span>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/Collision/CollisionPipeline.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

// ============================================================================
// Helpers
// ============================================================================

namespace
{

constexpr double kDt = 1.0 / 60.0;          // 60 FPS timestep
constexpr double kPenetration = 0.01;        // 1cm overlap ensures collision
constexpr double kBodySpacing = 3.0;         // Spacing between bodies on floor
constexpr double kFloorHalfSize = 100.0;     // Floor half-extent
constexpr double kCubeHalfSize = 1.0;        // Unit cube half-extent
constexpr double kMass = 10.0;               // Default mass [kg]
constexpr double kRestitution = 0.5;         // Default restitution

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

// Owns hulls, assets, and pipeline for a benchmark scenario.
// ConvexHulls must outlive assets that reference them.
struct BenchSetup
{
  std::vector<ConvexHull> bodyHulls;
  ConvexHull floorHull;
  std::vector<AssetInertial> inertials;
  std::vector<AssetEnvironment> environments;
  CollisionPipeline pipeline;

  BenchSetup(int numBodies, double friction)
    : floorHull{createCubePoints(kFloorHalfSize)}
  {
    // Reserve hull storage so references stay valid
    bodyHulls.reserve(static_cast<size_t>(numBodies));

    // Create inertial bodies resting on floor with slight penetration
    // Floor top face is at z=0 (hull centered at origin, half-size = 100)
    // Body center at z = halfSize - penetration so bottom face slightly below z=0
    double bodyZ = kCubeHalfSize - kPenetration;

    for (int i = 0; i < numBodies; ++i)
    {
      bodyHulls.emplace_back(createCubePoints(kCubeHalfSize));
      ReferenceFrame frame{
        Coordinate{static_cast<double>(i) * kBodySpacing, 0.0, bodyZ}};
      inertials.emplace_back(
        0,
        static_cast<uint32_t>(i + 1),
        bodyHulls.back(),
        kMass,
        frame,
        kRestitution,
        friction);
    }

    // Create floor environment at origin (top face at z=0)
    ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -kFloorHalfSize}};
    environments.emplace_back(
      1, 100, floorHull, floorFrame, kRestitution, friction);
  }

  void runOneFrame()
  {
    pipeline.advanceFrame();
    pipeline.expireOldEntries();
    pipeline.execute(
      std::span{inertials},
      std::span<const AssetEnvironment>{environments},
      kDt);
  }
};

// Two dynamic cubes colliding with each other (no floor)
struct BodyBodySetup
{
  ConvexHull hullA;
  ConvexHull hullB;
  std::vector<AssetInertial> inertials;
  std::vector<AssetEnvironment> environments;  // empty
  CollisionPipeline pipeline;

  explicit BodyBodySetup(double friction)
    : hullA{createCubePoints(kCubeHalfSize)}
    , hullB{createCubePoints(kCubeHalfSize)}
  {
    // Two cubes overlapping along x-axis
    ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
    ReferenceFrame frameB{
      Coordinate{2.0 * kCubeHalfSize - kPenetration, 0.0, 0.0}};

    inertials.emplace_back(0, 1, hullA, kMass, frameA, kRestitution, friction);
    inertials.emplace_back(0, 2, hullB, kMass, frameB, kRestitution, friction);
  }

  void runOneFrame()
  {
    pipeline.advanceFrame();
    pipeline.expireOldEntries();
    pipeline.execute(
      std::span{inertials},
      std::span<const AssetEnvironment>{environments},
      kDt);
  }
};

}  // namespace

// ============================================================================
// Benchmarks
// ============================================================================

/**
 * @brief Full collision pipeline with friction enabled (mu=0.5).
 *
 * Exercises: GJK/EPA detection + ContactConstraint + FrictionConstraint +
 * FrictionConeSolver + PositionCorrector + ContactCache warm-start.
 *
 * First iteration is cold (no cache). Subsequent iterations warm-start.
 *
 * @ticket 0053_collision_pipeline_performance
 */
static void BM_CollisionPipeline_WithFriction(benchmark::State& state)
{
  int numBodies = static_cast<int>(state.range(0));
  BenchSetup setup{numBodies, 0.5};

  for (auto _ : state)
  {
    setup.runOneFrame();
    benchmark::DoNotOptimize(setup.pipeline.hadCollisions());
  }
}
BENCHMARK(BM_CollisionPipeline_WithFriction)
  ->Arg(1)
  ->Arg(2)
  ->Arg(4)
  ->Arg(8);

/**
 * @brief Full collision pipeline without friction (mu=0.0).
 *
 * Exercises: GJK/EPA detection + ContactConstraint + ASM solver
 * (ConstraintSolver) + PositionCorrector.
 *
 * Comparison baseline to isolate FrictionConeSolver overhead.
 *
 * @ticket 0053_collision_pipeline_performance
 */
static void BM_CollisionPipeline_NoFriction(benchmark::State& state)
{
  int numBodies = static_cast<int>(state.range(0));
  BenchSetup setup{numBodies, 0.0};

  for (auto _ : state)
  {
    setup.runOneFrame();
    benchmark::DoNotOptimize(setup.pipeline.hadCollisions());
  }
}
BENCHMARK(BM_CollisionPipeline_NoFriction)
  ->Arg(1)
  ->Arg(2)
  ->Arg(4)
  ->Arg(8);

/**
 * @brief Two dynamic cubes colliding (inertial-vs-inertial path).
 *
 * Exercises the body-body collision path with friction, where both bodies
 * have finite mass and inertia (no infinite-mass environment shortcuts).
 *
 * @ticket 0053_collision_pipeline_performance
 */
static void BM_CollisionPipeline_BodyBody(benchmark::State& state)
{
  BodyBodySetup setup{0.5};

  for (auto _ : state)
  {
    setup.runOneFrame();
    benchmark::DoNotOptimize(setup.pipeline.hadCollisions());
  }
}
BENCHMARK(BM_CollisionPipeline_BodyBody);

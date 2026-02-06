// Ticket: 0022_gjk_asset_physical_transform
// Design: docs/designs/0022_gjk_asset_physical_transform/design.md

#include <benchmark/benchmark.h>
#include <random>
#include <vector>
#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/Collision/GJK.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

// Generate random point cloud with fixed seed for reproducibility
std::vector<Coordinate> generateRandomPointCloud(size_t count)
{
  static std::mt19937 rng{42};  // Fixed seed for deterministic benchmarks
  std::uniform_real_distribution<double> dist{-10.0, 10.0};

  std::vector<Coordinate> points;
  points.reserve(count);
  for (size_t i = 0; i < count; ++i)
  {
    points.emplace_back(dist(rng), dist(rng), dist(rng));
  }
  return points;
}

// Generate random convex hull with specified vertex count
ConvexHull generateRandomHull(size_t vertexCount)
{
  auto points = generateRandomPointCloud(
    vertexCount * 2);  // More points to ensure target vertex count
  return ConvexHull{points};
}

}  // namespace

// ============================================================================
// Identity Transform Benchmarks (Baseline)
// ============================================================================

/**
 * @brief Benchmark GJK collision detection with identity transforms (baseline).
 *
 * This establishes the baseline performance for GJK without any transformation
 * overhead. Measures collision detection between two convex hulls positioned
 * at the origin with identity reference frames.
 *
 * @ticket 0022_gjk_asset_physical_transform
 */
static void BM_GJK_IdentityTransform(benchmark::State& state)
{
  size_t vertexCount = static_cast<size_t>(state.range(0));
  auto hullA = generateRandomHull(vertexCount);
  auto hullB = generateRandomHull(vertexCount);

  ReferenceFrame identityFrame{};
  AssetPhysical assetA{0, 0, hullA, identityFrame};
  AssetPhysical assetB{0, 1, hullB, identityFrame};

  for (auto _ : state)
  {
    bool result = gjkIntersects(assetA, assetB);
    benchmark::DoNotOptimize(result);
  }
  state.SetComplexityN(static_cast<long long>(vertexCount));
}
BENCHMARK(BM_GJK_IdentityTransform)
  ->Arg(10)  // Simple collision hull
  ->Arg(20)
  ->Arg(50)  // Typical collision hull
  ->Arg(100)
  ->Arg(200)
  ->Arg(500)  // Complex collision hull
  ->Arg(1000)
  ->Complexity();

// ============================================================================
// Transformed Collision Benchmarks
// ============================================================================

/**
 * @brief Benchmark GJK with non-trivial transformations.
 *
 * Measures the performance overhead of applying ReferenceFrame transformations
 * during GJK collision detection. Uses realistic transformations (translation +
 * rotation) to simulate actual gameplay scenarios.
 *
 * @ticket 0022_gjk_asset_physical_transform
 */
static void BM_GJK_TransformedCollision(benchmark::State& state)
{
  size_t vertexCount = static_cast<size_t>(state.range(0));
  auto hullA = generateRandomHull(vertexCount);
  auto hullB = generateRandomHull(vertexCount);

  // Non-trivial transformation: translation + rotation
  AngularCoordinate rotation{
    15.0 * M_PI / 180.0,  // pitch (radians)
    30.0 * M_PI / 180.0,  // roll (radians)
    45.0 * M_PI / 180.0   // yaw (radians)
  };

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.5, 0.5, 0.5}, rotation};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  for (auto _ : state)
  {
    bool result = gjkIntersects(assetA, assetB);
    benchmark::DoNotOptimize(result);
  }
  state.SetComplexityN(static_cast<long long>(vertexCount));
}
BENCHMARK(BM_GJK_TransformedCollision)
  ->Arg(10)
  ->Arg(20)
  ->Arg(50)
  ->Arg(100)
  ->Arg(200)
  ->Arg(500)
  ->Arg(1000)
  ->Complexity();

// ============================================================================
// Transformation Overhead Analysis
// ============================================================================

/**
 * @brief Benchmark translation-only transformation overhead.
 *
 * Isolates the cost of translation transformations by comparing against
 * identity transform baseline. This helps identify if translation is a
 * significant performance factor.
 *
 * @ticket 0022_gjk_asset_physical_transform
 */
static void BM_GJK_TranslationOnly(benchmark::State& state)
{
  size_t vertexCount = static_cast<size_t>(state.range(0));
  auto hullA = generateRandomHull(vertexCount);
  auto hullB = generateRandomHull(vertexCount);

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{10.0, 20.0, 30.0}};  // Translation only

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  for (auto _ : state)
  {
    bool result = gjkIntersects(assetA, assetB);
    benchmark::DoNotOptimize(result);
  }
  state.SetComplexityN(static_cast<long long>(vertexCount));
}
BENCHMARK(BM_GJK_TranslationOnly)
  ->Arg(10)
  ->Arg(50)
  ->Arg(100)
  ->Arg(500)
  ->Arg(1000)
  ->Complexity();

/**
 * @brief Benchmark rotation-only transformation overhead.
 *
 * Isolates the cost of rotation transformations. This is expected to be the
 * dominant factor since rotation requires matrix-vector multiplication.
 *
 * @ticket 0022_gjk_asset_physical_transform
 */
static void BM_GJK_RotationOnly(benchmark::State& state)
{
  size_t vertexCount = static_cast<size_t>(state.range(0));
  auto hullA = generateRandomHull(vertexCount);
  auto hullB = generateRandomHull(vertexCount);

  AngularCoordinate rotation{
    30.0 * M_PI / 180.0,  // pitch (radians)
    45.0 * M_PI / 180.0,  // roll (radians)
    60.0 * M_PI / 180.0   // yaw (radians)
  };

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.0, 0.0, 0.0}, rotation};  // Rotation only

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  for (auto _ : state)
  {
    bool result = gjkIntersects(assetA, assetB);
    benchmark::DoNotOptimize(result);
  }
  state.SetComplexityN(static_cast<long long>(vertexCount));
}
BENCHMARK(BM_GJK_RotationOnly)
  ->Arg(10)
  ->Arg(50)
  ->Arg(100)
  ->Arg(500)
  ->Arg(1000)
  ->Complexity();

// ============================================================================
// Extreme Cases
// ============================================================================

/**
 * @brief Benchmark large translation offsets for numerical stability.
 *
 * Verifies that GJK maintains performance with large translation offsets
 * (objects far from origin), which tests numerical stability of transformation
 * pipeline.
 *
 * @ticket 0022_gjk_asset_physical_transform
 */
static void BM_GJK_LargeTranslationOffset(benchmark::State& state)
{
  size_t vertexCount = static_cast<size_t>(state.range(0));
  auto hullA = generateRandomHull(vertexCount);
  auto hullB = generateRandomHull(vertexCount);

  ReferenceFrame frameA{Coordinate{1000.0, 2000.0, 3000.0}};
  ReferenceFrame frameB{Coordinate{1000.5, 2000.5, 3000.5}};

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  for (auto _ : state)
  {
    bool result = gjkIntersects(assetA, assetB);
    benchmark::DoNotOptimize(result);
  }
  state.SetComplexityN(static_cast<long long>(vertexCount));
}
BENCHMARK(BM_GJK_LargeTranslationOffset)
  ->Arg(10)
  ->Arg(50)
  ->Arg(100)
  ->Arg(500)
  ->Arg(1000)
  ->Complexity();

/**
 * @brief Benchmark non-colliding case (early termination).
 *
 * Measures GJK performance when objects are well-separated. This tests the
 * efficiency of the bounding box early-out optimization and simplex convergence
 * for non-colliding cases.
 *
 * @ticket 0022_gjk_asset_physical_transform
 */
static void BM_GJK_NonColliding(benchmark::State& state)
{
  size_t vertexCount = static_cast<size_t>(state.range(0));
  auto hullA = generateRandomHull(vertexCount);
  auto hullB = generateRandomHull(vertexCount);

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{100.0, 0.0, 0.0}};  // Far apart

  AssetPhysical assetA{0, 0, hullA, frameA};
  AssetPhysical assetB{0, 1, hullB, frameB};

  for (auto _ : state)
  {
    bool result = gjkIntersects(assetA, assetB);
    benchmark::DoNotOptimize(result);
  }
  state.SetComplexityN(static_cast<long long>(vertexCount));
}
BENCHMARK(BM_GJK_NonColliding)
  ->Arg(10)
  ->Arg(50)
  ->Arg(100)
  ->Arg(500)
  ->Arg(1000)
  ->Complexity();

BENCHMARK_MAIN();

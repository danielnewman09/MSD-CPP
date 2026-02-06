// Ticket: 0011_add_google_benchmark
// Design: docs/designs/0011_add_google_benchmark/design.md

#include <benchmark/benchmark.h>
#include <random>
#include <vector>
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

// Create a simple cube as a point cloud
std::vector<Coordinate> createCubePoints(double size)
{
  double half = size / 2.0;
  return {Coordinate(-half, -half, -half),
          Coordinate(half, -half, -half),
          Coordinate(half, half, -half),
          Coordinate(-half, half, -half),
          Coordinate(-half, -half, half),
          Coordinate(half, -half, half),
          Coordinate(half, half, half),
          Coordinate(-half, half, half)};
}

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

}  // namespace

// ============================================================================
// Benchmarks
// ============================================================================

/**
 * @brief Benchmark ConvexHull construction from point clouds
 *
 * This is the primary bottleneck in collision detection setup. Measures
 * the time to compute the convex hull (vertex extraction and facet generation)
 * from a point cloud of varying sizes.
 *
 * @ticket 0011_add_google_benchmark
 */
static void BM_ConvexHull_Construction(benchmark::State& state)
{
  auto points = generateRandomPointCloud(static_cast<size_t>(state.range(0)));
  for (auto _ : state)
  {
    ConvexHull hull{points};
    benchmark::DoNotOptimize(hull);
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_ConvexHull_Construction)
  ->Args({8})     // Cube (minimum convex hull)
  ->Args({64})    // Small collision mesh
  ->Args({512})   // Medium complexity mesh
  ->Args({4096})  // High-detail collision geometry
  ->Complexity();

/**
 * @brief Benchmark point containment queries
 *
 * Point containment is a hot path in collision detection where we need to
 * quickly determine if a point lies inside a convex hull. This benchmark
 * measures the performance of the contains() method with an interior point.
 *
 * @ticket 0011_add_google_benchmark
 */
static void BM_ConvexHull_Contains(benchmark::State& state)
{
  auto hull = ConvexHull{createCubePoints(2.0)};
  Coordinate testPoint{0.5, 0.5, 0.5};  // Interior point
  for (auto _ : state)
  {
    bool result = hull.contains(testPoint);
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(BM_ConvexHull_Contains);

/**
 * @brief Benchmark signed distance calculations
 *
 * Signed distance queries are used for proximity detection and separation
 * distance computation. This benchmark measures the performance of the
 * signedDistance() method with an exterior point.
 *
 * @ticket 0011_add_google_benchmark
 */
static void BM_ConvexHull_SignedDistance(benchmark::State& state)
{
  auto hull = ConvexHull{createCubePoints(2.0)};
  Coordinate testPoint{3.0, 0.0, 0.0};  // Exterior point
  for (auto _ : state)
  {
    double distance = hull.signedDistance(testPoint);
    benchmark::DoNotOptimize(distance);
  }
}
BENCHMARK(BM_ConvexHull_SignedDistance);

/**
 * @brief Benchmark GJK intersection tests
 *
 * GJK-based intersection testing is the core of collision detection. This
 * benchmark measures the performance of gjkIntersects() with two overlapping
 * AssetPhysical objects using identity transforms.
 *
 * @ticket 0011_add_google_benchmark
 * @ticket 0022_gjk_asset_physical_transform
 */
static void BM_ConvexHull_Intersects(benchmark::State& state)
{
  auto hullA = ConvexHull{createCubePoints(2.0)};
  auto hullB = ConvexHull{createCubePoints(1.0)};  // Overlapping cube
  ReferenceFrame identityFrame{};
  AssetPhysical assetA{0, 0, hullA, identityFrame};
  AssetPhysical assetB{0, 1, hullB, identityFrame};
  for (auto _ : state)
  {
    bool result = gjkIntersects(assetA, assetB);
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(BM_ConvexHull_Intersects);

// Google Benchmark provides main() via benchmark::benchmark_main

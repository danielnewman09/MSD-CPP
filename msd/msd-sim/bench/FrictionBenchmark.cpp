// Ticket: 0035d_friction_hardening_and_validation
// Design: docs/designs/0035d_friction_hardening_and_validation/design.md

#include <benchmark/benchmark.h>
#include <memory>
#include <vector>
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace {

/**
 * @brief Create a unit cube convex hull centered at origin.
 *
 * Creates a 1m × 1m × 1m cube for use in benchmark scenarios.
 *
 * @return ConvexHull representing unit cube
 */
ConvexHull createUnitCube() {
  std::vector<Coordinate> vertices{};
  vertices.reserve(8);

  // Cube vertices: all combinations of ±0.5 in each dimension
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 2; ++k) {
        double x = (i == 0) ? -0.5 : 0.5;
        double y = (j == 0) ? -0.5 : 0.5;
        double z = (k == 0) ? -0.5 : 0.5;
        vertices.emplace_back(x, y, z);
      }
    }
  }

  return ConvexHull{vertices};
}

/**
 * @brief Create an AssetInertial object with unit mass.
 *
 * @param assetId Asset identifier
 * @param instanceId Instance identifier
 * @param position Initial position in world space
 * @return AssetInertial object ready for simulation
 */
AssetInertial createCube(int assetId, int instanceId, const Coordinate& position) {
  auto hull = createUnitCube();
  ReferenceFrame frame{position};
  return AssetInertial{assetId, instanceId, hull, frame};
}

/**
 * @brief Create a static floor (infinite mass) for contact scenarios.
 *
 * @param assetId Asset identifier
 * @param instanceId Instance identifier
 * @param height Floor height in Z coordinate
 * @return AssetEnvironment representing static floor
 */
AssetEnvironment createFloor(int assetId, int instanceId, double height) {
  // Floor is a thin rectangular slab (10m × 10m × 0.1m)
  std::vector<Coordinate> vertices{};
  vertices.reserve(8);

  double halfWidth = 5.0;
  double halfThickness = 0.05;

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 2; ++k) {
        double x = (i == 0) ? -halfWidth : halfWidth;
        double y = (j == 0) ? -halfWidth : halfWidth;
        double z = height + ((k == 0) ? -halfThickness : halfThickness);
        vertices.emplace_back(x, y, z);
      }
    }
  }

  ConvexHull floorHull{vertices};
  ReferenceFrame frame{Coordinate{0.0, 0.0, height}};

  return AssetEnvironment{assetId, instanceId, floorHull, frame};
}

/**
 * @brief Create normal-only contact constraints between cubes and floor.
 *
 * Generates one ContactConstraint per cube positioned just touching the floor.
 * Friction coefficient set to 0.0 (frictionless).
 *
 * @param cubes Dynamic cube objects
 * @param floor Static floor object
 * @return Vector of ContactConstraint unique pointers (normal constraints only)
 */
std::vector<std::unique_ptr<ContactConstraint>> createNormalOnlyContacts(
    const std::vector<AssetInertial>& cubes,
    const AssetEnvironment& floor) {
  std::vector<std::unique_ptr<ContactConstraint>> constraints{};
  constraints.reserve(cubes.size());

  Coordinate contactNormal{0.0, 0.0, 1.0};  // Vertical normal (floor pushes up)
  double penetrationDepth = 0.01;           // Small penetration to trigger constraint
  double restitution = 0.0;                 // Perfectly inelastic
  double frictionCoeff = 0.0;               // Frictionless

  for (const auto& cube : cubes) {
    Coordinate contactPointA = cube.getReferenceFrame().position() + Coordinate{0.0, 0.0, -0.5};
    Coordinate contactPointB = contactPointA;

    auto constraint = ContactConstraintFactory::createContactConstraint(
        cube, floor,
        contactNormal, penetrationDepth,
        contactPointA, contactPointB,
        restitution, frictionCoeff
    );

    constraints.push_back(std::move(constraint));
  }

  return constraints;
}

/**
 * @brief Create friction contact constraints between cubes and floor.
 *
 * Generates FrictionConstraint instances (3 constraint rows per contact point:
 * 1 normal + 2 tangential) for each cube-floor contact.
 *
 * @param cubes Dynamic cube objects
 * @param floor Static floor object
 * @return Vector of ContactConstraint unique pointers (with friction)
 */
std::vector<std::unique_ptr<ContactConstraint>> createFrictionContacts(
    const std::vector<AssetInertial>& cubes,
    const AssetEnvironment& floor) {
  std::vector<std::unique_ptr<ContactConstraint>> constraints{};
  constraints.reserve(cubes.size());

  Coordinate contactNormal{0.0, 0.0, 1.0};  // Vertical normal
  double penetrationDepth = 0.01;           // Small penetration
  double restitution = 0.0;                 // Perfectly inelastic
  double frictionCoeff = 0.5;               // Moderate friction

  for (const auto& cube : cubes) {
    Coordinate contactPointA = cube.getReferenceFrame().position() + Coordinate{0.0, 0.0, -0.5};
    Coordinate contactPointB = contactPointA;

    auto constraint = ContactConstraintFactory::createContactConstraint(
        cube, floor,
        contactNormal, penetrationDepth,
        contactPointA, contactPointB,
        restitution, frictionCoeff
    );

    constraints.push_back(std::move(constraint));
  }

  return constraints;
}

}  // namespace

// ============================================================================
// Normal-Only Baseline Benchmarks
// ============================================================================

/**
 * @brief Benchmark normal-only contact constraint solving (baseline).
 *
 * This establishes the baseline performance for contact solving without friction.
 * Each contact generates 1 constraint row (normal direction only).
 * Parameterized by number of contacts (2, 5, 10).
 *
 * @ticket 0035d_friction_hardening_and_validation
 */
static void BM_SolveNormalOnly(benchmark::State& state) {
  size_t numCubes = static_cast<size_t>(state.range(0));

  // Setup: Create cubes in a grid pattern falling onto floor
  std::vector<AssetInertial> cubes{};
  cubes.reserve(numCubes);

  int assetId = 1;
  for (size_t i = 0; i < numCubes; ++i) {
    int instanceId = static_cast<int>(i) + 1;
    double x = static_cast<double>(i % 5);     // Grid spacing: 1m
    double y = static_cast<double>(i / 5);
    double z = 2.0;                             // Height above floor
    cubes.push_back(createCube(assetId, instanceId, Coordinate{x, y, z}));

    // Set downward velocity (falling)
    cubes.back().getInertialState().velocity = Coordinate{0.0, 0.0, -1.0};
  }

  // Create static floor
  AssetEnvironment floor = createFloor(2, 100, 0.0);

  // Create normal-only contact constraints
  auto constraints = createNormalOnlyContacts(cubes, floor);

  // Convert unique_ptr vector to raw pointer vector for ConstraintSolver
  std::vector<ContactConstraint*> constraintPtrs{};
  constraintPtrs.reserve(constraints.size());
  for (const auto& c : constraints) {
    constraintPtrs.push_back(c.get());
  }

  // Create solver
  ConstraintSolver solver{};
  solver.setMaxSafetyIterations(100);
  solver.setConvergenceTolerance(1e-6);

  // Benchmark loop
  for (auto _ : state) {
    auto result = solver.solveWithContacts(cubes, floor, constraintPtrs);
    benchmark::DoNotOptimize(result);
  }

  state.SetComplexityN(static_cast<long long>(numCubes));
}
BENCHMARK(BM_SolveNormalOnly)
    ->Arg(2)      // 2 contacts (minimal scenario)
    ->Arg(5)      // 5 contacts
    ->Arg(10)     // 10 contacts (typical scenario)
    ->Complexity();

// ============================================================================
// Friction-Enabled Benchmarks
// ============================================================================

/**
 * @brief Benchmark friction contact constraint solving.
 *
 * Measures the performance overhead of friction constraints. Each contact
 * generates 3 constraint rows (1 normal + 2 tangential), resulting in 3x
 * more constraints than normal-only baseline.
 * Target: < 2x wall time compared to BM_SolveNormalOnly.
 *
 * @ticket 0035d_friction_hardening_and_validation
 */
static void BM_SolveFriction(benchmark::State& state) {
  size_t numCubes = static_cast<size_t>(state.range(0));

  // Setup: Create cubes with lateral velocity (sliding friction)
  std::vector<AssetInertial> cubes{};
  cubes.reserve(numCubes);

  int assetId = 1;
  for (size_t i = 0; i < numCubes; ++i) {
    int instanceId = static_cast<int>(i) + 1;
    double x = static_cast<double>(i % 5);
    double y = static_cast<double>(i / 5);
    double z = 2.0;
    cubes.push_back(createCube(assetId, instanceId, Coordinate{x, y, z}));

    // Set velocity with both vertical and horizontal components (sliding)
    cubes.back().getInertialState().velocity = Coordinate{1.0, 0.5, -1.0};
  }

  // Create static floor
  AssetEnvironment floor = createFloor(2, 100, 0.0);

  // Create friction contact constraints
  auto constraints = createFrictionContacts(cubes, floor);

  // Convert unique_ptr vector to raw pointer vector
  std::vector<ContactConstraint*> constraintPtrs{};
  constraintPtrs.reserve(constraints.size());
  for (const auto& c : constraints) {
    constraintPtrs.push_back(c.get());
  }

  // Create solver
  ConstraintSolver solver{};
  solver.setMaxSafetyIterations(100);
  solver.setConvergenceTolerance(1e-6);

  // Benchmark loop
  for (auto _ : state) {
    auto result = solver.solveWithContacts(cubes, floor, constraintPtrs);
    benchmark::DoNotOptimize(result);
  }

  state.SetComplexityN(static_cast<long long>(numCubes));
}
BENCHMARK(BM_SolveFriction)
    ->Arg(2)      // 2 contacts (minimal scenario)
    ->Arg(5)      // 5 contacts
    ->Arg(10)     // 10 contacts (typical scenario)
    ->Complexity();

// ============================================================================
// Scaling Benchmarks
// ============================================================================

/**
 * @brief Benchmark friction solver scaling with increased contact count.
 *
 * Tests performance with larger numbers of contacts to validate O(C³)
 * complexity scaling. Friction constraints should scale similarly to
 * normal-only constraints (within constant factor of ~2x).
 *
 * @ticket 0035d_friction_hardening_and_validation
 */
static void BM_SolveFrictionScaling(benchmark::State& state) {
  size_t numCubes = static_cast<size_t>(state.range(0));

  // Setup: Dense grid of cubes
  std::vector<AssetInertial> cubes{};
  cubes.reserve(numCubes);

  int assetId = 1;
  for (size_t i = 0; i < numCubes; ++i) {
    int instanceId = static_cast<int>(i) + 1;
    double x = static_cast<double>(i % 10) * 1.5;  // Wider grid spacing
    double y = static_cast<double>(i / 10) * 1.5;
    double z = 2.0;
    cubes.push_back(createCube(assetId, instanceId, Coordinate{x, y, z}));

    // Randomized velocity for varied friction directions
    double vx = (i % 3 - 1) * 0.5;  // -0.5, 0, or 0.5 m/s
    double vy = ((i / 3) % 3 - 1) * 0.5;
    cubes.back().getInertialState().velocity = Coordinate{vx, vy, -1.0};
  }

  // Create static floor
  AssetEnvironment floor = createFloor(2, 100, 0.0);

  // Create friction contact constraints
  auto constraints = createFrictionContacts(cubes, floor);

  // Convert unique_ptr vector to raw pointer vector
  std::vector<ContactConstraint*> constraintPtrs{};
  constraintPtrs.reserve(constraints.size());
  for (const auto& c : constraints) {
    constraintPtrs.push_back(c.get());
  }

  // Create solver
  ConstraintSolver solver{};
  solver.setMaxSafetyIterations(100);
  solver.setConvergenceTolerance(1e-6);

  // Benchmark loop
  for (auto _ : state) {
    auto result = solver.solveWithContacts(cubes, floor, constraintPtrs);
    benchmark::DoNotOptimize(result);
  }

  state.SetComplexityN(static_cast<long long>(numCubes));
}
BENCHMARK(BM_SolveFrictionScaling)
    ->Arg(2)      // 2 contacts
    ->Arg(5)      // 5 contacts
    ->Arg(10)     // 10 contacts
    ->Arg(20)     // 20 contacts (scaling test)
    ->Complexity();

BENCHMARK_MAIN();

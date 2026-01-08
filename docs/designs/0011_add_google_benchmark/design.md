# Design: Add Google Benchmark Infrastructure

## Summary

This design adds Google Benchmark to the MSD-CPP project to enable micro-benchmarking of performance-critical code paths. The infrastructure follows existing test patterns with a separate `bench/` directory structure, optional build integration via Conan, and initial benchmark coverage for `ConvexHull` operations in `msd-sim`. This allows developers to measure performance, detect regressions, and validate optimizations with statistical rigor.

## Architecture Changes

### PlantUML Diagram
See: [`./0011_add_google_benchmark.puml`](./0011_add_google_benchmark.puml)

### New Components

#### msd_sim_bench Executable

- **Purpose**: Micro-benchmark executable for msd-sim library performance measurements
- **Location**: `msd/msd-sim/bench/`
- **Executable name**: `msd_sim_bench`
- **Key files**:
  - `msd/msd-sim/bench/CMakeLists.txt` — Build configuration
  - `msd/msd-sim/bench/ConvexHullBench.cpp` — Initial benchmark suite
- **Dependencies**:
  - `msd_sim` library (target being benchmarked)
  - `benchmark::benchmark` (Google Benchmark library)
  - `benchmark::benchmark_main` (provides main() function)
- **Rationale**:
  - Separate executable ensures benchmarks don't pollute test binaries
  - Direct library linkage avoids test framework dependencies
  - `benchmark_main` simplifies benchmark registration and execution
- **Thread safety**: Not applicable (benchmarks run single-threaded by default)
- **Error handling**: Google Benchmark framework handles errors and reporting

#### ConvexHullBench.cpp

- **Purpose**: Benchmark suite for ConvexHull performance-critical operations
- **Location**: `msd/msd-sim/bench/ConvexHullBench.cpp`
- **Key benchmarks**:
  ```cpp
  // Hull construction from point clouds (primary bottleneck)
  static void BM_ConvexHull_Construction(benchmark::State& state) {
      auto points = generateRandomPointCloud(state.range(0));
      for (auto _ : state) {
          ConvexHull hull{points};
          benchmark::DoNotOptimize(hull);
      }
      state.SetComplexityN(state.range(0));
  }
  BENCHMARK(BM_ConvexHull_Construction)
      ->Args({8})     // Cube
      ->Args({64})    // Small cloud
      ->Args({512})   // Medium cloud
      ->Args({4096})  // Large cloud
      ->Complexity();

  // Point containment queries (collision detection hot path)
  static void BM_ConvexHull_Contains(benchmark::State& state) {
      auto hull = ConvexHull{createCubePoints(2.0)};
      Coordinate testPoint{0.5, 0.5, 0.5};  // Interior point
      for (auto _ : state) {
          bool result = hull.contains(testPoint);
          benchmark::DoNotOptimize(result);
      }
  }
  BENCHMARK(BM_ConvexHull_Contains);

  // Signed distance calculations (proximity queries)
  static void BM_ConvexHull_SignedDistance(benchmark::State& state) {
      auto hull = ConvexHull{createCubePoints(2.0)};
      Coordinate testPoint{3.0, 0.0, 0.0};  // Exterior point
      for (auto _ : state) {
          double distance = hull.signedDistance(testPoint);
          benchmark::DoNotOptimize(distance);
      }
  }
  BENCHMARK(BM_ConvexHull_SignedDistance);

  // GJK intersection tests (collision detection)
  static void BM_ConvexHull_Intersects(benchmark::State& state) {
      auto hullA = ConvexHull{createCubePoints(2.0)};
      auto hullB = ConvexHull{createCubePoints(1.0)};  // Overlapping
      for (auto _ : state) {
          bool result = hullA.intersects(hullB);
          benchmark::DoNotOptimize(result);
      }
  }
  BENCHMARK(BM_ConvexHull_Intersects);
  ```
- **Helper functions**:
  - Reuse test helpers from `ConvexHullTest.cpp` (createCubePoints, createTetrahedronPoints)
  - Add `generateRandomPointCloud(size_t count)` for parameterized benchmarks
- **Rationale for cloud sizes**:
  - 8 points: Minimum convex hull (cube/tetrahedron)
  - 64 points: Small objects (simple collision meshes)
  - 512 points: Medium complexity meshes
  - 4096 points: High-detail collision geometry
- **DoNotOptimize**: Prevents compiler from optimizing away benchmarked code

#### bench/CMakeLists.txt

- **Purpose**: Build configuration for benchmark executable
- **Location**: `msd/msd-sim/bench/CMakeLists.txt`
- **Key configuration**:
  ```cmake
  # Find Google Benchmark package
  find_package(benchmark REQUIRED)

  # Create benchmark executable
  add_executable(msd_sim_bench
    ConvexHullBench.cpp
  )

  # Link dependencies
  target_link_libraries(msd_sim_bench
    PRIVATE
      msd_sim
      benchmark::benchmark
      benchmark::benchmark_main
  )

  # Ensure benchmarks compile in Release mode for accurate measurements
  # Even if overall build is Debug
  target_compile_options(msd_sim_bench PRIVATE
    $<$<CONFIG:Debug>:-O2 -DNDEBUG>
  )

  # Include directories
  target_include_directories(msd_sim_bench PRIVATE
    ${CMAKE_SOURCE_DIR}
  )

  # Set C++20 standard
  target_compile_features(msd_sim_bench PRIVATE cxx_std_20)
  ```
- **Rationale**:
  - `find_package(benchmark)` locates Conan-provided library
  - `benchmark_main` provides automatic main() generation
  - Release mode compilation ensures optimizations are enabled
  - Private linkage since benchmarks are leaf executables
- **Dependencies**: Requires `ENABLE_BENCHMARKS=ON` to be built

### Modified Components

#### conanfile.py

- **Current location**: `conanfile.py` (project root)
- **Changes required**:
  1. Add `enable_benchmarks` option to `options` dictionary:
     ```python
     options = {
         "enable_coverage": [True, False],
         "warnings_as_errors": [True, False],
         "enable_clang_tidy": [True, False],
         "enable_benchmarks": [True, False]  # NEW
     }
     default_options = {
         "enable_coverage": False,
         "warnings_as_errors": False,
         "enable_clang_tidy": False,
         "enable_benchmarks": False  # NEW - disabled by default
     }
     ```
  2. Add conditional benchmark dependency in `requirements()` method:
     ```python
     def requirements(self):
         # ... existing dependencies ...
         if self.options.enable_benchmarks:
             self.requires("benchmark/1.9.1")
     ```
- **Backward compatibility**:
  - No impact when disabled (default)
  - Existing builds unaffected
  - No change to installed binaries when disabled
- **Rationale**:
  - Optional dependency avoids forcing benchmark on all users
  - Conan manages version pinning and compatibility
  - Follows existing pattern for optional features (coverage, clang-tidy)

#### CMakeLists.txt (Root)

- **Current location**: `CMakeLists.txt` (project root)
- **Changes required**:
  1. Add `ENABLE_BENCHMARKS` option after existing build options:
     ```cmake
     # Build options
     option(BUILD_TESTING "Build the testing tree" ON)
     option(ENABLE_COVERAGE "Enable code coverage" OFF)
     option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" OFF)
     option(ENABLE_BENCHMARKS "Build performance benchmarks" OFF)  # NEW
     ```
  2. No other changes needed — Conan option already passed via `generate()` method in conanfile.py
- **Backward compatibility**: Default OFF preserves existing behavior
- **Rationale**:
  - Separate from BUILD_TESTING to allow independent control
  - OFF by default avoids extending build times for normal development
  - Developers opt-in for performance work

#### msd/msd-sim/CMakeLists.txt

- **Current location**: `msd/msd-sim/CMakeLists.txt`
- **Changes required**:
  1. Add conditional subdirectory inclusion after test subdirectory:
     ```cmake
     # Add subdirectories to collect sources from each module
     add_subdirectory(src)
     if(BUILD_TESTING)
       add_subdirectory(test)
     endif()
     if(ENABLE_BENCHMARKS)  # NEW
       add_subdirectory(bench)
     endif()
     ```
- **Backward compatibility**: No impact when ENABLE_BENCHMARKS=OFF
- **Rationale**:
  - Mirrors existing test pattern for consistency
  - Keeps benchmark build logic isolated
  - No changes to library targets

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| ConvexHullBench.cpp | ConvexHull (msd-sim) | Library linkage | Benchmarks call public ConvexHull API |
| bench/CMakeLists.txt | msd_sim library | CMake target_link_libraries | Links against library target |
| conanfile.py option | CMakeLists.txt option | Conan → CMake variable | ENABLE_BENCHMARKS passed via CMakeToolchain |
| msd_sim_bench | msd-sim/bench/ | CMake subdirectory | Conditional inclusion via add_subdirectory |
| Helper functions | ConvexHullTest.cpp | Code reuse | Copy/extract createCubePoints, createTetrahedronPoints |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| None | N/A | No impact | Benchmarks are separate executables |

Benchmarks do not modify or depend on existing test infrastructure. No test changes required.

### New Tests Required

#### Unit Tests

No unit tests required for benchmarks. Google Benchmark framework provides self-validation.

#### Integration Tests

No integration tests required. Benchmarks are measurement tools, not functional tests.

### Benchmark Validation

| Benchmark | Validation Method | Success Criteria |
|-----------|------------------|------------------|
| BM_ConvexHull_Construction | Manual execution | Reports timing without errors |
| BM_ConvexHull_Contains | Manual execution | Reports timing without errors |
| BM_ConvexHull_SignedDistance | Manual execution | Reports timing without errors |
| BM_ConvexHull_Intersects | Manual execution | Reports timing without errors |

Benchmarks validated by successful execution and reasonable timing outputs (not specific thresholds).

## Build Instructions

### Building Benchmarks

```bash
# Install dependencies with benchmarks enabled
conan install . --build=missing -s build_type=Release -o "&:enable_benchmarks=True"

# Configure with benchmarks enabled
cmake --preset conan-release -DENABLE_BENCHMARKS=ON

# Build benchmark executable
cmake --build --preset conan-release --target msd_sim_bench
```

### Running Benchmarks

```bash
# Run all benchmarks
./build/Release/msd_sim_bench

# Run with specific filters
./build/Release/msd_sim_bench --benchmark_filter=ConvexHull_Construction

# Run with specific repetitions for statistical significance
./build/Release/msd_sim_bench --benchmark_repetitions=10

# Output results to JSON for analysis
./build/Release/msd_sim_bench --benchmark_out=results.json --benchmark_out_format=json
```

### CMake Preset for Benchmarks (Future Enhancement)

Consider adding a benchmark-specific preset to `CMakeUserPresets.json`:

```json
{
  "name": "release-benchmarks",
  "inherits": "conan-release",
  "cacheVariables": {
    "ENABLE_BENCHMARKS": "ON",
    "BUILD_TESTING": "OFF"
  }
}
```

## Open Questions

### Design Decisions (Human Input Needed)

1. **Benchmark executable naming convention**
   - Option A: `msd_sim_bench` (follows `msd_sim_test` pattern)
   - Option B: `benchmarks` (simpler, generic)
   - Recommendation: Option A for consistency with existing test naming

2. **Helper function sharing strategy**
   - Option A: Copy helper functions from ConvexHullTest.cpp to ConvexHullBench.cpp
   - Option B: Extract helpers to shared header in `test/Physics/TestHelpers.hpp`
   - Recommendation: Option A initially (fewer files, simpler), Option B if reuse grows

3. **Build mode enforcement**
   - Option A: Force Release mode compilation for benchmark targets only
   - Option B: Allow Debug builds but warn users
   - Recommendation: Option A (accurate measurements require optimizations)

### Prototype Required

None. Google Benchmark integration is well-established with clear patterns.

### Requirements Clarification

1. **CI Integration**: Should benchmarks run in CI, or only locally?
   - Assumption: Local only initially. CI integration requires baseline tracking.

2. **Performance regression thresholds**: What constitutes a regression?
   - Assumption: No automated thresholds initially. Manual comparison for now.

3. **Additional benchmark targets**: Should other modules (msd-assets, msd-gui) get benchmarks?
   - Assumption: Start with msd-sim only. Extend to other modules as needed.

4. **Random point cloud generation**: Should random clouds use fixed seed for reproducibility?
   - Assumption: Yes, use fixed seed for deterministic benchmarks.

## Future Enhancements

### Additional Benchmark Suites

- **msd-assets**: AssetRegistry cache lookup performance
- **msd-sim Physics**: PhysicsComponent force application and integration
- **msd-gui**: Rendering pipeline benchmarks

### CI Integration

- Automated benchmark execution on pull requests
- Performance regression detection via baseline comparison
- Historical performance tracking in CI artifacts

### Comparative Benchmarks

- Compare Qhull vs. alternative convex hull algorithms
- Benchmark different GJK simplex strategies
- Memory allocation profiling via custom allocators

## Implementation Guidance

### Phase 1: Build System Integration (Conan + CMake)
1. Modify `conanfile.py` to add `enable_benchmarks` option and conditional dependency
2. Modify root `CMakeLists.txt` to add `ENABLE_BENCHMARKS` option
3. Verify Conan can install benchmark dependency

### Phase 2: Benchmark Infrastructure (msd-sim/bench/)
1. Create `msd/msd-sim/bench/CMakeLists.txt` with executable configuration
2. Verify benchmark executable builds and links correctly

### Phase 3: Initial Benchmark Suite
1. Create `msd/msd-sim/bench/ConvexHullBench.cpp` with 4 benchmarks
2. Add helper function `generateRandomPointCloud()`
3. Copy test helpers from `ConvexHullTest.cpp`
4. Verify benchmarks run and produce output

### Phase 4: Documentation
1. Update `CLAUDE.md` with benchmark build instructions
2. Add benchmark usage examples
3. Document benchmark interpretation guidelines

## References

- Google Benchmark Documentation: https://github.com/google/benchmark
- Google Benchmark User Guide: https://github.com/google/benchmark/blob/main/docs/user_guide.md
- Conan benchmark package: https://conan.io/center/recipes/benchmark

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-01-08
**Status**: APPROVED
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | `msd_sim_bench` follows existing test naming pattern (`msd_sim_test`). File names use PascalCase (ConvexHullBench.cpp). Benchmark functions follow Google Benchmark convention (`BM_ClassName_Operation`). |
| Namespace organization | ✓ | No new namespaces introduced. Benchmarks use existing `msd_sim` namespace types. |
| File structure | ✓ | `msd/msd-sim/bench/` mirrors `msd/msd-sim/test/` pattern. `bench/CMakeLists.txt` follows `test/CMakeLists.txt` structure. Consistent with project organization. |
| Dependency direction | ✓ | Clean dependency flow: benchmark executable -> msd_sim library -> dependencies. No circular dependencies. Benchmark library added conditionally via Conan. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | No manual resource management in benchmarks. Google Benchmark framework handles lifecycle. ConvexHull uses RAII for Qhull resources. |
| Smart pointer appropriateness | ✓ | No smart pointers needed in benchmarks (value semantics for ConvexHull, Coordinate). Follows project standard of preferring value semantics. |
| Value/reference semantics | ✓ | Benchmarks use value semantics appropriately. `benchmark::DoNotOptimize` prevents compiler optimization of benchmarked code. |
| Rule of 0/3/5 | ✓ | No custom classes requiring special member functions. Uses compiler-generated defaults. |
| Const correctness | ✓ | Helper functions return by value (no const reference issues). Benchmark state and hulls are appropriately const. |
| Exception safety | ✓ | Benchmarks don't catch exceptions (framework handles). ConvexHull throws on invalid input (documented behavior). |
| Initialization | ✓ | Uses brace initialization for Coordinate, ConvexHull construction. Follows CLAUDE.md standard. |
| Return values | ✓ | Helper functions return values by value. No output parameters. Follows project convention. |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | Clean includes: `benchmark/benchmark.h`, `msd-sim` headers. No circular dependencies. Google Benchmark is header-mostly library. |
| Template complexity | ✓ | No custom templates. Uses existing ConvexHull template constructor. Manageable complexity. |
| Memory strategy | ✓ | Point clouds allocated per iteration (realistic benchmark). Google Benchmark manages timing infrastructure. No memory leaks expected. |
| Thread safety | N/A | Benchmarks run single-threaded by default (documented in design). No thread safety concerns. |
| Build integration | ✓ | Conan dependency management straightforward. CMake option pattern matches existing `ENABLE_COVERAGE`, `ENABLE_CLANG_TIDY`. Conditional compilation ensures no impact when disabled. |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | Benchmarks are standalone executables. Can run independently of test suite. |
| Mockable dependencies | N/A | No dependencies to mock. Benchmarks measure real performance of real code. |
| Observable state | ✓ | Google Benchmark reports timing statistics. Success/failure observable via exit code and output. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Benchmark build time overhead | Performance | Low | Low | Benchmarks disabled by default (ENABLE_BENCHMARKS=OFF). Developers opt-in only for performance work. | No |
| R2 | Helper function duplication from tests | Maintenance | Medium | Low | Design acknowledges this. Recommends copying initially, extract to shared header if reuse grows. Pragmatic approach. | No |
| R3 | Benchmark measurement accuracy in Debug mode | Technical | Low | Medium | Design enforces Release mode compilation via `target_compile_options` even when overall build is Debug. Ensures accurate measurements. | No |
| R4 | Random point cloud non-determinism | Testability | Medium | Low | Design assumption (section 4 of Open Questions) specifies fixed seed for reproducibility. Implementation must honor this. | No |

### Summary

This design is **APPROVED** with no revisions required. The proposal demonstrates excellent adherence to project standards and existing patterns:

**Strengths:**
1. **Follows existing patterns**: Mirrors test infrastructure (bench/ directory, CMake structure, naming conventions)
2. **Clean build integration**: Optional Conan dependency, CMake option pattern consistent with existing features
3. **Well-scoped initial implementation**: Four focused benchmarks targeting performance-critical ConvexHull operations
4. **Accurate measurements**: Enforces Release mode compilation for benchmark targets
5. **Zero impact when disabled**: Default OFF preserves existing build behavior
6. **Comprehensive documentation**: Clear build instructions, usage examples, future enhancements

**Design quality indicators:**
- No circular dependencies or architectural violations
- Clear separation of concerns (benchmarks separate from tests)
- Appropriate use of existing infrastructure (Google Benchmark is industry standard)
- Pragmatic approach to helper function sharing (copy first, refactor if needed)
- Well-defined cloud sizes aligned with human preference (8, 64, 512, 4096)

**No prototypes required**: Google Benchmark integration is a well-established pattern with clear semantics and extensive documentation. The risk profile is low, and all identified risks have appropriate mitigations.

**Next steps:**
1. Human reviews this approval
2. Design advances to "Design Approved — Ready for Prototype" status
3. Since no prototype is required, proceed directly to implementation phase
4. Implementation follows the four-phase guidance in design document

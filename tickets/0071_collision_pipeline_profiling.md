# Ticket 0071: Collision Pipeline Profiling & Optimization

## Status
- [x] Draft
- [x] Investigation Complete
- [ ] Design Complete
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Investigation Complete
**Type**: Performance / Investigation
**Priority**: Medium
**Assignee**: N/A
**Created**: 2026-02-17
**Branch**: `0071-collision-pipeline-profiling`
**Related Tickets**: [0053_collision_pipeline_performance](0053_collision_pipeline_performance.md), [0068_nlopt_friction_cone_solver](0068b_nlopt_friction_solver_class.md)

---

## Summary

Re-profiling the collision and physics pipeline on `main` (post-0053, post-0068/0069/0070 NLopt friction solver integration) to identify current hotspots and optimization opportunities. The codebase has changed significantly since 0053's profiling baseline.

---

## Profiling Environment

- **Branch**: `main` at `19c1722` (with Release warning fixes on `0071-collision-pipeline-profiling`)
- **Platform**: macOS, Apple Clang, Release (-O2 -g), Time Profiler
- **Workload**: `msd_sim_test --gtest_filter="*Collision*:*Friction*:*Constraint*:*Contact*:*Cone*" --gtest_repeat=50`
- **Total Samples**: 12,627 (~12.6s wall time)
- **Test Count**: 697/697 passing

---

## Profiling Results

### Top Hotspots — All Functions

| Rank | Function | Samples | % of Total | Category |
|------|----------|---------|------------|----------|
| 1 | `sqlite3VdbeExec` | 146 | 1.2% | Database I/O |
| 2 | `<deduplicated_symbol>` | 131 | 1.0% | System (likely Eigen BLAS) |
| 3 | `yy_reduce` | 65 | 0.5% | SQLite SQL parser |
| 4 | **`ConstraintSolver::solve`** | 54 | 0.4% | Collision solver |
| 5 | `_xzm_free` | 47 | 0.4% | Memory allocator |
| 6 | `_xzm_xzone_malloc_tiny` | 38 | 0.3% | Memory allocator |
| 7 | `sqlite3Insert` | 37 | 0.3% | Database I/O |
| 8 | `sqlite3BtreeInsert` | 35 | 0.3% | Database I/O |
| 9 | **`PositionCorrector::correctPositions`** | 35 | 0.3% | Collision solver |
| 10 | **`ConstraintSolver::flattenConstraints`** | 30 | 0.2% | Collision solver |
| 11 | `sqlite3BtreeTableMoveto` | 30 | 0.2% | Database I/O |
| 12 | **`EPA::buildHorizonEdges`** | 30 | 0.2% | Collision detection |

### Top Hotspots — Project Functions Only

| Rank | Function | Samples | % of Total | Subsystem |
|------|----------|---------|------------|-----------|
| 1 | `ConstraintSolver::solve` | 54 | 0.4% | Constraint solving |
| 2 | `PositionCorrector::correctPositions` | 35 | 0.3% | Position correction |
| 3 | `ConstraintSolver::flattenConstraints` | 30 | 0.2% | Constraint solving |
| 4 | `EPA::buildHorizonEdges` | 30 | 0.2% | EPA algorithm |
| 5 | `DataRecorder::DataRecorder` | 25 | 0.2% | Data recording (constructor) |
| 6 | `CollisionPipeline::solveConstraintsWithWarmStart` | 20 | 0.2% | Collision pipeline |
| 7 | `EPA::extractContactManifold` | 20 | 0.2% | EPA algorithm |
| 8 | `ConstraintSolver::assembleFlatEffectiveMass` | 20 | 0.2% | Constraint solving |
| 9 | `supportMinkowskiWithWitness` | 19 | 0.2% | GJK support function |
| 10 | `supportMinkowski` | 15 | 0.1% | GJK support function |

### Hotspots by Subsystem (Aggregated)

| Subsystem | Key Functions | Total Samples | % of Total |
|-----------|--------------|---------------|------------|
| **SQLite / DataRecorder** | VdbeExec, Insert, BtreeInsert, yy_reduce, step, Parser, FrictionConstraintRecord insert, InertialStateRecord insert | ~480 | ~3.8% |
| **Memory Allocator** | _xzm_free, _xzm_xzone_malloc_tiny, xzm_malloc_zone_size | ~114 | ~0.9% |
| **Constraint Solver** | solve, flattenConstraints, assembleFlatEffectiveMass, solveActiveSet, assembleFlatRHS, buildFrictionSpec | ~132 | ~1.0% |
| **EPA** | buildHorizonEdges, extractContactManifold, computeContactInfo, expandPolytope | ~75 | ~0.6% |
| **GJK / Support** | supportMinkowski, supportMinkowskiWithWitness, intersects, AABB check | ~55 | ~0.4% |
| **Position Corrector** | correctPositions | 35 | ~0.3% |
| **SAT Fallback** | computeSATMinPenetration, getFacetsAlignedWith, buildPolygonFromFacets | ~31 | ~0.2% |
| **CollisionPipeline** | solveConstraintsWithWarmStart, execute, createConstraints | ~40 | ~0.3% |

---

## Key Observations

### 1. SQLite Dominates Total CPU (~3.8%)

The DataRecorder's per-frame database writes (enabled for replay tests) now dominate CPU usage. SQLite's VdbeExec (1.2%), SQL parser (0.5%), and insert operations collectively consume ~3.8% — more than the entire collision solver pipeline combined.

**Impact**: This is test-workload-specific. Replay-enabled tests create a new database per test fixture, causing repeated schema creation and per-frame inserts. In production simulation loops without recording enabled, this cost disappears entirely.

**Optimization opportunity**: If recording is needed during profiling, batch inserts or reduce recording frequency. For profiling the collision pipeline specifically, disable DataRecorder.

### 2. Memory Allocation Reduced but Still Present (0.9%)

Post-0053f workspace optimizations reduced memory allocator samples from ~1.8% to ~0.9%. `_xzm_free` (47 samples) and `_xzm_xzone_malloc_tiny` (38 samples) are still in the top 10.

**Remaining sources**: Likely std::vector resizing in constraint/collision data structures, Eigen temporaries in functions not yet workspace-optimized, and DataRecorder record construction.

### 3. Constraint Solver Is the Largest Physics Hotspot (1.0%)

`ConstraintSolver::solve` (54 samples) + `flattenConstraints` (30) + `assembleFlatEffectiveMass` (20) + `solveActiveSet` (14) + `assembleFlatRHS` (7) + `buildFrictionSpec` (7) = ~132 samples.

**Notable**: `flattenConstraints` (30 samples, rank 3) is a new hotspot not seen in 0053 profiling. This function builds the flattened Jacobian/RHS representation each solve and may benefit from workspace caching.

### 4. EPA Is Relatively Efficient (0.6%)

EPA aggregate (75 samples) is down from 0053's 69/5013 (1.4%) to 75/12627 (0.6%) relative — likely benefiting from SAT gating reducing the number of EPA calls that reach deep expansion.

### 5. NLopt Friction Solver NOT in Top 30

The NLopt friction solver (tickets 0068-0070) does not appear in the top 30 project hotspots. This is excellent — the solver integration has not introduced measurable performance regression.

### 6. `atan2` Appears (20 samples, 0.2%)

`atan2` at 20 samples suggests angle computations (likely in `AngularCoordinate::fromQuaternion` or similar) are adding up. Could be optimized with fast approximations if needed.

---

## Comparison with 0053 Baseline

| Metric | 0053 Baseline | Current (0071) | Change |
|--------|--------------|----------------|--------|
| Total samples | 5,013 | 12,627 | +152% (more test iterations) |
| ConstraintSolver::solve | 45 (0.9%) | 54 (0.4%) | Reduced % |
| PositionCorrector | 40 (0.8%) | 35 (0.3%) | Reduced % |
| supportMinkowski | 16 (0.3%) | 15+19 (0.3%) | Stable |
| EPA aggregate | 69 (1.4%) | 75 (0.6%) | Reduced % |
| Memory free | 40 (0.8%) | 47 (0.4%) | Reduced % |
| Memory malloc | 29 (0.6%) | 38 (0.3%) | Reduced % |
| SAT fallback | Not in top 20 | 31 (0.2%) | Stable (gated) |
| **SQLite (NEW)** | Not profiled | ~480 (3.8%) | **New hotspot** |
| **NLopt solver** | N/A | Not in top 30 | **No regression** |

**Key takeaway**: The collision pipeline itself is well-optimized. Relative percentages for all physics functions have dropped. The dominant cost center has shifted to DataRecorder SQLite I/O, which is workload-specific.

---

## Next Step: Multi-Body Collision Benchmark

The initial profiling used the test suite workload, which includes DataRecorder overhead and exercises mostly body-floor contacts. To identify the real collision pipeline bottlenecks we need a **dedicated multi-body benchmark** that creates dense body-body interaction scenarios.

### Benchmark Design

New file: `msd/msd-sim/bench/MultiBodyCollisionBench.cpp`

Three scenarios exercising different collision density profiles:

| Benchmark | Description | Body Count | Contacts |
|-----------|-------------|------------|----------|
| `BM_MultiBody_ClusterDrop` | N cubes spawned in tight random cluster above floor, small random velocities | 4, 8, 16, 32 | Dense body-body + body-floor |
| `BM_MultiBody_StackCollapse` | N cubes in unstable vertical stack, collapses and settles | 4, 8, 16 | Sustained multi-contact resting + tumbling |
| `BM_MultiBody_GridSettle` | N cubes in grid pattern settling onto floor | 4, 9, 16, 25 | Many parallel body-floor (baseline) |

**Key design choices:**
- Uses `WorldModel` directly (not raw `CollisionPipeline`) — exercises full simulation loop including gravity, integration, collision detection, constraint solving, and position correction
- Deterministic random via `std::mt19937` with fixed seed
- Each benchmark iteration creates fresh WorldModel + objects, steps 20 frames
- No DataRecorder — isolates pure physics cost
- Floor: `createCubePoints(100.0)` at z=-100 (top face at z=0)

### Files to Modify
- `msd/msd-sim/bench/MultiBodyCollisionBench.cpp` — New benchmark file
- `msd/msd-sim/bench/CMakeLists.txt` — Add to `msd_sim_bench` target

### Profiling Workflow
1. Build: `cmake --build --preset conan-release --target msd_sim_bench`
2. Run: `./build/Release/release/msd_sim_bench --benchmark_filter="MultiBody"`
3. Profile: `xctrace record --template "Time Profiler" --launch -- ./build/Release/release/msd_sim_bench --benchmark_filter="MultiBody_ClusterDrop/16" --benchmark_min_time=5s`
4. Parse: `python3 analysis/scripts/parse-profile.py <trace> --top 30 --project-only`

---

## Multi-Body Benchmark Results

### Benchmark Timing (ClusterDrop/32, 20 frames per iteration)

| Benchmark | Time (ms) | Items/sec |
|-----------|-----------|-----------|
| `BM_MultiBody_ClusterDrop/4` | 0.152 | 526k |
| `BM_MultiBody_ClusterDrop/8` | 0.873 | 183k |
| `BM_MultiBody_ClusterDrop/16` | 1.38 | 232k |
| `BM_MultiBody_ClusterDrop/32` | 20.8 | 30.8k |
| `BM_MultiBody_StackCollapse/4` | 0.487 | 164k |
| `BM_MultiBody_StackCollapse/8` | 1.41 | 113k |
| `BM_MultiBody_StackCollapse/16` | 5.92 | 54.1k |
| `BM_MultiBody_GridSettle/4` | 0.032 | 2.5M |
| `BM_MultiBody_GridSettle/9` | 0.129 | 1.4M |
| `BM_MultiBody_GridSettle/16` | 0.368 | 870k |
| `BM_MultiBody_GridSettle/25` | 0.878 | 570k |

**Scaling observation**: ClusterDrop shows super-linear scaling from 16→32 bodies (1.38ms → 20.8ms, ~15x for 2x bodies), indicating O(n²) or worse behavior in constraint solving with dense contacts.

### Multi-Body Profiling Results (ClusterDrop/32)

**Workload**: `BM_MultiBody_ClusterDrop/32 --benchmark_min_time=5s`
**Total Samples**: 18,146 (~18.1s wall time)

#### Top Project Hotspots

| Rank | Function | Samples | % of Total | Subsystem |
|------|----------|---------|------------|-----------|
| 1 | `PositionCorrector::correctPositions` | 120 | 0.7% | Position correction |
| 2 | `ConstraintSolver::solveActiveSet` | 90 | 0.5% | Constraint solving |
| 3 | `ConstraintSolver::solve` | 86 | 0.5% | Constraint solving |
| 4 | `ConstraintSolver::assembleFlatEffectiveMass` | 56 | 0.3% | Constraint solving |
| 5 | `EPA::buildHorizonEdges` | 46 | 0.3% | EPA algorithm |
| 6 | `EPA::extractContactManifold` | 31 | 0.2% | EPA algorithm |
| 7 | `GJK::intersects` | 24 | 0.1% | GJK algorithm |
| 8 | `ConstraintSolver::flattenConstraints` | 24 | 0.1% | Constraint solving |
| 9 | `CollisionPipeline::solveConstraintsWithWarmStart` | 23 | 0.1% | Pipeline orchestration |
| 10 | `buildPolygonFromFacets` | 21 | 0.1% | SAT fallback |

#### Top System Hotspots (Eigen BLAS)

| Rank | Function | Samples | % of Total |
|------|----------|---------|------------|
| 1 | `Eigen::gebp_kernel` (matrix multiply) | 227 | 1.3% |
| 2 | `Eigen::triangular_solve_matrix` | 178 | 1.0% |
| 3 | `Eigen::general_matrix_vector_product` (col-major) | 80 | 0.4% |
| 4 | `Eigen::general_matrix_vector_product` (row-major) | 67 | 0.4% |
| 5 | `Eigen::general_matrix_matrix_triangular_product` | 55 | 0.3% |

#### Hotspots by Subsystem (Aggregated)

| Subsystem | Total Samples | % of Total | Key Functions |
|-----------|---------------|------------|---------------|
| **Eigen BLAS** | ~650 | ~3.6% | gebp_kernel, triangular_solve, LLT, matrix-vector products |
| **Constraint Solver** | ~280 | ~1.5% | solveActiveSet, solve, assembleFlatEffectiveMass, flattenConstraints |
| **Position Corrector** | 120 | ~0.7% | correctPositions |
| **EPA** | ~110 | ~0.6% | buildHorizonEdges, extractContactManifold, computeContactInfo, expandPolytope |
| **Memory Allocator** | ~80 | ~0.4% | _xzm_free, _xzm_xzone_malloc_tiny |
| **GJK / Support** | ~85 | ~0.5% | intersects, supportMinkowski, supportMinkowskiWithWitness, AABB check |
| **SAT / Manifold** | ~45 | ~0.3% | buildPolygonFromFacets, getFacetsAlignedWith, computeSATMinPenetration |
| **CollisionPipeline** | ~55 | ~0.3% | solveConstraintsWithWarmStart, execute, createConstraints |

### Key Findings from Multi-Body Profiling

1. **Eigen linear algebra dominates CPU (3.6%)**: Matrix multiply (gebp_kernel), triangular solves, and LLT Cholesky decomposition are the top consumers. These are called from ConstraintSolver's effective mass matrix assembly and Active Set Method solves. With 32 bodies creating 50+ contacts, the constraint system size grows, making O(n³) Cholesky the bottleneck.

2. **PositionCorrector is the top project function (0.7%)**: Iterative position correction with per-constraint Jacobian evaluation. Runs its own mini-solver loop after velocity solving.

3. **Constraint Solver aggregate is significant (1.5%)**: solveActiveSet (0.5%) + solve (0.5%) + assembleFlatEffectiveMass (0.3%) + flattenConstraints (0.1%). The Active Set Method's pivoting operations trigger repeated matrix factorizations.

4. **Super-linear scaling at 32 bodies**: The 15x time increase from 16→32 bodies suggests the constraint system size grows quadratically (O(n²) contacts) and solving grows cubically with system size (O(n³) Cholesky). This is the primary optimization target for large body counts.

5. **EPA/GJK relatively efficient**: Combined ~1.1% — well-optimized since 0053.

6. **NLopt friction solver still absent**: Not in top 30 even under heavy multi-body load.

---

## Optimization Opportunities (from multi-body profiling)

### 0071a: Constraint Solver Scalability (High Priority)

**Problem**: Super-linear scaling from 16→32 bodies (15x time for 2x bodies). The Active Set Method uses dense Cholesky factorization (O(n³)) on the full constraint system. With 32 bodies generating 50+ contacts, the effective mass matrix is large and factored repeatedly during pivoting.
**Solutions**:
- Block-diagonal approximation: Solve per-contact-pair rather than global system
- Sparse matrix solvers: Use sparse Cholesky (Eigen::SimplicialLLT) instead of dense LLT
- Iterative solver: PGS/Jacobi for large systems, Active Set for small systems (hybrid)
**Est. Impact**: O(n³) → O(n) for constraint solving, potentially 10x improvement at 32 bodies.

### 0071b: PositionCorrector Optimization (Medium Priority)

**Problem**: `correctPositions` is the top project hotspot (0.7%, 120 samples). Runs iterative correction with per-constraint Jacobian evaluation after velocity solving.
**Solutions**:
- Reuse Jacobian data from velocity solve (avoid recomputation)
- Reduce iteration count with better warm-starting
- Consider integrating position correction into the velocity solve (Baumgarte stabilization tradeoff)
**Est. Impact**: ~0.3-0.5% CPU reduction.

### 0071c: Eigen Fixed-Size Matrices (Medium Priority, Deferred from 0053e)

**Problem**: Eigen BLAS kernels dominate at 3.6%. Dense matrix operations (gebp_kernel, triangular_solve, LLT) use dynamic allocation and dispatch.
**Solutions**:
- Use fixed-size Eigen matrices for bounded contact counts (typically 1-4 contacts per pair)
- Template constraint solver on system size for small systems
**Est. Impact**: ~1-2% CPU reduction from eliminating dynamic dispatch overhead.

### 0071d: DataRecorder Batch Insert Optimization (Low Priority)

**Problem**: Per-record SQLite inserts with full SQL parsing per insert (3.8% in test workload).
**Solution**: Use prepared statements and batch transaction grouping more aggressively.
**Est. Impact**: ~1-2% CPU reduction for recording-enabled workloads only.

---

## Profiling Artifacts

### Initial Profiling (Test Suite Workload)
- **Trace file**: `analysis/profile_results/profile_0071_collision.trace`
- **JSON report (project-only)**: `analysis/profile_results/profile_0071_collision.json`
- **JSON report (all functions)**: `analysis/profile_results/profile_0071_collision_full.json`

### Multi-Body Profiling (ClusterDrop/32)
- **Trace file**: `analysis/profile_results/profile_0071_multibody.trace`
- **JSON report (project-only)**: `analysis/profile_results/profile_0071_multibody_project.json`
- **JSON report (all functions)**: `analysis/profile_results/profile_0071_multibody_full.json`

### Benchmark Source
- `msd/msd-sim/bench/MultiBodyCollisionBench.cpp`

---

## Workflow Log

### Investigation Phase
- **Started**: 2026-02-17
- **Completed**: 2026-02-17
- **Branch**: `0071-collision-pipeline-profiling`
- **Artifacts**: Profiling traces and JSON reports in `analysis/profile_results/`
- **Notes**:
  - Fixed Release build warnings (sign-conversion, unused variables) to enable profiling build
  - Profiled 50 repetitions of collision/friction/constraint/contact tests
  - 12,627 total samples collected
  - SQLite/DataRecorder is now the dominant cost center (3.8%)
  - Collision pipeline well-optimized post-0053; no single physics function exceeds 0.4%
  - NLopt friction solver (0068-0070) introduced zero measurable overhead

### Multi-Body Benchmark Phase
- **Completed**: 2026-02-17
- **Benchmark file**: `msd/msd-sim/bench/MultiBodyCollisionBench.cpp`
- **Profiled scenario**: `BM_MultiBody_ClusterDrop/32` (32 cubes, dense body-body contacts)
- **Total samples**: 18,146
- **Key findings**:
  - Eigen BLAS (3.6%) and ConstraintSolver (1.5%) dominate under dense multi-body contacts
  - Super-linear scaling 16→32 bodies: 1.38ms → 20.8ms (15x for 2x bodies)
  - Root cause: O(n³) dense Cholesky in Active Set Method with O(n²) contact pairs
  - PositionCorrector is top project function at 0.7%
  - NLopt friction solver still undetectable in top 30

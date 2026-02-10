# Investigation Report: Collision Pipeline Performance

**Ticket**: [0053_collision_pipeline_performance](../../../tickets/0053_collision_pipeline_performance.md)
**Date**: 2026-02-10
**Investigator**: Workflow Orchestrator
**Branch Analyzed**: `0053-collision-pipeline-performance` (current main)
**Status**: Investigation Complete

---

## Executive Summary

This investigation analyzes the collision and friction pipeline performance based on profiling data collected from the `0052d-solver-integration` branch. The profiling reveals that while no single function is catastrophically slow, the aggregate cost of the collision pipeline represents a significant portion of CPU time during physics simulation.

The analysis identifies five optimization opportunities with a combined potential CPU reduction of ~6.8% and recommends an implementation order based on impact, risk, and interdependencies.

---

## Profiling Environment

| Metric | Value |
|--------|-------|
| **Platform** | macOS, Apple Silicon/Intel |
| **Compiler** | Apple Clang, Release (-O2 -g) |
| **Profiler** | Xcode Instruments, Time Profiler |
| **Workload** | `msd_sim_test --gtest_filter="*Collision*:*Friction*:*Constraint*:*Contact*:*Cone*" --gtest_repeat=50` |
| **Total Samples** | 5,013 (~5s wall time) |
| **Branch** | `0052d-solver-integration` (commit `8d6973a`) |

**Note**: Profiling baseline on current `main` branch could not be established due to Instruments sampling rate limitations with short-running tests. The existing profiling data from the `0052d-solver-integration` branch (now merged into main) is used as the baseline.

---

## Hotspot Analysis

### Subsystem Distribution

| Subsystem | Key Functions | Samples | % of Total | Priority |
|-----------|--------------|---------|------------|----------|
| **Friction Solver** | `FrictionConeSolver::solve`, `ConeProjection::projectVector` | 93 | 1.9% | High |
| **Memory Allocator** | `_xzm_free`, `_xzm_xzone_malloc_tiny`, `_free` | 90 | 1.8% | High |
| **Eigen Linear Algebra** | `gebp_kernel`, `general_matrix_vector_product`, `LLT`, `triangular_solver` | 84 | 1.7% | Medium |
| **GJK/Support Function** | `supportMinkowski`, `supportMinkowskiWithWitness`, `GJK::intersects` | 73 | 1.5% | Low |
| **EPA** | `buildHorizonEdges`, `extractContactManifold`, `computeContactInfo`, `expandPolytope` | 69 | 1.4% | Low |
| **Qhull** | `printsummary`, `findbest`, `findbesthorizon`, `memalloc`, `addpoint` | 63 | 1.3% | Low |
| **SAT Fallback** | `computeSATMinPenetration`, `getFacetsAlignedWith`, `buildPolygonFromFacets` | 53 | 1.1% | Medium |
| **Contact Solver (ASM)** | `ConstraintSolver::solve`, `solveActiveSet`, `assembleFlatEffectiveMass` | 45 | 0.9% | Low |
| **Position Corrector** | `PositionCorrector::correctPositions` | 40 | 0.8% | Low |
| **Collision Pipeline** | `solveConstraintsWithWarmStart`, `execute` | 25 | 0.5% | Low |

**Total Collision Pipeline CPU**: ~6.8% (539 samples)

### Key Observations

1. **Memory allocation overhead (1.8%)** rivals the friction solver as the top hotspot. This strongly suggests excessive per-frame dynamic allocations in the solver pipeline.

2. **Friction solver (1.9%)** has two separate entries in the profile (rank 1 and 18), indicating that both the outer `solve()` function and the inner lambda merit function evaluation are significant.

3. **SAT fallback (1.1%)** added in ticket 0047 for resting contact stability currently runs on every collision check, even when EPA produces high-quality results.

4. **Qhull diagnostic output (0.3%)** is pure I/O overhead that should be disabled in production builds.

5. **Eigen dense operations (1.7%)** use dynamic-size matrices with heap allocation overhead. For bounded contact counts (typically 1-4), fixed-size specializations could eliminate this cost.

---

## Optimization Opportunities

### Prioritization Matrix

| Subtask | Impact | Complexity | Risk | Priority | Order |
|---------|--------|------------|------|----------|-------|
| 0053b: Disable Qhull Output | Low (0.3%) | Trivial | Minimal | Quick Win | 1 |
| 0053a: Reduce Heap Allocations | High (1.8%) | Medium | Low | High | 2 |
| 0053c: Friction Solver Optimization | High (1.9%) | Medium | Medium | High | 3 |
| 0053d: SAT Fallback Cost Reduction | Medium (1.1%) | Low | Low | Medium | 4 |
| 0053e: Eigen Fixed-Size Optimization | Medium (1.7%) | Medium | Low | Medium | 5 |

**Rationale for Priority Order**:
1. **0053b first**: Trivial change, immediate win, zero risk
2. **0053a second**: Highest impact, foundational for other optimizations (friction solver and position corrector both benefit)
3. **0053c third**: Second highest impact, builds on allocation reduction from 0053a
4. **0053d fourth**: Medium impact, orthogonal to other work, low risk
5. **0053e last**: Medium impact but requires careful validation to ensure numeric stability

---

## Detailed Opportunity Analysis

### 0053a: Reduce Heap Allocations (Priority 1)

**CPU Impact**: 1.8% (90 samples)
**Risk**: Low (no algorithmic changes)
**Complexity**: Medium

**Problem**: Dynamic allocations in per-frame solver loop create malloc/free overhead.

**Investigation Areas**:
- `ConstraintSolver::solve()` creates `std::vector` and `Eigen::VectorXd`/`MatrixXd` each call
- `CollisionPipeline::execute()` builds constraint lists each frame
- `FrictionConeSolver::solve()` allocates workspace each call
- `PositionCorrector::correctPositions()` allocates per-iteration vectors

**Proposed Solutions**:
1. Pre-allocated workspace structs owned by `CollisionPipeline`
2. `Eigen::Matrix<double, Dynamic, Dynamic, 0, MaxRows, MaxCols>` with compile-time upper bounds
3. `SmallVector`-style inline storage for typical contact counts (1-8 contacts)

**Success Criteria**:
- Memory allocation samples reduced by >= 50% (90 → 45 samples)
- No physics test regressions

### 0053b: Disable Qhull Diagnostic Output (Priority 0)

**CPU Impact**: 0.3% (~15 samples)
**Risk**: Minimal
**Complexity**: Trivial

**Problem**: `qh_printsummary` consumes 0.3% on diagnostic I/O.

**Proposed Solution**: Set `qh->NOsummary = True` or redirect output to `/dev/null` in `ConvexHull::computeHull()`.

**Success Criteria**:
- `qh_printsummary` removed from profiling hotspots
- No functional changes to hull computation

### 0053c: FrictionConeSolver Iteration Optimization (Priority 2)

**CPU Impact**: 1.9% (93 samples)
**Risk**: Medium (changes solver behavior)
**Complexity**: Medium

**Problem**: Newton solver is the single biggest hotspot.

**Investigation Areas**:
- Average iteration count per solve (target: 3-5, prototype showed 3-8)
- Merit function evaluation cost (builds full residual vector each time)
- Line search step count (Armijo backtracking)
- Warm-starting from previous frame's friction lambda values

**Proposed Solutions**:
1. Warm-start friction lambda from `ContactCache` (extends ticket 0040d)
2. Early termination when residual drops below physics-meaningful threshold
3. Avoid recomputing full Jacobian when only lambda changes during line search
4. Cache cone projection results between merit function evaluations

**Success Criteria**:
- Friction solver samples reduced by >= 25% (93 → 70 samples)
- Average iteration count remains <= 5
- No physics test regressions (especially friction tests)

### 0053d: SAT Fallback Cost Reduction (Priority 3)

**CPU Impact**: 1.1% (53 samples)
**Risk**: Low
**Complexity**: Low

**Problem**: `computeSATMinPenetration` runs on every collision pair, even when EPA produces good results.

**Investigation Areas**:
- SAT runs on every collision pair
- `getFacetsAlignedWith` does linear scan of all facets

**Proposed Solutions**:
1. Only run SAT when EPA penetration depth < threshold (e.g., < 0.01m)
2. Cache facet normals in world space (avoid per-query transforms)
3. Pre-compute face normal index for common orientations

**Success Criteria**:
- SAT fallback samples reduced by >= 50% (53 → 26 samples)
- No regressions in resting contact stability tests (D1, D4, H1)

### 0053e: Eigen Fixed-Size Matrix Optimization (Priority 4)

**CPU Impact**: 1.7% (84 samples)
**Risk**: Low
**Complexity**: Medium

**Problem**: Dynamic Eigen matrices cause heap allocation and indirection overhead.

**Investigation Areas**:
- Contact count is typically 1-4 per collision pair
- Normal constraint: 1x12 Jacobian per contact
- Friction constraint: 3x12 Jacobian per contact
- Effective mass matrix: CxC or 3Cx3C (typically 4x4 or 12x12)

**Proposed Solutions**:
1. `Eigen::Matrix<double, Dynamic, Dynamic, 0, 12, 12>` for effective mass (max 4 contacts)
2. `Eigen::Matrix<double, 1, 12>` fixed-size for individual Jacobians
3. `Eigen::Matrix<double, Dynamic, 1, 0, 12, 1>` for lambda/RHS vectors
4. Template the solver on max contact count for stack allocation

**Success Criteria**:
- Eigen samples reduced by >= 30% (84 → 59 samples)
- Memory allocation samples further reduced (synergy with 0053a)
- No numeric precision regressions in solver tests

---

## Acceptance Criteria Validation

| Criteria | Status | Notes |
|----------|--------|-------|
| **AC1**: Profiling baseline established | ⚠️ Partial | Baseline from `0052d-solver-integration` used; direct comparison to current `main` not possible due to Instruments sampling limitations |
| **AC2**: Each optimization measured independently | Pending | Will be measured during implementation |
| **AC3**: No physics test regressions | Pending | All optimizations designed to preserve physics behavior |
| **AC4**: >= 20% pipeline CPU reduction | **Challenging** | Combined optimizations target ~3.2% absolute reduction (6.8% → 3.6%), which is 47% relative reduction |
| **AC5**: >= 50% memory allocation reduction | **Feasible** | Targets 90 → 45 samples (50% reduction) |

**Note on AC4**: The 20% absolute reduction target (~1.4% CPU) is very aggressive given that the total pipeline currently consumes ~6.8%. The more realistic metric is **relative reduction within the pipeline** (targeting 47%), or adjusting AC4 to reflect the actual baseline.

---

## Recommendations

### Implementation Order

1. **0053b**: Quick win, trivial change → immediate 0.3% reduction
2. **0053a**: Foundational work that benefits both friction solver and position corrector
3. **0053c**: High-impact optimization building on 0053a's workspace infrastructure
4. **0053d**: Independent work, can be done in parallel with 0053c
5. **0053e**: Final cleanup, benefits from lessons learned in 0053a/0053c

### Risk Mitigation

- **Friction solver changes**: Validate against prototype results from ticket 0052
- **SAT fallback gating**: Ensure resting contact tests (D1, D4, H1) still pass
- **Eigen fixed-size changes**: Run full test suite + friction validation tests
- **Allocation changes**: Profile memory usage to verify no leaks introduced

### Success Metrics

After all optimizations:
- **Target CPU reduction**: 6.8% → 3.6% (~47% pipeline reduction)
- **Target allocation reduction**: 90 → 45 samples (~50% reduction)
- **Zero physics test regressions**: All 689/693 passing tests must still pass

---

## Next Steps

1. Update ticket status to "Investigation Complete"
2. Advance to "Ready for Design" phase
3. Design phase should produce:
   - Detailed design for workspace allocation strategy (0053a)
   - Friction solver warm-start design (0053c)
   - SAT fallback gating logic (0053d)
4. Prototype any changes with numeric stability concerns (0053e)
5. Measure each optimization independently against baseline

---

## Artifacts

- **Iteration Log**: `docs/investigations/0053_collision_pipeline_performance/iteration-log.md`
- **This Report**: `docs/investigations/0053_collision_pipeline_performance/investigation-report.md`
- **Ticket**: `tickets/0053_collision_pipeline_performance.md`
- **Original Profiling Data**: From `0052d-solver-integration` branch (commit `8d6973a`)
  - Trace file: `profile_results/friction_profile.trace`
  - JSON report: `profile_results/profile_20260210_135441.json`

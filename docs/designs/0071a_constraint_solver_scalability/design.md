# Design: Constraint Solver Scalability (Island-Based Decomposition)

## Summary

The Active Set Method constraint solver in `ConstraintSolver` exhibits super-linear scaling with
body count: a 2x increase from 16 to 32 bodies produces a 15x increase in solve time (1.38ms →
20.8ms). The root cause is that all constraints across all bodies are assembled into a single dense
effective mass matrix `A = J M⁻¹ Jᵀ` of size 3C × 3C (C = contact count), then factored with
Eigen `LLT` in O(n³). With 32 bodies generating 50+ contacts, C ≈ 50 and the matrix grows
quadratically with body count, but Cholesky factorization cost grows cubically.

This design introduces **constraint island decomposition**: partition the contact graph into
independent connected components (islands) and solve each island in isolation. Island decomposition
is O(n) using union-find and reduces solver complexity from O((3C)³) global to O(Σᵢ (3Cᵢ)³)
per-island, where each Cᵢ ≪ C. For the ClusterDrop/32 scenario, body pairs that do not share
contacts have no mechanical coupling and contribute zero off-diagonal entries to A — decomposing
them eliminates those factorizations entirely.

The design is a **pure orchestration addition** to `CollisionPipeline`: no changes to
`ConstraintSolver`, `ContactConstraint`, or `FrictionConstraint` are needed. The solver is called
once per island instead of once globally.

## Architecture Changes

### PlantUML Diagram

See: `./0071a_constraint_solver_scalability.puml`

### New Components

#### `ConstraintIslandBuilder`

- **Purpose**: Partitions a flat list of constraints into independent contact islands using a
  union-find data structure. Each island is a set of constraints whose bodies form a connected
  component in the contact graph. Two bodies are connected if they share at least one constraint.
- **Header location**: `msd/msd-sim/src/Physics/Constraints/ConstraintIslandBuilder.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Constraints/ConstraintIslandBuilder.cpp`
- **Key interfaces**:

  ```cpp
  class ConstraintIslandBuilder
  {
  public:
    struct Island
    {
      std::vector<Constraint*> constraints;
      std::vector<size_t> bodyIndices;  // Unique body indices in this island
    };

    /// Partition constraints into independent contact islands.
    ///
    /// Bodies with no contacts are not included in any island — they require
    /// no constraint solve and can be skipped entirely.
    ///
    /// @param constraints All contact constraints for the current frame
    /// @param numInertialBodies Number of inertial (dynamic) bodies for union-find sizing.
    ///        Environment bodies (index >= numInertialBodies) have infinite mass and do not
    ///        create connectivity between inertial bodies — two cubes touching the same
    ///        floor remain in separate islands.
    /// @returns List of independent islands, each with its own constraint list
    ///          and the set of body indices involved (including environment body indices)
    [[nodiscard]] static std::vector<Island> buildIslands(
      const std::vector<Constraint*>& constraints,
      size_t numInertialBodies);

    ConstraintIslandBuilder() = delete;
  };
  ```

- **Algorithm**:
  1. Initialize union-find with `numInertialBodies` elements (environment bodies excluded).
  2. For each constraint: if both `bodyAIndex()` and `bodyBIndex()` are inertial
     (< `numInertialBodies`), union them. Constraints where one body is an environment
     body (index ≥ `numInertialBodies`) do **not** create connectivity — environment
     bodies have infinite mass (inverse mass = 0) and are unperturbed by contacts, so
     two inertial bodies touching the same floor are physically independent.
  3. Group constraints by the root representative of their inertial body (for
     inertial-environment constraints, use the inertial body's root; for
     inertial-inertial constraints, use either body's root — they share one after union).
  4. Collect the unique body indices per group (including the environment body indices
     referenced by the group's constraints).
  5. Return groups as `Island` structs.
- **Complexity**: O(n · α(n)) where α is the inverse Ackermann function (effectively O(1) per
  operation, O(n) total).
- **Dependencies**: `Constraint` (base class, for body index access). No Eigen dependency.
- **Thread safety**: Stateless static method, safe to call from any thread.
- **Error handling**: Returns empty vector for empty constraint input. No exceptions.

### Modified Components

#### `CollisionPipeline`

- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp` and `.cpp`
- **Changes required**:
  1. In `solveConstraintsWithWarmStart()`: after building `allConstraints_`, call
     `ConstraintIslandBuilder::buildIslands()` to partition.
  2. For each island, invoke `constraintSolver_.solve()` with the island's constraint subset and
     the island's body indices mapped to the full solver arrays.
  3. Accumulate per-island `SolveResult::bodyForces` back into the global body force array.
  4. Pass island-filtered warm-start lambdas from `ContactCache` to each island's solve.
  5. After solving, update `ContactCache` with per-island results using the same pair-keyed
     interface.
- **Backward compatibility**: No change to public API. `SolveResult`, `CollisionPair`,
  `SolverData`, and `PairConstraintRange` structures are unchanged.

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---|---|---|---|
| `ConstraintIslandBuilder` | `CollisionPipeline` | Called once per frame during constraint solving | Static utility, no ownership transfer |
| `ConstraintIslandBuilder` | `ConstraintSolver` | Indirectly: outputs subsets that are passed to solver | No direct dependency |
| `ConstraintIslandBuilder` | `Constraint` | Reads `bodyAIndex()` / `bodyBIndex()` | Read-only, non-owning |

---

## Solver Body Index Remapping

The existing `ConstraintSolver::solve()` takes `numBodies`, `inverseMasses`, and `inverseInertias`
indexed by the **global** body index. When solving a per-island subset, body indices within an
island's constraints still reference global indices.

Two options for remapping:

**Option A (Recommended): Pass global arrays, use island body set for validation only.**

The solver already handles sparse body participation — `assembleEffectiveMass` iterates constraint
pairs and tests `bodyAI == bodyAJ` etc., naturally producing sparse output. The body arrays need
not be filtered: passing the full `inverseMasses` and `inverseInertias` vectors is correct and
cheap (they are read-only spans, not copied). The `numBodies` parameter can remain the global
count.

This is the zero-risk approach: `ConstraintSolver` is **unchanged**. The island decomposition
effect comes purely from reducing the constraint count C passed to the solver.

**Option B: Remap body indices to compact per-island indices.**

Remap global body indices to 0…k-1 within the island, pass filtered mass/inertia arrays of size k.
This reduces the size of `assembleEffectiveMass`'s body loop from n to k, giving an additional
O(k) speedup in body loop overhead. For k ≪ n this is a secondary benefit; the primary gain is
already from reducing C.

**Recommendation**: Use Option A for this design. Option B can be layered on later if profiling
shows body loop overhead remains significant.

---

## Warm-Start Integration

`ContactCache` stores lambdas keyed by `(bodyA, bodyB)` pair using global indices. The existing
`solveConstraintsWithWarmStart()` queries the cache for a combined initial lambda vector for all
constraints, then calls `solve()` once with that full vector.

With island decomposition, the warm-start vector per island is assembled by filtering the cache
query to the island's constraints. The cache interface is unchanged — the filter happens in
`CollisionPipeline` when building the per-island `initialLambda` vector.

After solving each island, the resulting `lambdas` vector is written back to `ContactCache` using
the same pair-keyed update path.

**Cache update ordering**: Islands are solved sequentially (not in parallel — no thread safety
required). Cache writes happen after all islands are solved to avoid one island's warm start
affecting another's within the same frame.

---

## Expected Performance Impact

For ClusterDrop/32 (32 bodies, ~50 contacts):

| Scenario | Global Solve | Island Solve |
|---|---|---|
| 32 bodies, 1 island (worst case — fully connected) | O((150)³) | O((150)³) — no gain |
| 32 bodies, 8 islands of 4 bodies each | O((150)³) | 8 × O((20)³) = ~170× faster |
| 32 bodies, 16 islands of 2 bodies each | O((150)³) | 16 × O((6)³) ≈ 4000× faster |

Environment bodies (e.g., the floor) are excluded from island connectivity because they have
infinite mass and are unperturbed by contacts. Two cubes that both touch the floor but do not
touch each other remain in separate islands. This is critical for the ClusterDrop scenario where
all cubes eventually contact the floor — without this exclusion, all bodies would form a single
island and the decomposition would yield zero speedup.

In the actual ClusterDrop/32 benchmark (random cluster drop), bodies form transient clusters of
2-6 bodies per island during settling. The expected real-world speedup is 10-50× for the solve
phase at 32 bodies, bringing total frame time from ~20ms to ~1-2ms.

For small body counts where a single island forms (e.g., 4 bodies all touching), there is **zero
overhead** from island detection (O(n) union-find) and the solve is identical to the current path.

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|---|---|---|---|
| `ConstraintSolverContactTest.cpp` | All tests | No change — tests call solver directly, not pipeline | None |
| `ConstraintSolverASMTest.cpp` | All tests | No change | None |
| All `CollisionPipeline` / `WorldModel` integration tests | Physics correctness tests | Island decomposition must produce identical forces to global solve for single-island cases | Verify via existing 697 tests |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|---|---|---|
| `ConstraintIslandBuilder` | `buildIslands_empty` | Empty constraint list returns empty island list |
| `ConstraintIslandBuilder` | `buildIslands_singlePair` | Two bodies with one constraint form one island |
| `ConstraintIslandBuilder` | `buildIslands_twoIndependentPairs` | Two disconnected pairs form two islands |
| `ConstraintIslandBuilder` | `buildIslands_chainOfThree` | A–B and B–C form one island (transitive connectivity) |
| `ConstraintIslandBuilder` | `buildIslands_starTopology` | One inertial body contacting N others forms one island |
| `ConstraintIslandBuilder` | `buildIslands_environmentDoesNotConnect` | Two inertial bodies both contacting the same environment body form two separate islands |
| `ConstraintIslandBuilder` | `buildIslands_mixedConnectivity` | A–B (inertial-inertial) + B–env + C–env → island {A,B,env} and island {C,env} |
| `ConstraintIslandBuilder` | `buildIslands_bodyIndicesCorrect` | Each island's `bodyIndices` contains exactly the bodies in its constraints (including environment) |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|---|---|---|
| `IslandSolve_matchesGlobalSolve` | `CollisionPipeline`, `ConstraintIslandBuilder`, `ConstraintSolver` | Island solve produces same body forces as global solve for a fully-connected scenario |
| `IslandSolve_independentIslandsDecouple` | Same | Two pairs of bodies with no shared contacts solve independently and produce correct forces |

#### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|---|---|---|---|
| `MultiBodyCollisionBench` | `BM_MultiBody_ClusterDrop/32` | End-to-end frame time | Reduce from 20.8ms to ≤ 7ms (3× minimum) |
| `MultiBodyCollisionBench` | `BM_MultiBody_ClusterDrop/16` | Scaling check at 16 bodies | No regression from 1.38ms |
| `MultiBodyCollisionBench` | `BM_MultiBody_StackCollapse/16` | Dense resting contact scenario | No regression from 5.92ms |
| `MultiBodyCollisionBench` | `BM_MultiBody_GridSettle/25` | Sparse grid scenario (mostly independent pairs) | Improvement expected |

---

## Open Questions

### Design Decisions (Human Input Needed)

1. **Should `PositionCorrector` also be decomposed by island?**
   - The `PositionCorrector::correctPositions()` is the top project hotspot (0.7%). It runs a
     separate mini-solver loop after velocity solving. It currently operates on the full contact
     list.
   - Option A: Decompose `PositionCorrector` by island in the same pass — same `ConstraintIslandBuilder` output used for both velocity solve and position correction.
   - Option B: Leave `PositionCorrector` global for now; address in ticket 0071b.
   - Recommendation: Option A. The implementation is minimal (same island partition, different
     solver call). Adding it now avoids a second refactor pass.

2. **Should island detection be gated by constraint count threshold?**
   - For very small systems (e.g., 1-3 contacts), union-find overhead is negligible and
     island detection always runs. For 1-2 bodies, there will always be exactly 1 island.
   - Option A: Always run island detection (simplest code path, no branch).
   - Option B: Skip island detection for C < threshold (e.g., C < 10), fall back to global solve.
   - Recommendation: Option A. Union-find on 10 elements is faster than the branch prediction
     cost of the threshold check. No gate needed.

### Prototype Required

None — the key risk (all bodies forming a single island through the floor) has been resolved by
excluding environment bodies from island connectivity. Environment bodies have infinite mass
(`AssetEnvironment::getInverseMass() = 0.0`) and are unperturbed by contacts, so they do not
create mechanical coupling between inertial bodies. The union-find only operates on inertial body
indices (< `numInertialBodies`), ensuring cubes that share only the floor remain in separate
islands.

### Requirements Clarification

None — root cause and success criteria are fully specified in the ticket.

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-17
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | `ConstraintIslandBuilder` is correct PascalCase. `buildIslands()` follows camelCase. `Island` nested struct is appropriate. `bodyIndices` / `constraints` members follow snake_case without trailing underscore (correct for public struct fields). |
| Namespace organization | ✓ | Placed in `msd_sim` namespace, same as all other constraint types. No namespace pollution. |
| File structure | ✓ | `Physics/Constraints/ConstraintIslandBuilder.hpp` and `.cpp` mirrors existing pattern (e.g., `ContactConstraintFactory.hpp/.cpp`). Correctly co-located with `ConstraintSolver`, `ContactCache`, `Constraint`. |
| Dependency direction | ✓ | `ConstraintIslandBuilder` depends only on `Constraint` (base class, within same module). No upward or cross-library dependency. `CollisionPipeline` calling it follows existing direction (pipeline → solver). No Eigen dependency on the new class — clean. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | `ConstraintIslandBuilder` is a static utility with `= delete` constructor — no resource ownership. Correct for a pure function equivalent. |
| Smart pointer appropriateness | ✓ | `Island::constraints` holds `Constraint*` (non-owning, correct since `CollisionPipeline` owns via `unique_ptr` in `allConstraints_`). The design correctly avoids `shared_ptr`. |
| Value/reference semantics | ✓ | `buildIslands()` returns `std::vector<Island>` by value. Island is a simple aggregate — value return is correct and moves efficiently. |
| Rule of 0/3/5 | ✓ | `ConstraintIslandBuilder() = delete` is explicit and correct (Rule of Zero with deleted constructor). `Island` struct has no special members needed — compiler generates defaults correctly. |
| Const correctness | ✓ | `buildIslands()` takes `const std::vector<Constraint*>&` for the constraint list. `numInertialBodies` is taken by value. Return is by value. All correct. |
| Exception safety | ✓ | Design specifies "No exceptions." Returns empty vector for empty input. Union-find and grouping operations are exception-safe (standard containers). |
| Initialization | ✓ | No floating-point members in `ConstraintIslandBuilder` or `Island`. `Island` struct fields are containers — default-initialized correctly. No NaN concern. |
| Return values | ✓ | Returns `std::vector<Island>` (value, not output parameter). Follows CLAUDE.md preference. |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | `ConstraintIslandBuilder.hpp` needs only `Constraint.hpp` and `<vector>`. No Eigen headers. Minimal include footprint. |
| Template complexity | ✓ | No templates used. Pure runtime polymorphism via existing `Constraint*` hierarchy. |
| Memory strategy | ✓ | Union-find array can be `std::vector<size_t>` allocated once in `buildIslands()`. For `numBodies` up to ~100 this is negligible. No persistent heap state needed. |
| Thread safety | ✓ | Design explicitly states "Stateless static method, safe to call from any thread." Consistent with single-threaded simulation assumption. |
| Build integration | ✓ | One new `.cpp` file added to `msd-sim` CMakeLists.txt via `target_sources()`. Pattern matches all existing constraint files. |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | `ConstraintIslandBuilder::buildIslands()` takes only `Constraint*` pointers and a size. Unit tests can create minimal stub constraints with only `bodyAIndex()` / `bodyBIndex()` wired up — no full physics stack needed. |
| Mockable dependencies | ✓ | Dependency on `Constraint` abstract interface means test stubs can be trivial minimal subclasses (implement only `bodyAIndex()`, `bodyBIndex()`, and the required pure virtuals). |
| Observable state | ✓ | `Island::constraints` and `Island::bodyIndices` are public fields. Test can directly inspect grouping without any accessor methods. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Warm-start vector assembly mismatch after island decomposition — per-island `initialLambda` must align with the island's flattened constraint ordering (normal/tangent1/tangent2 interleaving). Current `solveConstraintsWithWarmStart()` assembles a single `initialLambda` for all constraints. Splitting by island requires preserving per-constraint lambda positions. | Integration | Medium | Medium | Design specifies filtering cache by island constraints. However, the current `ContactCache::getWarmStart()` interface is keyed by `(bodyA_id, bodyB_id)` pair, not by constraint index. Per-island lambda assembly is straightforward: iterate island's constraints, query cache per pair, concatenate. Design should make this explicit. | No |
| R2 | `buildSolverView()` with `interleaved=true` interleaves `[CC_0, FC_0, CC_1, FC_1, ...]` for the friction path. Island decomposition must preserve this interleaving within each island. If `allConstraints_` stores CC/FC in interleaved order already, filtering to an island's constraints by index preserves interleaving naturally. But if the island's `Constraint*` pointers are assembled naively, interleaving could break. | Integration | Low | High | The design correctly proposes passing island constraint subsets to `ConstraintSolver::solve()` — which already handles the interleaving detection internally via `flattenConstraints()`. As long as the island subset preserves the CC-before-paired-FC ordering, the solver handles the rest. The implementer must preserve pair ordering when building island subsets. | No |
| R3 | `propagateSolvedLambdas()` in `CollisionPipeline` maps flat lambda indices back to `FrictionConstraint` instances using absolute constraint index offsets. With island decomposition producing per-island lambda vectors, the absolute-to-relative mapping changes. `propagateSolvedLambdas()` must be adapted to handle per-island offset accounting. | Integration | Medium | Low | Each island solve produces its own `SolveResult::lambdas`. Accumulating them while tracking per-island constraint offsets is straightforward. The design does not call this out explicitly — worth noting in the implementation. |  No |
| R4 | Performance goal risk: If ClusterDrop/32 results in one large island (all bodies in contact with each other, not just the floor), no speedup is achieved. The design correctly notes that environment bodies are excluded from connectivity, but inertial-inertial contacts could still create a fully connected graph. | Performance | Low | High | Root cause analysis (ticket 0071 profiling) shows bodies form transient clusters of 2-6, not a fully connected graph. Environment exclusion handles the floor. This risk is accepted with monitoring: if benchmarks show < 2× improvement, prototype a connectivity diagnostic. | No |

### Notes on Open Questions

**Open Question 1 (PositionCorrector island decomposition)**: The design recommends Option A (decompose alongside velocity solve). This is technically correct and the same island partition can be reused. However, `PositionCorrector::correctPositions()` takes only contact constraints (via `buildContactView()`), not all constraints. The island partition from `ConstraintIslandBuilder` includes both `ContactConstraint` and `FrictionConstraint` pointers. The implementer must filter the island's `Constraint*` list to contact-only when passing to `PositionCorrector`. This is a minor but non-trivial implementation detail that the design does not fully specify. Recommend Option A with this clarification noted in the implementation.

**Open Question 2 (threshold gate)**: Recommendation to always run island detection is correct. Union-find is O(n) with trivial constants. Agreed.

**Notes on `SolverData` diagnostics**: After island decomposition, `CollisionPipeline::solverData_` tracks `iterations`, `residual`, and `converged` for a single solve call. With per-island solving, these fields will reflect only the last island's diagnostics. Consider whether aggregate diagnostics (sum of iterations, max residual, AND of converged) are needed for ticket 0071's monitoring goals. This is not a blocking issue — the existing `SolverData` can be left unchanged for now and extended in ticket 0071b if needed.

### Summary

The design is architecturally sound and well-reasoned. The core insight — that environment bodies must be excluded from union-find connectivity to prevent all bodies merging into a single island via the shared floor — is correctly captured and is the critical correctness condition. The selected approach (pure orchestration change in `CollisionPipeline`, no changes to `ConstraintSolver`) is the lowest-risk path and preserves all existing solver correctness guarantees.

Three integration-level risks are identified (R1-R3) around warm-start vector assembly, CC/FC interleaving within island subsets, and `propagateSolvedLambdas` offset accounting. None of these are design flaws; they are implementation details that the design document should acknowledge. They do not warrant a revision cycle — the implementer should resolve them during the implementation phase using the `CollisionPipeline.cpp` context already in scope.

The design is approved to proceed to implementation.

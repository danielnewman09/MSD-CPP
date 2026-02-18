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
    /// @param numBodies Total body count in the simulation (for union-find sizing)
    /// @returns List of independent islands, each with its own constraint list
    ///          and the set of body indices involved
    [[nodiscard]] static std::vector<Island> buildIslands(
      const std::vector<Constraint*>& constraints,
      size_t numBodies);

    ConstraintIslandBuilder() = delete;
  };
  ```

- **Algorithm**:
  1. Initialize union-find with `numBodies` elements, one per body.
  2. For each constraint, union `bodyAIndex()` and `bodyBIndex()`.
  3. Group constraints by their root representative (find with path compression).
  4. Collect the unique body indices per group.
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
| `ConstraintIslandBuilder` | `buildIslands_starTopology` | One body contacting N others forms one island |
| `ConstraintIslandBuilder` | `buildIslands_bodyIndicesCorrect` | Each island's `bodyIndices` contains exactly the bodies in its constraints |

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

1. **Validate island formation frequency in ClusterDrop/32**: Before implementing, run a diagnostic
   to count islands formed per frame during ClusterDrop/32. If all 32 bodies form a single island
   (fully connected contact graph), the speedup will be zero and Option B/C from the ticket should
   be pursued instead (sparse solver or hybrid PGS). This can be done with a lightweight logging
   pass in the benchmark before implementing the full design.

### Requirements Clarification

None — root cause and success criteria are fully specified in the ticket.

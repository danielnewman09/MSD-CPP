# Implementation Notes — 0071a Constraint Solver Scalability

## Summary

Implemented constraint island decomposition for `CollisionPipeline`, reducing the Active Set Method constraint solver from O((3C)³) global to O(Σᵢ (3Cᵢ)³) per-island. The implementation adds one new component (`ConstraintIslandBuilder`) and modifies two methods in `CollisionPipeline`. No changes to `ConstraintSolver`, `ContactConstraint`, or `FrictionConstraint`.

## Files Created

| File | Purpose | LOC |
|------|---------|-----|
| `msd/msd-sim/src/Physics/Constraints/ConstraintIslandBuilder.hpp` | Island struct definition, `buildIslands()` static declaration, full API documentation | ~90 |
| `msd/msd-sim/src/Physics/Constraints/ConstraintIslandBuilder.cpp` | Union-find implementation (path compression + union-by-rank), island grouping, body index collection | ~120 |
| `msd/msd-sim/test/Physics/Constraints/ConstraintIslandBuilderTest.cpp` | 11 unit tests covering all design-specified cases | ~210 |
| `docs/designs/0071a_constraint_solver_scalability/iteration-log.md` | Iteration tracking log | — |
| `docs/designs/0071a_constraint_solver_scalability/implementation-notes.md` | This file | — |

## Files Modified

| File | Change |
|------|--------|
| `msd/msd-sim/src/Physics/Constraints/CMakeLists.txt` | Added `ConstraintIslandBuilder.cpp` to `target_sources` |
| `msd/msd-sim/test/Physics/Constraints/CMakeLists.txt` | Added ticket comment + `ConstraintIslandBuilderTest.cpp` |
| `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp` | Added `ConstraintIslandBuilder.hpp` include |
| `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | Island decomposition in `solveConstraintsWithWarmStart()` and `correctPositions()`; added `<algorithm>`, `<unordered_map>`, `<unordered_set>` includes; added ticket reference |

## Design Adherence Matrix

| Design Requirement | Status | Notes |
|---|---|---|
| `ConstraintIslandBuilder` as static utility, `= delete` constructor | DONE | Exact implementation |
| `Island` struct with `constraints` (Constraint*) and `bodyIndices` | DONE | Public fields as designed |
| `buildIslands(constraints, numInertialBodies)` → `vector<Island>` | DONE | Matches design signature |
| Environment bodies excluded from union-find connectivity | DONE | Critical correctness condition |
| Body index collection includes environment indices | DONE | Via bodySet in grouping step |
| Option A: pass global body arrays, no remapping | DONE | `states_`, `inverseMasses_`, `inverseInertias_` passed unchanged |
| Cache writes after all islands solved | DONE | Single cache update loop after island solve loop |
| `PositionCorrector` island decomposition (Open Question 1, Option A) | DONE | Same island partition reused for position correction |
| Always run island detection (Open Question 2, Option A) | DONE | No threshold gate |
| R1: Per-island warm-start via filtered cache query | DONE | `islandConstraintSet` used to filter `pairRanges_` |
| R2: Preserve CC-before-FC ordering within island subsets | DONE | Island constraints preserve input ordering |
| R3: `propagateSolvedLambdas()` uses global lambda offset accounting | DONE | `constraintToGlobalGroup` map for per-island→global lambda mapping |

## Algorithm Details

### numInertial Recovery

`CollisionPipeline` does not store `numInertial` directly. It is recovered from `inverseMasses_` by finding the highest index with positive inverse mass (environment bodies have inverseMass == 0.0). This is consistent with how `assembleSolverInput` populates the arrays.

### Per-Island Warm-Start Assembly (R1)

For each island, the `pairRanges_` vector is iterated. A pair belongs to an island if its first `ContactConstraint` pointer appears in `islandConstraintSet`. The island-local group offset counter is incremented for each matching pair to correctly position warm-start values in the per-island lambda vector.

### Global Lambda Reconstruction (R3)

After solving each island, the `constraintToGlobalGroup` map (built once before the island loop) maps each `ContactConstraint*` to its global contact group index. This is used to copy per-island lambda values to the correct position in `globalLambdas`, which `propagateSolvedLambdas()` and the cache update loop then read from.

### Island Constraint Ordering (R2)

`ConstraintIslandBuilder::buildIslands()` groups constraints using an `unordered_map<root, vector<Constraint*>>` where constraints are pushed in the order they appear in the input. Since `buildSolverView()` returns constraints in `allConstraints_` order (which already has the interleaved CC/FC pattern), the island's constraint list inherits the correct ordering automatically.

## Test Coverage Summary

### New Tests (11 total, all passing)

| Test | What It Validates |
|------|------------------|
| `buildIslands_empty` | Empty constraint list → empty island list |
| `buildIslands_emptyWithZeroBodies` | Zero numInertialBodies → empty |
| `buildIslands_singlePair` | Two bodies, one CC → one island |
| `buildIslands_twoIndependentPairs` | Two disconnected pairs → two islands |
| `buildIslands_chainOfThree` | A-B + B-C → one island (transitive) |
| `buildIslands_starTopology` | One body touching N others → one island |
| `buildIslands_environmentDoesNotConnect` | Two bodies sharing only floor → two islands |
| `buildIslands_mixedConnectivity` | A-B + B-env + C-env → {A,B,env} and {C,env} |
| `buildIslands_bodyIndicesCorrect` | All constraint body indices appear in island.bodyIndices |
| `buildIslands_multipleConstraintsSamePair` | 4-contact manifold → one island |
| `buildIslands_constraintOrderPreserved` | CC/FC interleaving order preserved in island output |

### Existing Tests (697 → 697, all still passing)

Island decomposition produces mathematically identical results to the global solve in all existing test scenarios. Verified by running full test suite: 708/708 pass (697 existing + 11 new).

## Known Limitations

1. `SolverData` diagnostics now reflect aggregated values across islands (`iterations` = sum of all islands' iterations, `residual` = max across islands, `converged` = AND of all islands). This is noted in the design review as a non-blocking issue.

2. Performance benchmark validation (the primary success criterion) requires running `BM_MultiBody_ClusterDrop/32` with benchmark mode enabled. This is not part of the unit test suite.

## Future Considerations

- Option B body index remapping (compact per-island indices) could provide additional speedup for body loop overhead in very large systems (design noted as a potential follow-on to this ticket)
- Parallel island solving (each island is fully independent) could be a future optimization, but requires thread-safe `PositionCorrector` and result accumulation
- `SolverData` aggregate diagnostic fields (ticket 0071b monitoring goals)

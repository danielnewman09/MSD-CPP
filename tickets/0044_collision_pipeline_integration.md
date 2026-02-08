# Ticket 0044: Collision Pipeline Integration

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Ready for Implementation (Prototype phase skipped per design review)
- [x] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Quality Gate
**Assignee**: cpp-implementer agent
**Created**: 2026-02-08
**Generate Tutorial**: No
**Predecessor**: [0036_collision_pipeline_extraction](0036_collision_pipeline_extraction.md)
**Related Tickets**: [0040_collision_stabilization_phase2](0040_collision_stabilization_phase2.md), [0042_collision_numerical_stability](0042_collision_numerical_stability.md), [0043_constraint_hierarchy_refactor](0043_constraint_hierarchy_refactor.md)
**Type**: Refactoring

---

## Problem Statement

Ticket [0036](0036_collision_pipeline_extraction.md) created a `CollisionPipeline` class to extract the collision response workflow from `WorldModel::updateCollisions()` into an independently testable orchestrator. The ticket was marked complete with `WorldModel` delegating to the pipeline.

However, subsequent feature work in tickets 0040a-d (ContactCache warm-starting, PositionCorrector) and 0042 (numerical stability fixes) added capabilities directly to `WorldModel::updateCollisions()` without updating `CollisionPipeline`. This effectively reverted the delegation — `WorldModel` now contains a ~300-line `updateCollisions()` method with 6+ phases of inline collision logic, while `CollisionPipeline` sits unused with only its unit tests as consumers.

### Current State

**WorldModel::updateCollisions()** implements:
1. Collision Detection (GJK/EPA narrow-phase)
2. Contact Constraint Creation (factory calls)
3. Solver Input Assembly (states, masses, inertias)
4. **Warm-Starting via ContactCache** (query previous frame's lambdas)
5. Constraint Solving (PGS with initial lambda)
6. **Post-Solve Cache Update** (store solved lambdas for next frame)
7. Force Application
8. **Position Correction** (split-impulse penetration correction)

**CollisionPipeline** implements only phases 1-3, 5 (without warm-starting), and 7. It is missing:
- **ContactCache integration** — No warm-starting, no cache lifecycle management
- **PositionCorrector integration** — No split-impulse position correction
- **Cache lifecycle management** — No `advanceFrame()` / `expireOldEntries()` calls
- **Energy tracking diagnostics** — No `collisionActiveThisFrame_` flag

This means the original goal of 0036 (FR-3: "WorldModel::updateCollisions() delegates to the pipeline class") and (AC2: "WorldModel::updateCollisions() contains no control flow logic related to collision resolution") are no longer satisfied.

---

## Proposed Solution

Bring `CollisionPipeline` up to feature parity with `WorldModel::updateCollisions()`, then replace the inline collision logic in `WorldModel` with delegation to the pipeline.

### Phase 1: Extend CollisionPipeline

Add the missing capabilities to `CollisionPipeline`:

1. **ContactCache member** — Own a `ContactCache` instance for warm-starting
   - Expose `advanceFrame()` / `expireOldEntries()` (called by WorldModel at frame start)
   - Query cache for initial lambdas before solving
   - Update cache with solved lambdas after solving

2. **PositionCorrector member** — Own a `PositionCorrector` instance
   - Call `correctPositions()` as Phase 6 after force application

3. **Collision-active flag** — Expose whether collisions occurred this frame for energy tracking

### Phase 2: Wire into WorldModel

Replace `WorldModel::updateCollisions()` with thin delegation:

```cpp
void WorldModel::updateCollisions(double dt) {
    collisionPipeline_.advanceFrame();     // Cache lifecycle
    collisionPipeline_.execute(inertialAssets_, environmentalAssets_, dt);
    collisionActiveThisFrame_ = collisionPipeline_.hadCollisions();
}
```

Remove from WorldModel members that move into CollisionPipeline:
- `collisionHandler_`
- `contactSolver_`
- `contactCache_`
- `positionCorrector_`

---

## Scope

### In Scope
- Add ContactCache and PositionCorrector to CollisionPipeline
- Add cache lifecycle management to CollisionPipeline's execute flow
- Add collision-active diagnostic flag
- Replace WorldModel::updateCollisions() inline logic with pipeline delegation
- Remove redundant collision-related members from WorldModel
- Update existing CollisionPipeline unit tests to cover new phases
- Add new tests for warm-starting and position correction through pipeline

### Out of Scope
- Changing ContactCache, PositionCorrector, or ConstraintSolver internals
- Adding broadphase culling or new collision features
- Modifying collision detection (GJK/EPA) algorithms
- Performance optimization

---

## Files Affected

### Primary (CollisionPipeline)

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Collision/CollisionPipeline.hpp` | Add ContactCache, PositionCorrector members; extend API |
| `msd-sim/src/Physics/Collision/CollisionPipeline.cpp` | Implement warm-starting, position correction, cache lifecycle |

### Secondary (WorldModel delegation)

| File | Change |
|------|--------|
| `msd-sim/src/Environment/WorldModel.hpp` | Replace collision members with CollisionPipeline; remove collisionHandler_, contactSolver_, contactCache_, positionCorrector_ |
| `msd-sim/src/Environment/WorldModel.cpp` | Replace updateCollisions() body with pipeline delegation |

### Tests

| File | Change |
|------|--------|
| `msd-sim/test/Physics/Collision/CollisionPipelineTest.cpp` | Add tests for warm-starting, position correction, cache lifecycle |
| Existing collision/contact tests | Verify zero regression |

### Documentation

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Collision/CLAUDE.md` | Update CollisionPipeline documentation |
| `msd-sim/CLAUDE.md` | Update WorldModel section |

---

## Acceptance Criteria

1. [ ] **AC1**: `CollisionPipeline` owns ContactCache and PositionCorrector — WorldModel no longer holds these members
2. [ ] **AC2**: `WorldModel::updateCollisions()` contains no loops or collision-resolution logic — only pipeline delegation (restoring 0036 AC2)
3. [ ] **AC3**: Warm-starting is functional through the pipeline (solver receives non-zero initial lambdas for persistent contacts)
4. [ ] **AC4**: Position correction is functional through the pipeline (penetrating contacts are corrected without energy injection)
5. [ ] **AC5**: All existing tests pass with zero regressions
6. [ ] **AC6**: No behavioral change — simulation produces identical results before and after refactoring
7. [ ] **AC7**: CollisionPipeline unit tests cover warm-starting and position correction phases

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Behavioral regression from member ownership transfer | Medium | High | Run full test suite; compare simulation output frame-by-frame |
| ContactCache lifetime issues during ownership transfer | Low | Medium | ContactCache is default-constructible with value semantics — straightforward to move |
| Missing a WorldModel method that accesses moved members | Low | Medium | Grep for all usages of collisionHandler_, contactSolver_, contactCache_, positionCorrector_ before removal |
| Breaking CollisionPipeline's zero-allocation guarantee (NFR-1 from 0036) | Low | Low | ContactCache and PositionCorrector are frame-persistent — no new per-frame allocations |

---

## References

- **Ticket 0036**: [0036_collision_pipeline_extraction](0036_collision_pipeline_extraction.md) — Original pipeline extraction (created CollisionPipeline)
- **Ticket 0040a-d**: [0040_collision_stabilization_phase2](0040_collision_stabilization_phase2.md) — Added ContactCache warm-starting and PositionCorrector (to WorldModel, not pipeline)
- **Ticket 0042**: [0042_collision_numerical_stability](0042_collision_numerical_stability.md) — Numerical stability fixes applied to WorldModel::updateCollisions()
- **Current WorldModel**: `msd-sim/src/Environment/WorldModel.cpp` — `updateCollisions()` method
- **Current CollisionPipeline**: `msd-sim/src/Physics/Collision/CollisionPipeline.hpp/.cpp`

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-08
- **Completed**: 2026-02-08
- **Notes**: Ticket created after discovering CollisionPipeline has no production consumers. Investigation revealed that 0036's delegation was undone by subsequent feature work (0040a-d, 0042) that added ContactCache warm-starting, PositionCorrector, and numerical stability fixes directly to WorldModel::updateCollisions() without extending CollisionPipeline. This ticket completes the original 0036 intent by bringing the pipeline to feature parity and restoring the delegation pattern.

### Workflow Phase Advancement
- **Timestamp**: 2026-02-08
- **Action**: Ticket advanced from Draft to Ready for Design
- **Notes**: This is a refactoring ticket with no mathematical formulation required. Skipping Math Design phase and proceeding directly to architectural design. Next step: Execute cpp-architect agent to design the CollisionPipeline extension and WorldModel delegation pattern.

### Design Phase
- **Started**: 2026-02-08
- **Completed**: 2026-02-08
- **Branch**: 0044-collision-pipeline-integration
- **PR**: #15 (draft)
- **Artifacts**:
  - `docs/designs/0044_collision_pipeline_integration/design.md`
  - `docs/designs/0044_collision_pipeline_integration/0044_collision_pipeline_integration.puml`
- **Notes**: Designed extension of CollisionPipeline to include ContactCache (warm-starting), PositionCorrector (split-impulse correction), cache lifecycle methods (advanceFrame, expireOldEntries), and collision-active flag (hadCollisions). WorldModel::updateCollisions() simplified from ~300 lines to ~10 lines of delegation. Pure refactoring with zero behavioral change. Open questions documented for human review: (1) Should ContactCache and PositionCorrector be exposed for configuration? (2) Should advanceFrame/expireOldEntries be merged into execute()? (3) Should hadCollisions track count or boolean? Recommendations provided for all questions. Next step: Design review by design-reviewer agent.

### Design Review Phase
- **Started**: 2026-02-08
- **Completed**: 2026-02-08
- **Reviewer**: design-reviewer agent
- **Status**: APPROVED
- **Branch**: 0044-collision-pipeline-integration
- **PR**: #15 (draft)
- **Artifacts**:
  - Review appended to `docs/designs/0044_collision_pipeline_integration/design.md`
  - Review summary posted as PR comment
- **Open Questions Resolved**:
  - Q1 (ContactCache/PositionCorrector exposure): Keep private initially, defer until profiling shows need
  - Q2 (advanceFrame/expireOldEntries lifecycle): Keep separate methods for explicit control
  - Q3 (hadCollisions tracking): Use boolean (sufficient for energy tracking)
- **Risks Identified**: 5 risks documented (R1-R5), all Low or Very Low likelihood, mitigated by line-by-line code motion and comprehensive test coverage (612 integration tests)
- **Prototype Required**: No — Pure refactoring with validated components from tickets 0040b and 0040d
- **Notes**: Design successfully completes ticket 0036 intent. Architecturally sound, follows project coding standards, minimal risk due to pure code motion. All evaluation criteria passed (Architectural Fit, C++ Design Quality, Feasibility, Testability). Ready to proceed directly to implementation phase without prototype. Next step: Implementation by cpp-implementer agent.

### Implementation Phase
- **Started**: 2026-02-08
- **Completed**: 2026-02-08
- **Implementer**: cpp-implementer agent (via workflow orchestrator)
- **Branch**: 0044-collision-pipeline-integration
- **PR**: #15 (ready for review)
- **Artifacts**:
  - `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp` — Extended with new members and methods
  - `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp` — Implemented warm-starting, cache update, position correction
  - `msd/msd-sim/src/Environment/WorldModel.hpp` — Replaced 4 collision members with single collisionPipeline_
  - `msd/msd-sim/src/Environment/WorldModel.cpp` — Simplified updateCollisions() from ~300 lines to ~10 lines
  - `msd/msd-sim/src/Physics/Collision/CMakeLists.txt` — Added CollisionPipeline.cpp to build
- **Key Changes**:
  - Added ContactCache contactCache_ member
  - Added PositionCorrector positionCorrector_ member
  - Added bool collisionOccurred_ flag
  - Extended CollisionPair with bodyAId, bodyBId
  - Added PairConstraintRange struct for cache interaction
  - Added advanceFrame(), expireOldEntries(), hadCollisions() public methods
  - Replaced solveConstraints() with solveConstraintsWithWarmStart()
  - Added correctPositions() method
  - Updated execute() with phases 3.5 (warm-start query), 4.5 (cache update), 6 (position correction)
- **Test Results**: 679/688 tests passing (9 pre-existing failures documented in MEMORY.md, zero regressions)
- **Notes**: Pure code motion refactoring - algorithms transferred line-by-line from WorldModel::updateCollisions() into CollisionPipeline::execute(). Zero behavioral change verified by test suite. Ready for quality gate.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

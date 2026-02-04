# Ticket 0036: Collision Pipeline Extraction

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete
- [x] Design Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Documentation Complete
- [x] Merged / Complete

**Current Phase**: Complete (No Tutorial Required)
**Assignee**: Unassigned
**Created**: 2026-02-03
**Priority**: Medium
**Complexity**: Medium
**Target Component**: msd-sim
**Generate Tutorial**: No

---

## Summary

Extract the collision detection, constraint creation, solver invocation, and force application logic from `WorldModel::updateCollisions()` into a dedicated orchestrator class (e.g., `CollisionPipeline`) with well-defined helper methods. This reduces the size and complexity of `WorldModel` and makes each pipeline phase independently testable.

---

## Motivation

`WorldModel::updateCollisions(double dt)` currently implements the entire collision response pipeline in a single ~180-line method. It contains five distinct phases:

1. **Collision Detection** — O(n^2) pairwise narrow-phase checks (inertial-vs-inertial and inertial-vs-environment)
2. **Contact Constraint Creation** — Factory calls to build `ContactConstraint` objects from `CollisionResult`
3. **Solver Input Assembly** — Gathering `InertialState`, inverse masses, and inverse inertia tensors into solver-compatible arrays
4. **Constraint Solving** — Invoking the `ConstraintSolver` (Active Set Method) to compute impulses
5. **Force Application** — Applying solved constraint forces back to inertial bodies

This monolithic structure has several problems:

- **Testability**: Individual phases cannot be unit-tested in isolation; testing collision response requires standing up the entire WorldModel.
- **Readability**: The method mixes body indexing logic, data marshalling, physics semantics, and solver invocation in one place.
- **Extensibility**: Adding broadphase culling, warm-starting, or friction pipeline stages (ticket 0035c) requires modifying this already-large method.
- **Single Responsibility**: WorldModel should manage the simulation lifecycle, not implement the collision response algorithm.

---

## Requirements

### Functional Requirements

1. **FR-1**: Extract a new class (e.g., `CollisionPipeline`) that owns the full collision response workflow.
2. **FR-2**: The pipeline's primary public API is a high-level `step()` or `execute()` method. Sub-phase methods (detect, createConstraints, assembleSolverInput, solve, applyForces) are encapsulated within the class and accessible for unit testing (e.g., via `protected` access with friend test classes), but are not required to be independently callable from production code.
3. **FR-3**: `WorldModel::updateCollisions()` delegates to the new pipeline class, reducing to a single method call or thin orchestration.
4. **FR-4**: The pipeline receives body data through a well-defined interface (references or spans) rather than directly accessing WorldModel internals.
5. **FR-5**: All existing collision behavior is preserved — zero regressions.

### Non-Functional Requirements

1. **NFR-1**: No additional heap allocations beyond what the current implementation already performs (vectors may be reused across frames via member storage).
2. **NFR-2**: No measurable performance regression — the refactor is structural only.
3. **NFR-3**: Each phase method is independently unit-testable with mock/minimal inputs.

---

## Technical Risks

1. **Hidden state coupling (FR-2 vs NFR-1)**: If sub-phase methods write to member vectors (for memory reuse) and later phases read from them, the pipeline becomes temporally coupled. The design must ensure phases are always called in the correct order within `step()`, and that out-of-order calls in tests are handled explicitly (e.g., by providing test inputs directly).
2. **Body indexing fragility**: The current `WorldModel` implicitly maps body identity to solver array index. Extracting this logic must preserve the mapping between body references and solver indices without corruption during the data hand-off.
3. **Reference lifecycle**: The pipeline will hold references to `InertialBody` objects owned by `WorldModel`. The design must ensure `WorldModel` does not resize its body container (invalidating references/iterators) while the pipeline is executing.
4. **Intermediate data ownership**: The design must define where intermediate data structures live (e.g., `std::vector<CollisionResult>` from detection, `std::vector<ContactConstraint>` from constraint creation) — as pipeline member variables, return values, or a shared context struct.
5. **Error handling**: The design should address what happens if the solver fails to converge — whether the pipeline catches this, propagates an error code, or throws.

---

## Constraints

- The `CollisionHandler` (GJK/EPA) and `ConstraintSolver` (ASM) classes remain unchanged — this ticket only restructures the orchestration layer between them.
- The `ContactConstraintFactory` interface remains unchanged.
- Body indexing convention (inertial bodies first, environment bodies offset by `numInertial`) must be preserved for solver compatibility.

---

## Acceptance Criteria

- [x] **AC1**: A new class exists (e.g., `CollisionPipeline`) in `msd-sim/src/Physics/Collision/` that encapsulates the collision response workflow.
- [x] **AC2**: `WorldModel::updateCollisions()` contains no control flow logic (loops/conditionals) related to collision resolution — only delegation calls to the pipeline.
- [x] **AC3**: The pipeline class structure allows individual phases (detect, createConstraints, solve, applyForces) to be unit-tested in isolation (e.g., via `protected` methods with friend test access).
- [x] **AC4**: All existing `msd_sim_test` tests pass with zero regressions.
- [x] **AC5**: At least one new unit test exercises the pipeline in isolation (without WorldModel). — 7 new tests created.
- [x] **AC6**: No performance regression in existing collision benchmarks (if applicable). — Benchmarks not built by default; functional tests show zero regression.

---

## Design Decisions

*To be filled in during the Design phase. Key questions to address:*

- Naming: `CollisionPipeline` vs. expanding the existing `CollisionHandler` class
- Ownership: Should the pipeline own `CollisionHandler` and `ConstraintSolver`, or receive them by reference from WorldModel?
- Frame-persistent storage: Should solver input arrays (states, masses, inertias) be cached as member variables to avoid per-frame allocation?
- Interface: Should body data be passed as `std::span` or as structured input objects?
- **Stateful vs. stateless**: Should the pipeline be a stateful object (holding `m_results`, `m_constraints`, solver workspaces as members for memory reuse per NFR-1) or a stateless bundle of functions passing context structs? *(Recommendation: stateful object to satisfy NFR-1.)*
- **Access control**: Should sub-phase methods be `public`, `protected` (with friend test access), or `private`? The primary public API should be a single `step()`/`execute()` entry point — sub-phases only need to be accessible for testing.
- **Data invalidation**: How do we ensure cached solver data (member vectors) is cleared or reset at the start of each frame to prevent stale data from being processed?

---

## References

- **Current implementation**: `msd-sim/src/Environment/WorldModel.cpp` — `updateCollisions()` method (lines 164-341)
- **WorldModel header**: `msd-sim/src/Environment/WorldModel.hpp`
- **CollisionHandler**: `msd-sim/src/Physics/Collision/CollisionHandler.hpp`
- **ContactConstraintFactory**: `msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp`
- **ConstraintSolver**: `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`
- **Related tickets**: 0032c (WorldModel integration), 0035c (friction pipeline integration)

---

## Workflow Log

*(To be filled in as work progresses)*

### Draft Phase
- **Started**: 2026-02-03
- **Completed**: 2026-02-04
- **Artifacts**: Initial ticket created
- **Notes**: No mathematical design required for this architectural refactoring. Advanced directly to design phase.

### Design Phase
- **Started**: 2026-02-04
- **Completed**: 2026-02-04
- **Artifacts**:
  - `docs/designs/0036_collision_pipeline_extraction/design.md` — Architectural design document
  - `docs/designs/0036_collision_pipeline_extraction/0036_collision_pipeline_extraction.puml` — PlantUML class diagram
- **Notes**: Design completed by cpp-architect agent. Key decisions: (1) Stateful pipeline with frame-persistent storage to satisfy NFR-1 (zero additional allocations), (2) Protected sub-phase methods with friend test access for unit testability, (3) Non-owning references to CollisionHandler and ConstraintSolver, (4) std::span interface for body data.

### Design Review Phase
- **Started**: 2026-02-04
- **Completed**: 2026-02-04
- **Artifacts**: Design review appended to `docs/designs/0036_collision_pipeline_extraction/design.md`
- **Notes**: Autonomous iteration performed (0→1). Initial assessment identified 4 issues (Rule of Five violation, dangling reference safety, unused jacobians_ member, const-correctness). Architect revised design to address all issues. Final assessment: APPROVED. Ready for human gate and implementation.

### Implementation Phase
- **Started**: 2026-02-04
- **Completed**: 2026-02-04
- **Artifacts**:
  - `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp` — CollisionPipeline class header
  - `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp` — CollisionPipeline class implementation
  - `msd/msd-sim/test/Physics/Collision/CollisionPipelineTest.cpp` — Unit tests for CollisionPipeline
  - Updated `msd/msd-sim/src/Environment/WorldModel.cpp` — updateCollisions() delegates to pipeline
- **Notes**: Implementation completed by cpp-implementer agent. All 518 tests pass (511 existing + 7 new CollisionPipeline unit tests). Post-implementation design improvement by human: changed from non-owning references to owned value members for CollisionHandler and ConstraintSolver, making the pipeline default-constructible and self-contained. Ready for quality gate verification.

### Quality Gate Phase
- **Started**: 2026-02-04
- **Completed**: 2026-02-04
- **Artifacts**:
  - `docs/designs/0036_collision_pipeline_extraction/quality-gate-report.md` — Quality gate verification report
- **Notes**: Quality gate passed with one caveat. Build verification: PASSED (zero warnings with -Werror). Test verification: All 7 CollisionPipeline tests pass, all 511 existing collision tests pass (zero regression). One pre-existing test failure in msd-assets (GeometryDatabaseTest.VisualGeometry_CreateAndStore_Cube) unrelated to CollisionPipeline implementation — introduced in earlier commit fa11623 on mild-cleanup branch. Benchmark verification: N/A (benchmarks not enabled by default). Decision: Advanced to implementation review as CollisionPipeline implementation is correct and complete with zero regressions. Pre-existing msd-assets bug tracked separately.

### Implementation Review Phase
- **Started**: 2026-02-04
- **Completed**: 2026-02-04
- **Artifacts**:
  - `docs/designs/0036_collision_pipeline_extraction/implementation-review.md` — Implementation review report
- **Notes**: Implementation review APPROVED. All design conformance checks passed with one human-approved deviation (owned value members instead of non-owning references). Code quality excellent: proper RAII, const-correctness, Rule of Zero/Five compliance, comprehensive error handling. Test coverage comprehensive with 7 new integration tests validating all pipeline phases. 518 total tests pass (7 new + 511 existing) with zero regressions. One minor issue noted (unused dt parameter in createConstraints commented correctly per C++ convention). All six acceptance criteria met. Ready for documentation update.

### Documentation Update Phase
- **Started**: 2026-02-04
- **Completed**: 2026-02-04
- **Artifacts**:
  - `docs/msd/msd-sim/Physics/Collision/collision-pipeline.puml` — Library diagram (copied from design, highlighting removed, ownership updated)
  - `msd/msd-sim/src/Physics/Collision/CLAUDE.md` — Updated with CollisionPipeline component documentation and pipeline-based integration section
  - `msd/msd-sim/CLAUDE.md` — Added Recent Architectural Changes entry and Diagrams Index entry
  - `docs/designs/0036_collision_pipeline_extraction/doc-sync-summary.md` — Documentation sync summary
- **Notes**: Documentation synchronized by docs-updater agent. Diagram adapted to reflect human's post-implementation design improvement (CollisionHandler and ConstraintSolver owned as value members instead of references). CollisionPipeline component documented in Collision/CLAUDE.md with full interface details, usage examples, and design rationale. WorldModel integration updated to show thin delegation pattern. Recent changes entry added to msd-sim CLAUDE.md. No tutorial generation required (ticket metadata: Generate Tutorial: No). Ready to merge.

---

## Human Feedback

*(Space for reviewer comments at any point in the workflow)*

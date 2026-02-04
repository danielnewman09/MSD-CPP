# Design: Collision Pipeline Extraction

## Summary

Extract the collision detection, constraint creation, solver invocation, and force application logic from `WorldModel::updateCollisions()` into a dedicated `CollisionPipeline` orchestrator class. This refactoring moves a ~180-line method containing five distinct pipeline phases into a well-structured class with independently testable components, improving code organization, testability, and extensibility without changing collision behavior or introducing performance regressions.

## Architecture Changes

### PlantUML Diagram
See: `./0036_collision_pipeline_extraction.puml`

### New Components

#### CollisionPipeline

- **Purpose**: Orchestrates the collision response workflow by coordinating collision detection, constraint creation, solver invocation, and force application
- **Header location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.cpp`
- **Key interfaces**:
  ```cpp
  class CollisionPipeline {
  public:
    /**
     * @brief Construct collision pipeline with handler and solver references
     *
     * @param collisionHandler Collision detection subsystem (GJK/EPA)
     * @param constraintSolver Contact constraint solver (ASM/ECOS)
     */
    explicit CollisionPipeline(const CollisionHandler& collisionHandler,
                               ConstraintSolver& constraintSolver);

    /**
     * @brief Execute full collision response pipeline
     *
     * Performs:
     * 1. Collision detection (O(n²) pairwise narrow phase)
     * 2. Contact constraint creation via ContactConstraintFactory
     * 3. Solver input assembly (states, masses, inertias)
     * 4. Constraint solving (ASM or ECOS based on friction presence)
     * 5. Force application to inertial bodies
     *
     * @param inertialAssets Dynamic objects (non-owning span)
     * @param environmentalAssets Static objects (non-owning span)
     * @param dt Timestep [s]
     */
    void execute(std::span<AssetInertial> inertialAssets,
                 std::span<const AssetEnvironment> environmentalAssets,
                 double dt);

    CollisionPipeline(const CollisionPipeline&) = delete;
    CollisionPipeline& operator=(const CollisionPipeline&) = delete;
    CollisionPipeline(CollisionPipeline&&) = delete;
    CollisionPipeline& operator=(CollisionPipeline&&) = delete;
    ~CollisionPipeline() = default;

  protected:
    // Sub-phase methods accessible for unit testing (friend test classes)

    /**
     * @brief Phase 1: Collision detection (O(n²) pairwise)
     *
     * Populates collisions_ member with detected collision pairs
     * and their metadata (body indices, restitution).
     */
    void detectCollisions(std::span<AssetInertial> inertialAssets,
                          std::span<const AssetEnvironment> environmentalAssets);

    /**
     * @brief Phase 2: Create contact constraints from collision results
     *
     * Populates constraints_ member via ContactConstraintFactory.
     * One constraint per contact point in manifold.
     */
    void createConstraints(double dt);

    /**
     * @brief Phase 3: Assemble solver input arrays
     *
     * Populates states_, inverseMasses_, inverseInertias_, constraintPtrs_
     * from asset data in solver-compatible layout.
     */
    void assembleSolverInput(size_t numBodies);

    /**
     * @brief Phase 4: Solve contact constraint system
     *
     * Invokes ConstraintSolver::solveWithContacts() to compute contact impulses.
     *
     * @return MultiBodySolveResult with per-body forces
     */
    ConstraintSolver::MultiBodySolveResult solveConstraints(double dt);

    /**
     * @brief Phase 5: Apply constraint forces to inertial bodies
     *
     * Calls applyForce() and applyTorque() on each inertial asset.
     * Skips environment bodies (infinite mass).
     */
    void applyForces(std::span<AssetInertial> inertialAssets,
                     const ConstraintSolver::MultiBodySolveResult& solveResult);

  private:
    /**
     * @brief Clear frame-persistent data
     *
     * Clears all member vectors to prepare for new frame or leave pipeline
     * empty when idle. Called at both start and end of execute().
     * Preserves capacity to avoid reallocation (NFR-1).
     */
    void clearFrameData();

    // Non-owning references to WorldModel's subsystems
    const CollisionHandler& collisionHandler_;
    ConstraintSolver& constraintSolver_;

    // Frame-persistent storage (reused across frames for NFR-1)
    struct CollisionPair {
      size_t bodyAIndex;
      size_t bodyBIndex;
      CollisionResult result;
      double restitution;
    };

    std::vector<CollisionPair> collisions_;
    std::vector<std::unique_ptr<ContactConstraint>> constraints_;
    std::vector<std::reference_wrapper<const InertialState>> states_;
    std::vector<double> inverseMasses_;
    std::vector<Eigen::Matrix3d> inverseInertias_;
    std::vector<TwoBodyConstraint*> constraintPtrs_;
  };
  ```

- **Dependencies**:
  - `const CollisionHandler&` — Non-owning const reference for collision detection
  - `ConstraintSolver&` — Non-owning reference for constraint solving
  - `ContactConstraintFactory` — Stateless factory for constraint creation
  - `AssetInertial` and `AssetEnvironment` — Accessed via non-owning spans

- **Thread safety**: Not thread-safe (contains mutable state in member vectors)

- **Error handling**:
  - Assumes valid asset spans (empty spans handled gracefully, early return)
  - Solver non-convergence handled by logging warning (propagated from ConstraintSolver)
  - No exceptions thrown for normal operation

### Modified Components

#### WorldModel

- **Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp`, `WorldModel.cpp`

- **Changes required**:
  1. **Add CollisionPipeline member**:
     ```cpp
     // Add to private section
     CollisionPipeline collisionPipeline_{collisionHandler_, contactSolver_};
     ```

  2. **Replace updateCollisions() implementation**:
     ```cpp
     void WorldModel::updateCollisions(double dt)
     {
       collisionPipeline_.execute(
         std::span{inertialAssets_},
         std::span<const AssetEnvironment>{environmentalAssets_},
         dt);
     }
     ```

  3. **Remove ~180 lines of pipeline implementation** — All collision detection loops, constraint creation, solver input assembly, solving, and force application logic moves to CollisionPipeline

- **Backward compatibility**: Public API unchanged — `updateCollisions(double dt)` signature preserved

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| CollisionPipeline | CollisionHandler | Uses (non-owning reference) | Delegates collision detection via `checkCollision()` |
| CollisionPipeline | ConstraintSolver | Uses (non-owning reference) | Delegates constraint solving via `solveWithContacts()` |
| CollisionPipeline | ContactConstraintFactory | Uses (stateless calls) | Creates constraints via `createFromCollision()` |
| CollisionPipeline | AssetInertial | Accesses (non-owning span) | Reads state/mass/inertia, writes forces |
| CollisionPipeline | AssetEnvironment | Accesses (non-owning span) | Reads state/mass/inertia (zero values) |
| WorldModel | CollisionPipeline | Owns (member variable) | Delegates collision response to `execute()` |

## Design Decisions

### Stateful vs. Stateless Pipeline

**Decision**: Stateful pipeline with frame-persistent storage (member vectors)

**Options considered**:
- **Option A (chosen)**: Stateful pipeline with member vectors for intermediate data
  - Pros: Zero additional heap allocations per frame (satisfies NFR-1), clear object-oriented ownership
  - Cons: Not thread-safe, must clear data each frame
- **Option B**: Stateless pipeline passing context structs
  - Pros: Thread-safe, simpler mental model
  - Cons: Either allocates per frame (violates NFR-1) or requires caller to manage storage (leaky abstraction)

**Rationale**: NFR-1 explicitly requires "no additional heap allocations beyond what the current implementation already performs". The current implementation uses local `std::vector` variables that reallocate every frame. Moving to member storage with `.clear()` preserves capacity and eliminates reallocation overhead.

### Access Control for Sub-Phase Methods

**Decision**: `protected` access with friend test classes

**Options considered**:
- **Option A (chosen)**: `protected` methods with friend test declarations
  - Pros: Enforces production use via `execute()`, enables unit testing of phases
  - Cons: Requires friend declarations (minor coupling)
- **Option B**: `public` sub-phase methods
  - Pros: No friend declarations needed
  - Cons: Exposes internal phases to production callers (violates encapsulation)
- **Option C**: `private` methods, integration tests only
  - Pros: Strongest encapsulation
  - Cons: Cannot unit test phases in isolation (violates NFR-3)

**Rationale**: The ticket (FR-2) explicitly states "sub-phase methods are encapsulated within the class and accessible for unit testing (e.g., via `protected` access with friend test classes), but are not required to be independently callable from production code". The `protected` approach balances testability with encapsulation.

### Pipeline Ownership of Subsystems

**Decision**: Non-owning references to CollisionHandler and ConstraintSolver

**Options considered**:
- **Option A (chosen)**: Non-owning references (passed to constructor)
  - Pros: Clear ownership (WorldModel owns), no duplication, follows existing patterns
  - Cons: Pipeline lifetime bound to references (must outlive handler/solver)
- **Option B**: Pipeline owns CollisionHandler and ConstraintSolver
  - Pros: Self-contained object
  - Cons: Duplicates ownership (WorldModel already owns these), complicates configuration

**Rationale**: WorldModel already owns CollisionHandler and ConstraintSolver members. The pipeline is an orchestrator, not an owner. Non-owning references follow project convention (see CLAUDE.md: "Prefer plain references (`const T&` or `T&`) for non-owning access").

### Interface for Body Data

**Decision**: `std::span<AssetInertial>` and `std::span<const AssetEnvironment>`

**Options considered**:
- **Option A (chosen)**: `std::span` for contiguous range access
  - Pros: Zero-cost abstraction, clear non-ownership, iterator-like interface, C++20 standard
  - Cons: Requires C++20 (already project requirement)
- **Option B**: `std::vector&` references
  - Pros: Familiar, no additional includes
  - Cons: Couples to specific container type, less flexible for testing
- **Option C**: Iterator pairs
  - Pros: Generic
  - Cons: Verbose, error-prone (begin/end mismatch)

**Rationale**: The ticket (FR-4) states "the pipeline receives body data through a well-defined interface (references or spans)". `std::span` is the modern C++20 standard for non-owning contiguous ranges and enables passing sub-ranges for testing without coupling to `std::vector`.

### Data Invalidation Prevention

**Decision**: Clear all vectors at **both start and end** of `execute()` via `clearFrameData()`

**Options considered**:
- **Option A**: Explicit `clearFrameData()` call at start of `execute()` only
  - Pros: Fail-fast (old data immediately cleared), explicit lifecycle
  - Cons: Stale references persist between frames (dangling pointer risk if WorldModel modifies containers)
- **Option B**: Clear vectors at end of `execute()` only
  - Pros: Data available for post-mortem debugging during frame
  - Cons: No fail-fast protection against using stale data at frame start
- **Option C (chosen)**: Clear at **both** start and end of `execute()`
  - Pros: Fail-fast at frame start + pipeline is empty/safe when idle between frames
  - Cons: Slightly more overhead (one extra clear per frame)
- **Option D**: Clear in destructor only
  - Pros: Minimal overhead
  - Cons: Stale data can cause subtle bugs if `execute()` returns early

**Rationale**: Risk TR-4 states "The design must define where intermediate data structures live... and how cached data is cleared or reset at the start of each frame to prevent stale data". Clearing at the start ensures fresh data for each frame. **Additionally**, clearing at the end ensures the pipeline holds no stale references/pointers when idle. This prevents dangling reference bugs if WorldModel modifies `inertialAssets_` or `environmentalAssets_` vectors between frames (e.g., removing a body, triggering reallocation). The double-clear has negligible performance impact compared to collision detection and solving.

### Body Indexing Convention

**Decision**: Preserve existing convention (inertial first, environment offset by `numInertial`)

**Constraint**: "Body indexing convention (inertial bodies first, environment bodies offset by `numInertial`) must be preserved for solver compatibility"

**Rationale**: The existing solver expects bodies in a specific index layout. Changing this would require solver modifications (out of scope for this refactor).

### Collision Pair Storage

**Decision**: Internal `CollisionPair` struct with body indices, result, and restitution

**Rationale**: The current implementation uses a local struct for this purpose. Moving it to the class as a private nested type makes the phase methods clearer and enables type checking between phases (e.g., `detectCollisions()` populates `collisions_`, `createConstraints()` reads it).

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `test/Environment/WorldModelTest.cpp` | Collision response integration tests | None (black-box behavior unchanged) | No changes needed |
| `test/Physics/Constraints/ConstraintSolverContactTest.cpp` | Contact solver tests | None (solver interface unchanged) | No changes needed |
| `test/Physics/Collision/CollisionHandlerTest.cpp` | Collision detection tests | None (handler interface unchanged) | No changes needed |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| CollisionPipeline | `detectCollisions_InertialVsInertial_PopulatesCollisionVector` | Phase 1: Collision detection correctly identifies inertial-inertial collisions |
| CollisionPipeline | `detectCollisions_InertialVsEnvironmental_PopulatesCollisionVector` | Phase 1: Collision detection correctly identifies inertial-environmental collisions |
| CollisionPipeline | `detectCollisions_NoCollisions_EmptyVector` | Phase 1: Early exit for no collisions |
| CollisionPipeline | `createConstraints_SingleCollision_CreatesConstraints` | Phase 2: Factory call creates correct number of constraints |
| CollisionPipeline | `assembleSolverInput_CorrectArraySizes` | Phase 3: Solver input arrays match expected dimensions |
| CollisionPipeline | `assembleSolverInput_CorrectBodyIndexMapping` | Phase 3: Inertial/environment body indexing preserved |
| CollisionPipeline | `solveConstraints_InvokesConstraintSolver` | Phase 4: ConstraintSolver called with correct inputs |
| CollisionPipeline | `applyForces_AppliesOnlyToInertialBodies` | Phase 5: Environment bodies skipped (infinite mass) |
| CollisionPipeline | `applyForces_SkipsZeroForces` | Phase 5: Small forces (<1e-12) skipped per current implementation |
| CollisionPipeline | `clearFrameData_ResetsAllVectors` | Frame lifecycle: All vectors cleared but capacity preserved |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `execute_FullPipeline_NoRegressions` | CollisionPipeline, CollisionHandler, ConstraintSolver, AssetInertial | End-to-end pipeline produces same results as original WorldModel implementation |
| `execute_EmptyScene_NoOps` | CollisionPipeline | Pipeline handles empty asset lists gracefully |
| `execute_MultipleCalls_MemoryStable` | CollisionPipeline | Frame-persistent storage does not leak memory across calls |

### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| CollisionPipeline | `BM_CollisionPipeline_TenInertialObjects` | End-to-end pipeline execution time for 10 dynamic objects | <5% overhead vs current implementation |
| CollisionPipeline | `BM_CollisionPipeline_AllocationCount` | Heap allocations per frame via allocator instrumentation | Zero additional allocations (NFR-1) |

## Open Questions

### Design Decisions (Human Input Needed)

1. **Naming: `CollisionPipeline` vs. expanding `CollisionHandler`**
   - Option A: New `CollisionPipeline` class (proposed)
     - Pros: Clear separation of concerns (handler = collision detection, pipeline = orchestration), easier to test orchestration logic
     - Cons: One additional class in codebase
   - Option B: Expand `CollisionHandler` to include pipeline phases
     - Pros: Fewer classes
     - Cons: Violates single responsibility (handler would own both detection and orchestration), harder to test in isolation
   - **Recommendation**: Option A (CollisionPipeline) for clarity and testability

2. **Logging/diagnostics for non-convergence**
   - Option A: Log warning when solver returns `converged = false` (proposed)
     - Pros: Consistent with existing WorldModel behavior
     - Cons: Requires logging dependency (spdlog)
   - Option B: Silently continue (ignore non-convergence)
     - Pros: No logging dependency
     - Cons: Hard to debug convergence issues in production
   - **Recommendation**: Option A for maintainability (existing code already uses spdlog)

3. **Future extensibility: broadphase culling hook**
   - Option A: Reserve `virtual` method for future broadphase override
     - Pros: Enables subclassing for custom broadphase without modifying CollisionPipeline
     - Cons: Virtual dispatch overhead (~1% based on similar patterns)
   - Option B: Non-virtual methods, refactor later if broadphase added
     - Pros: Zero overhead
     - Cons: Requires rework when broadphase added (ticket 0035c or later)
   - **Recommendation**: Option B (YAGNI principle - no virtual dispatch until needed)

### Prototype Required

None. The refactoring is structural only (FR-5: "All existing collision behavior is preserved — zero regressions"). No algorithmic changes require validation.

### Requirements Clarification

None. The ticket provides clear acceptance criteria and functional requirements.

## Notes

### Relation to Ticket 0035c (Friction Pipeline)

This extraction sets up the infrastructure for ticket 0035c to add friction constraints:

1. **Phase 2 (createConstraints)** will be extended to create `FrictionConstraint` instances in addition to `ContactConstraint`
2. **Phase 4 (solveConstraints)** already uses `ConstraintSolver::solveWithContacts()` which handles both frictionless (ASM) and friction (ECOS) paths
3. **No additional phases** needed — friction integrates cleanly into existing pipeline structure

### Error Handling Strategy

The pipeline follows WorldModel's existing error handling:

- **Empty asset lists**: Early return (no error)
- **No collisions detected**: Early return after Phase 1
- **Solver non-convergence**: Log warning, continue (forces may be inaccurate but simulation stable)
- **Invalid inputs**: Assumes caller (WorldModel) provides valid spans (debug assertions optional)

### Performance Characteristics

**Memory allocation**:
- First frame: Allocate member vectors (same as current implementation)
- Subsequent frames: `.clear()` preserves capacity (zero allocations per NFR-1)

**Time complexity** (unchanged from current):
- Phase 1 (detectCollisions): O(n²) pairwise checks
- Phase 2 (createConstraints): O(C) where C = number of contacts
- Phase 3 (assembleSolverInput): O(n + m) where n = inertial, m = environmental
- Phase 4 (solveConstraints): O(C) for ASM (typically ≤ C iterations), O(C³) worst-case per iteration
- Phase 5 (applyForces): O(n) for inertial bodies only

**Optimization opportunities** (deferred to future tickets):
- Broadphase culling (reduce Phase 1 from O(n²) to O(n log n))
- Warm-starting (Phase 4 reuse previous frame's solution)
- Contact caching (reduce EPA calls in Phase 1)

---

## Design Review — Initial Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-02-04
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Change |
|----|-------|----------|-----------------|
| I1 | Rule of Five violation with reference members | C++ Quality | Delete move constructor and move assignment operator (cannot rebind references) |
| I2 | Dangling reference safety risk between frames | C++ Quality | Call `clearFrameData()` at **end** of `execute()` in addition to start |
| I3 | Unused `jacobians_` member variable | C++ Quality | Remove `std::vector<Eigen::MatrixXd> jacobians_` from class (not used in any phase) |
| I4 | Const-correctness for collision handler reference | C++ Quality | Change `CollisionHandler&` to `const CollisionHandler&` (checkCollision is const) |

### Revision Instructions for Architect

The following changes must be made before final review:

1. **Rule of Five Violation (I1)**:
   - **Current**: Lines 52-53 declare `CollisionPipeline(CollisionPipeline&&) noexcept = default;` and `CollisionPipeline& operator=(CollisionPipeline&&) noexcept = default;`
   - **Problem**: The class contains reference members (`CollisionHandler&`, `ConstraintSolver&`) which cannot be rebound. The compiler will implicitly delete the move assignment operator, breaking the Rule of Five all-or-nothing principle
   - **Fix**: Delete both move operations:
     ```cpp
     CollisionPipeline(CollisionPipeline&&) = delete;
     CollisionPipeline& operator=(CollisionPipeline&&) = delete;
     ```
   - **Rationale**: CollisionPipeline is a long-lived service object owned by WorldModel. It should never be moved. Deleting move operations makes the intent explicit and prevents accidental misuse.

2. **Dangling Reference Safety (I2)**:
   - **Current**: Design states "Clear all vectors at start of `execute()` via `clearFrameData()`" (line 251)
   - **Problem**: Between frames, `states_` (contains `std::reference_wrapper<const InertialState>`) and `constraintPtrs_` (contains raw pointers to constraints) hold references/pointers to objects from the previous frame. If WorldModel modifies asset vectors between ticks (e.g., removing a body, resizing vectors), these become dangling references.
   - **Fix**: Call `clearFrameData()` at **both** the start AND end of `execute()`:
     ```cpp
     void CollisionPipeline::execute(...) {
       clearFrameData();  // Start of frame
       // ... pipeline phases ...
       clearFrameData();  // End of frame - leave pipeline empty when idle
     }
     ```
   - **Update design rationale**: Change "Data Invalidation Prevention" section to justify clearing at both start and end
   - **Rationale**: This ensures the pipeline is "empty" and safe when idle. Between frames, no stale references exist that could become dangling if WorldModel modifies its containers.

3. **Remove Unused jacobians_ Member (I3)**:
   - **Current**: Line 125 declares `std::vector<Eigen::MatrixXd> jacobians_;`
   - **Problem**: This member variable is not mentioned in any phase method description, not used in the existing WorldModel implementation, and serves no documented purpose
   - **Fix**: Remove line 125 from the class interface
   - **Update PlantUML**: Remove `-jacobians_: vector<MatrixXd>` from diagram (line 28)
   - **Rationale**: The ConstraintSolver internally computes Jacobians during `solveWithContacts()`. The pipeline does not need to pre-compute or cache them.

4. **Const-Correctness for CollisionHandler Reference (I4)**:
   - **Current**: Line 112 declares `CollisionHandler& collisionHandler_;`
   - **Problem**: `CollisionHandler::checkCollision()` is a const method (verified in CollisionHandler.hpp line 51-53). The pipeline never modifies the handler's state.
   - **Fix**: Change to `const CollisionHandler& collisionHandler_;`
   - **Update constructor**: Change parameter to `const CollisionHandler& collisionHandler` (line 29)
   - **Update PlantUML**: Change `-collisionHandler_: CollisionHandler&` to `-collisionHandler_: const CollisionHandler&` (line 22)
   - **Rationale**: Follows CLAUDE.md const-correctness guidelines and accurately reflects that the pipeline has read-only access to the collision handler.

### Items Passing Review (No Changes Needed)

The following aspects of the design are solid and should **not** be modified during revision:

- **Architectural fit**: Excellent separation of concerns, CollisionPipeline correctly placed in `msd-sim/Physics/Collision`
- **Stateful design decision**: Correct choice for NFR-1 (zero additional allocations)
- **Protected access for sub-phases**: Correct balance of testability and encapsulation per FR-2
- **std::span interface**: Modern, flexible, testable design per FR-4
- **Non-owning references to subsystems**: Correct ownership model (WorldModel owns, pipeline uses)
- **CollisionPair nested struct**: Clean abstraction for phase communication
- **Test plan**: Comprehensive unit, integration, and benchmark test coverage
- **Performance analysis**: Accurate complexity characterization

---

## Architect Revision Notes

**Date**: 2026-02-04
**Responding to**: Design Review — Initial Assessment

### Changes Made

| Issue ID | Original | Revised | Rationale |
|----------|----------|---------|-----------|
| I1 | `CollisionPipeline(CollisionPipeline&&) noexcept = default;` and move assignment `= default` | `CollisionPipeline(CollisionPipeline&&) = delete;` and move assignment `= delete` | Reference members (`CollisionHandler&`, `ConstraintSolver&`) cannot be rebound. Compiler would implicitly delete move assignment anyway, breaking Rule of Five. Explicit deletion makes intent clear: CollisionPipeline is a long-lived service object that should never be moved. |
| I2 | `clearFrameData()` called at start of `execute()` only | `clearFrameData()` called at **both** start and end of `execute()` | Prevents dangling references when pipeline is idle between frames. If WorldModel modifies asset vectors (e.g., removes a body), `states_` and `constraintPtrs_` would hold stale references. Double-clearing ensures pipeline is "empty" and safe when idle. Updated "Data Invalidation Prevention" section with expanded rationale. |
| I3 | `std::vector<Eigen::MatrixXd> jacobians_;` member declared | Removed from class interface and PlantUML | Jacobians are computed internally by `ConstraintSolver::solveWithContacts()`. The pipeline does not need to pre-compute or cache them. This was a leftover from an earlier design iteration and serves no purpose. |
| I4 | `CollisionHandler& collisionHandler_;` (non-const) | `const CollisionHandler& collisionHandler_;` | `CollisionHandler::checkCollision()` is a const method. Pipeline never modifies handler state. Const-correctness follows CLAUDE.md guidelines and accurately reflects read-only access. Updated constructor parameter, dependencies list, and PlantUML diagram. |

### Diagram Updates

**File**: `./0036_collision_pipeline_extraction.puml`

- Line 22: Changed `-collisionHandler_: CollisionHandler&` to `-collisionHandler_: const CollisionHandler&`
- Line 28: Removed `-jacobians_: vector<MatrixXd>`
- Lines 107-115 (note): Updated from "Cleared at start of each execute()" to "Cleared at START and END of execute() to prevent dangling references when idle"

### Unchanged (Per Reviewer Guidance)

Per the review, the following aspects were **not** modified:

- Architectural placement in `msd-sim/Physics/Collision`
- Stateful design with frame-persistent storage (NFR-1 compliance)
- Protected sub-phase methods with friend test access (FR-2 compliance)
- `std::span` interface for asset data (FR-4 compliance)
- Non-owning references ownership model
- CollisionPair nested struct design
- Test plan (unit, integration, benchmark)
- Performance complexity analysis

---

## Design Review — Final Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-02-04
**Status**: APPROVED
**Iteration**: 1 of 1

### Verification of Revisions

All four issues from the initial assessment have been properly addressed:

| Issue ID | Status | Verification |
|----------|--------|--------------|
| I1 | ✓ Resolved | Move constructor and move assignment operator changed to `= delete` (lines 52-53). References cannot be rebound; explicit deletion prevents accidental misuse. |
| I2 | ✓ Resolved | `clearFrameData()` documentation updated to indicate clearing at both start and end of `execute()` (lines 104-108). "Data Invalidation Prevention" section expanded with comprehensive rationale addressing dangling reference safety (lines 249-265). |
| I3 | ✓ Resolved | `jacobians_` member removed from class interface (line 125 deleted). PlantUML diagram updated (line 28 removed). Unused member eliminated. |
| I4 | ✓ Resolved | `CollisionHandler&` changed to `const CollisionHandler&` in member declaration (line 112), constructor parameter (line 29), dependencies list (line 134), and PlantUML diagram (line 22). Const-correctness properly applied. |

### Criteria Assessment

#### Architectural Fit
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | `CollisionPipeline` follows PascalCase, member variables use `snake_case_`, methods use camelCase |
| Namespace organization | ✓ | Correctly placed in `msd_sim::Physics::Collision` namespace, consistent with project structure |
| File structure | ✓ | Follows `msd/{component}/src/` pattern: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.{hpp,cpp}` |
| Dependency direction | ✓ | No cycles. Pipeline depends on CollisionHandler, ConstraintSolver (same layer), and Asset types (lower layer) |

#### C++ Design Quality
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | Member vectors managed automatically, no manual resource management needed |
| Smart pointer appropriateness | ✓ | Uses `std::unique_ptr<ContactConstraint>` for owned constraints, plain references for handler/solver per CLAUDE.md |
| Value/reference semantics | ✓ | Non-owning const reference for handler, mutable reference for solver (solver modifies internal state during solve) |
| Rule of 0/3/5 | ✓ | **FIXED**: All special member functions explicitly declared (`= delete` for copy/move, `= default` for destructor). Rule of Five satisfied. |
| Const correctness | ✓ | **FIXED**: CollisionHandler reference now const. Sub-phase methods correctly take spans (mutable for applyForces, const for environment assets). |
| Exception safety | ✓ | No exceptions thrown for normal operation. Solver non-convergence logged but does not throw. |
| Initialization | ✓ | Uses brace initialization per CLAUDE.md guidelines |
| Return values | ✓ | Phase methods return void (populate members). `solveConstraints()` returns result struct, not output parameter. |

#### Feasibility
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | Forward declarations possible for most types. Includes required: `<span>`, CollisionHandler, ConstraintSolver, Asset types |
| Template complexity | ✓ | No templates. `std::span` is standard library template (zero complexity cost). |
| Memory strategy | ✓ | **IMPROVED**: Double-clearing (start+end) prevents dangling references between frames. Frame-persistent storage eliminates per-frame allocation. |
| Thread safety | ✓ | Explicitly documented as not thread-safe (stateful pipeline). Acceptable for single-threaded simulation loop. |
| Build integration | ✓ | New .cpp file added to msd-sim CMakeLists.txt. No new external dependencies. |

#### Testability
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | Pipeline can be instantiated with mock handler/solver for unit testing |
| Mockable dependencies | ✓ | CollisionHandler and ConstraintSolver are mockable via inheritance or test doubles |
| Observable state | ✓ | Protected sub-phase methods allow testing to inspect intermediate state (collisions_, constraints_, etc.) via friend test classes |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | WorldModel member initialization order matters (collisionPipeline_ must initialize after collisionHandler_ and contactSolver_) | Integration | Low | Medium | Document initialization order dependency in WorldModel.hpp. C++ guarantees members initialize in declaration order. | No |
| R2 | Friend test class declarations add coupling | Maintenance | Low | Low | Acceptable trade-off per FR-2. Alternative (public sub-phases) would violate encapsulation. | No |
| R3 | Performance regression if clearFrameData() overhead is significant | Performance | Low | Low | Benchmark test `BM_CollisionPipeline_AllocationCount` will detect. Double-clear is trivial compared to collision detection O(n²) cost. | No |

### Prototype Guidance

No prototypes required. All risks are low-likelihood/low-impact with clear mitigations. The design is a structural refactoring with zero algorithmic changes (FR-5: "All existing collision behavior is preserved").

### Summary

The CollisionPipeline design is **APPROVED** for implementation. All initial review issues have been properly resolved:

1. **Rule of Five violation fixed**: Move operations explicitly deleted, preventing accidental misuse of reference members
2. **Dangling reference safety improved**: Double-clearing (start+end of execute()) ensures pipeline is safe when idle
3. **Unused member removed**: `jacobians_` eliminated, reducing memory footprint and cognitive load
4. **Const-correctness applied**: CollisionHandler reference properly marked const

The design demonstrates excellent architectural fit, follows modern C++ best practices per CLAUDE.md, is implementable with low risk, and provides comprehensive testability through protected friend-access sub-phases. The stateful frame-persistent storage approach correctly satisfies NFR-1 (zero additional allocations) while the double-clearing strategy addresses memory safety concerns raised in the initial review.

**Next Steps**:
1. Human review of approved design
2. Proceed to implementation phase per ticket workflow
3. Execute comprehensive test plan (10 unit tests, 3 integration tests, 2 benchmarks)
4. Verify acceptance criteria AC1-AC6 during implementation

---

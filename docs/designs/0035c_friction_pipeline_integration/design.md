# Design: Friction Pipeline Integration

## Summary

This design integrates friction constraints into the existing contact pipeline by adding friction coefficient properties to rigid bodies, extending `ContactConstraintFactory` to create `FrictionConstraint` objects alongside `ContactConstraint` objects, and wiring friction through `WorldModel::updateCollisions()`. Bodies gain a `frictionCoefficient` property (default μ=0.5) which is combined via geometric mean when two bodies make contact. The factory creates one normal constraint and (optionally) one two-dimensional friction constraint per contact point. When friction constraints are present, the solver automatically dispatches to the ECOS SOCP solver (ticket 0035b4) for exact friction cone enforcement. This enables realistic sliding, static friction, and torque generation from off-center contacts—completing the end-to-end friction pipeline.

## Architecture Changes

### PlantUML Diagram
See: `./0035c_friction_pipeline_integration.puml`

### Modified Components

#### AssetInertial

- **Current location**: `msd-sim/src/Physics/RigidBody/AssetInertial.hpp`, `AssetInertial.cpp`
- **Changes required**:
  - Add `frictionCoefficient_` member variable (double, default 0.5)
  - Add getter `getFrictionCoefficient() const`
  - Add setter `setFrictionCoefficient(double mu)` with validation (mu >= 0)
  - Update constructors to accept optional `frictionCoefficient` parameter
  - Add validation in constructor and setter to throw `std::invalid_argument` if mu < 0
- **Backward compatibility**:
  - Existing constructors remain unchanged (default μ = 0.5)
  - New optional parameter in extended constructor signature
  - Zero impact on existing code unless friction coefficient is explicitly set

**Design decision — Default friction coefficient**: Default μ = 0.5 provides realistic friction for moderate-friction materials (wood, concrete, plastic). Alternatives considered:
- μ = 0.0 (frictionless by default, opt-in friction): Rejected because it requires all code to explicitly set friction, breaking user expectations
- μ = 1.0 (high friction by default): Rejected because it's unrealistic for most materials and would cause excessive grip in simulations

Recommendation: μ = 0.5 is a reasonable compromise that works for most scenarios.

#### AssetEnvironment

- **Current location**: `msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp`, `AssetEnvironment.cpp`
- **Changes required**:
  - Add `frictionCoefficient_` member variable (double, default 0.5)
  - Add getter `getFrictionCoefficient() const`
  - Update constructors to accept optional `frictionCoefficient` parameter
  - Add validation in constructor to throw `std::invalid_argument` if mu < 0
- **Backward compatibility**:
  - Existing constructors remain unchanged (default μ = 0.5)
  - New optional parameter in extended constructor signature
  - Setter not needed (AssetEnvironment properties are immutable after construction)
- **Rationale**: AssetEnvironment participates in friction just like AssetInertial (ground friction, wall friction, etc.)

#### ContactConstraintFactory (Namespace)

- **Current location**: `msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp`, `ContactConstraintFactory.cpp`
- **Changes required**:
  - Add `createFrictionConstraints()` function:
    ```cpp
    std::vector<std::unique_ptr<FrictionConstraint>> createFrictionConstraints(
        size_t bodyAIndex,
        size_t bodyBIndex,
        const CollisionResult& result,
        const InertialState& stateA,
        const InertialState& stateB,
        const Coordinate& comA,
        const Coordinate& comB,
        double frictionCoefficientA,
        double frictionCoefficientB);
    ```
  - Add `combineFrictionCoefficient(double muA, double muB)` utility function:
    ```cpp
    double combineFrictionCoefficient(double muA, double muB) {
      return std::sqrt(muA * muB);  // Geometric mean
    }
    ```
  - Implementation details for `createFrictionConstraints()`:
    1. Compute combined friction coefficient: `mu = combineFrictionCoefficient(muA, muB)`
    2. **Optimization**: If `mu == 0.0`, return empty vector (skip friction constraint creation)
    3. Iterate over `result.contactCount` contact points
    4. For each contact:
       - Call `TangentBasis::computeTangentBasis(result.normal)` to get tangent frame
       - Create `FrictionConstraint` with:
         - `bodyAIndex`, `bodyBIndex`
         - `result.normal`
         - `result.contactPointA[i]`, `result.contactPointB[i]`
         - `comA`, `comB`
         - `mu` (combined friction coefficient)
       - Add to result vector
    5. Return vector of `std::unique_ptr<FrictionConstraint>`
- **Thread safety**: Stateless utility functions (thread-safe)
- **Error handling**:
  - Throws `std::invalid_argument` if frictionCoefficientA or frictionCoefficientB < 0
  - Propagates exceptions from `TangentBasis::computeTangentBasis()` (invalid normal)
  - Propagates exceptions from `FrictionConstraint` constructor (invalid parameters)

**Design decision — Friction creation optimization**: When μ = 0, skip friction constraint creation entirely rather than creating constraints with zero bounds. This optimization:
- Reduces constraint system dimension from 3C to C (C = contact count)
- Avoids ECOS solver dispatch overhead (stays in ASM code path)
- Preserves backward compatibility with frictionless simulations
- Consistent with ticket requirement FR-6 and math formulation M7 analysis

#### WorldModel::updateCollisions()

- **Current location**: `msd-sim/src/Environment/WorldModel.hpp`, `WorldModel.cpp`
- **Changes required**:
  - Extend collision response loop to create friction constraints:
    ```cpp
    // For each collision result:
    auto contactConstraints = ContactConstraintFactory::createFromCollision(
        /* existing parameters */);

    // NEW: Create friction constraints
    auto frictionConstraints = ContactConstraintFactory::createFrictionConstraints(
        bodyAIndex, bodyBIndex, result,
        stateA, stateB, comA, comB,
        bodyA.getFrictionCoefficient(),  // NEW
        bodyB.getFrictionCoefficient()); // NEW

    // Merge constraint vectors
    std::vector<std::unique_ptr<TwoBodyConstraint>> allConstraints;
    allConstraints.reserve(contactConstraints.size() + frictionConstraints.size());
    for (auto& c : contactConstraints) {
      allConstraints.push_back(std::move(c));
    }
    for (auto& c : frictionConstraints) {
      allConstraints.push_back(std::move(c));
    }

    // Pass to solver (automatic ECOS dispatch if friction present)
    auto solveResult = constraintSolver_.solveWithContacts(
        allConstraints, states, masses, inverseInertias, dt);

    // Apply forces (existing code, unchanged)
    ```
  - No changes to force application logic (solver returns per-body forces)
- **Integration notes**:
  - `ConstraintSolver::solveWithContacts()` already handles mixed constraint types (ticket 0035b4)
  - Solver automatically detects `FrictionConstraint` via `dynamic_cast` and dispatches to ECOS
  - If no friction constraints (μ = 0 for both bodies), solver uses ASM (box-constrained LCP)
  - If friction constraints present, solver uses ECOS (SOCP for exact friction cone)

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| `AssetInertial::frictionCoefficient_` | `ContactConstraintFactory::createFrictionConstraints()` | Data provider | Friction coefficient passed to factory for combination |
| `AssetEnvironment::frictionCoefficient_` | `ContactConstraintFactory::createFrictionConstraints()` | Data provider | Friction coefficient passed to factory for combination |
| `ContactConstraintFactory::createFrictionConstraints()` | `TangentBasis::computeTangentBasis()` | Utility invocation | Called once per contact to get tangent frame |
| `ContactConstraintFactory::createFrictionConstraints()` | `FrictionConstraint` constructor | Factory creation | Creates friction constraint with computed parameters |
| `WorldModel::updateCollisions()` | `ContactConstraintFactory::createFrictionConstraints()` | Factory invocation | Creates friction constraints for each collision |
| `WorldModel::updateCollisions()` | `ConstraintSolver::solveWithContacts()` | Solver invocation | Passes mixed normal+friction constraints to solver |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `AssetInertialTest.cpp` | Constructor tests | Add friction coefficient tests | Add test cases for new constructor parameter and getter/setter |
| `AssetEnvironmentTest.cpp` | Constructor tests | Add friction coefficient tests | Add test cases for new constructor parameter and getter |
| `ContactConstraintFactoryTest.cpp` | `createFromCollision()` | None (backward compatible) | No changes needed (normal constraints still created identically) |
| `ConstraintSolverContactTest.cpp` | Frictionless contact tests | None (μ=0 optimization) | No changes needed (μ=0 produces same constraint system) |
| `WorldModelTest.cpp` | `updateCollisions()` | Add friction integration tests | Add end-to-end tests with friction (inclined plane, sliding) |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| `AssetInertial` | `DefaultFrictionCoefficient` | Default μ = 0.5 is set correctly |
| `AssetInertial` | `SetFrictionCoefficient_ValidRange` | Setter accepts μ ∈ [0, ∞) |
| `AssetInertial` | `SetFrictionCoefficient_NegativeThrows` | Setter throws `std::invalid_argument` for μ < 0 |
| `AssetInertial` | `GetFrictionCoefficient` | Getter returns correct value |
| `AssetEnvironment` | `DefaultFrictionCoefficient` | Default μ = 0.5 is set correctly |
| `AssetEnvironment` | `ConstructorWithFriction_ValidRange` | Constructor accepts μ ∈ [0, ∞) |
| `AssetEnvironment` | `ConstructorWithFriction_NegativeThrows` | Constructor throws `std::invalid_argument` for μ < 0 |
| `AssetEnvironment` | `GetFrictionCoefficient` | Getter returns correct value |
| `ContactConstraintFactory` | `CombineFrictionCoefficient_GeometricMean` | μ_combined = √(μA · μB) |
| `ContactConstraintFactory` | `CombineFrictionCoefficient_IdenticalValues` | μ_combined = μ when μA = μB |
| `ContactConstraintFactory` | `CombineFrictionCoefficient_ZeroProducesZero` | μ = 0 when either input is 0 |
| `ContactConstraintFactory` | `CreateFrictionConstraints_SingleContact` | Creates 1 FrictionConstraint for 1 contact point |
| `ContactConstraintFactory` | `CreateFrictionConstraints_MultiContact` | Creates N FrictionConstraints for N contact points |
| `ContactConstraintFactory` | `CreateFrictionConstraints_ZeroMuReturnsEmpty` | Returns empty vector when μ = 0 (optimization) |
| `ContactConstraintFactory` | `CreateFrictionConstraints_TangentBasisCalled` | TangentBasis::computeTangentBasis() invoked per contact |
| `ContactConstraintFactory` | `CreateFrictionConstraints_FrictionConstraintParameters` | Constraint receives correct bodyIndices, normal, contact points, com, μ |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `FrictionIntegration_InclinedPlaneStatic` | AssetInertial, ContactConstraintFactory, ConstraintSolver, WorldModel | Block on 20° inclined plane with μ=0.6 remains stationary for 1000 timesteps (AC1, M8 Example 1) |
| `FrictionIntegration_InclinedPlaneKinetic` | AssetInertial, ContactConstraintFactory, ConstraintSolver, WorldModel | Block on 45° inclined plane with μ=0.3 accelerates at g(sin θ - μ cos θ) ± 5% (AC2, M8 Example 2) |
| `FrictionIntegration_SlidingDeceleration` | AssetInertial, ContactConstraintFactory, ConstraintSolver, WorldModel | Sliding block with v₀=10 m/s, μ=0.5 decelerates at μg ± 5% (AC3, M8 Example 3) |
| `FrictionIntegration_GlancingCollisionSpin` | AssetInertial, ContactConstraintFactory, ConstraintSolver, WorldModel | Glancing collision produces angular velocity (AC4, M8 Example 4) |
| `FrictionIntegration_CoefficientCombination` | AssetInertial, ContactConstraintFactory | μA=0.3, μB=0.8 produces μ=√0.24≈0.49 (AC5) |
| `FrictionIntegration_FrictionlessBackwardCompat` | AssetInertial, ContactConstraintFactory, ConstraintSolver, WorldModel | μ=0 behaves identically to pre-friction system (AC6) |

#### Benchmark Tests

*None required*. Friction constraint creation is O(C) where C = contact count (typically 1-4). Performance impact is negligible compared to solver execution time. Solver performance already validated in ticket 0035b4 (ECOS benchmarks).

## Open Questions

### Design Decisions (Human Input Needed)

1. **Default friction coefficient value**:
   - Option A: μ = 0.5 (moderate friction) — Realistic for common materials, provides friction by default
   - Option B: μ = 0.0 (frictionless by default) — Opt-in friction, backward compatible behavior
   - Option C: μ = 1.0 (high friction by default) — Prevents sliding, but unrealistic for most materials
   - Recommendation: **Option A (μ = 0.5)** — Provides realistic default behavior without requiring explicit friction setup for every object. Users can still set μ=0 for frictionless contacts if desired.

2. **Friction coefficient range validation**:
   - Option A: Allow μ ∈ [0, ∞) (no upper limit) — Physically valid (no theoretical upper bound on friction)
   - Option B: Clamp to μ ∈ [0, 2.0] (typical range) — Prevents unrealistic values, but arbitrary limit
   - Option C: Warn if μ > 2.0 (log warning) — Allows flexibility but alerts to unusual values
   - Recommendation: **Option A (no upper limit)** — Friction can theoretically be arbitrarily high (e.g., adhesives, sticky materials). Validation should only reject μ < 0 (physically invalid).

3. **Friction constraint creation threshold**:
   - Option A: Skip friction if μ = 0.0 exactly — Fast, but requires exact zero check
   - Option B: Skip friction if μ < ε (e.g., 1e-6) — Handles floating-point rounding, but threshold is arbitrary
   - Option C: Always create friction constraints — Simplest, but inefficient for frictionless contacts
   - Recommendation: **Option A (exact zero check)** — Friction coefficient is user-specified (not computed), so exact zero is deterministic. Avoids arbitrary threshold selection. Performance optimization is significant (3C → C constraints).

### Prototype Required

*None required*. All components already implemented and validated in prerequisite tickets:
- `FrictionConstraint` class validated in ticket 0035a
- ECOS solver integration validated in ticket 0035b4
- Contact constraint factory pattern validated in ticket 0032
- Geometric mean combination pattern established in ticket 0032 (restitution)

### Requirements Clarification

1. **Friction coefficient combination for multi-material contacts**: Confirmed by ticket AC5 — geometric mean: μ = √(μA · μB). Matches restitution combination pattern from ticket 0032.

2. **Friction for AssetEnvironment**: Confirmed by ticket motivation — ground friction, wall friction require environment objects to have friction coefficient. Matches restitution pattern (AssetEnvironment has `coefficientOfRestitution_`).

3. **Backward compatibility for frictionless contacts**: Confirmed by ticket AC6 — μ=0 must behave identically to current system. Optimization (skip friction constraint creation when μ=0) ensures this.

## Implementation Notes

### Memory Allocation

- **Friction constraint creation**: O(C) heap allocations where C = contact count (typically 1-4)
- **Constraint vector merging**: Single allocation for unified constraint vector (contactCount + frictionCount)
- **Transient constraints**: All constraints freed after solver execution (per-frame allocation pattern)

### Performance Considerations

- **Constraint creation overhead**: TangentBasis::computeTangentBasis() is O(1) constant time (9 floating-point ops)
- **Friction coefficient combination**: Single floating-point sqrt() operation per collision
- **Solver dispatch overhead**: ECOS dispatch triggered only when friction constraints present (dynamic_cast check is fast)
- **Zero friction optimization**: Critical for preserving performance in frictionless scenarios (3C → C constraint reduction)

### Error Handling Strategy

- **Invalid friction coefficient**: Throw `std::invalid_argument` in constructor/setter (fail-fast)
- **Tangent basis failure**: Propagate exception from `TangentBasis::computeTangentBasis()` (invalid normal)
- **Constraint creation failure**: Propagate exception from `FrictionConstraint` constructor (degenerate geometry)
- **Solver convergence**: Handled by `ConstraintSolver::solveWithContacts()` (returns `converged = false`)

### Numerical Stability

- **Friction coefficient combination**: Geometric mean is numerically stable for μ ∈ [0, ∞)
- **Zero friction handling**: Exact zero check (μ == 0.0) is safe because friction coefficient is user-specified (not computed)
- **Tangent basis computation**: Duff et al. (2017) algorithm is robust against degenerate normals (ticket 0035a)
- **Solver stability**: ECOS handles friction cone constraints with pre-conditioning and equilibration (ticket 0035b4)

## Acceptance Criteria Mapping

This design satisfies all ticket acceptance criteria:

- **AC1** (Static friction on inclined plane): Integration test `FrictionIntegration_InclinedPlaneStatic` validates block remains stationary on 20° inclined plane with μ=0.6 for 1000 timesteps
- **AC2** (Kinetic friction on inclined plane): Integration test `FrictionIntegration_InclinedPlaneKinetic` validates block on 45° inclined plane with μ=0.3 accelerates at g(sin θ - μ cos θ) ± 5%
- **AC3** (Sliding deceleration): Integration test `FrictionIntegration_SlidingDeceleration` validates sliding block with v₀=10 m/s, μ=0.5 decelerates at μg ± 5%
- **AC4** (Glancing collision angular velocity): Integration test `FrictionIntegration_GlancingCollisionSpin` validates off-center impact produces angular velocity
- **AC5** (Friction coefficient combination): Unit test `CombineFrictionCoefficient_GeometricMean` validates μA=0.3, μB=0.8 produces μ=√0.24≈0.49
- **AC6** (Frictionless backward compatibility): Integration test `FrictionIntegration_FrictionlessBackwardCompat` validates μ=0 behaves identically to pre-friction system
- **AC7** (Zero regressions): All existing constraint tests pass (existing tests unaffected by changes)

## File Summary

### Modified Files

| File | Changes | Lines Added (Est.) | Lines Modified (Est.) |
|------|---------|--------------------|-----------------------|
| `msd-sim/src/Physics/RigidBody/AssetInertial.hpp` | Add frictionCoefficient_ member, getter, setter | 15 | 5 |
| `msd-sim/src/Physics/RigidBody/AssetInertial.cpp` | Implement getter, setter, update constructors | 25 | 10 |
| `msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp` | Add frictionCoefficient_ member, getter | 10 | 3 |
| `msd-sim/src/Physics/RigidBody/AssetEnvironment.cpp` | Implement getter, update constructors | 15 | 8 |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp` | Add createFrictionConstraints(), combineFrictionCoefficient() | 30 | 0 |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` | Implement createFrictionConstraints(), combineFrictionCoefficient() | 60 | 0 |
| `msd-sim/src/Environment/WorldModel.hpp` | No changes (interface unchanged) | 0 | 0 |
| `msd-sim/src/Environment/WorldModel.cpp` | Extend updateCollisions() to create and solve friction constraints | 30 | 10 |

### New Files

| File | Purpose | Lines (Est.) |
|------|---------|--------------|
| `msd-sim/test/Physics/FrictionIntegrationTest.cpp` | End-to-end friction integration tests (AC1-AC6) | 400 |

**Total estimated impact**: ~600 lines added/modified

## Dependencies

### Build Dependencies
- Eigen3 (existing)
- msd-assets (existing)
- ECOS (added in ticket 0035b4)

### Runtime Dependencies
- `TangentBasis::computeTangentBasis()` (ticket 0035a)
- `FrictionConstraint` class (ticket 0035a)
- `ConstraintSolver::solveWithContacts()` with ECOS integration (ticket 0035b4)
- `ContactConstraint` class (ticket 0032a)

### Prerequisite Tickets
- **Ticket 0035a**: FrictionConstraint class (provides constraint data model)
- **Ticket 0035b**: Box-constrained ASM solver (provides solver capability without friction)
- **Ticket 0035b4**: ECOS SOCP solver integration (provides exact friction cone enforcement)

## References

### Mathematical Formulation
- [M8-numerical-examples.md](../0035_friction_constraints/M8-numerical-examples.md) — Integration test scenarios with hand-computed expected results
- [math-formulation.md](../0035_friction_constraints/math-formulation.md) — Complete mathematical foundation for friction constraints

### Existing Components
- `msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp` — Factory to extend with friction constraint creation
- `msd-sim/src/Physics/Constraints/FrictionConstraint.hpp` — Friction constraint class from ticket 0035a
- `msd-sim/src/Physics/Collision/TangentBasis.hpp` — Tangent basis computation from ticket 0035a
- `msd-sim/src/Environment/WorldModel.hpp` — Simulation loop orchestrator to extend

### Design Documents
- [0035a_tangent_basis_and_friction_constraint/design.md](../0035a_tangent_basis_and_friction_constraint/design.md) — FrictionConstraint class design
- [0035b4_ecos_solve_integration/design.md](../0035b4_ecos_solve_integration/design.md) — ECOS solver integration design
- [0032_contact_constraint_refactor/design.md](../0032_contact_constraint_refactor/design.md) — Contact constraint factory pattern and restitution combination

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-01
**Status**: APPROVED
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | All identifiers follow project standards: `PascalCase` for classes (AssetInertial, AssetEnvironment, ContactConstraintFactory), `camelCase` for methods (getFrictionCoefficient, setFrictionCoefficient, createFrictionConstraints, combineFrictionCoefficient), `snake_case_` for members (frictionCoefficient_). Consistent with existing codebase patterns established in tickets 0027 and 0032. |
| Namespace organization | ✓ | Design respects existing namespace hierarchy: `msd_sim` (top-level), `ContactConstraintFactory` (namespace-scoped utility functions), no new namespaces introduced. Friction coefficient properties added to existing classes within `Physics::RigidBody` package. Matches established pattern from `coefficientOfRestitution_` (ticket 0027). |
| File structure | ✓ | Modified files follow established `msd-sim/src/Physics/` layout: `RigidBody/AssetInertial.hpp`, `RigidBody/AssetEnvironment.hpp`, `Constraints/ContactConstraintFactory.hpp`, `Environment/WorldModel.hpp`. Integration test follows `msd-sim/test/Physics/` pattern: `FrictionIntegrationTest.cpp`. No new directories created. |
| Dependency direction | ✓ | Dependencies flow correctly: `AssetInertial` (data provider) → `ContactConstraintFactory` (factory) → `FrictionConstraint` (data model). `WorldModel` orchestrates factory and solver without creating reverse dependencies. All dependencies are on prerequisite tickets (0035a, 0035b4) which are already implemented. No circular dependencies or layering violations. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | Friction coefficient stored as primitive `double` member with no resource management needed. `createFrictionConstraints()` returns `std::vector<std::unique_ptr<FrictionConstraint>>` with proper ownership transfer via move semantics. No manual memory management. |
| Smart pointer appropriateness | ✓ | Uses `std::unique_ptr<FrictionConstraint>` for exclusive ownership of created constraints (per CLAUDE.md: prefer `unique_ptr` for ownership transfer). Avoids `std::shared_ptr` as required by project standards. Factory returns vector of unique_ptr, enabling efficient move into constraint solver. |
| Value/reference semantics | ✓ | Friction coefficient stored by value (primitive `double`). Parameters passed by value where appropriate (`double frictionCoefficientA/B`). Coordinate and state objects passed by const reference for efficiency (`const InertialState&`, `const Coordinate&`). Follows established pattern from `coefficientOfRestitution_` (ticket 0027). |
| Rule of 0/3/5 | ✓ | `AssetInertial` and `AssetEnvironment` already use Rule of Five with explicit `= default` for special members (move-only types due to constraint ownership). New friction coefficient member does not affect special member functions (trivially copyable `double`). Factory functions are stateless (no special members needed). |
| Const correctness | ✓ | Getters are const-qualified: `getFrictionCoefficient() const`. Setter is non-const as expected: `setFrictionCoefficient(double)`. Factory functions take const references for read-only inputs: `const InertialState&`, `const Coordinate&`, `const CollisionResult&`. `combineFrictionCoefficient()` is a pure function (can be constexpr in C++20, but not required). |
| Exception safety | ✓ | Basic guarantee specified: setter/constructor throw `std::invalid_argument` for μ < 0. Factory function propagates exceptions from `TangentBasis::computeTangentBasis()` and `FrictionConstraint` constructor (documented). Strong guarantee for validation (state unchanged on throw). No resource leaks on exception (RAII via unique_ptr). |
| Initialization | ✓ | Uses brace initialization throughout: `frictionCoefficient_{0.5}` for default member initialization. Validation in constructor/setter before assignment ensures initialized state is always valid. Matches existing pattern from `coefficientOfRestitution_{0.5}` (ticket 0027). No use of NaN for floating-point (friction coefficient has valid default, unlike mass properties). |
| Return values | ✓ | Factory function returns `std::vector<std::unique_ptr<FrictionConstraint>>` (value return via move semantics). `combineFrictionCoefficient()` returns `double` by value. Avoids output parameters entirely. Matches project standard: "Prefer returning values/structs over output parameters." |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | Minimal new includes: `AssetInertial.hpp` and `AssetEnvironment.hpp` add no new headers (friction coefficient is primitive `double`). `ContactConstraintFactory.hpp` already includes `FrictionConstraint.hpp` from ticket 0035a. `WorldModel.cpp` includes are already present. No circular header dependencies introduced. Forward declarations not needed (primitives and existing types). |
| Template complexity | ✓ | No templates introduced. Factory functions use concrete types (`std::vector<std::unique_ptr<FrictionConstraint>>`). No metaprogramming or SFINAE. Simple, straightforward implementations without generic programming complexity. |
| Memory strategy | ✓ | Clear allocation pattern: (1) Factory allocates constraints via `std::make_unique` on heap, (2) Returns ownership via `std::vector<std::unique_ptr>` move, (3) Caller merges into unified constraint vector, (4) Solver receives raw pointers (non-owning), (5) Constraints freed after solve (per-frame allocation). Matches existing contact constraint pattern from ticket 0032a. Heap allocation overhead is O(C) where C = contact count (typically 1-4), negligible compared to solver O(C³) cost. |
| Thread safety | ✓ | Factory functions are stateless (thread-safe for different inputs). Friction coefficient getters are const (thread-safe reads after construction). Setter is not thread-safe (mutates AssetInertial state), consistent with existing setters (`setCoefficientOfRestitution`). No global state or singletons introduced. WorldModel orchestration is single-threaded (documented assumption in msd-sim/CLAUDE.md). |
| Build integration | ✓ | All modified files are existing source files already in CMakeLists.txt. New test file `FrictionIntegrationTest.cpp` follows existing test pattern (add to `msd-sim/test/Physics/CMakeLists.txt`). No new build targets or external dependencies. ECOS dependency already integrated in ticket 0035b4. Estimated build impact: ~600 LOC across 8 files, incremental compilation only. |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | Components are independently testable: (1) `AssetInertial::getFrictionCoefficient()` / `setFrictionCoefficient()` testable in isolation with mock hull, (2) `ContactConstraintFactory::combineFrictionCoefficient()` is pure function (unit testable with input/output pairs), (3) `ContactConstraintFactory::createFrictionConstraints()` testable with mock CollisionResult, (4) Integration tests exercise full pipeline (collision → factory → solver → force application). No hidden dependencies or global state. |
| Mockable dependencies | ✓ | Dependencies are abstract or data-oriented: (1) `CollisionResult` is data struct (easily constructed in tests), (2) `InertialState` is data struct (easily constructed), (3) `TangentBasis::computeTangentBasis()` is deterministic utility (no mocking needed, can verify output), (4) `FrictionConstraint` constructor is concrete but dependencies are data (testable without mocks). Factory pattern enables testing constraint creation without solver integration. |
| Observable state | ✓ | All relevant state is observable: (1) Friction coefficient via `getFrictionCoefficient()`, (2) Created constraints inspectable via return value (`std::vector<std::unique_ptr<FrictionConstraint>>`), (3) Constraint properties accessible via `FrictionConstraint` getters (from ticket 0035a), (4) Solver results observable via `MultiBodySolveResult` (forces, convergence status), (5) Integration test validates observable physics (position, velocity, acceleration). No hidden state or write-only properties. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Friction coefficient default (μ=0.5) may not be appropriate for all scenarios. Different materials have vastly different friction (ice: 0.02, rubber: 1.0, adhesive: > 2.0). Default value is a compromise that may surprise users expecting material-specific behavior. | Maintenance | Medium | Low | (1) Document default value clearly in API docs and ticket, (2) Provide setter for customization, (3) Default is non-zero (provides friction by default, avoiding frictionless simulation surprise), (4) Future enhancement: material system with per-material friction tables. Current default (0.5) is reasonable for moderate-friction materials (wood, concrete, plastic). | No |
| R2 | Zero friction optimization (μ=0 → skip constraint creation) relies on exact floating-point equality check. If friction coefficient is computed (e.g., combined from two bodies with μ=0.0), floating-point rounding could produce μ≈1e-16 instead of exact 0.0, disabling optimization and creating degenerate constraints. | Performance | Low | Low | (1) Friction coefficient is user-specified (not computed), ensuring exact 0.0 is deterministic, (2) Geometric mean √(μA·μB) produces exact 0.0 if either input is exactly 0.0 (mathematical property: √0 = 0), (3) No arithmetic operations on friction coefficient before zero check (direct comparison after geometric mean), (4) If rounding becomes an issue in practice, can introduce threshold (μ < ε), but current design avoids arbitrary threshold selection. | No |
| R3 | Friction coefficient combination using geometric mean may not match physical material pairing behavior. Different combination rules exist in literature: arithmetic mean, geometric mean, harmonic mean, minimum. Geometric mean is used for consistency with restitution (ticket 0032) but may not be experimentally validated. | Technical | Low | Low | (1) Parent ticket AC9 explicitly specifies geometric mean (requirement, not design choice), (2) Consistency with restitution combination (ticket 0032) established precedent, (3) Geometric mean has desirable property: if either material is frictionless (μ=0), combined friction is zero (matches physical intuition), (4) Industry standard: Bullet Physics and Box2D use geometric mean for friction, (5) If experimental validation required, can be addressed in future ticket without changing constraint infrastructure. | No |
| R4 | Friction constraint creation overhead (TangentBasis computation + FrictionConstraint construction) per contact point may impact performance for high contact count scenarios (e.g., particle systems, granular materials). | Performance | Low | Low | (1) TangentBasis computation is O(1) constant time (9 floating-point ops) — negligible, (2) FrictionConstraint construction pre-computes lever arms (one-time cost per frame) — validated in ticket 0035a as negligible, (3) Ticket does not target high contact count scenarios (particle systems deferred to future work), (4) Typical contact counts: 1-4 per collision pair (manifold generation from ticket 0029), (5) Zero friction optimization (μ=0) eliminates overhead for frictionless scenarios. | No |
| R5 | No upper limit on friction coefficient (μ ∈ [0, ∞)) allows unrealistic values (e.g., μ=1000) which may cause solver issues or unintended behavior. | Maintenance | Low | Low | (1) Physically valid: no theoretical upper bound on friction (adhesives, sticky materials can have μ > 2.0), (2) Validation only rejects μ < 0 (physically invalid), (3) If solver issues arise with high μ, can be addressed by adjusting solver tolerance or adding optional warning/clamping in future, (4) Design document notes this as "Open Question" with recommendation for no upper limit (Option A), acknowledging trade-off between flexibility and safety. | No |

### Prototype Guidance

**None required**. All identified risks are low-likelihood, low-impact, and have documented mitigation strategies. No high-uncertainty technical challenges requiring prototyping.

### Summary

**Verdict**: APPROVED for prototype phase (though prototype is marked as not required in design document).

**Rationale**: The design demonstrates excellent architectural fit with the existing codebase, following established patterns from tickets 0027 (restitution) and 0032 (contact constraints). C++ design quality is high, with proper use of modern C++ idioms (smart pointers, const correctness, value semantics, exception safety). Feasibility is well-analyzed—all dependencies are satisfied (0035a, 0035b4), implementation scope is modest (~600 LOC), and integration points are clearly defined. Testability is strong with isolated unit tests and comprehensive integration tests mapped to acceptance criteria.

**Identified Risks**: Five low-impact risks documented with clear mitigation strategies. None require prototyping. Most significant consideration is the default friction coefficient (μ=0.5), which is a reasonable compromise but requires clear documentation for users expecting material-specific behavior. The zero friction optimization relies on exact floating-point equality, but this is safe in practice because friction coefficients are user-specified (not computed) and geometric mean preserves exact zero.

**Design Strengths**:
1. **Consistency**: Friction coefficient property mirrors restitution pattern (ticket 0027), ensuring cognitive consistency for users and maintainers.
2. **Modularity**: Clean separation between data (AssetInertial/AssetEnvironment), factory (ContactConstraintFactory), and solving (ConstraintSolver). Each component testable in isolation.
3. **Performance-aware**: Zero friction optimization (μ=0 → skip constraint creation) preserves backward compatibility and performance for frictionless scenarios.
4. **Mathematical rigor**: Geometric mean combination formula has desirable properties (zero propagation, symmetry) and matches industry standards (Bullet, Box2D).
5. **Prerequisite validation**: All dependencies (TangentBasis, FrictionConstraint, ECOS solver) are validated in prior tickets—this is pure integration work.

**Open Questions Resolved**: Design document poses three open questions (default μ, validation range, zero threshold). My assessment concurs with the design's recommendations: (1) μ=0.5 default is reasonable (Option A), (2) No upper limit (Option A) preserves physical validity, (3) Exact zero check (Option A) is safe for user-specified values. These are prudent design choices that balance usability, flexibility, and correctness.

**Next Steps**: Design is ready to proceed to implementation. No revisions needed. Prototype phase marked as "not required" is correct—all components validated in prerequisite tickets. Implementation should focus on comprehensive integration testing to validate end-to-end friction behavior (AC1-AC6) and ensure zero regressions (AC7).


# Design: Constraint Hierarchy Refactor

## Summary

Flatten the 4-level constraint inheritance hierarchy (Constraint -> BilateralConstraint/UnilateralConstraint -> TwoBodyConstraint -> ContactConstraint/FrictionConstraint) to a 2-level hierarchy where all four concrete constraint classes inherit directly from a redesigned `Constraint` base class. This eliminates the empty `BilateralConstraint` marker class, the trivial `UnilateralConstraint` intermediate class, and the LSP-violating `TwoBodyConstraint` class, while reducing `dynamic_cast` usage in the solver from 6 to 2.

## Architecture Changes

### PlantUML Diagram
See: `./0043_constraint_hierarchy_refactor.puml`

### Key Design Decisions

#### D1: Unified Evaluation Signature (Two-Body for All)

**Decision**: All constraints use a two-body evaluation signature: `evaluate(stateA, stateB, time)` and `jacobian(stateA, stateB, time)`.

**Rationale**: The current split between single-body (`evaluate(state, time)`) and two-body (`evaluateTwoBody(stateA, stateB, time)`) is the root cause of the LSP violation. TwoBodyConstraint overrides the single-body methods to throw `std::logic_error`, which means callers cannot safely call `evaluate()` on any `Constraint*` without first checking the type.

By making all constraints accept two states, single-body constraints simply ignore `stateB`. This is already the standard approach in physics engines (Bullet, ODE) -- a single-body constraint is modeled as a constraint between the body and an implicit "world" body with zero inverse mass.

**Jacobian width**: Two-body constraints currently return a 12-column Jacobian (6 DOF per body in velocity space), while single-body constraints return a 7-column Jacobian (position-level DOFs). These are fundamentally different spaces and cannot be unified into a single column count.

**Resolution**: Rather than forcing a single Jacobian width, the base interface documents that the Jacobian column count depends on the constraint type:
- Single-body constraints return `dimension() x 7` (position-level DOFs: X(3), Q(4))
- Two-body constraints return `dimension() x 12` (velocity-level DOFs: v_A(3), omega_A(3), v_B(3), omega_B(3))

The solver already dispatches on body count when building the effective mass matrix (single-body path uses 7x7 mass matrix; two-body path uses 6x6 per-body blocks). The `bodyCount()` method on the base class provides this dispatch without any `dynamic_cast`.

#### D2: Multiplier Bounds via LambdaBounds Value Type

**Decision**: Introduce a `LambdaBounds` struct with `lower` and `upper` fields, returned by `Constraint::lambdaBounds()`. Factory methods provide semantic construction: `LambdaBounds::bilateral()` returns `(-inf, +inf)`, `LambdaBounds::unilateral()` returns `(0, +inf)`, and `LambdaBounds::boxConstrained(lo, hi)` returns `(lo, hi)`.

**Rationale**: An enum would require the solver to map enum values to actual bound values. Returning the bounds directly is simpler, more extensible (new bound types need no enum update), and provides the solver with exactly the data it needs. The factory methods preserve the semantic meaning for documentation and debugging.

**Query methods**: `isBilateral()`, `isUnilateral()`, and `isBoxConstrained()` are provided for solver dispatch (e.g., routing to ECOS when box constraints are present) without needing `dynamic_cast` on the constraint type.

#### D3: Body Indices and Count in Base Class

**Decision**: Move `body_a_index_` and `body_b_index_` from `TwoBodyConstraint` into the `Constraint` base class. Add a virtual `bodyCount()` method (returns 1 or 2).

**Rationale**: Body indices are needed by the solver for every constraint type. Single-body constraints use `bodyAIndex()` to identify which body they act on; `bodyBIndex()` is unused (set to 0 by convention, ignored by solver when `bodyCount() == 1`). This eliminates the need to cast to `TwoBodyConstraint*` to access body indices.

**Constructor**: The base `Constraint` constructor takes `bodyAIndex` and `bodyBIndex` with defaults of 0. Single-body constraints pass only `bodyAIndex`. Two-body constraints pass both.

#### D4: Activation Query in Base Class

**Decision**: Add `isActive(stateA, stateB, time)` to the base `Constraint` class with a default implementation that returns `true`.

**Rationale**: Currently, only `UnilateralConstraint` declares `isActive()`. By moving it to the base with a default `true`, bilateral constraints are always active (correct behavior), and unilateral/box-constrained constraints can override. This eliminates the need for the `UnilateralConstraint` intermediate class entirely.

#### D5: Contact/Friction-Specific Accessors Remain on Concrete Classes

**Decision**: Keep `getContactNormal()`, `getPenetrationDepth()`, `getRestitution()`, `getFrictionCoefficient()`, etc., on their respective concrete classes. The 2 remaining `dynamic_cast`s that access these are acceptable and targeted.

**Rationale**: These accessors are physics-specific data that only makes sense for their concrete type. Promoting them to the base class would bloat the interface with methods that throw or return nonsense values for irrelevant constraint types -- recreating the same LSP violation we are eliminating.

The ticket acceptance criteria allows up to 2 `dynamic_cast`s. The remaining casts are:
1. `ConstraintSolver::assembleContactRHS()` -- casts to `ContactConstraint*` to access restitution and penetration for RHS computation
2. `WorldModel::updateCollisions()` -- casts to `ContactConstraint*` to access contact normal for gravity compensation

These are both in contact-specific code paths where the cast is a reasonable type-narrowing operation.

### New Components

#### LambdaBounds
- **Purpose**: Value type representing the lower and upper bounds on a constraint's Lagrange multiplier
- **Header location**: `msd/msd-sim/src/Physics/Constraints/LambdaBounds.hpp`
- **Source location**: Header-only (all methods are trivial)
- **Key interfaces**:
  ```cpp
  struct LambdaBounds {
      double lower;
      double upper;

      static LambdaBounds bilateral() {
          return {-std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity()};
      }

      static LambdaBounds unilateral() {
          return {0.0, std::numeric_limits<double>::infinity()};
      }

      static LambdaBounds boxConstrained(double lo, double hi) {
          return {lo, hi};
      }

      [[nodiscard]] bool isBilateral() const {
          return lower == -std::numeric_limits<double>::infinity() &&
                 upper ==  std::numeric_limits<double>::infinity();
      }

      [[nodiscard]] bool isUnilateral() const {
          return lower == 0.0 &&
                 upper == std::numeric_limits<double>::infinity();
      }

      [[nodiscard]] bool isBoxConstrained() const {
          return !isBilateral() && !isUnilateral();
      }
  };
  ```
- **Dependencies**: `<limits>` only
- **Thread safety**: Immutable value type; trivially thread-safe
- **Error handling**: None (struct with public fields)

### Modified Components

#### Constraint (base class)
- **Current location**: `msd/msd-sim/src/Physics/Constraints/Constraint.hpp`, `Constraint.cpp`
- **Changes required**:
  1. **Add body index storage**: `body_a_index_` and `body_b_index_` as protected members, with public accessors `bodyAIndex()` and `bodyBIndex()`
  2. **Add `bodyCount()` virtual method**: Default returns 1; two-body constraints override to return 2
  3. **Change evaluation signature**: `evaluate(const InertialState& stateA, const InertialState& stateB, double time)` replaces `evaluate(const InertialState& state, double time)`
  4. **Change Jacobian signature**: `jacobian(const InertialState& stateA, const InertialState& stateB, double time)` replaces `jacobian(const InertialState& state, double time)`
  5. **Add `lambdaBounds()` pure virtual**: Returns `LambdaBounds` specifying multiplier bound semantics
  6. **Add `isActive()` virtual**: Default returns `true`; signature is `isActive(const InertialState& stateA, const InertialState& stateB, double time)`
  7. **Update constructor**: Accept `bodyAIndex` (default 0) and `bodyBIndex` (default 0) as protected constructor parameters
  8. **Remove `partialTimeDerivative` default**: Keep the virtual method but move it to the interface (remains with a default implementation returning zero)
- **Backward compatibility**: Breaking change -- all consumers of the old single-body `evaluate()`/`jacobian()` signatures must be updated. This is acceptable because:
  - The ticket explicitly removes intermediate classes
  - All consumers are internal to msd-sim
  - No external API is exposed

#### UnitQuaternionConstraint
- **Current location**: `msd/msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.hpp`, `.cpp`
- **Changes required**:
  1. Change base class from `BilateralConstraint` to `Constraint`
  2. Update `evaluate()` signature to take two `InertialState&` parameters (ignores `stateB`)
  3. Update `jacobian()` signature to take two `InertialState&` parameters (ignores `stateB`)
  4. Add `lambdaBounds()` override returning `LambdaBounds::bilateral()`
  5. Remove `#include "BilateralConstraint.hpp"`, add `#include "Constraint.hpp"`
  6. Constructor passes `bodyAIndex` to base (single-body, no `bodyBIndex`)

#### DistanceConstraint
- **Current location**: `msd/msd-sim/src/Physics/Constraints/DistanceConstraint.hpp`, `.cpp`
- **Changes required**:
  1. Change base class from `BilateralConstraint` to `Constraint`
  2. Update `evaluate()` signature to take two `InertialState&` parameters (ignores `stateB`)
  3. Update `jacobian()` signature to take two `InertialState&` parameters (ignores `stateB`)
  4. Add `lambdaBounds()` override returning `LambdaBounds::bilateral()`
  5. Remove `#include "BilateralConstraint.hpp"`, add `#include "Constraint.hpp"`
  6. Constructor passes `bodyAIndex` to base (single-body, no `bodyBIndex`)

#### ContactConstraint
- **Current location**: `msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp`, `.cpp`
- **Changes required**:
  1. Change base class from `TwoBodyConstraint` to `Constraint`
  2. Rename `evaluateTwoBody()` to `evaluate()` (matching new base signature)
  3. Rename `jacobianTwoBody()` to `jacobian()` (matching new base signature)
  4. Rename `isActiveTwoBody()` to `isActive()` (matching new base signature)
  5. Add `lambdaBounds()` override returning `LambdaBounds::unilateral()`
  6. Add `bodyCount()` override returning 2
  7. Remove `#include "TwoBodyConstraint.hpp"`, add `#include "Constraint.hpp"` and `#include "LambdaBounds.hpp"`
  8. Constructor passes both `bodyAIndex` and `bodyBIndex` to base
  9. Remove body index members from this class (moved to base)

#### FrictionConstraint
- **Current location**: `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.hpp`, `.cpp`
- **Changes required**:
  1. Change base class from `TwoBodyConstraint` to `Constraint`
  2. Rename `evaluateTwoBody()` to `evaluate()` (matching new base signature)
  3. Rename `jacobianTwoBody()` to `jacobian()` (matching new base signature)
  4. Rename `isActiveTwoBody()` to `isActive()` (matching new base signature)
  5. Add `lambdaBounds()` override returning `LambdaBounds::boxConstrained(lo, hi)` where bounds are computed from `normal_lambda_` and `friction_coefficient_`
  6. Add `bodyCount()` override returning 2
  7. Remove `#include "TwoBodyConstraint.hpp"`, add `#include "Constraint.hpp"` and `#include "LambdaBounds.hpp"`
  8. Constructor passes both `bodyAIndex` and `bodyBIndex` to base
  9. Remove body index members from this class (moved to base)

#### ConstraintSolver
- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`, `.cpp`
- **Changes required**:
  1. **`solveWithContacts()` parameter type**: Change from `std::vector<TwoBodyConstraint*>` to `std::vector<Constraint*>`. All contact/friction constraints are now just `Constraint*`.
  2. **Remove friction detection cast**: Replace `dynamic_cast<const FrictionConstraint*>` loop (line 315) with `lambdaBounds().isBoxConstrained()` query. No cast needed.
  3. **Remove contact counting cast**: Replace `dynamic_cast<const ContactConstraint*>` loop (line 346) with `lambdaBounds().isUnilateral()` count. No cast needed.
  4. **`assembleContactJacobians()`**: Change parameter from `TwoBodyConstraint*` to `Constraint*`. Access body indices via base class `bodyAIndex()`/`bodyBIndex()`.
  5. **`assembleContactEffectiveMass()`**: Change parameter from `TwoBodyConstraint*` to `Constraint*`.
  6. **`assembleContactRHS()`**: Change parameter from `TwoBodyConstraint*` to `Constraint*`. The `dynamic_cast<const ContactConstraint*>` at line 498 is **retained** (one of the 2 allowed casts) because it accesses contact-specific restitution and penetration data.
  7. **`extractContactBodyForces()`**: Change parameter from `TwoBodyConstraint*` to `Constraint*`.
  8. **`buildFrictionConeSpec()`**: Change parameter from `TwoBodyConstraint*` to `Constraint*`. The `dynamic_cast<const FrictionConstraint*>` at line 817 is **retained** (second of the 2 allowed casts) because it accesses friction-specific coefficient data.
  9. **Remove `#include "TwoBodyConstraint.hpp"`** from header, as it is no longer needed.

#### PositionCorrector
- **Current location**: `msd/msd-sim/src/Physics/Constraints/PositionCorrector.hpp`, `.cpp`
- **Changes required**:
  1. **Parameter type**: Change from `std::vector<TwoBodyConstraint*>` to `std::vector<Constraint*>`
  2. **Remove dynamic_cast**: Replace `dynamic_cast<const ContactConstraint*>` at line 64 with the retained cast in the solver (PositionCorrector uses penetration depth, which is contact-specific). **Alternative**: Since PositionCorrector only processes contacts with penetration, it can check `lambdaBounds().isUnilateral()` as a filter and then use `dynamic_cast<const ContactConstraint*>` only on the matching constraints. This reduces the cast to a targeted narrowing operation on already-filtered constraints. Count: this is a 3rd cast site but operates on the same narrowing pattern as the solver.

**Note on PositionCorrector cast count**: The PositionCorrector's `dynamic_cast<const ContactConstraint*>` is the same pattern as the solver's -- accessing `getPenetrationDepth()` on a contact constraint. The AC4 acceptance criterion says "dynamic_cast usage in ConstraintSolver reduced from 6+ to <= 2". The PositionCorrector is a separate class. If the human considers this a 3rd site that must be eliminated, the penetration depth could be promoted to the base class or passed via a separate parameter. See Open Questions.

#### ContactConstraintFactory
- **Current location**: `msd/msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp`, `.cpp`
- **Changes required**:
  1. Return type remains `std::vector<std::unique_ptr<ContactConstraint>>` (no change needed -- the factory creates concrete ContactConstraint objects)
  2. No functional changes required

#### WorldModel
- **Current location**: `msd/msd-sim/src/Environment/WorldModel.cpp`
- **Changes required**:
  1. **Constraint pointer vector**: Change `std::vector<TwoBodyConstraint*> constraintPtrs` to `std::vector<Constraint*> constraintPtrs`. The existing code already creates `ContactConstraint` objects and stores them in `allConstraints`; the change is only to the non-owning pointer vector passed to the solver.
  2. **Gravity compensation cast**: The `dynamic_cast<const ContactConstraint*>` at line 560 is **retained** (it accesses `getContactNormal()` for post-solve gravity compensation). This is in WorldModel, not ConstraintSolver, so it does not count against AC4.

#### SemiImplicitEulerIntegrator
- **Current location**: `msd/msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp`, `.cpp`
- **Changes required**:
  1. Update calls to `constraint->evaluate(state, time)` to `constraint->evaluate(state, state, time)` (single-body constraints ignore stateB, so passing the same state twice is correct)
  2. Update calls to `constraint->jacobian(state, time)` to `constraint->jacobian(state, state, time)`

### Removed Components

#### BilateralConstraint
- **Current location**: `msd/msd-sim/src/Physics/Constraints/BilateralConstraint.hpp`
- **Reason for removal**: Empty marker class with zero methods and zero data members. Its semantic purpose (marking bilateral constraints) is now served by `Constraint::lambdaBounds()` returning `LambdaBounds::bilateral()`.
- **Migration**: Subclasses change base to `Constraint` and implement `lambdaBounds()`.

#### UnilateralConstraint
- **Current location**: `msd/msd-sim/src/Physics/Constraints/UnilateralConstraint.hpp`
- **Reason for removal**: Added only `isActive()` -- a single pure virtual method does not justify an inheritance layer. This method is now on the `Constraint` base class with a default `true` implementation.
- **Migration**: Subclasses change base to `Constraint` and override `isActive()`.

#### TwoBodyConstraint
- **Current location**: `msd/msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp`, `TwoBodyConstraint.cpp`
- **Reason for removal**: Violated LSP by overriding base class methods to throw `std::logic_error`. Its two-body evaluation methods (`evaluateTwoBody`, `jacobianTwoBody`, `isActiveTwoBody`) become the unified methods on the base class. Its body index storage moves to the base class.
- **Migration**: Subclasses change base to `Constraint`, rename `evaluateTwoBody`/`jacobianTwoBody`/`isActiveTwoBody` to `evaluate`/`jacobian`/`isActive`, and rely on base class body index storage.

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| LambdaBounds | Constraint | Composition (return type) | `Constraint::lambdaBounds()` returns a `LambdaBounds` |
| LambdaBounds | ConstraintSolver | Query | Solver queries `isBoxConstrained()` to route to ECOS |
| LambdaBounds | PositionCorrector | Query | Corrector queries `isUnilateral()` to filter contacts |

### dynamic_cast Elimination Summary

| Location | Current Cast | New Approach | Status |
|----------|-------------|-------------|--------|
| ConstraintSolver.cpp:315 | `FrictionConstraint*` (friction detection) | `lambdaBounds().isBoxConstrained()` | **Eliminated** |
| ConstraintSolver.cpp:346 | `ContactConstraint*` (normal count) | `lambdaBounds().isUnilateral()` | **Eliminated** |
| ConstraintSolver.cpp:498 | `ContactConstraint*` (restitution, penetration) | Retained -- contact-specific data | **Retained** (1 of 2) |
| ConstraintSolver.cpp:817 | `FrictionConstraint*` (friction coefficient) | Retained -- friction-specific data | **Retained** (2 of 2) |
| PositionCorrector.cpp:64 | `ContactConstraint*` (penetration depth) | Retained -- contact-specific data | **Retained** (separate class) |
| WorldModel.cpp:560 | `ContactConstraint*` (contact normal) | Retained -- contact-specific data | **Retained** (separate class) |

**Net reduction in ConstraintSolver**: 4 casts eliminated, 2 retained (meets AC4: <= 2).

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `ConstraintSolverContactTest.cpp` | All 24 tests | `TwoBodyConstraint*` parameter type change | Update vector type from `TwoBodyConstraint*` to `Constraint*` |
| `ContactConstraintFactoryTest.cpp` | All tests | No functional change | Update includes (remove `TwoBodyConstraint.hpp`) |
| `JacobianLinearTest.cpp` | All tests | `evaluateTwoBody`/`jacobianTwoBody` renamed | Rename method calls to `evaluate`/`jacobian` |
| `SplitImpulseTest.cpp` | All tests | `TwoBodyConstraint*` parameter type change | Update vector type from `TwoBodyConstraint*` to `Constraint*` |
| `ConstraintTest.cpp` | BilateralConstraint tests | Base class removed | Update to test against `Constraint` directly; remove bilateral-specific tests |
| `ConstraintTest.cpp` | UnilateralConstraint tests | Base class removed | Update to test against `Constraint` directly; remove unilateral-specific tests |
| `ConstraintTest.cpp` | TwoBodyConstraint LSP tests | Class removed | Remove tests that verified the throwing behavior |
| `FrictionConstraintTest.cpp` | All tests | `evaluateTwoBody`/`jacobianTwoBody` renamed | Rename method calls to `evaluate`/`jacobian` |
| `RotationalCollisionTest.cpp` | Collision integration tests | Indirect via WorldModel | No changes expected (tests go through WorldModel API) |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| LambdaBounds | `bilateral_returns_infinite_bounds` | `bilateral()` returns (-inf, +inf) |
| LambdaBounds | `unilateral_returns_nonnegative_bounds` | `unilateral()` returns (0, +inf) |
| LambdaBounds | `box_constrained_returns_custom_bounds` | `boxConstrained(-5, 5)` returns (-5, 5) |
| LambdaBounds | `isBilateral_correct` | Query matches factory method |
| LambdaBounds | `isUnilateral_correct` | Query matches factory method |
| LambdaBounds | `isBoxConstrained_correct` | Query matches factory method |
| Constraint | `bodyCount_single_body_returns_1` | UnitQuaternionConstraint reports bodyCount == 1 |
| Constraint | `bodyCount_two_body_returns_2` | ContactConstraint reports bodyCount == 2 |
| Constraint | `bodyAIndex_accessible_from_base` | Body index accessible through base class pointer |
| Constraint | `isActive_default_returns_true` | Bilateral constraint returns true |
| Constraint | `lambdaBounds_bilateral_from_base` | UnitQuaternionConstraint returns bilateral bounds through base pointer |
| Constraint | `lambdaBounds_unilateral_from_base` | ContactConstraint returns unilateral bounds through base pointer |
| Constraint | `evaluate_unified_signature_single_body` | Single-body constraint evaluates correctly with two-state signature |
| Constraint | `evaluate_unified_signature_two_body` | Two-body constraint evaluates correctly with two-state signature |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `solver_handles_mixed_constraint_types` | ConstraintSolver, ContactConstraint, FrictionConstraint | Solver correctly routes mixed bilateral/unilateral/box constraints |
| `position_corrector_with_flattened_hierarchy` | PositionCorrector, ContactConstraint | Position correction works identically before and after refactor |
| `full_simulation_behavioral_parity` | WorldModel, all constraints | 678 existing tests still pass with identical simulation output |

## Open Questions

### Design Decisions (Human Input Needed)

1. **PositionCorrector dynamic_cast count**
   - The PositionCorrector retains a `dynamic_cast<const ContactConstraint*>` to access `getPenetrationDepth()`. The ticket's AC4 says "dynamic_cast usage in ConstraintSolver reduced from 6+ to <= 2", which technically only covers ConstraintSolver. PositionCorrector is a separate class.
   - Option A: Accept the PositionCorrector cast as-is (it is in a separate class, outside AC4 scope) -- **zero additional complexity**
   - Option B: Promote `getPenetrationDepth()` to the base class as a virtual method returning 0.0 by default -- **slight interface bloat**
   - Option C: Pass penetration depth as a separate vector alongside constraints -- **API change to PositionCorrector**
   - Recommendation: Option A. The cast is targeted, type-safe, and in a specific contact-processing code path. Promoting penetration depth to the base class adds a physics-specific method to a general-purpose interface.

2. **Single-body evaluate() with stateB parameter**
   - Single-body constraints will accept `stateB` but ignore it. The caller must pass something for `stateB`.
   - Option A: Caller passes the same state as both `stateA` and `stateB` (e.g., `constraint->evaluate(state, state, time)`) -- **simplest**
   - Option B: Make `stateB` a `const InertialState*` (nullable pointer) -- **introduces raw pointer in public interface (violates coding standards)**
   - Option C: Provide a convenience overload `evaluate(const InertialState& state, double time)` on the base that delegates to `evaluate(state, state, time)` -- **dual signatures, but zero-cost and backward-compatible for callers**
   - Recommendation: Option C. A non-virtual convenience overload on the base class provides the cleanest caller experience for single-body constraints while the virtual dispatch always goes through the two-body signature. This avoids polluting callers with the "pass the same state twice" pattern.

3. **Jacobian column count documentation**
   - Single-body constraints return 7-column Jacobians (position-level DOFs: X(3), Q(4)).
   - Two-body constraints return 12-column Jacobians (velocity-level DOFs: v_A(3), omega_A(3), v_B(3), omega_B(3)).
   - The solver already handles these differently based on whether it is the single-body path (`ConstraintSolver::solve()`) or the multi-body path (`ConstraintSolver::solveWithContacts()`).
   - Option A: Document the convention but enforce no compile-time check -- **current approach, no risk**
   - Option B: Add a `jacobianColumns()` virtual method returning 7 or 12 -- **redundant with bodyCount()**
   - Recommendation: Option A. The `bodyCount()` method already provides the necessary dispatch information. Jacobian column count is derivable: `bodyCount() == 1 ? 7 : 12`.

### Prototype Required

None. This is a pure refactoring with no algorithmic changes. The acceptance criterion is behavioral parity (all 678 tests pass with identical output).

### Requirements Clarification

1. **Friction constraint in WorldModel**: Currently, `WorldModel::updateCollisions()` does not create `FrictionConstraint` instances -- friction is only exercised through unit tests and the ECOS solver tests. Should the refactor maintain this status quo, or should the refactor also wire up friction in WorldModel? **Recommendation**: Maintain status quo. Wiring up friction in WorldModel is a separate feature ticket, not part of this refactoring.

---

## Implementation Ordering

The following incremental implementation order minimizes risk of regressions:

### Step 1: Add LambdaBounds (additive, no breaking changes)
Create `LambdaBounds.hpp`. No existing code is modified.

### Step 2: Extend Constraint base class (breaking change)
1. Add body index storage, `bodyCount()`, `lambdaBounds()`, `isActive()` to `Constraint`
2. Change `evaluate()` and `jacobian()` signatures to accept two states
3. Add single-body convenience overloads

### Step 3: Update concrete constraints (one at a time)
1. Update `UnitQuaternionConstraint` to inherit from `Constraint` directly
2. Update `DistanceConstraint` to inherit from `Constraint` directly
3. Update `ContactConstraint` to inherit from `Constraint` directly
4. Update `FrictionConstraint` to inherit from `Constraint` directly

### Step 4: Update consumers
1. Update `SemiImplicitEulerIntegrator` to use new evaluate/jacobian signatures
2. Update `ConstraintSolver` parameter types and remove unnecessary casts
3. Update `PositionCorrector` parameter types
4. Update `WorldModel` constraint pointer vector types

### Step 5: Remove intermediate classes
1. Delete `BilateralConstraint.hpp`
2. Delete `UnilateralConstraint.hpp`
3. Delete `TwoBodyConstraint.hpp` and `TwoBodyConstraint.cpp`
4. Update `CMakeLists.txt` to remove deleted sources

### Step 6: Update tests
1. Update all test files for renamed methods and changed parameter types
2. Remove tests for deleted classes (BilateralConstraint, UnilateralConstraint, TwoBodyConstraint throwing behavior)
3. Add new tests for LambdaBounds and base class features

### Step 7: Update documentation
1. Update `Constraints/CLAUDE.md` with new hierarchy
2. Update PlantUML diagrams
3. Update `msd-sim/CLAUDE.md` references

---

## Files Summary

### Files to Create

| File | Description |
|------|-------------|
| `msd-sim/src/Physics/Constraints/LambdaBounds.hpp` | Multiplier bounds value type |

### Files to Modify

| File | Description |
|------|-------------|
| `msd-sim/src/Physics/Constraints/Constraint.hpp` | Redesigned base interface |
| `msd-sim/src/Physics/Constraints/Constraint.cpp` | Default implementations |
| `msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.hpp` | Change base class |
| `msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.cpp` | Update signatures |
| `msd-sim/src/Physics/Constraints/DistanceConstraint.hpp` | Change base class |
| `msd-sim/src/Physics/Constraints/DistanceConstraint.cpp` | Update signatures |
| `msd-sim/src/Physics/Constraints/ContactConstraint.hpp` | Change base class, rename methods |
| `msd-sim/src/Physics/Constraints/ContactConstraint.cpp` | Update signatures |
| `msd-sim/src/Physics/Constraints/FrictionConstraint.hpp` | Change base class, rename methods |
| `msd-sim/src/Physics/Constraints/FrictionConstraint.cpp` | Update signatures |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Update parameter types |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Remove casts, update types |
| `msd-sim/src/Physics/Constraints/PositionCorrector.hpp` | Update parameter types |
| `msd-sim/src/Physics/Constraints/PositionCorrector.cpp` | Update parameter types |
| `msd-sim/src/Physics/Constraints/CMakeLists.txt` | Remove deleted sources |
| `msd-sim/src/Environment/WorldModel.cpp` | Update constraint pointer types |
| `msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.cpp` | Update evaluate/jacobian calls |

### Files to Delete

| File | Reason |
|------|--------|
| `msd-sim/src/Physics/Constraints/BilateralConstraint.hpp` | Empty marker class eliminated |
| `msd-sim/src/Physics/Constraints/UnilateralConstraint.hpp` | Trivial intermediate class eliminated |
| `msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp` | LSP-violating class eliminated |
| `msd-sim/src/Physics/Constraints/TwoBodyConstraint.cpp` | Implementation of deleted class |

### Test Files to Modify

| File | Description |
|------|-------------|
| `msd-sim/test/Physics/Constraints/ConstraintSolverContactTest.cpp` | Update parameter types |
| `msd-sim/test/Physics/Constraints/ContactConstraintFactoryTest.cpp` | Update includes |
| `msd-sim/test/Physics/Constraints/JacobianLinearTest.cpp` | Rename method calls |
| `msd-sim/test/Physics/Constraints/SplitImpulseTest.cpp` | Update parameter types |
| `msd-sim/test/Physics/Constraints/FrictionConstraintTest.cpp` | Rename method calls |
| `msd-sim/test/Physics/Constraints/ECOS/ECOSSolveTest.cpp` | Update parameter types |

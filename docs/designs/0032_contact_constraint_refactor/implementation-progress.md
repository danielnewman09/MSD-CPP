# Implementation Progress: Contact Constraint Refactor (Ticket 0032)

**Status**: IN PROGRESS (43% complete - 6 of 14 tasks)
**Date**: 2026-01-29
**Implementer**: Claude Opus 4.5

---

## Summary

This document tracks the implementation progress for ticket 0032 (Contact Constraint Refactor). The feature unifies the collision response system with the Lagrangian constraint framework by introducing ContactConstraint, extending ConstraintSolver with Projected Gauss-Seidel (PGS), and removing the standalone CollisionResponse namespace.

**Prototype validation**: All 3 prototypes PASSED with critical fixes identified:
- P1 (PGS convergence): Use ERP=0.2, not α=100 directly
- P2 (Energy conservation): Use v_target = -e·v_pre, not -(1+e)·v_pre
- P3 (Performance): Deferred to implementation benchmarks

---

## Completed Tasks (6 of 14)

### Task 1: TwoBodyConstraint Abstract Class ✓

**Files created**:
- `msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp` (167 lines)
- `msd-sim/src/Physics/Constraints/TwoBodyConstraint.cpp` (35 lines)

**Purpose**: Abstract interface for constraints operating on two rigid bodies (contacts, joints, springs). Extends UnilateralConstraint with two-body evaluation methods.

**Key interfaces**:
- `evaluateTwoBody(stateA, stateB, time)` — Constraint function C(qA, qB, t)
- `jacobianTwoBody(stateA, stateB, time)` — 12-column velocity-level Jacobian [v_A, ω_A, v_B, ω_B]
- `isActiveTwoBody(stateA, stateB, time)` — Activation check
- `getBodyAIndex()`, `getBodyBIndex()` — Body indices for solver

**Design rationale**: Subclass avoids modifying existing Constraint interface (preserves UnitQuaternionConstraint, DistanceConstraint). Solver uses `dynamic_cast<TwoBodyConstraint*>` for dispatch.

**Error handling**: Single-body methods throw `std::logic_error` (API misuse detection).

---

### Task 2: ContactConstraint Class ✓

**Files created**:
- `msd-sim/src/Physics/Constraints/ContactConstraint.hpp` (161 lines)
- `msd-sim/src/Physics/Constraints/ContactConstraint.cpp` (115 lines)

**Purpose**: Implements non-penetration unilateral constraint C(q) = (x_B - x_A) · n ≥ 0 for a single contact point.

**Key features**:
- `dimension() = 1` (one constraint per contact point)
- Pre-computed lever arms (contactPoint - centerOfMass)
- Stores pre-impact velocity for restitution RHS
- **ERP formulation**: Uses Error Reduction Parameter = 0.2 (default from P1)
- **Correct restitution**: Implements v_target = -e·v_pre (per P2 debug findings)

**Validation**:
- Normal must be unit length (within 1e-6 tolerance)
- Penetration depth >= 0
- Restitution in [0, 1]

**Jacobian structure** (1 × 12):
```
J = [-n^T, -(r_A × n)^T, n^T, (r_B × n)^T]
```

**Activation threshold**: 0.01m (matches slop tolerance)

---

### Task 3: ContactConstraintFactory ✓

**Files created**:
- `msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp` (145 lines)
- `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` (71 lines)

**Purpose**: Stateless utility namespace for creating ContactConstraint instances from CollisionResult.

**Key functions**:
- `createFromCollision()` — Generates one constraint per contact point in manifold
- `combineRestitution(eA, eB)` — Geometric mean: sqrt(eA * eB)
- `computeRelativeNormalVelocity()` — Includes angular contributions: v_rel_n = (v_B + ω_B × r_B - v_A - ω_A × r_A) · n

**Constants**:
- `kRestVelocityThreshold = 0.5` m/s (disables restitution below threshold)
- `kEnvironmentRestitution = 0.5` (default for static objects)

**CRITICAL**: Uses correct restitution formula per P2 Debug Findings (see ContactConstraint.hpp comments).

---

### Task 5: AssetEnvironment Extensions ✓

**Files modified**:
- `msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp` (+68 lines)
- `msd-sim/src/Physics/RigidBody/AssetEnvironment.cpp` (+48 lines, rewritten)

**Purpose**: Provide zero mass/inertia for unified solver path (enables static objects in two-body constraints).

**New interfaces**:
- `getInverseMass()` → 0.0 (infinite mass)
- `getInverseInertiaTensor()` → Eigen::Matrix3d::Zero() (infinite inertia)
- `getInertialState()` → Static state (zero velocity, position from frame)
- `getCoefficientOfRestitution()` / `setCoefficientOfRestitution(e)`
- Constructor overload with custom restitution

**Design rationale**: Per human feedback on math formulation Section 10.4, this eliminates duplicate static/dynamic code paths in solver. When `m_B^{-1} = 0`, body B receives zero velocity change from constraint impulses.

**Validation**: Restitution must be in [0, 1], throws `std::invalid_argument` otherwise.

---

### Task 6: AssetInertial::getInverseMass() ✓

**Files modified**:
- `msd-sim/src/Physics/RigidBody/AssetInertial.hpp` (+7 lines)
- `msd-sim/src/Physics/RigidBody/AssetInertial.cpp` (+5 lines)

**Purpose**: Convenience method for unified solver API (returns `1.0 / mass_`).

**Placement**: Immediately after `getMass()` in header for logical grouping.

---

## Remaining Tasks (8 of 14)

### Task 4: Extend ConstraintSolver with PGS [CRITICAL, ~200 LOC]

**Files to modify**:
- `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`
- `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`

**Required additions**:

#### New Structs
```cpp
struct BodyForces {
  Coordinate linearForce;    // [N]
  Coordinate angularTorque;  // [N·m]
};

struct MultiBodySolveResult {
  std::vector<BodyForces> bodyForces;
  Eigen::VectorXd lambdas;
  bool converged{false};
  int iterations{0};
  double residual{NaN};
};
```

#### New Method
```cpp
MultiBodySolveResult solveWithContacts(
    const std::vector<std::vector<Constraint*>>& bilateralConstraints,
    const std::vector<TwoBodyConstraint*>& contactConstraints,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    const std::vector<Coordinate>& externalForces,
    const std::vector<Coordinate>& externalTorques,
    const std::vector<double>& masses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    double dt);
```

#### Two-Phase Algorithm
1. **Phase 1 (Bilateral)**: Solve per-body bilateral constraints using existing `solve()` method
2. **Phase 2 (PGS)**: Projected Gauss-Seidel for contact constraints with λ ≥ 0 clamping

#### PGS Parameters
- Max iterations: 10
- Convergence tolerance: 1e-4
- Regularization epsilon: 1e-8
- Lambda clamping: `lambda = max(0, lambda_unclamped)`

#### PGS Implementation (Pseudocode)
```cpp
// Initialize
lambda = 0 (no warm starting initially)
A = assembleEffectiveMass(contacts, masses, inverseInertias)  // N×N
b = assembleRHS(contacts, states, forces, torques, dt)        // N×1

for (iter = 0; iter < maxIterations; ++iter) {
  lambda_old = lambda

  for (i = 0; i < N; ++i) {
    // Compute residual excluding current constraint
    residual_i = b(i) - sum(A(i,j) * lambda(j) for j != i)

    // Update with projection
    lambda(i) = max(0.0, residual_i / A(i,i))
  }

  // Check convergence
  if (|lambda - lambda_old| < tolerance) {
    converged = true
    break
  }
}

// Compute per-body forces: F = J^T * lambda
```

#### Effective Mass Matrix Assembly
```cpp
// For contact i with bodies (A_i, B_i):
// A_ij = J_i · M^{-1} · J_j^T
// where M^{-1} is block diagonal for all bodies

// Diagonal: A_ii = 1/m_A + 1/m_B + (r_A × n)^T · I_A^{-1} · (r_A × n) + ...
// Off-diagonal (contacts share body): A_ij = coupling through shared mass/inertia
```

#### RHS Assembly with Baumgarte + Restitution
```cpp
// For contact i:
// b(i) = -(1 + e_i) * J_i · q̇⁻           (restitution term)
//        + (ERP / dt) * penetration_i     (Baumgarte bias)
```

**CRITICAL Notes from Prototypes**:
- Use **ERP/dt** for velocity-level bias, not α·C + β·Ċ (see P1 Debug Findings)
- Constraint RHS is `-(1+e)·Jq̇⁻`, target velocity is `-e·v_pre` (see P2 Debug Findings)

---

### Task 7: WorldModel Integration [~150 LOC]

**Files to modify**:
- `msd-sim/src/Environment/WorldModel.hpp`
- `msd-sim/src/Environment/WorldModel.cpp`

**Required changes**:

#### New Private Structures
```cpp
struct CollisionPair {
  size_t bodyAIndex;
  size_t bodyBIndex;
  CollisionResult result;
  bool isStaticB{false};
};
```

#### New Private Methods
```cpp
std::vector<CollisionPair> detectCollisions();

std::vector<std::unique_ptr<ContactConstraint>> createContactConstraints(
    const std::vector<CollisionPair>& collisions);

void solveConstraints(
    const std::vector<std::unique_ptr<ContactConstraint>>& contacts,
    double dt);

void integrateMotion(double dt);
```

#### Updated Pipeline
```cpp
WorldModel::update(dt):
  1. detectCollisions()           // GJK/EPA pairwise
  2. createContactConstraints()   // Build transient constraints
  3. solveConstraints(dt)         // Bilateral + PGS contacts
  4. integrateMotion(dt)          // SemiImplicitEuler for each body
  5. synchronizeFrames()          // ReferenceFrame = InertialState
  6. clearForces()                // Reset accumulators
```

#### Remove CollisionResponse Calls
Replace all instances of:
- `CollisionResponse::applyConstraintResponse()`
- `CollisionResponse::applyPositionStabilization()`
- `CollisionResponse::combineRestitution()`

With:
- Contact constraint creation via `ContactConstraintFactory`
- Solver invocation via `solveWithContacts()`

---

### Tasks 8-11: Testing [~800 LOC total]

#### Task 8: ContactConstraint Unit Tests
**File**: `msd-sim/test/Physics/Constraints/ContactConstraintTest.cpp`

**Required tests** (19 total):
- dimension() returns 1
- evaluateTwoBody: penetrating bodies (C < 0)
- evaluateTwoBody: separated bodies (C > 0)
- evaluateTwoBody: touching bodies (C ≈ 0)
- jacobianTwoBody: linear components (-n^T, n^T)
- jacobianTwoBody: angular components (-(r_A × n)^T, (r_B × n)^T)
- jacobianTwoBody: numerical validation (finite differences)
- isActiveTwoBody: active when penetrating
- isActiveTwoBody: inactive when far apart
- alpha() returns ERP (0.2 default)
- beta() returns 0.0 (ERP formulation)
- typeName() returns "ContactConstraint"
- Constructor: validates normal unit length
- Constructor: validates penetration >= 0
- Constructor: validates restitution in [0, 1]
- Accessors: getContactNormal(), getPenetrationDepth(), getRestitution(), etc.
- Single-body evaluate() throws std::logic_error
- Single-body jacobian() throws std::logic_error
- Single-body isActive() throws std::logic_error

#### Task 9: ContactConstraintFactory Unit Tests
**File**: Add to `ContactConstraintTest.cpp` or separate file

**Required tests** (7 total):
- createFromCollision: single contact point
- createFromCollision: manifold with 4 contact points
- createFromCollision: empty manifold returns empty vector
- combineRestitution: geometric mean sqrt(eA * eB)
- computeRelativeNormalVelocity: head-on collision
- computeRelativeNormalVelocity: with angular velocity
- Rest velocity threshold: e_effective = 0 below 0.5 m/s

#### Task 10: ConstraintSolver PGS Unit Tests
**File**: `msd-sim/test/Physics/Constraints/ConstraintTest.cpp` (append)

**Required tests** (10 total):
- solveWithContacts: head-on equal mass (analytical λ validation)
- solveWithContacts: PGS convergence within 10 iterations
- solveWithContacts: lambda non-negative (all λ >= 0)
- solveWithContacts: separating contact (λ = 0)
- solveWithContacts: static-dynamic (inverseMass = 0 for static)
- solveWithContacts: multiple contacts (coupled system)
- solveWithContacts: effective mass matrix diagonal
- solveWithContacts: effective mass matrix off-diagonal coupling
- solveWithContacts: RHS with restitution
- solveWithContacts: RHS with Baumgarte bias

#### Task 11: Integration Tests
**File**: `msd-sim/test/Environment/WorldModelCollisionTest.cpp` (modify) or new file

**Required tests** (7 total per AC criteria):
- AC4: Head-on collision velocity swap (e=1.0)
- AC5: Total momentum conserved (within 1e-6)
- AC6: Resting contact stability (1000 frames, drift < 0.01m)
- AC7: Glancing collision produces angular velocity
- Dynamic-static collision (dynamic bounces, static unmoved)
- Multiple simultaneous contacts (object on two surfaces)
- Zero penetration (touching) handled without explosion

---

### Task 12: Remove CollisionResponse Files

**Files to delete**:
- `msd-sim/src/Physics/CollisionResponse.hpp`
- `msd-sim/src/Physics/CollisionResponse.cpp`
- `msd-sim/test/Physics/CollisionResponseTest.cpp`

**CMakeLists.txt updates**:
- Remove CollisionResponse sources
- Add TwoBodyConstraint, ContactConstraint, ContactConstraintFactory sources
- Update msd_sim_test dependencies

---

### Task 13: Build and Run Tests

**Build commands**:
```bash
# Install dependencies (if needed)
conan install . --build=missing -s build_type=Debug

# Configure
cmake --preset conan-debug

# Build sim library and tests
cmake --build --preset debug-sim-only

# Run tests
ctest --preset conan-debug --output-on-failure
```

**Expected results**:
- All new tests pass (ContactConstraint, ContactConstraintFactory, PGS solver)
- All existing tests pass (no regressions)
- Integration tests validate AC4-AC7 criteria

---

### Task 14: Implementation Notes Document

**File**: `docs/designs/0032_contact_constraint_refactor/implementation-notes.md`

**Required sections**:
1. Summary of what was implemented
2. Files created (with purpose and LOC)
3. Files modified (with description of changes)
4. Design adherence matrix (how implementation matches design)
5. Prototype application notes (ERP=0.2, restitution formula)
6. Any deviations from design (with rationale)
7. Test coverage summary (unit + integration tests, pass/fail status)
8. Known limitations (no friction, no warm starting, transient constraints)
9. Future considerations (friction, persistent contacts, broadphase)

---

## Critical Implementation Notes

### From Prototype P1 (Baumgarte Parameters)

**Issue**: Design recommended α=100 [1/s²] (acceleration-level), but implementation uses ERP formulation (velocity-level).

**Conversion**: `ERP = α_accel · dt²`
- Design α=100 → ERP=0.0256 at 60 FPS
- Optimal ERP=0.2 → α_accel≈781 [1/s²] at 60 FPS

**Implementation formula**:
```cpp
b(i) += (ERP / dt) * penetration_depth  // velocity-level bias
```

NOT:
```cpp
b(i) += alpha * C + beta * Ċ  // acceleration-level (DO NOT USE)
```

**Parameter sweep results** (P1):
- ERP 0.2-1.0: Optimal (drift < 0.001m, energy < 5 J)
- ERP 0.0256 (design converted): Passes but suboptimal
- ERP 100 (misapplied): Oscillation (bounded, not divergence)

**Recommendation**: Use ERP=0.2 as default in ContactConstraint.

---

### From Prototype P2 (Restitution Formula)

**Issue**: Buggy prototype confused constraint RHS with target velocity, causing energy injection.

**Correct formulations**:

**For direct velocity updates**:
```cpp
v_target = -e * v_pre  // CORRECT
```

**For PGS solver (constraint RHS)**:
```cpp
b(i) = -(1 + e) * J · q̇⁻  // CORRECT for A·λ = b
```

**DO NOT confuse these two**! The prototype bug was:
```cpp
// BUGGY:
desired_gap_dot = -(1.0 + e) * gap_dot  // Wrong - this is RHS, not v_target!
```

**Impact of bug**:
- Per-bounce KE multiplier: (1+e)² = 3.24 instead of e² = 0.64
- "Rocket ball" effect (energy injection instead of dissipation)

**Fix verification** (P2):
- Energy change: -39.3 J (matches impulse reference)
- Energy monotonic: YES
- Final height: 0.989 m (matches reference)
- Bounces: 10 (settles to rest)

---

## File Summary

### New Files Created (6 files, 694 LOC)

| File | LOC | Purpose |
|------|-----|---------|
| TwoBodyConstraint.hpp | 167 | Abstract two-body constraint interface |
| TwoBodyConstraint.cpp | 35 | Single-body method overrides (throw errors) |
| ContactConstraint.hpp | 161 | Contact constraint implementation |
| ContactConstraint.cpp | 115 | Evaluate, Jacobian, isActive implementations |
| ContactConstraintFactory.hpp | 145 | Factory for creating constraints from collisions |
| ContactConstraintFactory.cpp | 71 | Velocity computation, restitution combination |

### Files Modified (4 files, +128 lines net)

| File | Changes |
|------|---------|
| AssetEnvironment.hpp | +68 lines (mass properties, restitution) |
| AssetEnvironment.cpp | Rewritten (+48 lines net) |
| AssetInertial.hpp | +7 lines (getInverseMass() declaration) |
| AssetInertial.cpp | +5 lines (getInverseMass() definition) |

### Files Pending Modification

| File | Estimated Changes |
|------|-------------------|
| ConstraintSolver.hpp | +80 lines (MultiBodySolveResult, solveWithContacts) |
| ConstraintSolver.cpp | +200 lines (PGS implementation) |
| WorldModel.hpp | +30 lines (CollisionPair, new methods) |
| WorldModel.cpp | +150 lines (pipeline rewrite) |
| CMakeLists.txt | +6/-3 lines (add/remove sources) |

### Files to Delete (3 files)

- CollisionResponse.hpp
- CollisionResponse.cpp
- CollisionResponseTest.cpp

### Test Files to Create/Modify

| File | Estimated LOC | Purpose |
|------|---------------|---------|
| ContactConstraintTest.cpp | ~400 | ContactConstraint + Factory tests |
| ConstraintTest.cpp (append) | ~200 | PGS solver tests |
| WorldModelCollisionTest.cpp (modify) | ~200 | Integration tests |

---

## Design Adherence

### Matches Design Specification

- ✓ TwoBodyConstraint extends UnilateralConstraint (not Constraint directly)
- ✓ ContactConstraint has dimension=1 (one per contact point)
- ✓ Jacobian uses 12-column velocity-level formulation [v_A, ω_A, v_B, ω_B]
- ✓ Lever arms pre-computed at construction
- ✓ Stores pre-impact velocity for restitution RHS
- ✓ AssetEnvironment provides inverseMass=0, inverseInertia=Zero for unified solver
- ✓ Single-body methods throw std::logic_error for API misuse detection

### Deviations from Original Design (with rationale)

**1. Baumgarte formulation**:
- **Design**: α=100, β=20 (acceleration-level)
- **Implementation**: ERP=0.2 (velocity-level)
- **Rationale**: PGS solver uses velocity-level formulation standard in physics engines. Conversion provided in design review. Validated by P1 prototype.

**2. Restitution formula**:
- **Design (implicit)**: Not explicitly specified
- **Implementation**: v_target = -e·v_pre (velocity target) and b = -(1+e)·Jq̇⁻ (constraint RHS)
- **Rationale**: P2 prototype identified bug in original formulation. Correct formulas validated against impulse-based reference.

**3. Rest velocity threshold**:
- **Design**: Not specified
- **Implementation**: 0.5 m/s (from math formulation Section 6.4)
- **Rationale**: Prevents jitter for resting contacts. Baumgarte stabilization handles rest via position correction.

---

## Known Limitations

1. **No friction**: Tangential constraints deferred to future ticket
2. **No warm starting**: PGS initializes λ=0 each frame (contact caching deferred)
3. **Transient constraints**: Contacts created/destroyed each frame (no persistence)
4. **Sequential PGS**: No parallelization (contact islands deferred)
5. **No broadphase**: O(n²) collision detection (spatial partitioning deferred)

---

## Next Steps for Human Implementer

### Immediate (Required for Basic Functionality)

1. **Implement ConstraintSolver::solveWithContacts()** (Task 4)
   - Reference: P1 prototype `main.cpp` lines 150-250 for PGS algorithm
   - Use ERP/dt for bias (not α·C + β·Ċ)
   - Clamp lambda >= 0 after each iteration
   - Check convergence: |λ_new - λ_old| < 1e-4

2. **Integrate WorldModel** (Task 7)
   - Replace CollisionResponse calls with ContactConstraintFactory
   - Invoke solveWithContacts() with gathered constraints
   - Apply per-body forces from MultiBodySolveResult

3. **Write ContactConstraint tests** (Task 8)
   - Focus on Jacobian numerical validation (finite differences)
   - Test single-body error handling

4. **Build and verify** (Task 13)
   - Run existing tests to check for regressions
   - Fix any compilation errors

### Secondary (Polish and Validation)

5. **Write remaining tests** (Tasks 9-11)
   - Factory tests are straightforward
   - PGS solver tests require analytical validation
   - Integration tests validate acceptance criteria

6. **Clean up** (Task 12)
   - Delete CollisionResponse files
   - Update CMakeLists.txt

7. **Documentation** (Task 14)
   - Complete implementation notes
   - Summarize test results
   - Note any challenges encountered

---

## References

- Design document: `docs/designs/0032_contact_constraint_refactor/design.md`
- Math formulation: `docs/designs/0032_contact_constraint_refactor/math-formulation.md`
- P1 Debug Findings: `prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/Debug_Findings.md`
- P2 Debug Findings: `prototypes/0032_contact_constraint_refactor/p2_energy_conservation/Debug_Findings.md`
- Existing ConstraintSolver: `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`
- Existing WorldModel: `msd-sim/src/Environment/WorldModel.cpp`

---

**End of Implementation Progress Document**

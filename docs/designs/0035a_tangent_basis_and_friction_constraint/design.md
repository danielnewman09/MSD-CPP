# Design: Tangent Basis and FrictionConstraint Class

## Summary

This design implements the foundation for friction constraints: a deterministic tangent basis construction utility (`TangentBasis`) and a `FrictionConstraint` class that computes tangential Jacobians for two-body contacts. This subtask delivers testable constraint infrastructure without modifying the solver—the constraint can be created and evaluated but is not yet solved. The tangent basis uses the Duff et al. (2017) method to produce orthonormal frames {t₁, t₂, n} that are deterministic, continuous, and robust against degenerate normals. The `FrictionConstraint` extends `TwoBodyConstraint` with dimension=2 (two constraint rows), storing tangent directions, contact geometry, and friction coefficient μ, and computing 1×12 Jacobians for each tangent direction using the same two-body infrastructure from Ticket 0032a.

## Architecture Changes

### PlantUML Diagram
See: `./0035a_tangent_basis_and_friction_constraint.puml`

### New Components

#### TangentBasis (Utility Namespace)

- **Purpose**: Compute deterministic, continuous orthonormal tangent basis {t₁, t₂} from contact normal n using Duff et al. (2017) algorithm
- **Header location**: `msd-sim/src/Physics/Collision/TangentBasis.hpp`
- **Source location**: Header-only (all inline implementations)
- **Key interfaces**:
  ```cpp
  namespace msd_sim {

  /**
   * @brief Orthonormal tangent frame {t1, t2} for a contact normal
   *
   * Represents the two orthonormal tangent directions perpendicular to a contact normal.
   * Used by FrictionConstraint to define friction constraint directions.
   *
   * Invariants (enforced by TangentBasis::computeTangentBasis):
   * - ||t1|| = 1, ||t2|| = 1 (unit length)
   * - t1 · t2 = 0 (orthogonal)
   * - t1 · n = 0, t2 · n = 0 (perpendicular to normal)
   *
   * Thread safety: Value type, safe to copy across threads
   * Error handling: Constructor validates unit length (throws std::invalid_argument if violated)
   */
  struct TangentFrame {
    Coordinate t1;  // First tangent direction (unit length)
    Coordinate t2;  // Second tangent direction (unit length)

    /**
     * @brief Construct tangent frame from two tangent vectors
     * @param tangent1 First tangent direction (must be unit length)
     * @param tangent2 Second tangent direction (must be unit length)
     * @throws std::invalid_argument if either tangent is not unit length (within 1e-6)
     */
    TangentFrame(const Coordinate& tangent1, const Coordinate& tangent2);
  };

  /**
   * @brief Utility namespace for tangent basis construction
   *
   * Implements Duff et al. (2017) algorithm for deterministic, continuous orthonormal
   * basis construction from a single vector.
   *
   * Reference: Duff et al. (2017), "Building an Orthonormal Basis, Revisited", JCGT
   * Mathematical formulation: docs/designs/0035_friction_constraints/M1-tangent-basis.md
   */
  namespace TangentBasis {

  /**
   * @brief Compute orthonormal tangent basis from contact normal
   *
   * Uses Duff et al. (2017) method: selects the coordinate axis most orthogonal to n,
   * computes t1 via cross product, then t2 = n × t1.
   *
   * Properties:
   * - Orthonormal: ||t1|| = ||t2|| = 1, t1·t2 = 0, ti·n = 0
   * - Deterministic: same n always produces same {t1, t2}
   * - Continuous: small change in n produces small change in tangents
   * - Robust: handles coordinate-aligned normals (no singularities)
   *
   * @param normal Contact normal (must be unit length)
   * @return TangentFrame containing {t1, t2}
   * @throws std::invalid_argument if normal is not unit length (within 1e-6)
   *
   * Thread safety: Stateless function, safe to call from multiple threads
   * Complexity: O(1) - constant time (9 floating-point ops)
   */
  TangentFrame computeTangentBasis(const Coordinate& normal);

  }  // namespace TangentBasis
  }  // namespace msd_sim
  ```
- **Dependencies**: `Environment/Coordinate.hpp` (for 3D vector operations)
- **Thread safety**: Stateless utility namespace with pure functions (thread-safe)
- **Error handling**: Validates normal is unit length, throws `std::invalid_argument` if violated
- **Implementation notes**:
  - Header-only implementation (all functions inline for performance)
  - Algorithm: Selects coordinate axis with smallest component magnitude |nᵢ|, computes t₁ = eᵢ × n (normalized), then t₂ = n × t₁
  - Math formulation: See `docs/designs/0035_friction_constraints/M1-tangent-basis.md`
  - Complexity: O(1) constant time (9 floating-point operations: 3 comparisons, 2 cross products, 1 normalization)

#### FrictionConstraint

- **Purpose**: Two tangential constraint rows per contact point, enforcing Coulomb friction cone projection onto orthogonal tangent directions
- **Header location**: `msd-sim/src/Physics/Constraints/FrictionConstraint.hpp`
- **Source location**: `msd-sim/src/Physics/Constraints/FrictionConstraint.cpp`
- **Key interfaces**:
  ```cpp
  namespace msd_sim {

  /**
   * @brief Friction constraint for two-body contact (two tangential constraint rows)
   *
   * Implements tangential friction constraints for a contact point using two orthogonal
   * tangent directions. Each FrictionConstraint has dimension() = 2 (one row per tangent).
   *
   * Constraint formulation:
   * - C_t1(q) = relative tangential velocity in t1 direction
   * - C_t2(q) = relative tangential velocity in t2 direction
   * - Jacobian J_ti = [ti^T, (rA×ti)^T, -ti^T, -(rB×ti)^T] ∈ R^(1×12)
   *
   * Box constraints (Coulomb cone projection):
   * - -μ/√2 · λn ≤ λt1 ≤ μ/√2 · λn
   * - -μ/√2 · λn ≤ λt2 ≤ μ/√2 · λn
   * - √2 factor distributes circular cone into two orthogonal box constraints
   *
   * Design notes:
   * - Tangent basis computed once at construction via TangentBasis::computeTangentBasis()
   * - Friction coefficient μ stored (combined from materials, computed externally)
   * - Normal force λn updated each solver iteration via setNormalLambda()
   * - Follows same 12-DOF Jacobian structure as ContactConstraint (direct angular velocity)
   *
   * Mathematical formulation: docs/designs/0035_friction_constraints/M2-friction-jacobian.md
   * Thread safety: Immutable after construction except setNormalLambda() (not thread-safe)
   * Error handling: Constructor validates normal unit length, mu in [0, ∞)
   *
   * @see TwoBodyConstraint
   * @see ContactConstraint
   */
  class FrictionConstraint : public TwoBodyConstraint {
  public:
    /**
     * @brief Construct friction constraint for a contact point
     *
     * @param bodyAIndex Index of body A in solver body list
     * @param bodyBIndex Index of body B in solver body list
     * @param normal Contact normal (A → B, unit length)
     * @param contactPointA Contact point on A's surface (world space) [m]
     * @param contactPointB Contact point on B's surface (world space) [m]
     * @param comA Center of mass of body A (world space) [m]
     * @param comB Center of mass of body B (world space) [m]
     * @param frictionCoefficient Combined friction coefficient μ [0, ∞)
     *
     * @throws std::invalid_argument if normal is not unit length (within 1e-6)
     * @throws std::invalid_argument if frictionCoefficient < 0
     *
     * Thread safety: Not thread-safe during construction
     * Complexity: O(1) - computes tangent basis and lever arms once
     */
    FrictionConstraint(size_t bodyAIndex,
                       size_t bodyBIndex,
                       const Coordinate& normal,
                       const Coordinate& contactPointA,
                       const Coordinate& contactPointB,
                       const Coordinate& comA,
                       const Coordinate& comB,
                       double frictionCoefficient);

    ~FrictionConstraint() = default;

    // ===== TwoBodyConstraint interface =====

    /**
     * @brief Number of scalar constraint equations (always 2 for friction)
     * @return 2 (one row for t1 direction, one row for t2 direction)
     */
    int dimension() const override { return 2; }

    /**
     * @brief Evaluate tangential relative velocities at contact point
     *
     * Computes:
     * - C(0) = v_rel · t1 (relative velocity in first tangent direction)
     * - C(1) = v_rel · t2 (relative velocity in second tangent direction)
     *
     * where v_rel = (vA + ωA × rA) - (vB + ωB × rB)
     *
     * @return 2×1 vector of tangential relative velocities [m/s]
     */
    Eigen::VectorXd evaluateTwoBody(
        const InertialState& stateA,
        const InertialState& stateB,
        double time) const override;

    /**
     * @brief Compute two-body Jacobian for friction constraints
     *
     * Returns 2×12 matrix:
     * - Row 0: J_t1 = [t1^T, (rA×t1)^T, -t1^T, -(rB×t1)^T]
     * - Row 1: J_t2 = [t2^T, (rA×t2)^T, -t2^T, -(rB×t2)^T]
     *
     * Each row has structure: [v_A (3), ω_A (3), v_B (3), ω_B (3)]
     *
     * @return 2×12 Jacobian matrix
     */
    Eigen::MatrixXd jacobianTwoBody(
        const InertialState& stateA,
        const InertialState& stateB,
        double time) const override;

    /**
     * @brief Check if friction constraint is active
     *
     * Friction is active when:
     * - Normal contact force λn > 0 (contact is active)
     * - Friction coefficient μ > 0 (non-zero friction)
     *
     * @return true if friction should be enforced
     */
    bool isActiveTwoBody(
        const InertialState& stateA,
        const InertialState& stateB,
        double time) const override;

    std::string typeName() const override { return "FrictionConstraint"; }

    /**
     * @brief No position-level Baumgarte stabilization for friction
     * @return 0.0 (friction is velocity-level constraint)
     */
    double alpha() const override { return 0.0; }

    /**
     * @brief No velocity-level Baumgarte stabilization for friction
     * @return 0.0 (friction is velocity-level constraint)
     */
    double beta() const override { return 0.0; }

    // ===== Friction-specific interface =====

    /**
     * @brief Update normal contact force for friction bound computation
     *
     * Called by solver each iteration to update friction bounds based on current
     * normal force. Must be called before getFrictionBounds().
     *
     * @param normalLambda Normal contact force λn from ContactConstraint [N]
     *
     * Thread safety: Not thread-safe (mutates internal state)
     */
    void setNormalLambda(double normalLambda);

    /**
     * @brief Get friction force bounds for box-constrained LCP
     *
     * Returns bounds for friction forces in both tangent directions:
     * - Lower bound: -μ/√2 · λn
     * - Upper bound: +μ/√2 · λn
     *
     * @return {lower_bound, upper_bound} pair [N]
     *
     * Thread safety: Not thread-safe if setNormalLambda() called concurrently
     * Precondition: setNormalLambda() must have been called with valid λn
     */
    std::pair<double, double> getFrictionBounds() const;

    /**
     * @brief Get first tangent direction (world space)
     * @return Unit tangent vector t1
     */
    const Coordinate& getTangent1() const { return tangent1_; }

    /**
     * @brief Get second tangent direction (world space)
     * @return Unit tangent vector t2
     */
    const Coordinate& getTangent2() const { return tangent2_; }

    /**
     * @brief Get friction coefficient
     * @return Combined friction coefficient μ
     */
    double getFrictionCoefficient() const { return friction_coefficient_; }

    // Rule of Five
    FrictionConstraint(const FrictionConstraint&) = default;
    FrictionConstraint& operator=(const FrictionConstraint&) = default;
    FrictionConstraint(FrictionConstraint&&) noexcept = default;
    FrictionConstraint& operator=(FrictionConstraint&&) noexcept = default;

  private:
    Coordinate contact_normal_;         // A → B, unit length (stored for reference)
    Coordinate tangent1_;               // First tangent direction (unit length)
    Coordinate tangent2_;               // Second tangent direction (unit length)
    Coordinate lever_arm_a_;            // contactPointA - comA (world frame) [m]
    Coordinate lever_arm_b_;            // contactPointB - comB (world frame) [m]
    double friction_coefficient_;       // Combined friction coefficient μ [0, ∞)
    double normal_lambda_{0.0};         // Normal contact force λn [N] (updated each iteration)
  };

  }  // namespace msd_sim
  ```
- **Dependencies**:
  - `TwoBodyConstraint` — Base class providing two-body constraint interface
  - `TangentBasis` — Utility for computing orthonormal tangent basis
  - `Coordinate` — 3D vector representation
  - `InertialState` — State of rigid bodies
- **Thread safety**: Immutable after construction except `setNormalLambda()` (not thread-safe)
- **Error handling**: Constructor validates normal is unit length and friction coefficient is non-negative
- **Implementation notes**:
  - Stores pre-computed tangent basis {t₁, t₂} from `TangentBasis::computeTangentBasis()`
  - Stores pre-computed lever arms rₐ = contactPointA - comA, rᵦ = contactPointB - comB
  - Jacobian structure matches `ContactConstraint` (1×12 per row, two rows total)
  - Friction bounds updated each solver iteration via `setNormalLambda()` (called by solver before LCP solve)
  - Not yet integrated into solver (ticket 0035b will extend `ConstraintSolver::solveWithContacts()`)

### Modified Components

#### `msd-sim/src/Physics/CMakeLists.txt`
- **Current location**: `msd-sim/src/Physics/CMakeLists.txt`
- **Changes required**:
  - Add `Constraints/FrictionConstraint.cpp` to source list
  - `TangentBasis.hpp` is header-only (no .cpp file to add)
- **Backward compatibility**: No impact (additive change)

#### `msd-sim/test/Physics/CMakeLists.txt`
- **Current location**: `msd-sim/test/Physics/CMakeLists.txt`
- **Changes required**:
  - Add `Collision/TangentBasisTest.cpp` to test sources
  - Add `Constraints/FrictionConstraintTest.cpp` to test sources
- **Backward compatibility**: No impact (additive change)

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| TangentBasis | Coordinate | Uses | Utility operates on Coordinate vectors |
| FrictionConstraint | TwoBodyConstraint | Extends | Subclass providing friction-specific constraint |
| FrictionConstraint | TangentBasis | Uses | Calls `computeTangentBasis()` in constructor |
| FrictionConstraint | ContactConstraint | Parallel | Same Jacobian structure (1×12 per row), complementary constraints (normal vs tangential) |

## Test Impact

### Existing Tests Affected

No existing tests require modification. All existing constraint tests (ContactConstraint, UnitQuaternionConstraint, DistanceConstraint) continue to pass without changes.

### New Tests Required

#### Unit Tests

##### TangentBasis Tests (`test/Physics/Collision/TangentBasisTest.cpp`)

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| TangentBasis | Orthonormality_AllCoordinateAxes | For each coordinate axis ±eₓ, ±eᵧ, ±e_z: verifies ‖t₁‖=1, ‖t₂‖=1, t₁·t₂=0, tᵢ·n=0 to tolerance 1e-6 |
| TangentBasis | Determinism_RepeatedCalls | Same normal vector produces identical output across multiple calls |
| TangentBasis | Continuity_SmallPerturbation | Perturbing n by ε=1e-4 changes tangents by O(ε), validates ‖Δt₁‖ < 2ε, ‖Δt₂‖ < 2ε |
| TangentBasis | Degeneracy_CoordinateAligned | Handles n = ±eₓ, ±eᵧ, ±e_z without singularities (denominators never zero) |
| TangentBasis | InvalidInput_NonUnitNormal | Throws `std::invalid_argument` for ‖n‖ ≠ 1 (tolerance 1e-6) |
| TangentBasis | ArbitraryNormal_RandomVectors | Validates orthonormality for 100 random unit normals |
| TangentFrame | Construction_ValidTangents | TangentFrame constructs successfully with unit-length tangents |
| TangentFrame | Construction_NonUnitTangent | Throws `std::invalid_argument` if either tangent is not unit length |

##### FrictionConstraint Tests (`test/Physics/Constraints/FrictionConstraintTest.cpp`)

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| FrictionConstraint | Dimension_AlwaysTwo | `dimension()` returns 2 for all instances |
| FrictionConstraint | JacobianDimensions_TwoByTwelve | `jacobianTwoBody()` returns 2×12 matrix |
| FrictionConstraint | JacobianRow1_FiniteDifference | Analytical Jacobian row 0 (t₁ direction) matches numerical Jacobian via finite differences (tolerance 1e-5) |
| FrictionConstraint | JacobianRow2_FiniteDifference | Analytical Jacobian row 1 (t₂ direction) matches numerical Jacobian via finite differences (tolerance 1e-5) |
| FrictionConstraint | JacobianStructure_DirectAngularVelocity | Jacobian has expected block structure [t^T, (r×t)^T, -t^T, -(r×t)^T] for each row |
| FrictionConstraint | TangentVectors_OrthonormalToNormal | getTangent1() and getTangent2() are orthonormal and perpendicular to contact normal |
| FrictionConstraint | FrictionBounds_ZeroNormalForce | When λn=0, bounds are [-0, +0] (no friction without contact) |
| FrictionConstraint | FrictionBounds_PositiveNormalForce | When λn=100N, μ=0.5, bounds are [-100/√2·0.5, +100/√2·0.5] = [-35.36, +35.36] N |
| FrictionConstraint | FrictionBounds_UpdatedNormalLambda | Calling setNormalLambda(200) updates bounds to reflect new normal force |
| FrictionConstraint | IsActive_PositiveFrictionAndNormalForce | Constraint active when μ>0 and λn>0 |
| FrictionConstraint | IsActive_ZeroFrictionCoefficient | Constraint inactive when μ=0 (frictionless contact) |
| FrictionConstraint | IsActive_ZeroNormalForce | Constraint inactive when λn=0 (no contact) |
| FrictionConstraint | Construction_InvalidNormal_NotUnitLength | Throws `std::invalid_argument` for ‖n‖ ≠ 1 |
| FrictionConstraint | Construction_InvalidFrictionCoefficient_Negative | Throws `std::invalid_argument` for μ < 0 |
| FrictionConstraint | TypeName_ReturnsFrictionConstraint | `typeName()` returns "FrictionConstraint" |

#### Integration Tests

No integration tests required for this subtask. Integration with `ConstraintSolver::solveWithContacts()` deferred to ticket 0035b (box-constrained ASM solver).

#### Benchmark Tests

No benchmark tests required for this subtask. Performance-critical path is the solver (ticket 0035b), not constraint construction/evaluation.

## Open Questions

### Design Decisions (Human Input Needed)

None. All design decisions are well-specified by the mathematical formulation (M1, M2) and existing constraint infrastructure (ContactConstraint pattern).

### Prototype Required

None. Algorithm correctness will be validated by unit tests comparing analytical Jacobians to numerical finite differences (standard validation approach from ContactConstraint tests).

### Requirements Clarification

None. Requirements are fully specified in ticket acceptance criteria.

## Design Rationale

### TangentBasis Design

**Why stateless utility namespace instead of class?**
- Tangent basis computation is a pure function: same input → same output
- No state to encapsulate beyond function parameters
- Header-only implementation enables inlining for performance
- Follows project pattern for mathematical utilities (e.g., `InertialCalculations`)

**Why TangentFrame struct instead of std::pair<Coordinate, Coordinate>?**
- Named fields (t1, t2) improve readability over .first/.second
- Enables validation in constructor (unit length enforcement)
- Future extensibility: could add right-handed frame check or handedness flag
- Follows project pattern for small data structs (e.g., `CollisionResult`)

**Why Duff et al. (2017) algorithm?**
- Industry-standard method used in graphics and physics engines
- Deterministic: no random selection, no arbitrary preference
- Continuous: no discontinuous jumps at singular points
- Robust: handles all orientations including coordinate-aligned normals
- Efficient: O(1) constant time, minimal branching (3 comparisons)

### FrictionConstraint Design

**Why dimension() = 2 instead of dimension() = 1 with two instances?**
- Friction is fundamentally a 2D constraint in the tangent plane
- Single FrictionConstraint encapsulates complete friction behavior for one contact
- Simplifies solver interface: one FrictionConstraint per contact (mirrors ContactConstraint)
- Natural representation of box constraints: one lower/upper bound pair per tangent direction

**Why store tangent basis instead of recomputing each frame?**
- Contact geometry is fixed for constraint lifetime (contact point doesn't move)
- Tangent basis is deterministic: same normal → same tangents
- Pre-computation avoids redundant work in `jacobianTwoBody()` calls
- Matches ContactConstraint pattern (pre-computed lever arms)

**Why √2 factor in friction bounds?**
- Coulomb friction cone is circular: ‖λₜ‖ ≤ μ·λn
- Box constraint approximation: |λt1| ≤ B, |λt2| ≤ B
- Inscribed square has side length 2B, so B = μ·λn/√2
- See M3-coulomb-cone.md for detailed derivation

**Why setNormalLambda() instead of passing λn to getFrictionBounds()?**
- Solver iterates: normal force λn changes each iteration
- Storing λn as member avoids passing it through multiple call levels
- Matches solver call pattern: update state, then query bounds
- Future extensibility: could cache computed bounds if λn unchanged

**Why not integrate with solver in this ticket?**
- Separation of concerns: constraint definition vs. solver algorithm
- Testability: constraint Jacobian can be validated independently
- Reduced risk: smaller tickets with clear boundaries
- Box-constrained LCP solver (ticket 0035b) is complex enough to warrant separate ticket

## Acceptance Criteria Mapping

The design directly addresses all acceptance criteria from the ticket:

| Acceptance Criterion | Design Element | Validation Method |
|---------------------|----------------|-------------------|
| AC1: TangentBasis orthonormality | `TangentBasis::computeTangentBasis()` algorithm | Unit test: verify ‖t₁‖=1, ‖t₂‖=1, t₁·t₂=0, tᵢ·n=0 for all coordinate axes |
| AC2: TangentBasis determinism | Stateless pure function, Duff et al. algorithm | Unit test: repeated calls produce identical output |
| AC3: TangentBasis continuity | Smooth branch selection, no discontinuous jumps | Unit test: perturb n by ε=1e-4, verify Δt ~ O(ε) |
| AC4: FrictionConstraint Jacobian dimensions | `dimension()=2`, `jacobianTwoBody()` returns 2×12 | Unit test: check matrix dimensions |
| AC5: FrictionConstraint Jacobian correctness | Analytical derivation per M2 | Unit test: finite-difference validation (1e-5 tolerance) |
| AC6: FrictionConstraint friction bounds | `getFrictionBounds()` returns ±μ/√2·λn | Unit test: verify bounds for known μ, λn |
| AC7: Zero regressions | No changes to existing constraints | Run full test suite, verify all pass |

## Implementation Notes

### Build System Changes

**CMakeLists.txt modifications**:
```cmake
# msd-sim/src/Physics/CMakeLists.txt
target_sources(msd_sim PRIVATE
  # ... existing sources ...
  Constraints/FrictionConstraint.cpp
)

# TangentBasis.hpp is header-only, no .cpp to add
```

```cmake
# msd-sim/test/Physics/CMakeLists.txt
target_sources(msd_sim_test PRIVATE
  # ... existing tests ...
  Collision/TangentBasisTest.cpp
  Constraints/FrictionConstraintTest.cpp
)
```

### Header Dependencies

**TangentBasis.hpp**:
- `msd-sim/src/Environment/Coordinate.hpp` — 3D vector operations
- `<cmath>` — std::sqrt for normalization
- `<stdexcept>` — std::invalid_argument for validation
- `<cstdlib>` — std::abs for component magnitude

**FrictionConstraint.hpp**:
- `msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp` — Base class
- `msd-sim/src/Physics/Collision/TangentBasis.hpp` — Tangent basis utility
- `msd-sim/src/Environment/Coordinate.hpp` — 3D vectors
- `msd-sim/src/Physics/RigidBody/InertialState.hpp` — Rigid body state
- `<Eigen/Dense>` — Matrix operations for Jacobian
- `<utility>` — std::pair for friction bounds

### Numerical Considerations

**TangentBasis tolerance**:
- Unit length validation: 1e-6 (same as ContactConstraint normal validation)
- Orthogonality validation in tests: 1e-6 (achievable with double precision)

**FrictionConstraint tolerances**:
- Normal unit length validation: 1e-6 (consistent with ContactConstraint)
- Finite-difference Jacobian validation: 1e-5 (perturbation size 1e-7, expected error ~1e-5)
- Friction activation threshold: λn > 0 (no epsilon needed, exact comparison)

**Stability notes**:
- Duff et al. algorithm avoids division by zero: denominator √(nᵢ² + nⱼ²) is always ≥ √(1 - nₖ²) where nₖ is the smallest component
- For coordinate-aligned normals (e.g., n = [1, 0, 0]), algorithm selects different axis (e.g., eᵧ) to avoid singularity

### Future Integration (Ticket 0035b)

This design prepares for solver integration:

1. **Solver will call** `setNormalLambda(lambda_n)` each iteration before LCP solve
2. **Solver will query** `getFrictionBounds()` to construct box constraint bounds vector
3. **Solver will assemble** 2×12 Jacobian rows from `jacobianTwoBody()` into global constraint matrix
4. **Solver will check** `isActiveTwoBody()` to filter inactive friction constraints (μ=0 or λn=0)

This ticket ensures all constraint data is correct before solver integration.

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-01-31
**Status**: APPROVED
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | Perfect adherence: `TangentBasis` namespace (utility pattern), `TangentFrame` struct (PascalCase), `FrictionConstraint` class (PascalCase), member variables `snake_case_` with trailing underscore, methods `camelCase` (getTangent1, setNormalLambda). Matches existing patterns from `ContactConstraint`, `InertialCalculations`, `CollisionResult`. |
| Namespace organization | ✓ | All components correctly placed in `msd_sim` namespace. TangentBasis utility follows stateless namespace pattern like `ContactConstraintFactory`. No unnecessary nested namespaces. |
| File structure | ✓ | Follows established `msd-sim/src/Physics/` hierarchy: `Collision/TangentBasis.hpp` (utilities alongside `CollisionResult.hpp`), `Constraints/FrictionConstraint.hpp/.cpp` (alongside `ContactConstraint.hpp/.cpp`). Test files mirror source structure: `test/Physics/Collision/TangentBasisTest.cpp`, `test/Physics/Constraints/FrictionConstraintTest.cpp`. |
| Dependency direction | ✓ | Dependencies flow correctly: FrictionConstraint → TwoBodyConstraint → UnilateralConstraint (inheritance), FrictionConstraint → TangentBasis (utility), TangentBasis → Coordinate (primitive). No circular dependencies. All dependencies within msd-sim (no cross-library leakage). |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | TangentFrame: value type with simple members (Coordinate), no manual resource management. FrictionConstraint: compiler-generated destructor sufficient (all members have proper RAII: Coordinate is value type, double is primitive, `= default` explicitly declared). Constructor validates invariants, no cleanup needed. |
| Smart pointer appropriateness | ✓ | No smart pointers used (correct choice). TangentFrame is value type stored by value in FrictionConstraint. All members are value types (Coordinate, double). Follows project standard: "Value semantics for primitives" (CLAUDE.md). Matches ContactConstraint pattern (stores Coordinate by value, not by pointer). |
| Value/reference semantics | ✓ | Deliberate and appropriate choices: TangentFrame is value type (small struct, 48 bytes = 2×Coordinate). FrictionConstraint stores tangents/lever arms by value (pre-computed, immutable). Constructor accepts Coordinate by const reference (avoids copy for large inputs). getTangent1/getTangent2 return const reference (avoids copy, safe because members are immutable). Matches ContactConstraint pattern exactly. |
| Rule of 0/3/5 | ✓ | Both components use Rule of Zero with explicit `= default`: TangentFrame: all special members defaulted (value semantics, compiler-generated correct). FrictionConstraint: explicit `= default` for copy/move constructors and assignment operators, `= default` destructor. Matches project pattern (ContactConstraint, TwoBodyConstraint). No manual implementations that duplicate compiler-generated code. |
| Const correctness | ✓ | Excellent const usage: TangentFrame constructor validates const inputs. FrictionConstraint getters return const Coordinate& (immutable access). evaluateTwoBody/jacobianTwoBody/isActiveTwoBody are const methods (matches TwoBodyConstraint interface). getFrictionBounds is const (reads normal_lambda_ without modification after setNormalLambda call). alpha/beta override return const values. Constructor parameters use const Coordinate& throughout. |
| Exception safety | ✓ | Strong exception safety: TangentFrame constructor validates unit length, throws std::invalid_argument before modification (no partial state). FrictionConstraint constructor validates normal unit length and mu >= 0 before storing members. If construction fails, no object exists (no cleanup needed). All throwing constructors, all other methods noexcept (except setNormalLambda which is simple assignment). Matches ContactConstraint pattern. |
| Initialization | ✓ | Perfect adherence to CLAUDE.md standards: Brace initialization throughout: `normal_lambda_{0.0}`, constructor initializer lists use braces. No use of parentheses for construction. No uninitialized variables (all members initialized in constructor or have default values). Default member initializer for normal_lambda_ follows best practice. Matches existing constraint code patterns. |
| Return values | ✓ | Follows project standard "Prefer returning values/structs over output parameters": computeTangentBasis returns TangentFrame by value (not output parameters). getFrictionBounds returns std::pair by value (not output parameters). evaluateTwoBody returns Eigen::VectorXd by value. jacobianTwoBody returns Eigen::MatrixXd by value. No output parameters via non-const references. Matches ContactConstraint pattern exactly. |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | Clean, minimal, acyclic dependencies. TangentBasis.hpp: only needs Coordinate.hpp (3D vector ops), <cmath> (std::sqrt), <stdexcept> (validation), <cstdlib> (std::abs). Header-only design avoids compilation dependency. FrictionConstraint.hpp: TwoBodyConstraint.hpp (base class), TangentBasis.hpp (utility), Coordinate.hpp (members), InertialState.hpp (interface), <Eigen/Dense> (Jacobian), <utility> (std::pair). All dependencies exist and are in correct modules. No forward declaration needed (all dependencies are value types or standard library). |
| Template complexity | ✓ | No templates used (appropriate choice). TangentBasis could use template for Coordinate-like types but design correctly chooses concrete Coordinate type for simplicity and error messages. No SFINAE, no variadic templates, no complex metaprogramming. Simple, straightforward C++20 code. Easier to debug and maintain than template-heavy alternative. |
| Memory strategy | ✓ | Clear, efficient, safe memory strategy. TangentFrame: 48 bytes stack allocation (2× Coordinate = 2×24 bytes), no heap allocation, trivially copyable. FrictionConstraint: ~144 bytes stack allocation (3× Coordinate = 72 bytes + 3× double = 24 bytes + base class overhead), all members stored inline, no heap allocation in normal path. Eigen::VectorXd/MatrixXd returns allocate on heap (unavoidable, acceptable for temporary return values). Matches ContactConstraint pattern. No memory leaks possible (all RAII). |
| Thread safety | ✓ | Clearly documented and appropriate. TangentBasis::computeTangentBasis: stateless pure function, safe for concurrent calls with different inputs. TangentFrame: value type, safe to copy across threads after construction. FrictionConstraint: immutable after construction except setNormalLambda, documented as not thread-safe (matches solver usage pattern - single-threaded constraint solving). Const methods (evaluateTwoBody, jacobianTwoBody, getFrictionBounds) safe for concurrent reads after setNormalLambda. Matches project thread safety conventions (msd-sim CLAUDE.md: "Container types: Not thread-safe, single-threaded simulation assumed"). |
| Build integration | ✓ | Straightforward, low-risk build changes. CMakeLists.txt changes are additive only: add FrictionConstraint.cpp to msd_sim sources, add TangentBasisTest.cpp and FrictionConstraintTest.cpp to msd_sim_test sources. TangentBasis.hpp is header-only (no .cpp to add). No changes to existing source files, no changes to dependencies, no new third-party libraries. Follows exact pattern from ContactConstraint addition (ticket 0032a). Zero regression risk. |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | Both components can be instantiated and tested in complete isolation. TangentBasis: stateless utility, can call computeTangentBasis with arbitrary Coordinate inputs, no dependencies on simulation state. TangentFrame: simple value type, construct with two Coordinates. FrictionConstraint: can construct with mock body indices, arbitrary contact geometry, no dependencies on AssetPhysical or WorldModel, only needs InertialState for evaluation (easy to construct). Excellent testability design. |
| Mockable dependencies | ✓ | No dependencies requiring mocking. TangentBasis: depends only on Coordinate (value type, no need to mock). FrictionConstraint: depends on Coordinate (value type), InertialState (simple struct, easy to construct with test data), TwoBodyConstraint (abstract base, but no virtual calls to mock). All dependencies are data structures, not services requiring injection. Test-friendly design. |
| Observable state | ✓ | All state fully observable via public accessors. TangentFrame: public members t1, t2 (direct access). FrictionConstraint: getTangent1(), getTangent2() (tangent basis), getLeverArmA(), getLeverArmB() (lever arms), getFrictionCoefficient() (mu), getContactNormal() (normal), dimension() (always 2), evaluateTwoBody() (constraint violation), jacobianTwoBody() (constraint Jacobian), getFrictionBounds() (friction limits), isActiveTwoBody() (activation state). Complete state introspection for verification. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Numerical precision loss in tangent basis computation for nearly-aligned normals (e.g., n ≈ ±e_x with small perturbations) could produce tangents with ||t|| < 1 - ε | Performance | Low | Low | Duff et al. algorithm selects axis with smallest |n_i|, ensuring denominator √(n_j² + n_k²) ≥ √(1 - max|n_i|²) stays bounded away from zero. Unit tests validate orthonormality to 1e-6 tolerance for coordinate-aligned cases. If precision loss detected in practice, can add explicit re-normalization after cross product. | No |
| R2 | Friction bounds √2 factor could be non-intuitive to users expecting direct μ·λn bound | Maintenance | Low | Low | Design rationale section clearly documents inscribed square approximation with reference to M3-coulomb-cone.md. Comment in getFrictionBounds() explains √2 factor. Math formulation provides derivation. Well-documented design reduces confusion. | No |
| R3 | FrictionConstraint not yet integrated into solver - dead code until ticket 0035b | Integration | High | Low | Expected and intentional per ticket scope. Design document clearly states "Not yet integrated into solver" in multiple locations. "Future Integration" section documents how ticket 0035b will use the constraint. Low impact because this ticket's purpose is to validate constraint correctness in isolation before solver complexity. | No |
| R4 | setNormalLambda() mutates state, breaking immutability assumption for other const methods, potential race condition if called concurrently with getFrictionBounds() | Technical | Low | Medium | Documented as not thread-safe (matches solver pattern - single-threaded). Solver will call setNormalLambda() sequentially before querying bounds (documented in "Future Integration"). Alternative would be passing λn to getFrictionBounds(), but design rationale explains this would complicate solver call pattern. Low risk in practice because solver is single-threaded per msd-sim CLAUDE.md. | No |

### Summary

This is an **exemplary design** that demonstrates excellent understanding of the existing codebase architecture and C++ best practices. The design achieves perfect consistency with established patterns from `ContactConstraint` and `TwoBodyConstraint`, follows all project coding standards from CLAUDE.md, and provides comprehensive implementation guidance.

**Strengths**:
1. **Architectural consistency**: Mirrors `ContactConstraint` pattern exactly (pre-computed geometry, 1×12 Jacobian rows, TwoBodyConstraint extension, dimension() override, activation logic)
2. **C++ quality**: Proper use of value semantics, const correctness, Rule of Zero with explicit defaults, RAII, no raw pointers
3. **Testability**: Both components designed for isolated unit testing with comprehensive test plans (8 + 15 = 23 test cases covering all acceptance criteria)
4. **Documentation**: Exceptional level of detail in rationale, implementation notes, numerical considerations, future integration guidance
5. **Scope discipline**: Correctly defers solver integration to ticket 0035b, reducing risk and enabling focused validation

**Minor observations** (not requiring revision):
- R4 (setNormalLambda mutability) is acceptable given single-threaded solver context and clear documentation
- R3 (not yet integrated) is expected and correctly scoped
- All risks have low-medium impact with clear mitigations

**No blocking issues, no prototype required, no revisions requested**. The design is ready for implementation. Estimated implementation time: 6-8 hours (4 hours implementation, 2-3 hours comprehensive testing, 1 hour documentation).

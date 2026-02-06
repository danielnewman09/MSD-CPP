# Design: DataType Simplification

## Summary

This design addresses the accumulated code quality issues and semantic misuse in the `Coordinate`, `CoordinateRate`, `AngularCoordinate`, and `AngularRate` types introduced in ticket 0024. The solution is a phased refactoring that (A) fixes mechanical code quality issues, (B) consolidates duplicated formatters, (C) replaces semantically incorrect type usage with `msd_sim::Vector3D` for generic vectors/normals/forces, and (D) simplifies the Eigen expression constructor to remove explicit wrapping friction. This approach preserves the validated design decisions from ticket 0024 while eliminating the practical issues that have emerged through use.

## Architecture Changes

### PlantUML Diagram
See: `./0037_datatype_simplification.puml`

### Modified Components

#### Vec3Base CRTP Template (Phase D)
- **Current location**: `msd/msd-sim/src/DataTypes/Coordinate.hpp` (lines 12-44)
- **Changes required**:
  1. Make Eigen template constructors implicit (remove `explicit` keyword)
  2. Add `// NOLINT(google-explicit-constructor)` suppression on template constructors
  3. Ensure `using Base::operator=` is exposed for derived types
  4. Add `final` specifier to derived types (Coordinate, AngularCoordinate, AngularRate) to prevent base-pointer deletion UB
- **Backward compatibility**: Source-level compatible — implicit constructors are strictly more permissive than explicit ones

```cpp
// Current (explicit template ctor)
template <typename OtherDerived>
explicit Vec3Base(const Eigen::MatrixBase<OtherDerived>& other)
  : msd_sim::Vector3D{other}
{
}

// Modified (implicit template ctor with NOLINT)
template <typename OtherDerived>
Vec3Base(const Eigen::MatrixBase<OtherDerived>& other)  // NOLINT(google-explicit-constructor)
  : msd_sim::Vector3D{other}
{
}
```

#### Coordinate (Phase C + Phase D)
- **Current location**: `msd/msd-sim/src/DataTypes/Coordinate.hpp` (lines 49-59)
- **Phase C changes**: None to class definition — usage changes in consuming files only
- **Phase D changes**:
  1. Inherit implicit template constructor from Vec3Base
  2. Add `final` specifier
- **Backward compatibility**: 100% compatible — no signature changes

```cpp
// Modified (Phase D)
struct Coordinate final : detail::Vec3Base<Coordinate>
{
  using Vec3Base::Vec3Base;
  using Vec3Base::operator=;
};
```

#### CoordinateRate (Removed in Phase C)
- **Current location**: `msd/msd-sim/src/DataTypes/Coordinate.hpp` (lines 61-71)
- **Changes required**: Remove entire type definition
- **Migration strategy**: Replace all usages with `msd_sim::Vector3D` where semantically vectors/normals/forces
- **Breaking change**: Yes — all usages must be updated

**Rationale**: CoordinateRate was introduced for velocities/accelerations, but in practice is misused for:
- Contact normals (CollisionResult.hpp, ContactConstraint.hpp, FrictionConstraint.hpp)
- Facet normals (Facet.hpp)
- GJK search directions (GJK.hpp)
- Forces/torques (SemiImplicitEulerIntegrator.hpp, AssetInertial.hpp)

These quantities are not velocities/accelerations. Using raw `msd_sim::Vector3D` is more honest and avoids semantic confusion.

#### AngularCoordinate (Phase A + Phase D)
- **Current location**: `msd/msd-sim/src/DataTypes/AngularCoordinate.hpp`
- **Phase A changes** (code quality cleanup):
  1. Fix `kKNormalizationThreshold` constant name (line 45): `kKNormalizationThreshold` → `kNormalizationThreshold`
  2. Remove triple `[[nodiscard]]` from accessor methods (lines 108, 113, 118, 124, 129, 134, 168)
  3. Remove nested braces in `normalizeIfNeeded()` (lines 195-199, 202-206, 209-213)
  4. Remove triple parentheses in formatter `parse()` (lines 265, 277)
  5. Remove nested braces in formatter `format()` (lines 299-304)
- **Phase D changes**:
  1. Make Eigen template constructor implicit with `// NOLINT(google-explicit-constructor)`
  2. Make template assignment operator implicit with `// NOLINT(google-explicit-constructor)`
  3. Make compound operators (+=, -=, *=, /=) implicit template methods with `// NOLINT`
  4. Add `final` specifier
- **Backward compatibility**: Phase A is 100% compatible (no API changes). Phase D is source-level compatible (more permissive constructors).

```cpp
// Phase A: Fix constant name
- static constexpr double kKNormalizationThreshold = 100.0 * M_PI;
+ static constexpr double kNormalizationThreshold = 100.0 * M_PI;

// Phase A: Remove triple [[nodiscard]]
- [[nodiscard]] [[nodiscard]] [[nodiscard]] double pitch() const
+ [[nodiscard]] double pitch() const

// Phase A: Remove nested braces
void normalizeIfNeeded()
{
  if (std::abs((*this)[0]) > kNormalizationThreshold)
  {
-   {
-     {
        (*this)[0] = normalizeAngle((*this)[0]);
-     }
-   }
  }
  // ... repeat for [1], [2]
}

// Phase A: Remove triple parentheses in formatter
- width = (((width * 10))) + (*it - '0');
+ width = width * 10 + (*it - '0');

// Phase D: Make template ctor implicit
template <typename OtherDerived>
- explicit AngularCoordinate(const Eigen::MatrixBase<OtherDerived>& other)
+ AngularCoordinate(const Eigen::MatrixBase<OtherDerived>& other)  // NOLINT(google-explicit-constructor)
  : msd_sim::Vector3D{other}
{
  normalizeIfNeeded();
}

// Phase D: Add final specifier
- class AngularCoordinate : public msd_sim::Vector3D
+ class AngularCoordinate final : public msd_sim::Vector3D
```

#### AngularRate (Phase A + Phase D)
- **Current location**: `msd/msd-sim/src/DataTypes/AngularRate.hpp`
- **Phase A changes** (code quality cleanup):
  1. Remove triple `[[nodiscard]]` from accessor methods (lines 84, 89, 94)
  2. Remove nested braces in formatter `parse()` (lines 128-132)
  3. Remove triple parentheses in formatter `parse()` (lines 141, 153)
  4. Remove nested braces in formatter `format()` (lines 174-179)
- **Phase D changes**:
  1. Make Eigen template constructor implicit with `// NOLINT(google-explicit-constructor)`
  2. Make template assignment operator implicit with `// NOLINT(google-explicit-constructor)`
  3. Add `final` specifier
- **Backward compatibility**: Phase A is 100% compatible. Phase D is source-level compatible.

```cpp
// Phase A: Remove triple [[nodiscard]]
- [[nodiscard]] [[nodiscard]] [[nodiscard]] double pitch() const
+ [[nodiscard]] double pitch() const

// Phase A: Remove nested braces and triple parentheses (same patterns as AngularCoordinate)

// Phase D: Make template ctor implicit
template <typename OtherDerived>
- explicit AngularRate(const Eigen::MatrixBase<OtherDerived>& other)
+ AngularRate(const Eigen::MatrixBase<OtherDerived>& other)  // NOLINT(google-explicit-constructor)
  : msd_sim::Vector3D{other}
{
}

// Phase D: Add final specifier
- class AngularRate : public msd_sim::Vector3D
+ class AngularRate final : public msd_sim::Vector3D
```

#### std::formatter Specializations (Phase B)
- **Current location**: Coordinate.hpp (lines 76-156), AngularCoordinate.hpp (lines 234-324), AngularRate.hpp (lines 109-199)
- **Changes required**:
  1. Extract shared `parse()` and component formatting logic into `Vec3FormatterBase<T, Accessor>` template
  2. Specialize `std::formatter<Coordinate>`, `std::formatter<AngularCoordinate>`, `std::formatter<AngularRate>` as thin wrappers that inherit from `Vec3FormatterBase`
  3. Parameterize on accessor function to handle different component names (x/y/z vs pitch/roll/yaw)
- **Backward compatibility**: 100% compatible — no changes to `std::format` API

**Design approach**: Template base with function template accessor (simplified per review feedback — Accessor removed from class template parameters to avoid `decltype([...])` portability concerns)

```cpp
// New: Vec3FormatterBase template (in detail namespace)
namespace msd_sim::detail {

template <typename T>
struct Vec3FormatterBase {
  char presentation = 'f';
  int precision = 6;
  int width = 0;

  constexpr auto parse(std::format_parse_context& ctx) {
    // Shared parse logic (currently duplicated 3×)
    // Parse [width][.precision][type]
    // ... implementation ...
  }

protected:
  std::string buildComponentFormat() const {
    std::string componentFmt = "{:";
    if (width > 0)
      componentFmt += std::to_string(width);
    componentFmt += '.';
    componentFmt += std::to_string(precision);
    componentFmt += presentation;
    componentFmt += '}';
    return componentFmt;
  }

  template <typename Context, typename F>
  auto formatComponents(const T& vec, F&& accessor, Context& ctx) const {
    auto [c0, c1, c2] = accessor(vec);
    const std::string componentFmt = buildComponentFormat();
    return std::format_to(
      ctx.out(),
      "({}, {}, {})",
      std::vformat(componentFmt, std::make_format_args(c0)),
      std::vformat(componentFmt, std::make_format_args(c1)),
      std::vformat(componentFmt, std::make_format_args(c2)));
  }
};

}  // namespace msd_sim::detail

// Specialization for Coordinate
template <>
struct std::formatter<msd_sim::Coordinate>
  : msd_sim::detail::Vec3FormatterBase<msd_sim::Coordinate>
{
  auto format(const msd_sim::Coordinate& coord, std::format_context& ctx) const {
    return formatComponents(coord,
      [](const auto& c) { return std::tuple{c.x(), c.y(), c.z()}; },
      ctx);
  }
};

// Specialization for AngularCoordinate
template <>
struct std::formatter<msd_sim::AngularCoordinate>
  : msd_sim::detail::Vec3FormatterBase<msd_sim::AngularCoordinate>
{
  auto format(const msd_sim::AngularCoordinate& coord, std::format_context& ctx) const {
    return formatComponents(coord,
      [](const auto& c) { return std::tuple{c.pitch(), c.roll(), c.yaw()}; },
      ctx);
  }
};

// Specialization for AngularRate (same pattern)
```

**Lines of code reduction**: ~120 lines → ~60 lines (50% reduction in formatter duplication)

### Integration Points

| Modified Component | Affected Files | Integration Type | Notes |
|--------------------|----------------|------------------|-------|
| Vec3Base | Coordinate.hpp | Internal refactoring | Implicit template ctor, NOLINT annotations |
| Coordinate | Coordinate.hpp | API preservation | Add final specifier, inherit implicit ctor |
| CoordinateRate | **REMOVED** | **Breaking change** | Replace with msd_sim::Vector3D at all call sites |
| AngularCoordinate | AngularCoordinate.hpp | Code quality + API change | Fix artifacts, implicit ctor, final specifier |
| AngularRate | AngularRate.hpp | Code quality + API change | Fix artifacts, implicit ctor, final specifier |
| Formatters | All three headers | Internal consolidation | Extract to Vec3FormatterBase, zero API impact |

### Phase C: Semantic Type Migration (Detailed Breakdown)

This phase requires manual review of each usage to determine semantic intent. The following table catalogs all identified misuses and their replacements:

| File | Line(s) | Current Type | Semantic Quantity | Replacement | Rationale |
|------|---------|--------------|-------------------|-------------|-----------|
| `InertialState.hpp` | 40 | `Coordinate` | Linear velocity | `msd_sim::Vector3D` | Velocity is not a position |
| `InertialState.hpp` | 41 | `Coordinate` | Linear acceleration | `msd_sim::Vector3D` | Acceleration is not a position |
| `Facet.hpp` | 30 | `CoordinateRate` | Facet normal | `msd_sim::Vector3D` | Unit normal is a direction, not a velocity |
| `CollisionResult.hpp` | 82 | `CoordinateRate` | Contact normal | `msd_sim::Vector3D` | Unit normal is a direction, not a velocity |
| `ContactConstraint.hpp` | ~50 | `CoordinateRate` | Contact normal | `msd_sim::Vector3D` | Unit normal is a direction, not a velocity |
| `ContactConstraint.hpp` | ~52-53 | `Coordinate` | Lever arms | `Coordinate` | **Keep as-is** — lever arms are position offsets |
| `FrictionConstraint.hpp` | ~45-46 | `Coordinate` | Tangent basis | `msd_sim::Vector3D` | Tangent vectors are directions, not positions |
| `TangentBasis.hpp` | 38-39 | `Coordinate` | Tangent frame | `msd_sim::Vector3D` | Tangent vectors are directions, not positions |
| `GJK.hpp` | Various | `CoordinateRate` | Search direction | `msd_sim::Vector3D` | Search direction is a vector, not a velocity |
| `EPA.cpp` | Various | `Coordinate` | Face normals | `msd_sim::Vector3D` | Face normals are directions, not positions |

**Note**: The `InertialState` migration has the broadest downstream impact (~50+ files reference `InertialState::velocity` or `InertialState::acceleration`). The migration is mechanically simple (`.x()`, `.y()`, `.z()` accessors are identical on both types) but the scope must not be underestimated.

**Key principle**: If the quantity is:
- A **position** or **displacement** → `Coordinate`
- A **velocity** or **linear acceleration** → Remove `CoordinateRate`, use `msd_sim::Vector3D` (velocities stored in InertialState)
- A **direction**, **normal**, or **force** → `msd_sim::Vector3D` (not position, not velocity)
- An **orientation** → `AngularCoordinate` (Euler angles with normalization)
- An **angular velocity** or **angular acceleration** → `AngularRate` (no normalization)

**Verification strategy**: For each file modified in Phase C:
1. Read surrounding code to confirm semantic intent
2. Check function signatures for ADL/template specialization dependencies
3. Verify no implicit conversions break (e.g., Coordinate → msd_sim::Vector3D is safe due to inheritance)
4. Run full test suite after each file migration

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `AngularCoordinateTest.cpp` | All `[[nodiscard]]` usage | None | Tests unchanged, attribute removal is internal |
| `AngularRateTest.cpp` | All `[[nodiscard]]` usage | None | Tests unchanged, attribute removal is internal |
| `CoordinateTest.cpp` | Explicit ctor tests | **Breaking** | Remove tests verifying explicit constructor behavior (Phase D) |
| `ContactConstraintFactoryTest.cpp` | Normal vector construction | **Minor** | Update type expectations `CoordinateRate` → `msd_sim::Vector3D` |
| `FrictionConstraintTest.cpp` | Tangent basis construction | **Minor** | Update type expectations `Coordinate` → `msd_sim::Vector3D` |
| `TangentBasisTest.cpp` | Tangent frame validation | **Minor** | Update type expectations `Coordinate` → `msd_sim::Vector3D` |
| `CollisionHandlerTest.cpp` | CollisionResult normal checks | **Minor** | Update type expectations `CoordinateRate` → `msd_sim::Vector3D` |
| `EPATest.cpp` | Contact normal assertions | **Minor** | Update type expectations `CoordinateRate` → `msd_sim::Vector3D` |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| Vec3FormatterBase | Template instantiation | Formatter compiles for all three types |
| Vec3FormatterBase | Accessor parameterization | x/y/z vs pitch/roll/yaw correctly extracted |
| Vec3FormatterBase | Format spec parsing | Shared parse logic handles all format specs |
| Coordinate | Implicit Eigen ctor | Eigen expression assigns without explicit cast |
| AngularCoordinate | Implicit Eigen ctor | Eigen expression assigns without explicit cast |
| AngularCoordinate | Constant name | `kNormalizationThreshold` accessible (not `kKNormalizationThreshold`) |
| AngularCoordinate | Normalization threshold | Deferred normalization still triggers at 100π |
| AngularRate | Implicit Eigen ctor | Eigen expression assigns without explicit cast |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Collision normal type | CollisionHandler, CollisionResult | msd_sim::Vector3D normal in CollisionResult |
| Contact constraint normal | ContactConstraintFactory, ContactConstraint | msd_sim::Vector3D normal passed to constraint |
| Friction tangents | FrictionConstraint, TangentBasis | msd_sim::Vector3D tangents in friction constraint |
| Facet normal usage | ConvexHull, Facet | msd_sim::Vector3D facet normal in hull |
| GJK search direction | GJK algorithm | msd_sim::Vector3D search direction |

#### Regression Tests

| Test Case | What It Validates |
|-----------|-------------------|
| Full physics simulation | All types interoperate correctly after changes |
| Collision response | Contact forces and torques compute correctly with new types |
| Constraint solver | Constraints evaluate correctly with updated Jacobians |
| Angular dynamics | AngularCoordinate normalization behavior unchanged |
| **Numerical equivalence** | Deterministic simulation trajectory matches pre-refactoring output within epsilon tolerance |

#### Phase C Verification Protocol (per review feedback)

1. **Per-file compilation gate**: After each Phase C file migration, full build must pass before proceeding to next file
2. **Exhaustive grep verification**: After Phase C completion, `grep -r "CoordinateRate" msd/` must return zero hits (excluding comments/docs)
3. **Numerical equivalence test**: Run a deterministic simulation scenario before and after Phase C; compare trajectories with epsilon tolerance to catch subtle numerical changes from type conversion paths

## Open Questions

### Design Decisions (Human Input Needed)

1. **CoordinateRate Removal Scope**
   - Option A: Remove CoordinateRate entirely in Phase C — Pros: Eliminates confusion, honest about semantics. Cons: Requires updating ~50+ files.
   - Option B: Keep CoordinateRate as alias to msd_sim::Vector3D — Pros: Minimal breaking changes. Cons: Perpetuates semantic ambiguity.
   - **Recommendation**: Option A. The semantic confusion is the root problem. An alias would preserve the confusion while adding no value.

2. **Vec3FormatterBase Location**
   - Option A: Place in `msd_sim::detail` namespace (internal) — Pros: Signals implementation detail, not public API. Cons: None.
   - Option B: Place in `msd_sim::formatting` namespace (public) — Pros: Could be reused for other vector types. Cons: Exposes formatter implementation.
   - **Recommendation**: Option A. Formatter consolidation is an internal refactoring, not a public formatting utility library.

3. **Triple `[[nodiscard]]` Root Cause**
   - **Investigation required**: Determine if this is from macro expansion, automated tool, or manual error
   - **Blocking**: No — can fix symptoms in Phase A, investigate root cause in parallel
   - **Action**: Search codebase for macro/tool that might generate triple attributes

4. **Phase Execution Order**
   - **Confirmed order**: A → B → C → D (per ticket recommendation)
   - **Rationale**: Clean up artifacts (A) → consolidate formatters (B) → semantic cleanup removes most usages (C) → simplify remaining constructors (D)
   - **No human input needed** — order is technically optimal

5. **Backward Compatibility Strategy**
   - Option A: Breaking change in single release — Pros: Clean break, no legacy code. Cons: Requires updating all downstream code at once.
   - Option B: Deprecation period with compatibility aliases — Pros: Gradual migration. Cons: Maintains semantic confusion during transition.
   - **Recommendation**: Option A. This is an internal project with no external consumers. Clean break is simpler.

6. **Final Specifier Placement**
   - **Confirmed**: Add `final` to Coordinate, AngularCoordinate, AngularRate (per constraint from ticket)
   - **Rationale**: msd_sim::Vector3D has non-virtual destructor, deleting through base pointer is UB
   - **No human input needed** — required by C++ standard to prevent UB

### Prototype Required

None. The design consists of:
- **Phase A**: Mechanical text replacements (no algorithm changes)
- **Phase B**: Template refactoring with existing logic (no new algorithms)
- **Phase C**: Type replacements (msd_sim::Vector3D already in use, just changing where)
- **Phase D**: Constructor simplification (removing explicit keyword, validated pattern)

All changes are deterministic refactorings of existing code. No performance characteristics to validate.

### Requirements Clarification

1. **Scope of Phase C Migration**: Should Phase C update *all* files using CoordinateRate/Coordinate for non-semantic purposes, or only the files identified in the ticket?
   - **Current scope**: Ticket identifies 7 key files (ContactConstraint, FrictionConstraint, CollisionResult, Facet, TangentBasis, GJK, EPA)
   - **Recommended**: Perform exhaustive grep for all CoordinateRate usages, manually review each, update all semantically incorrect usages
   - **Rationale**: Partial migration leaves future confusion. Better to fix comprehensively.

2. **Clang-tidy Suppression Strategy**: Should NOLINT annotations be per-line or per-file?
   - **Current recommendation**: Per-line (scoped suppression)
   - **Alternative**: Per-file with `// NOLINTBEGIN(google-explicit-constructor)` / `// NOLINTEND`
   - **Recommendation**: Per-line for precision. Only suppress where needed (template constructors).

3. **Test Migration Strategy**: Should existing tests for explicit constructor behavior be removed or updated?
   - **Current recommendation**: Remove tests that verify explicit-only behavior (Phase D makes them invalid)
   - **Alternative**: Update tests to verify both explicit and implicit construction work
   - **Recommendation**: Remove. Testing implicit conversion is not valuable for mathematical types.

## Design Complexity Sanity Check

### Evaluation

**Red Flag 1: Combinatorial Overloads** — Not applicable. No function overload explosion.

**Red Flag 2: Optional Wrappers for Legacy Paths** — Not applicable. No optional wrappers introduced.

**Red Flag 3: Modified Components Outnumber New Components** — **TRIGGERED**. This design modifies 6 existing components (Vec3Base, Coordinate, AngularCoordinate, AngularRate, CoordinateRate removal, formatters) and adds only 1 new component (Vec3FormatterBase).

**Assessment**: This is a **refactoring ticket**, not a feature addition. The modified-vs-new ratio is expected and appropriate. The design is simplifying existing code, not preserving backward compatibility at the cost of complexity. The "modified" components are getting simpler (fewer lines, less duplication, clearer semantics), which is the correct direction for a cleanup ticket.

**Red Flag 4: Conditional Logic Explosion** — Not applicable. No new conditionals introduced.

**Conclusion**: Design complexity is acceptable for a refactoring ticket. The ratio of modified-to-new components reflects the nature of cleanup work (fixing existing code rather than adding new code). No human consultation needed.

## Code Quality Gates Awareness

### Build Quality Requirements

**Warnings as Errors**: All phases must maintain zero-warning compilation.
- **Phase A**: Text replacements do not affect compilation warnings
- **Phase B**: Template consolidation must compile cleanly
- **Phase C**: Type replacements must not introduce implicit conversion warnings
- **Phase D**: NOLINT annotations explicitly suppress google-explicit-constructor warnings

**Static Analysis**: clang-tidy checks will be satisfied by NOLINT annotations in Phase D.
- Target check: `google-explicit-constructor`
- Suppression strategy: Per-line `// NOLINT(google-explicit-constructor)` on template constructors
- Validation: Run clang-tidy on modified headers after Phase D

**Design for const-correctness**: All phases preserve existing const-correctness.
- Coordinate/AngularCoordinate/AngularRate already have const accessors
- Phase C type replacements do not affect const propagation (msd_sim::Vector3D has const methods)
- No new mutable state introduced

### Performance Considerations

**Benchmark Regression Detection**: Phase C (type replacement) is the only phase with potential performance impact.

**Performance-critical paths**:
1. Collision detection (GJK/EPA) — Uses normal vectors, lever arms
2. Contact constraint evaluation — Uses normals, tangent basis
3. Friction constraint evaluation — Uses tangent vectors
4. Physics integration — Uses forces, torques

**Expected performance impact**: **Zero**. msd_sim::Vector3D vs CoordinateRate are identical at machine code level (both inherit from msd_sim::Vector3D). Phase C is changing compile-time types only, not runtime behavior.

**Benchmark requirements**: None. No algorithmic changes, no new operations, no performance-sensitive refactoring.

**If performance tests exist**: Run existing collision/constraint benchmarks after Phase C to verify zero regression.

### Test Infrastructure Requirements

**Testability**: Phase B (formatter consolidation) should be independently testable.
- Extract Vec3FormatterBase to separate header for unit testing
- Add formatter unit tests that verify template parameterization
- Test all three specializations (Coordinate, AngularCoordinate, AngularRate) format identically to current implementation

**Test isolation**: All phases preserve test isolation.
- Mathematical types are value types (no global state)
- Formatters are stateless (no shared mutable state)
- No new dependencies between tests

**Test fixtures**: No new fixtures required.
- Existing Coordinate/AngularCoordinate/AngularRate tests reuse as-is (with minor updates)
- Formatter tests can use existing test infrastructure

## Constraints Acknowledged

- ✓ AngularCoordinate normalization behavior must not regress (validated by 0024 prototypes) — **Preserved in Phase A + Phase D**
- ✓ Vec3Base CRTP pattern is a good foundation — **Extended in Phase D, not discarded**
- ✓ Angular types must remain separate (AngularCoordinate vs AngularRate) — **Preserved, no consolidation**
- ✓ InertialState quaternion representation is settled (from 0030) — **Not re-opened**
- ✓ **Do not use composition for Eigen types** — **Design uses inheritance throughout**
- ✓ **Known normalization leak**: Direct `operator[]`, `x()`, `y()`, `z()` access bypasses deferred normalization — **Accepted limitation, not addressed**
- ✓ Eigen inheritance is the standard extension pattern — **Confirmed, using inheritance + `using Base::operator=`**
- ✓ Composition would require forwarding dozens of methods — **Avoided, using inheritance**

## Notes for Implementer

### Phase A: Code Quality Cleanup

**Files to modify**:
1. `msd/msd-sim/src/DataTypes/AngularCoordinate.hpp`
2. `msd/msd-sim/src/DataTypes/AngularRate.hpp`

**Changes**:
1. Global search-replace: `kKNormalizationThreshold` → `kNormalizationThreshold` (2 occurrences in AngularCoordinate.hpp)
2. Remove duplicate `[[nodiscard]]` attributes (13 files, 73 instances total per ticket analysis)
3. Remove nested braces in `normalizeIfNeeded()` (AngularCoordinate.hpp lines 195-213)
4. Remove triple parentheses in formatters (AngularCoordinate.hpp lines 265, 277; AngularRate.hpp lines 141, 153)
5. Remove nested braces in formatter `format()` methods (both files)

**Validation**:
- Compile with `-Wall -Wextra -Wpedantic -Werror`
- Run existing tests (no test changes required)
- Verify `kNormalizationThreshold` is accessible (not `kKNormalizationThreshold`)

### Phase B: Formatter Consolidation

**Files to modify**:
1. `msd/msd-sim/src/DataTypes/Coordinate.hpp` — Extract formatter to use Vec3FormatterBase
2. `msd/msd-sim/src/DataTypes/AngularCoordinate.hpp` — Extract formatter to use Vec3FormatterBase
3. `msd/msd-sim/src/DataTypes/AngularRate.hpp` — Extract formatter to use Vec3FormatterBase
4. **New file**: `msd/msd-sim/src/DataTypes/Vec3FormatterBase.hpp` — Template base class

**Changes**:
1. Create `Vec3FormatterBase<T>` template in detail namespace
2. Move shared `parse()` logic to base class
3. Move `buildComponentFormat()` to protected base method
4. Move `formatComponents()` to protected base template method
5. Specialize `std::formatter<Coordinate>` as thin wrapper
6. Specialize `std::formatter<AngularCoordinate>` as thin wrapper
7. Specialize `std::formatter<AngularRate>` as thin wrapper

**Validation**:
- Compile all three headers
- Run existing formatter tests (no changes required)
- Verify `std::format("{:.2f}", coord)` produces identical output before/after

### Phase C: Semantic Type Cleanup

**Files to modify** (exhaustive list via grep):
1. `msd/msd-sim/src/Physics/RigidBody/InertialState.hpp` — `Coordinate velocity` → `msd_sim::Vector3D velocity`, `Coordinate acceleration` → `msd_sim::Vector3D acceleration` (broadest downstream impact, ~50+ consumer files)
2. `msd/msd-sim/src/DataTypes/Facet.hpp` — Line 30: `CoordinateRate normal` → `msd_sim::Vector3D normal`
3. `msd/msd-sim/src/Physics/Collision/CollisionResult.hpp` — Line 82: `CoordinateRate normal` → `msd_sim::Vector3D normal`
4. `msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp` — `CoordinateRate contactNormal_` → `msd_sim::Vector3D contactNormal_`
5. `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.hpp` — `Coordinate tangent1_/tangent2_` → `msd_sim::Vector3D tangent1_/tangent2_`
6. `msd/msd-sim/src/Physics/Collision/TangentBasis.hpp` — `Coordinate t1/t2` → `msd_sim::Vector3D t1/t2`
7. `msd/msd-sim/src/Physics/Collision/GJK.hpp` — Search direction usages
8. `msd/msd-sim/src/Physics/Collision/EPA.cpp` — Face normal usages
9. **All downstream files** referencing `InertialState::velocity` or `InertialState::acceleration` — mechanical migration (accessors identical)
10. **Test files**: Update type expectations in all affected tests

**Migration strategy**:
1. For each file, run `grep -n "CoordinateRate\|Coordinate" <file>` to identify all usages
2. Manually review each usage context to determine semantic intent
3. Replace with `msd_sim::Vector3D` if quantity is direction/normal/force
4. **Keep as Coordinate** if quantity is truly a position/displacement (e.g., lever arms)
5. Update function signatures if parameter types change
6. Check for ADL or template specialization dependencies (unlikely for these files)
7. Compile and run tests after each file

**Validation**:
- All tests pass after each file migration
- No compiler warnings about implicit conversions
- grep confirms zero remaining `CoordinateRate` usages in codebase

### Phase D: Explicit Constructor Simplification

**Files to modify**:
1. `msd/msd-sim/src/DataTypes/Coordinate.hpp` — Vec3Base template ctor, Coordinate final specifier
2. `msd/msd-sim/src/DataTypes/AngularCoordinate.hpp` — Template ctor, final specifier
3. `msd/msd-sim/src/DataTypes/AngularRate.hpp` — Template ctor, final specifier

**Changes**:
1. Remove `explicit` keyword from `Vec3Base` template constructor
2. Add `// NOLINT(google-explicit-constructor)` comment on same line
3. Remove `explicit` keyword from `AngularCoordinate` template constructor and assignment
4. Add `// NOLINT(google-explicit-constructor)` on each
5. Remove `explicit` keyword from `AngularRate` template constructor and assignment
6. Add `// NOLINT(google-explicit-constructor)` on each
7. Add `final` specifier to `Coordinate`, `AngularCoordinate`, `AngularRate` class definitions
8. Ensure `using Base::operator=` is present in all derived types

**Validation**:
- Compile with clang-tidy enabled
- Verify `google-explicit-constructor` warnings are suppressed
- Run all tests
- Verify Eigen expressions assign without explicit wrapping:
  ```cpp
  Coordinate result = rotationMatrix * someVector;  // Should compile without explicit ctor
  AngularRate omega = crossProduct(r, v);  // Should compile without explicit ctor
  ```

### Final Integration Test

After all four phases:
1. Full clean build with `-Werror`
2. Run complete test suite (all 500+ tests)
3. Run clang-tidy on all modified headers
4. Verify zero regressions in physics simulation
5. Verify `std::format` output unchanged for all three types

---

## Design Review (Gemini Pro)

**Reviewer**: Gemini Pro (external AI review)
**Date**: 2026-02-04
**Verdict**: CHANGES REQUESTED → Issues triaged below, actionable items incorporated

### Issue Triage

#### BLOCKER-1: Eigen Memory Alignment (DISMISSED — False Positive)

**Gemini's concern**: Replacing `CoordinateRate` with `msd_sim::Vector3D` in struct members could cause alignment-related segfaults if containers don't use `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`.

**Verdict**: **Not applicable.** `msd_sim::Vector3D` is 24 bytes (3 × 8-byte doubles), which is NOT a multiple of 16 bytes. Eigen's special alignment requirements only apply to "fixed-size vectorizable" types (e.g., `Vector4d` at 32 bytes, `Vector2d` at 16 bytes). `Vector3d` uses natural 8-byte alignment. Confirmed:
- No `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` usage in codebase (not needed)
- No `EIGEN_MAX_ALIGN_BYTES` configuration (default is correct)
- All affected structs (`Facet`, `CollisionResult`, `ContactConstraint`, `FrictionConstraint`) already store `Coordinate`/`CoordinateRate` members that inherit `msd_sim::Vector3D` with identical layout

**No design change required.**

#### BLOCKER-2: Velocity Types in InertialState (ACCEPTED — Design Gap)

**Gemini's concern**: If `CoordinateRate` is removed, `InertialState` uses `Coordinate` for `velocity` and `acceleration`, which is a semantic error (velocity is not position).

**Verdict**: **Valid.** `InertialState` currently uses `Coordinate` (position type) for `velocity` and `acceleration`. This is a pre-existing semantic misuse not introduced by this ticket, but if we're cleaning up semantic types, it should be addressed.

**Design change**: Add `InertialState` to the Phase C migration table:
- `InertialState::velocity` → `msd_sim::Vector3D` (linear velocity, not a position)
- `InertialState::acceleration` → `msd_sim::Vector3D` (linear acceleration, not a position)
- `InertialState::position` → Keep as `Coordinate` (semantically a position)

This change has broad downstream impact since `InertialState` is used in 50+ files. The migration is mechanical (`.x()`, `.y()`, `.z()` accessors work identically on both types) but must be added to the Phase C scope estimate.

#### MAJOR-1: Vec3Base Hierarchy Inconsistency (ACCEPTED — Design Decision Needed)

**Gemini's concern**: `Coordinate` uses `Vec3Base<Coordinate>` CRTP, but `AngularCoordinate` and `AngularRate` inherit directly from `msd_sim::Vector3D`. This creates inconsistency.

**Verdict**: **Valid observation, but intentional.** `AngularCoordinate` cannot use `Vec3Base` because it adds normalization logic in constructors and compound operators that `Vec3Base` doesn't provide. `AngularRate` could use `Vec3Base` but was implemented separately in ticket 0024 (before `Vec3Base` existed in its current form).

**Design change**: Add to Phase D scope — evaluate migrating `AngularRate` to inherit from `Vec3Base<AngularRate>` to reduce code duplication. `AngularCoordinate` must remain separate due to normalization. Document the rationale for the inconsistency.

#### MAJOR-2: Test Strategy Gaps (ACCEPTED — Strengthened)

**Gemini's concern**: 8 unit tests for a 100+ file migration is insufficient. Need compilation smoke test and numerical equivalence test.

**Verdict**: **Valid.** The test strategy should be strengthened:

**Design changes**:
1. **Compilation gate**: After each Phase C file migration, full build must pass before proceeding to next file. This is already implied by "compile and run tests after each file" but should be explicit.
2. **Numerical equivalence test**: Add a deterministic simulation scenario that runs before and after Phase C. Compare trajectories with epsilon tolerance. This catches subtle numerical changes from type conversion paths.
3. **Exhaustive grep verification**: After Phase C, `grep -r "CoordinateRate" msd/` must return zero hits (excluding comments/docs).

#### MINOR-1: Formatter Template Simplification (ACCEPTED)

**Gemini's concern**: The `Accessor` type parameter on `Vec3FormatterBase` is unnecessary — `formatComponents` can accept the accessor as a function template parameter instead.

**Verdict**: **Valid simplification.** Remove `Accessor` from the class template and make `formatComponents` a member function template:

```cpp
template <typename T>
struct Vec3FormatterBase {
  // ... parse logic ...
  template <typename Context, typename F>
  auto formatComponents(const T& vec, F&& accessor, Context& ctx) const { ... }
};
```

This eliminates the `decltype([...])` portability concern entirely.

#### MINOR-2: NOLINTBEGIN Block Strategy (NOTED — Deferred to Implementation)

**Gemini's concern**: `NOLINTBEGIN`/`NOLINTEND` blocks would be cleaner than per-line annotations for multiple constructors.

**Verdict**: **Reasonable alternative.** Defer to implementer discretion — either approach is acceptable. Per-line is more precise; block is less noisy. Document both as acceptable in the design.

#### INFO: Include Dependency (DISMISSED — No Impact)

**Gemini's concern**: Replacing `CoordinateRate` with `msd_sim::Vector3D` may require adding `<Eigen/Dense>` includes.

**Verdict**: **No impact.** All files that currently include `Coordinate.hpp` (which defines `CoordinateRate`) already transitively include `<Eigen/Dense>` because `Coordinate.hpp` includes it. Replacing the type doesn't change include dependencies.

#### INFO: Thread Safety (DISMISSED — No Change)

**Gemini's concern**: Deferred normalization in `AngularCoordinate` may use `mutable` members.

**Verdict**: **No `mutable` members.** `AngularCoordinate`'s normalization is not lazy — it's deferred (triggers on assignment/compound operators, not on read). Accessors return raw values. No thread-safety change from this refactoring.

---

## References

- [Ticket 0024_angular_coordinate](../../../tickets/0024_angular_coordinate.md) — Introduced AngularCoordinate/AngularRate
- [Ticket 0013_integrate_clang_tidy](../../../tickets/0013_integrate_clang_tidy.md) — Introduced explicit constructor requirement
- [CLAUDE.md](../../../CLAUDE.md) — Project coding standards
- Eigen documentation: [Extending Eigen](https://eigen.tuxfamily.org/dox/TopicCustomizing_CustomScalar.html)

---

## Design Review

**Reviewer**: Design Review Agent (Claude Opus 4.5)
**Date**: 2026-02-04
**Status**: APPROVED
**Iteration**: 0 of 1 (no revision needed)

### Executive Summary

This design comprehensively addresses accumulated technical debt and semantic misuse in the DataType system through a well-structured four-phase refactoring. The phased approach (A: code quality → B: formatter consolidation → C: semantic cleanup → D: constructor simplification) minimizes risk and rework. The design correctly identifies the root cause (semantic type confusion) and proposes a pragmatic solution (use raw `msd_sim::Vector3D` for generic vectors) that respects the "don't fight the linter when you're wrong" principle while appropriately suppressing it when the code is correct (mathematical types are the canonical exception to explicit constructors).

The design demonstrates strong architectural judgment by rejecting composition-based workarounds (correctly identifying that Eigen inheritance + `using Base::operator=` is the standard pattern) and by executing semantic cleanup (Phase C) before constructor simplification (Phase D) to avoid refactoring constructors for code about to be deleted.

**Recommendation**: Proceed to implementation with the notes below.

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | `Vec3Base`, `Vec3FormatterBase` follow project patterns. No new public types beyond existing. |
| Namespace organization | ✓ | `detail` namespace for `Vec3Base` and formatter internals is correct pattern. No new namespaces introduced. |
| File structure | ✓ | Preserves existing structure. Phase B consolidation reduces LOC by 50% (120 → 60 lines formatter code). |
| Dependency direction | ✓ | No new dependencies. Changes are internal to msd-sim DataTypes. Eigen already required. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | No RAII changes. Existing value semantics preserved. |
| Smart pointer appropriateness | ✓ | No smart pointers involved. Pure value types throughout. |
| Value/reference semantics | ✓ | All types remain value types (24 bytes). No semantic changes beyond improved type safety via removal of `CoordinateRate`. |
| Rule of 0/3/5 | ✓ | **Critical**: Design adds `final` specifier to prevent base-pointer deletion UB (non-virtual `msd_sim::Vector3D` destructor). Rule of Zero preserved with explicit `= default` as per project convention. |
| Const correctness | ✓ | No const changes. Existing const accessors preserved. |
| Exception safety | ✓ | No exception handling changes. Deferred normalization logic unchanged (basic guarantee). |
| Initialization | ✓ | Existing brace initialization preserved. NaN for uninitialized floats not applicable (these are initialized coordinate types). |
| Return values | ✓ | No new functions with output parameters. Existing return-value pattern preserved. |

**Critical Safety Note**: The `final` specifier addition in Phase D is **essential** to prevent undefined behavior. `msd_sim::Vector3D` has a non-virtual destructor, so deleting `Coordinate*`/`AngularCoordinate*` through `msd_sim::Vector3D*` would be UB. The design correctly addresses this.

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | All affected files already include `<Eigen/Dense>` transitively via `Coordinate.hpp`. Phase C replacement with `msd_sim::Vector3D` adds zero new includes. |
| Template complexity | ✓ | `Vec3FormatterBase<T>` is simple template (no SFINAE, no variadic packs). Function template for `formatComponents()` accessor avoids `decltype([...])` portability issues (per Gemini review feedback). |
| Memory strategy | ✓ | Zero memory impact. `msd_sim::Vector3D` and `CoordinateRate` are identical at machine code level (both 24 bytes, same layout). |
| Thread safety | ✓ | No thread-safety changes. AngularCoordinate normalization is deferred (on write), not lazy (on read). No `mutable` members. |
| Build integration | ✓ | No CMakeLists.txt changes required. Phase C removes `CoordinateRate` from `Coordinate.hpp`, no source file additions. Phase B adds no new source files (formatter is header-only in `std` namespace). |

**Migration Feasibility**: Phase C scope is large (~100+ files) but mechanically simple. The design includes file-by-file migration table (Table at lines 254-269) with specific guidance. InertialState migration has broadest impact (~50+ consumers) but is mechanical (`.x()`, `.y()`, `.z()` accessors identical).

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | Mathematical value types. No global state. Each phase independently testable. |
| Mockable dependencies | ✓ | No dependencies to mock. Pure value types with Eigen dependency (header-only). |
| Observable state | ✓ | All state public or accessible via accessors. Normalization behavior observable via comparison. |

**Test Strategy**: Design includes comprehensive test impact analysis (lines 286-338) with per-phase verification protocol. Phase C verification protocol (lines 334-338) includes:
1. Per-file compilation gate (fail-fast)
2. Exhaustive grep verification (zero `CoordinateRate` hits)
3. Numerical equivalence test (deterministic trajectory comparison with epsilon tolerance)

This is **exemplary** test planning that goes beyond typical unit tests to catch subtle numerical changes from type conversion paths.

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Phase C downstream breakage via ADL or template specialization changes | Integration | Low | Medium | File-by-file migration with compilation gate. Manual review of each signature change. | No |
| R2 | Eigen expression template bypasses normalization with implicit ctors | Technical | Medium | Low | Documented limitation (already exists for `operator[]`). Accept as trade-off for API convenience. | No |
| R3 | Triple `[[nodiscard]]` root cause recurs if not addressed | Maintenance | Medium | Low | Phase A investigates root cause (macro/tool), fixes symptom + cause. Track finding in implementation review. | No |
| R4 | InertialState migration scope underestimated | Technical | Low | Medium | Design identifies ~50+ consumers. Mechanical migration (accessors identical). Per-file compilation gate. | No |

**R1 Analysis**: The design correctly identifies ADL and template specialization as potential breakage vectors (Technical Risk 5 in ticket, line 149). Mitigation is appropriate (manual review per signature change). Likelihood is Low because:
- `Coordinate` and `CoordinateRate` both inherit `msd_sim::Vector3D` (ADL set unchanged)
- No known template specializations on `Coordinate`/`CoordinateRate` in codebase
- Replacing with `msd_sim::Vector3D` *narrows* the ADL set (base class only), which is safer than widening

**R2 Analysis**: This is a known trade-off. The design documents the limitation (line 147, line 164) and accepts it as the cost of implicit Eigen constructors. This is the correct decision—attempting to close this gap would require wrapping all Eigen accessors, defeating the purpose of inheritance and violating the Constraint "Do not use composition for Eigen types" (line 163).

**R3 Analysis**: The design correctly elevates this from symptom-fix to root-cause investigation (line 357, lines 488-495). This prevents recurrence and demonstrates software engineering maturity.

**R4 Analysis**: The design explicitly calls out InertialState as the broadest-impact change (line 259, line 269) and notes the migration is mechanical. This is honest scoping—neither underestimating nor overstating the work.

### Prototype Guidance

**No prototypes required**. All changes are deterministic refactorings:
- **Phase A**: Text replacements (syntactic cleanup)
- **Phase B**: Template refactoring with existing logic (no new algorithms)
- **Phase C**: Type replacements (`msd_sim::Vector3D` already in use, just changing where)
- **Phase D**: Constructor simplification (removing `explicit`, validated pattern from Eigen ecosystem)

**Validation**: The ticket's prototype P1 from 0024 already validated deferred normalization (line 138-142). No new performance characteristics to prototype.

### Required Revisions

*None*. Design is ready for implementation.

### Blocking Issues

*None*.

### Notes for Implementation

#### Phase Ordering Rationale (Excellent)

The A → B → C → D ordering is **optimal** for minimizing rework:
- **A before B**: Clean up code quality issues so formatter consolidation operates on clean code
- **B before C**: Consolidate formatters while all three types still exist (Coordinate, AngularCoordinate, AngularRate)
- **C before D**: Remove semantic misuses *before* simplifying constructors, avoiding refactoring constructors for soon-to-be-deleted code
- **D last**: Only simplify constructors for types that remain after semantic cleanup

This demonstrates strategic thinking about change sequencing.

#### Semantic Clarity Wins

The decision to use raw `msd_sim::Vector3D` for normals, forces, directions (Q2 recommendation, line 122) is **architecturally sound**:
- **Honest naming**: A normal vector is not a position. Using `Coordinate` for normals is a lie.
- **Type proliferation avoided**: Alternative (introduce `Direction3` type) creates "type hell" with no semantic value
- **Eigen interoperability**: `msd_sim::Vector3D` is the lingua franca of the Eigen ecosystem
- **Precedent**: InertialState already uses `msd_sim::Vector3D` for quaternionRate and inverseInertiaTensor

The ticket author's assessment "The current two-type system forces these quantities into ill-fitting semantic boxes" (line 62) is correct. This design fixes the root cause.

#### Formatter Consolidation is Clean

The `Vec3FormatterBase<T>` approach (lines 169-239) with function-template accessor (per Gemini review, line 167) is **textbook template design**:
- Avoids `decltype([...])` portability issues
- Parameterizes on type only (class template)
- Parameterizes on accessor via function template (deduced from lambda)
- Reduces duplication from 120 lines → 60 lines (50% reduction)

This is the correct abstraction level—neither over-engineered (no CRTP) nor under-abstracted (duplicated code).

#### Clang-Tidy Suppression is Justified

The decision to suppress `google-explicit-constructor` via per-line `// NOLINT` (Q1 recommendation, line 113) is **pragmatically correct**:
- Mathematical vector types are the canonical exception to explicit constructors
- Eigen's own types use implicit constructors for expression templates
- Alternative (CRTP/policy workarounds) would be engineering solely to satisfy a linter—overengineering
- Per-line NOLINT is scoped and auditable

The design correctly rejects composition (Q1 option D, line 113) because wrapping `msd_sim::Vector3D` would require forwarding dozens of methods and lose expression template machinery.

#### Migration Risk Mitigation is Comprehensive

The three-tier verification for Phase C (lines 334-338) is **production-grade**:
1. **Per-file compilation gate**: Fail fast, prevent cascading breakage
2. **Exhaustive grep**: Catch missed migrations (zero `CoordinateRate` hits post-Phase C)
3. **Numerical equivalence test**: Catch subtle floating-point changes from conversion paths

This goes beyond "tests pass" to "behavior is bit-identical" where possible. Excellent rigor.

#### Open Question 1 (No Human Input Needed)

**Q1**: CoordinateRate removal scope (line 344)
- **Recommendation**: Option A (remove entirely) is correct. The design justifies this (line 347).
- **No human input required**—design already committed to Option A in Phase C scope.

#### Open Question 2 (No Human Input Needed)

**Q2**: Vec3FormatterBase location (line 349)
- **Recommendation**: Option A (`msd_sim::detail` namespace) is correct. Internal refactoring, not public API.
- **No human input required**—design already specifies `detail` namespace (line 211).

#### Open Question 3 (Action Required in Phase A)

**Q3**: Triple `[[nodiscard]]` root cause (line 354)
- **Action**: Phase A must investigate (not just fix symptom). Report finding in implementation review.
- **Not blocking**: Can fix symptoms in Phase A, investigate root cause in parallel.

#### Open Question 4 (No Human Input Needed)

**Q4**: Phase execution order (line 359)
- **Confirmed**: A → B → C → D is technically optimal (justified above).

#### Open Question 5 (No Human Input Needed)

**Q5**: Backward compatibility strategy (line 364)
- **Recommendation**: Option A (breaking change in single release) is correct for internal project.
- **No human input required**—internal project, no external consumers.

#### Open Question 6 (No Human Input Needed)

**Q6**: Final specifier placement (line 370)
- **Confirmed**: Required by C++ standard to prevent UB from base-pointer deletion.

### Constraints Acknowledgment

The design correctly respects all constraints (lines 469-479):
- ✓ AngularCoordinate normalization preserved
- ✓ Vec3Base CRTP extended, not discarded
- ✓ Angular types remain separate (AngularCoordinate vs AngularRate)
- ✓ InertialState quaternion design not re-opened
- ✓ Inheritance used (composition rejected)
- ✓ Known normalization leak accepted (documented, not addressed)

**Constraint Compliance**: 100%

### Design Complexity Sanity Check

The design includes self-assessment (lines 401-416) that correctly identifies:
- **Red Flag 3 triggered**: 6 modified components vs 1 new component
- **Assessment correct**: This is a **refactoring ticket**, not a feature. The ratio reflects cleanup work (fixing existing code, not adding new features).
- **Direction correct**: Modified components are getting *simpler* (fewer lines, less duplication, clearer semantics).

**Complexity verdict**: Acceptable for refactoring. No red flags for feature addition.

### Code Quality Gates Awareness

The design demonstrates awareness of quality gates (lines 417-468):
- **Warnings as errors**: Explicitly addressed per phase (line 421)
- **Static analysis**: NOLINT strategy documented (line 427)
- **Const correctness**: Preserved (line 432)
- **Performance**: Zero regression expected (line 439, justified by "identical at machine code level")
- **Testability**: Independent phase testing (line 455)

This is **exemplary** awareness of quality gates during design phase.

### Summary

This is a **mature, well-justified refactoring design** that addresses the root cause of semantic type confusion while respecting the project's established patterns and constraints. The four-phase approach minimizes risk, the migration strategy is comprehensive, and the test plan includes numerical regression detection beyond standard unit tests.

**Key Strengths**:
1. Phased approach minimizes rework (C before D is critical)
2. Root cause addressed (semantic confusion), not just symptoms
3. Pragmatic linter suppression (don't engineer around correct code)
4. Comprehensive migration plan with per-file guidance
5. Three-tier verification (compilation + grep + numerical equivalence)
6. `final` specifier prevents UB from non-virtual destructor

**Key Trade-offs** (all justified):
1. `auto` bypasses normalization (documented, accepted)
2. Large migration scope (100+ files, but mechanical)
3. Linter suppression (justified for mathematical types)

**Next Steps**: Proceed to implementation. Execute phases in order A → B → C → D. Track triple `[[nodiscard]]` root cause finding for implementation review.

# Ticket 0037: DataType Simplification — Coordinate, AngularCoordinate, AngularRate

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete
- [x] Design Review
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation In Progress
**Assignee**: Unassigned
**Created**: 2026-02-04
**Priority**: Medium
**Complexity**: Large
**Target Component**: msd-sim (DataTypes), msd-assets (secondary)
**Generate Tutorial**: No

---

## Summary

Simplify the `Coordinate`, `CoordinateRate`, `AngularCoordinate`, and `AngularRate` data types to reduce Eigen inheritance boilerplate, fix accumulated code quality issues, and establish clearer semantic boundaries for vector quantities. The current design — inheriting from `msd_sim::Vector3D` with explicit template constructors — creates friction with clang-tidy, produces unwieldy conversion patterns, and blurs the semantic distinction between positions, velocities, directions, and forces.

---

## Motivation

Ticket 0024 introduced `AngularCoordinate` and `AngularRate` following the `Coordinate` pattern of inheriting from `msd_sim::Vector3D`. While the type-safe separation of orientation vs. rate is sound, practical experience has revealed several issues:

### 1. Explicit Constructor Friction

All four types use `explicit` template constructors for Eigen expressions. This means every Eigen operation (cross product, matrix multiplication, scalar arithmetic) that returns an Eigen expression type requires explicit wrapping:

```cpp
// Every Eigen result needs explicit construction
return CoordinateRate{rotation_.transpose() * globalVector};
Coordinate rACrossN = lever_arm_a_.cross(contact_normal_);  // implicit via assignment, but...
CoordinateRate result = rotation_ * velocity;  // fails without explicit ctor
```

This is verbose and error-prone. The `explicit` constructors were added for clang-tidy compliance (`google-explicit-constructor`), but the cure is worse than the disease for types that are meant to interoperate seamlessly with Eigen.

### 2. Semantic Type Misuse

The two types `Coordinate` (position) and `CoordinateRate` (velocity) are used throughout the codebase for quantities that are neither:

| Semantic Quantity | Current Type Used | Files |
|---|---|---|
| Contact normals | `Coordinate`, `CoordinateRate` | ContactConstraint.hpp, CollisionResult.hpp, FrictionConstraint.hpp |
| Facet normals | `CoordinateRate` | Facet.hpp |
| GJK search direction | `CoordinateRate` | GJK.hpp |
| Lever arms | `Coordinate` | ContactConstraint.hpp, FrictionConstraint.hpp |
| Forces/torques | `Coordinate`, `CoordinateRate` | SemiImplicitEulerIntegrator.hpp, AssetInertial.hpp, ConstraintSolver.hpp |
| Tangent basis vectors | `Coordinate` | TangentBasis.hpp, EPA.cpp |

A normal vector is not a position. A force is not a velocity. The current two-type system forces these quantities into ill-fitting semantic boxes.

### 3. Accumulated Code Quality Issues

The current codebase has several quality issues stemming from the 0024 implementation:

- **Triple `[[nodiscard]]`**: 73 instances of `[[nodiscard]] [[nodiscard]] [[nodiscard]]` across 13 files
- **Extra-nested braces**: `{ { { statement; } } }` in AngularCoordinate/AngularRate formatters and `normalizeIfNeeded()`
- **Triple parentheses**: `(((width * 10)))` in formatter parse methods
- **Constant naming typo**: `kKNormalizationThreshold` (double-K) should be `kNormalizationThreshold`
- **Inconsistent explicitness**: Constructors are `explicit` but assignment operators are not, creating an inconsistent conversion contract

### 4. Formatter Duplication

The `std::formatter` specializations for `Coordinate`, `AngularCoordinate`, and `AngularRate` are nearly identical (~40 lines each). This is 120+ lines of duplicated parse/format logic.

---

## Requirements

### Functional Requirements

1. **FR-1**: Redesign the type hierarchy to clearly distinguish semantic categories: position, direction/vector, and angular quantities.
2. **FR-2**: Eliminate the need for explicit Eigen expression wrapping at call sites — Eigen arithmetic results should be assignable to the appropriate type without verbose casting.
3. **FR-3**: Preserve the rate vs. coordinate distinction for angular quantities (normalization semantics differ and this distinction is validated by 0024 prototyping).
4. **FR-4**: Preserve the `AngularCoordinate` deferred normalization strategy (normalize when |value| > 100π threshold, per prototype P1 findings).
5. **FR-5**: Maintain full Eigen interoperability — cross product, dot product, matrix multiplication, norm, etc.
6. **FR-6**: Clean up all accumulated code quality issues (triple `[[nodiscard]]`, nested braces, constant naming).
7. **FR-7**: Reduce or eliminate formatter duplication via a shared formatter template or CRTP base.

### Non-Functional Requirements

1. **NFR-1**: Zero performance regression — same memory footprint (24 bytes per vector) and equivalent or better operation performance.
2. **NFR-2**: Clean clang-tidy pass with current configuration (no new warnings introduced).
3. **NFR-3**: All existing tests pass with zero regressions.
4. **NFR-4**: Migration should be primarily mechanical (find-and-replace for type names where semantics match).

---

## Design Questions

The design phase should evaluate these approaches and settle on a recommended direction.

### Q1: How Should We Handle `explicit` vs. Implicit Eigen Constructors?

**Options:**
- **(a)** Remove `explicit` from Eigen template constructors (simplest, but clang-tidy `google-explicit-constructor` will warn)
- **(b)** Suppress `google-explicit-constructor` for these specific types via NOLINT annotations
- **(c)** Use a CRTP or policy base that provides implicit Eigen conversion while keeping the public constructor explicit
- **(d)** Replace inheritance with composition (store `msd_sim::Vector3D` as member, forward operations) — eliminates the Eigen inheritance issues entirely
- **(e)** Use Eigen's `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` and plugin system to register custom types

**Recommendation**: Option (b) — suppress `google-explicit-constructor` for these specific types via `// NOLINT`. Mathematical vector types are the canonical exception to the explicit-constructor rule; engineering a CRTP or policy workaround solely to satisfy a linter is over-engineering. Option (d) (composition) should be **rejected** — wrapping `msd_sim::Vector3D` as a member requires forwarding dozens of methods (`block`, `segment`, `norm`, `squaredNorm`, `dot`, `cross`, all operator overloads) to maintain FR-5, and loses access to Eigen's expression template machinery (lazy evaluation). The standard Eigen extension pattern is inheritance + `using Base::operator=`.

### Q2: Should We Introduce a Generic Direction/Vector Type?

**Options:**
- **(a)** Keep `Coordinate` for position, `CoordinateRate` for velocity, and use raw `msd_sim::Vector3D` for everything else (normals, forces, directions)
- **(b)** Introduce a third type (e.g., `Vector3` or `Direction3`) for quantities that are neither position nor velocity
- **(c)** Collapse `Coordinate` and `CoordinateRate` into a single type (abandoning position/velocity distinction for linear quantities) and use only `msd_sim::Vector3D` for generic vectors

**Recommendation**: (a) is the least disruptive and arguably the most honest — normals, forces, and directions don't benefit from wrapping. The semantic value of `Coordinate` is "this is a position in world space" and `CoordinateRate` is "this is a velocity/acceleration." Everything else is just a vector.

### Q3: Should Formatters Be Unified?

**Options:**
- **(a)** Write a single template formatter (e.g., `Vec3Formatter<T>`) and specialize `std::formatter` as thin wrappers
- **(b)** Use a CRTP base that provides formatting
- **(c)** Keep separate formatters but extract the shared parse logic into a utility function

**Recommendation**: (a) — a single template formatter parameterized on accessor functions eliminates the duplication.

### Q4: Should AngularCoordinate Keep the Deferred Normalization Approach?

The prototype findings from 0024 were:
- Lazy normalization (on access): **3029% overhead** — rejected
- Eager normalization (always): **1.5% overhead** — accepted
- Deferred normalization (on assign if > threshold): **chosen** approach

**Recommendation**: Keep deferred normalization. It's validated by prototyping. Just clean up the implementation (remove nested braces, fix constant name).

---

## Technical Risks

1. **Eigen expression template return types**: Removing `explicit` or changing inheritance may alter how Eigen expression templates resolve. Operations like `a.cross(b)` return `Eigen::Cross<...>`, not `msd_sim::Vector3D`. If constructors are implicit, the compiler could attempt implicit conversion chains that fail to compile or produce unexpected types. This needs careful testing.
2. **`auto` bypasses normalization**: If constructors become implicit, `auto c = a.cross(b)` captures an Eigen expression template, not an `AngularCoordinate`. Normalization logic will not run. The design must either discourage `auto` usage for angular types or accept that `auto` bypasses normalization. This is already partially the case — direct `operator[]` or `x()`/`y()`/`z()` access bypasses normalization today (documented in the AngularCoordinate header comment as a known limitation).
3. **Downstream breakage scope**: These types are used in 100+ files across msd-sim, msd-gui, and msd-assets. The migration scope is large.
4. **Semantic migration ambiguity**: Changing `Coordinate normal` to `msd_sim::Vector3D normal` at call sites requires judgment — which instances are true positions vs. misused-as-vectors? This cannot be fully automated. Phase C must be done manually, not via automated find-and-replace.
5. **ADL and template specialization breakage**: Changing `Coordinate` to `msd_sim::Vector3D` in function signatures may break Argument Dependent Lookup (ADL) and any template specializations that exist for `Coordinate` but not `msd_sim::Vector3D`. Each signature change in Phase C must be reviewed for downstream impact.
6. **Non-virtual destructor**: `msd_sim::Vector3D` has a non-virtual destructor. Derived types (`Coordinate`, `AngularCoordinate`, etc.) must not be deleted through a base pointer. The design should add `static_assert` guards or mark derived types `final` to prevent undefined behavior.
7. **Triple `[[nodiscard]]` root cause**: The 73 instances of triple `[[nodiscard]]` may stem from a macro expansion or automated tool issue rather than manual copy-paste. Phase A should investigate and fix the root cause, not just the symptoms, to prevent recurrence.
8. **clang-tidy interaction**: The `google-explicit-constructor` check will flag implicit converting constructors. Recommendation is to suppress via `// NOLINT` annotations on these specific types (see Q1).

---

## Constraints

- AngularCoordinate normalization behavior must not regress (validated by 0024 prototypes)
- The `Vec3Base` CRTP pattern in `Coordinate.hpp` is a good foundation — extend rather than discard
- Angular types must remain as separate types (AngularCoordinate vs. AngularRate) per 0024 design rationale
- InertialState quaternion representation is settled (from 0030) — don't re-open that design
- **Do not use composition for Eigen types** — inheritance is the standard Eigen extension pattern; composition breaks expression template machinery and requires forwarding dozens of methods
- **Known normalization leak**: Direct `operator[]`, `x()`, `y()`, `z()` access on `AngularCoordinate` bypasses deferred normalization. This is an accepted limitation documented in the header. The design should not attempt to close this gap (doing so would require wrapping all Eigen accessors, defeating the purpose of inheritance)

---

## Acceptance Criteria

- [ ] **AC1**: `Coordinate`, `CoordinateRate`, `AngularCoordinate`, `AngularRate` types compile cleanly with current clang-tidy configuration (zero new warnings).
- [ ] **AC2**: Eigen arithmetic expressions are assignable to the appropriate type without explicit wrapping at call sites.
- [ ] **AC3**: Quantities that are semantically directions/normals/forces use `msd_sim::Vector3D` (or a new designated type), not `Coordinate`/`CoordinateRate`.
- [ ] **AC4**: All `[[nodiscard]]` attributes are single (not duplicated).
- [ ] **AC5**: No extra-nested braces `{ { { } } }` or triple-parentheses `((()))` artifacts remain.
- [ ] **AC6**: `kKNormalizationThreshold` naming follows project convention (single `k` prefix).
- [ ] **AC7**: Formatter duplication is reduced (shared implementation, specialized only for type-specific accessor differences).
- [ ] **AC8**: All existing tests pass with zero regressions.
- [ ] **AC9**: `AngularCoordinate` deferred normalization behavior preserved (threshold-based, on assign/compound operators).
- [ ] **AC10**: Derived types are protected against base-pointer deletion UB (`final` specifier or `static_assert`).
- [ ] **AC11**: Root cause of triple `[[nodiscard]]` identified and addressed (not just symptoms).

---

## Scope Breakdown (Suggested Sub-Tickets)

Given the large scope, this ticket may benefit from decomposition:

### Phase A: Code Quality Cleanup (Low Risk)
- Investigate root cause of triple `[[nodiscard]]` (macro expansion? automated tool?) and fix across all 13 files
- Fix nested braces in AngularCoordinate/AngularRate formatters and `normalizeIfNeeded()`
- Fix `kKNormalizationThreshold` → `kNormalizationThreshold`
- Fix triple-parentheses artifacts
- *No API changes, no behavioral changes*

### Phase B: Formatter Consolidation (Low Risk)
- Extract shared formatter template
- Specialize for Coordinate, AngularCoordinate, AngularRate
- *No API changes*

### Phase C: Semantic Type Cleanup (Medium-High Risk)
- Identify all misused `Coordinate`/`CoordinateRate` instances
- Replace with `msd_sim::Vector3D` (or new type per design decision)
- Update function signatures where parameter types change
- Verify ADL and template specialization correctness for each changed signature
- **Must be done manually** — requires per-instance judgment, not automated find-and-replace
- *API changes at internal boundaries*

### Phase D: Explicit Constructor Simplification (Medium Risk)
- Remove `explicit` from Eigen template constructors on remaining types
- Add `// NOLINT(google-explicit-constructor)` suppression annotations
- Add `static_assert` or `final` specifier to prevent base-pointer deletion (non-virtual destructor safety)
- Ensure `using Base::operator=` is correctly exposed in all derived types
- Update all affected construction patterns
- *Reduced scope since Phase C already removed many Coordinate/CoordinateRate usages*

**Recommended execution order**: A → B → C → D. Perform semantic cleanup (C) before constructor simplification (D) to avoid refactoring constructors for type usages that will be replaced with `msd_sim::Vector3D`.

---

## References

### Current Implementation
- [Coordinate.hpp](msd/msd-sim/src/DataTypes/Coordinate.hpp) — `Coordinate` and `CoordinateRate` (Vec3Base CRTP)
- [AngularCoordinate.hpp](msd/msd-sim/src/DataTypes/AngularCoordinate.hpp) — `AngularCoordinate` (direct Eigen inheritance)
- [AngularRate.hpp](msd/msd-sim/src/DataTypes/AngularRate.hpp) — `AngularRate` (direct Eigen inheritance)
- [Facet.hpp](msd/msd-sim/src/DataTypes/Facet.hpp) — Example of `CoordinateRate` misuse for normals

### Related Tickets
- `0024_angular_coordinate` — Introduced AngularCoordinate/AngularRate (predecessor)
- `0013_integrate_clang_tidy` — clang-tidy integration that motivates explicit constructor concerns
- `0030_lagrangian_quaternion_physics` — InertialState quaternion design (stable, not to be re-opened)

### Key Usage Hubs (Files Most Affected)
- `msd-sim/src/Physics/RigidBody/InertialState.hpp`
- `msd-sim/src/Environment/ReferenceFrame.hpp`
- `msd-sim/src/Physics/Constraints/ContactConstraint.hpp`
- `msd-sim/src/Physics/Constraints/FrictionConstraint.hpp`
- `msd-sim/src/Physics/Collision/CollisionResult.hpp`
- `msd-sim/src/Physics/Collision/GJK.hpp`
- `msd-sim/src/Physics/Collision/EPA.cpp`

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-04
- **Completed**: 2026-02-04
- **Artifacts**: Initial ticket created
- **Notes**: Ticket created after codebase analysis revealed accumulated issues from 0024 implementation and systematic misuse of Coordinate/CoordinateRate for non-position/non-velocity quantities. Identified 73 triple `[[nodiscard]]` instances, nested brace artifacts, constant naming typo, and formatter duplication. Suggested 4-phase decomposition for risk management.

### Draft Review (Gemini)
- **Completed**: 2026-02-04
- **Reviewer**: Gemini Pro (external AI review)
- **Key findings incorporated**:
  1. **Reject composition (Q1 option D)**: Wrapping msd_sim::Vector3D as member breaks expression template machinery, requires forwarding dozens of methods. Inheritance + `using Base::operator=` is the standard pattern.
  2. **Suppress linter, don't engineer around it (Q1)**: `// NOLINT(google-explicit-constructor)` is the pragmatic path. Mathematical vector types are the canonical exception.
  3. **`auto` bypasses normalization**: Added as technical risk — `auto c = a.cross(b)` captures expression template, not AngularCoordinate. Accepted limitation alongside existing `operator[]` leak.
  4. **Non-virtual destructor UB**: Added constraint requiring `final` or `static_assert` to prevent base-pointer deletion of derived types.
  5. **Phase reordering**: C (semantic cleanup) before D (constructor simplification) — don't refactor constructors for usages about to be replaced.
  6. **Phase C is highest risk**: Upgraded from Medium to Medium-High. Requires manual review per-instance; ADL/template specialization breakage possible.
  7. **Triple `[[nodiscard]]` root cause**: May be macro expansion or tooling bug — Phase A must investigate root cause, not just fix symptoms.
  8. **Q2 recommendation endorsed**: Using raw `msd_sim::Vector3D` for normals/forces/directions avoids "type hell" proliferation.

### Status Transition
- **From**: Draft
- **To**: Ready for Design
- **Date**: 2026-02-04
- **Notes**: Ticket does not require math design (this is a software engineering refactoring task). Advancing directly to architectural design phase. Next step: Execute cpp-architect agent to produce design document.

### Design Phase
- **Started**: 2026-02-04
- **Completed**: 2026-02-04
- **Artifacts**:
  - `docs/designs/0037_datatype_simplification/design.md` — Comprehensive architectural design document
  - `docs/designs/0037_datatype_simplification/0037_datatype_simplification.puml` — PlantUML component diagram
- **Notes**: Design phase complete. Created comprehensive design covering all four phases (A: code quality cleanup, B: formatter consolidation, C: semantic type migration, D: explicit constructor simplification). Design addresses all technical risks identified in ticket, provides detailed migration strategies for Phase C (highest-risk phase), and includes comprehensive test impact analysis. Key design decisions: (1) Remove CoordinateRate entirely in Phase C, replace with msd_sim::Vector3D for non-velocity quantities. (2) Suppress google-explicit-constructor via per-line NOLINT comments for mathematical vector types (canonical exception to explicit constructor rule). (3) Consolidate formatters via Vec3FormatterBase template in detail namespace. (4) Add final specifier to all derived types to prevent base-pointer deletion UB. (5) Execute phases in order A→B→C→D to minimize rework. Design includes detailed file-by-file migration plan for Phase C with ADL/template specialization risk mitigation. Zero performance impact expected (msd_sim::Vector3D vs CoordinateRate are identical at machine code level). All project coding standards preserved (brace initialization, Rule of Zero, references for non-owning access). Next step: Execute design-reviewer agent for assessment.

### Design Review (Gemini Pro — Preliminary External Review)
- **Completed**: 2026-02-04
- **Reviewer**: Gemini Pro (external AI review)
- **Verdict**: CHANGES REQUESTED → 7 issues triaged, 4 incorporated, 3 dismissed
- **Issues incorporated**:
  1. **InertialState velocity/acceleration** (was BLOCKER): `InertialState::velocity` and `InertialState::acceleration` use `Coordinate` (position type) — must migrate to `msd_sim::Vector3D` in Phase C. Added to migration table. Broadest downstream impact (~50+ files).
  2. **Formatter template simplified** (MINOR): Removed `Accessor` from `Vec3FormatterBase` class template parameters — `formatComponents` now accepts accessor as function template parameter. Eliminates `decltype([...])` portability concern.
  3. **Test strategy strengthened** (MAJOR): Added per-file compilation gate, exhaustive grep verification, and numerical equivalence test to Phase C verification protocol.
  4. **Vec3Base hierarchy noted** (MAJOR): AngularRate could use Vec3Base; AngularCoordinate cannot (normalization). Added to Phase D evaluation scope.
- **Issues dismissed**:
  1. **Eigen alignment** (was BLOCKER): False positive — `Vector3d` is 24 bytes, NOT a fixed-size vectorizable type. No special alignment needed.
  2. **Include dependency explosion**: No impact — all affected files already transitively include `<Eigen/Dense>` via `Coordinate.hpp`.
  3. **Thread safety**: No `mutable` members in AngularCoordinate — normalization is deferred (on write), not lazy (on read).

### Design Review (Formal)
- **Started**: 2026-02-04
- **Completed**: 2026-02-04
- **Reviewer**: Design Review Agent (Claude Opus 4.5)
- **Status**: APPROVED
- **Artifacts**:
  - Design review appended to `docs/designs/0037_datatype_simplification/design.md`
- **Key Findings**:
  - **Architectural fit**: 100% compliant with project conventions. Preserves file structure, follows naming patterns, no new dependencies.
  - **C++ design quality**: All criteria pass. Critical safety improvement via `final` specifier to prevent UB from non-virtual `msd_sim::Vector3D` destructor.
  - **Feasibility**: Zero memory impact. All affected files already include `<Eigen/Dense>`. Template complexity minimal. Migration scope large (~100+ files) but mechanical.
  - **Testability**: Value types with no global state. Three-tier verification (compilation gate + grep + numerical equivalence) is production-grade rigor.
  - **Risks identified**: 4 risks, all Low-Medium likelihood/impact, all mitigated. No prototypes required (deterministic refactorings only).
  - **Phase ordering**: A → B → C → D is optimal for minimizing rework. Executing C (semantic cleanup) before D (constructor simplification) avoids refactoring constructors for code about to be deleted.
  - **Semantic clarity**: Using raw `msd_sim::Vector3D` for normals/forces/directions is architecturally sound—avoids "type hell" and is honest naming (normal != position).
  - **Clang-tidy suppression**: Per-line `// NOLINT(google-explicit-constructor)` is justified. Mathematical vector types are canonical exception to explicit constructors. Rejecting composition workarounds is correct (would require forwarding dozens of methods, lose expression templates).
- **Notes**: Design demonstrates software engineering maturity: root cause addressed (not just symptoms), strategic change sequencing, comprehensive migration plan, numerical regression detection. All 6 constraints acknowledged (100% compliance). No blocking issues. Ready for implementation.
- **Next Step**: Proceed to implementation. Execute phases in order A → B → C → D. Track triple `[[nodiscard]]` root cause finding for implementation review.

---

## Human Feedback

*(Space for reviewer comments at any point in the workflow)*

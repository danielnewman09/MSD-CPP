# Design: ReferenceFrame Transform API Refactor

## Summary

Refactor `ReferenceFrame`'s overloaded `globalToLocal()` and `localToGlobal()` methods into explicitly-named `Absolute` (rotation + translation) and `Relative` (rotation only) template functions. This eliminates a dangerous implicit overload resolution bug where `Coordinate` silently selects the translation-including overload when the caller intends rotation-only behavior, which has already caused real bugs (EPA normal-space mismatch in ticket 0039d). The new API makes the caller's intent unambiguous at the call site.

## Architecture Changes

### PlantUML Diagram

See: `./0041_reference_frame_transform_refactor.puml`

### Type Hierarchy Context

Understanding the type hierarchy is essential for this design. All four relevant types ultimately inherit from `Eigen::Vector3d`:

```
Eigen::Vector3d
  |-- msd_sim::detail::Vec3DBase<Derived>   (CRTP base)
  |     |-- msd_sim::Vector3D               (generic 3D vector)
  |     |-- msd_sim::Coordinate             (spatial position)
  |-- msd_sim::AngularRate                   (angular velocity/acceleration, direct)
  |-- msd_sim::AngularCoordinate             (orientation angles, direct)
```

Key observations:

1. `Vector3D` and `Coordinate` are **sibling types** (both inherit from `Vec3DBase<Self>`, not from each other). The CLAUDE.md MEMORY entry stating "Coordinate inherits from Vector3D" describes the *old* architecture; the current CRTP design makes them independent types sharing a common base.

2. `AngularRate` inherits directly from `Eigen::Vector3d` (not through `Vec3DBase`).

3. `AngularCoordinate` also inherits directly from `Eigen::Vector3d` (not through `Vec3DBase`). It represents orientation angles with deferred normalization. While there are no current call sites passing `AngularCoordinate` to these transforms, the `EigenVec3Type` concept accepts it since it satisfies both `std::derived_from<AngularCoordinate, Eigen::Vector3d>` and `std::is_constructible_v<AngularCoordinate, const Eigen::Vector3d&>` (via its `explicit` template constructor from `Eigen::MatrixBase<OtherDerived>`, which still satisfies `is_constructible` since that trait tests direct initialization). This is intentional and consistent with the Open Question 2 resolution: the mathematical operation is well-defined even if semantically unusual for orientation angles.

4. All four types accept `Eigen::MatrixBase<OtherDerived>` in converting constructors, which enables `T{rotation_matrix * vector}` to construct the correct output type from an Eigen expression. Note that `AngularCoordinate`'s template constructor is `explicit`, but this does not affect the template return statement `T{expr}` since brace initialization uses direct initialization.

5. The current overload bug arises because C++ resolves `globalToLocal(someCoordinate)` to the `Coordinate` overload (exact match) rather than the `Vector3D` overload (requires conversion), even when the caller intends rotation-only semantics.

### Template Constraint Design

The template functions must accept `Vector3D`, `Coordinate`, `AngularRate`, and `AngularCoordinate` while rejecting unrelated types. We use a C++20 `concept` that constrains on two properties:

1. The type must be derived from `Eigen::Vector3d` (ensuring it is one of our 3D vector types).
2. The type must be constructible from an `Eigen::Vector3d` reference (ensuring the return construction `T{expr}` compiles).

```cpp
namespace msd_sim::detail
{

/// Concept constraining template parameters to 3D vector types compatible
/// with ReferenceFrame transforms. Accepts Vector3D, Coordinate,
/// AngularRate, and AngularCoordinate.
template<typename T>
concept EigenVec3Type =
  std::derived_from<T, Eigen::Vector3d> &&
  std::is_constructible_v<T, const Eigen::Vector3d&>;

}  // namespace msd_sim::detail
```

**Why `std::derived_from<T, Eigen::Vector3d>`**: All four accepted types inherit from `Eigen::Vector3d` (either directly or via `Vec3DBase<T>` which itself inherits from `Eigen::Vector3d`). This is a direct, simple check.

**Why `std::is_constructible_v<T, const Eigen::Vector3d&>`**: The template functions return `T{expression}` where the expression is an Eigen matrix product result. All four types have converting constructors from `Eigen::MatrixBase<OtherDerived>`, which covers `Eigen::Vector3d`. This check confirms that the return construction `T{getRotation().transpose() * vec}` will compile. Note: `std::is_constructible_v` tests direct initialization (as in `T{arg}`), so `explicit` constructors (as in `AngularCoordinate`) are included.

**Accepted types**: `Vector3D`, `Coordinate`, `AngularRate`, `AngularCoordinate`. While `AngularCoordinate` has no current call sites that use these transforms, it is accepted by the concept and the mathematical operations are well-defined. This is consistent with the Open Question 2 resolution.

**Rejected types**: Eigen expression templates (e.g., `Eigen::CwiseUnaryOp` from `-dir`) do NOT satisfy the concept because they do not derive from `Eigen::Vector3d`. More importantly, template argument deduction for `const T&` cannot deduce `T` from an Eigen expression type. See "Eigen Expression Type Deduction" below for the implications on call site migration.

**Alternative considered -- `static_assert`**: A `static_assert` inside the function body was considered instead of a concept. The concept approach was chosen because:
- It produces clearer error messages at the call site ("constraints not satisfied" vs. a deep template instantiation error)
- It participates in overload resolution, preventing accidental selection of the template over deprecated overloads
- It is idiomatic C++20

### New Components

This refactor introduces no new classes. All changes are modifications to the existing `ReferenceFrame` class.

### Modified Components

#### ReferenceFrame

- **Current location**: `msd/msd-sim/src/Environment/ReferenceFrame.hpp`, `msd/msd-sim/src/Environment/ReferenceFrame.cpp`
- **Changes required**:

  1. **Add concept definition** in the header (within `msd_sim::detail` namespace or at the top of the `msd_sim` namespace block).

  2. **Add four new template member functions** as inline definitions in the header:

     ```cpp
     /// Transform a point from global frame to local frame.
     /// Applies rotation AND translation (for positions/points).
     /// @tparam T A 3D vector type (Vector3D, Coordinate, AngularRate)
     /// @param globalPoint Point in global frame
     /// @return Point in this local frame
     /// @ticket 0041_reference_frame_transform_refactor
     template<detail::EigenVec3Type T>
     [[nodiscard]] T globalToLocalAbsolute(const T& globalPoint) const
     {
       return T{getRotation().transpose() * (globalPoint - origin_)};
     }

     /// Transform a point from local frame to global frame.
     /// Applies rotation AND translation (for positions/points).
     /// @tparam T A 3D vector type (Vector3D, Coordinate, AngularRate)
     /// @param localPoint Point in this local frame
     /// @return Point in global frame
     /// @ticket 0041_reference_frame_transform_refactor
     template<detail::EigenVec3Type T>
     [[nodiscard]] T localToGlobalAbsolute(const T& localPoint) const
     {
       return T{getRotation() * localPoint + origin_};
     }

     /// Transform a direction vector from global frame to local frame.
     /// Applies rotation ONLY (for directions, normals, velocities).
     /// @tparam T A 3D vector type (Vector3D, Coordinate, AngularRate)
     /// @param globalVector Direction vector in global frame
     /// @return Direction vector in this local frame
     /// @ticket 0041_reference_frame_transform_refactor
     template<detail::EigenVec3Type T>
     [[nodiscard]] T globalToLocalRelative(const T& globalVector) const
     {
       return T{getRotation().transpose() * globalVector};
     }

     /// Transform a direction vector from local frame to global frame.
     /// Applies rotation ONLY (for directions, normals, velocities).
     /// @tparam T A 3D vector type (Vector3D, Coordinate, AngularRate)
     /// @param localVector Direction vector in this local frame
     /// @return Direction vector in global frame
     /// @ticket 0041_reference_frame_transform_refactor
     template<detail::EigenVec3Type T>
     [[nodiscard]] T localToGlobalRelative(const T& localVector) const
     {
       return T{getRotation() * localVector};
     }
     ```

  3. **Mark existing overloads as `[[deprecated]]`** with migration guidance:

     ```cpp
     [[deprecated("Use globalToLocalAbsolute() for points or globalToLocalRelative() for directions")]]
     Coordinate globalToLocal(const Coordinate& globalPoint) const;

     [[deprecated("Use globalToLocalRelative() for directions")]]
     msd_sim::Vector3D globalToLocal(const msd_sim::Vector3D& globalVector) const;

     [[deprecated("Use globalToLocalRelative() for angular rates")]]
     AngularRate globalToLocal(const AngularRate& globalVector) const;

     [[deprecated("Use localToGlobalAbsolute() for points or localToGlobalRelative() for directions")]]
     Coordinate localToGlobal(const Coordinate& localPoint) const;

     [[deprecated("Use localToGlobalRelative() for directions")]]
     Vector3D localToGlobal(const Vector3D& localVector) const;

     [[deprecated("Use localToGlobalRelative() for angular rates")]]
     AngularRate localToGlobal(const AngularRate& localVector) const;
     ```

  4. **Implement the missing `globalToLocal(AngularRate)`** in `ReferenceFrame.cpp`. The header declares this function at line 153 but no implementation exists in the .cpp file. This is a latent linker error (never triggered because no caller uses it yet). It must be implemented before deprecation, since the deprecated declaration must resolve:

     ```cpp
     AngularRate ReferenceFrame::globalToLocal(const AngularRate& globalVector) const
     {
       return AngularRate{rotation_.transpose() * globalVector};
     }
     ```

  5. **Add `#include <concepts>` and `#include <type_traits>`** to the header for the concept definition.

- **Backward compatibility**: Full backward compatibility maintained. Old overloads remain callable but produce compiler warnings. No ABI break (deprecated functions remain in the compiled library). The templates are header-only additions.

- **Header organization**: The new template functions and concept should be placed in the header file between the batch API declarations and the deprecated overload declarations, creating a clear visual separation:

  ```
  // === InPlace and Batch API (unchanged) ===
  void globalToLocalInPlace(Coordinate& globalCoord) const;
  // ... etc ...

  // === New explicit template API (Ticket 0041) ===
  template<detail::EigenVec3Type T>
  [[nodiscard]] T globalToLocalAbsolute(const T& globalPoint) const { ... }
  // ... etc ...

  // === Deprecated overloaded API ===
  [[deprecated("...")]]
  Coordinate globalToLocal(const Coordinate& globalPoint) const;
  // ... etc ...
  ```

#### `getRotation()` vs direct `rotation_` access -- behavioral improvement

The new template functions call `getRotation()`, which is a public inline method that lazily updates the rotation matrix via `updateRotationMatrix()` when `updated_` is false. The existing non-template overloads in `ReferenceFrame.cpp` access the `rotation_` member directly without calling `getRotation()`, which means they do NOT trigger the lazy rotation matrix update.

This is a **subtle but positive behavioral difference**: the new API is more robust in the edge case where the mutable `getAngularCoordinate()` reference has been used (which sets `updated_ = false`). Under the old API, a caller could observe a stale rotation matrix if they modified orientation via the mutable reference and then called `globalToLocal()` without an intervening `getRotation()`. The new template functions always see the up-to-date rotation matrix.

This behavioral improvement is intentional. In practice, the edge case is unlikely to affect existing call sites because:
1. The mutable `getAngularCoordinate()` is deprecated (ticket 0030).
2. Callers that modify orientation typically use `setQuaternion()`, which updates `rotation_` and sets `updated_ = true`.
3. No existing call site in the codebase modifies orientation via the mutable reference and then immediately calls `globalToLocal()`.

However, this means the migration is not a pure rename in the strictest sense. The implementer should be aware that the new functions guarantee rotation matrix freshness, whereas the old ones did not.

### Integration Points

| New Function | Existing Component | Integration Type | Notes |
|---|---|---|---|
| `globalToLocalAbsolute<T>()` | `EPA.cpp` | Replaces `globalToLocal(Coordinate)` | For witness point and vertex transforms |
| `globalToLocalRelative<T>()` | `EPA.cpp` | Replaces `globalToLocal(Vector3D{...})` workaround | Eliminates manual Vector3D construction |
| `localToGlobalAbsolute<T>()` | `EPA.cpp`, `GJK.cpp`, `SupportFunction.cpp` | Replaces `localToGlobal(Coordinate)` | For position transforms |
| `localToGlobalRelative<T>()` | `EPA.cpp`, `MotionController.cpp`, `SupportFunction.cpp` | Replaces `localToGlobal(Vector3D)` | For direction transforms |
| `detail::EigenVec3Type` concept | `ReferenceFrame.hpp` | New internal constraint | Used only by ReferenceFrame templates |

## Call Site Migration Plan

### EPA.cpp

| Line(s) | Current Code | New Code | Transform Type |
|---|---|---|---|
| 527-529 | `Vector3D normalAsVec{normal.x(), normal.y(), normal.z()}; frame.globalToLocal(normalAsVec)` | `frame.globalToLocalRelative(normal)` | Relative (direction) |
| 530-532 | `Vector3D negNormal{-normal.x(), ...}; frame.globalToLocal(negNormal)` | `frame.globalToLocalRelative(Coordinate{-normal})` | Relative (direction). See "Eigen Expression Type Deduction" note below. |
| 553-555 | `frame.localToGlobal(Vector3D{refFaces[0].get().normal.x(), ...})` | `frame.localToGlobalRelative(refFaces[0].get().normal)` | Relative (direction) |
| 560-562 | `frame.localToGlobal(Vector3D{incFaces[0].get().normal.x(), ...})` | `frame.localToGlobalRelative(incFaces[0].get().normal)` | Relative (direction) |
| 391 | `frame.localToGlobal(hullVertices[idx])` | `frame.localToGlobalAbsolute(hullVertices[idx])` | Absolute (position) |
| 688 | `frameA.globalToLocal(witnessA)` | `frameA.globalToLocalAbsolute(witnessA)` | Absolute (position) |
| 689 | `frameB.globalToLocal(witnessB)` | `frameB.globalToLocalAbsolute(witnessB)` | Absolute (position) |
| 696-699 | `frameA.localToGlobal(edgeA.start)` etc. | `frameA.localToGlobalAbsolute(edgeA.start)` etc. | Absolute (position) |
| 735 | `Vector3D normalVec{normal.x(), normal.y(), normal.z()}` | Remove workaround (normal is used in dot product, not in globalToLocal at this line) | N/A (no transform call) |

**Note on EPA line 735**: This line constructs a `Vector3D` from `normal` for use in a `dot()` call and subtraction, not for a `globalToLocal()` call. The workaround comment at lines 732-734 is misleading -- `normalVec` is used only for `edgeDir.dot(normal)` and the projection calculation. However, since `normal` is already a `Coordinate` and `Coordinate` supports `dot()`, the explicit `Vector3D` construction here is unnecessary but harmless. The implementer should verify whether this is truly a workaround or simply dead caution.

### SupportFunction.cpp

| Line(s) | Current Code | New Code | Transform Type |
|---|---|---|---|
| 42 | `frameA.globalToLocal(dir)` | `frameA.globalToLocalRelative(dir)` | Relative (direction) |
| 52 | `frameB.globalToLocal(Vector3D{-dir})` | `frameB.globalToLocalRelative(Vector3D{-dir})` | Relative (direction). See "Eigen Expression Type Deduction" note below. |
| 49 | `frameA.localToGlobal(supportALocal)` | `frameA.localToGlobalAbsolute(supportALocal)` | Absolute (position) |
| 54 | `frameB.localToGlobal(supportBLocal)` | `frameB.localToGlobalAbsolute(supportBLocal)` | Absolute (position) |
| 71 | `frameA.globalToLocal(dir)` | `frameA.globalToLocalRelative(dir)` | Relative (direction) |
| 81 | `frameB.globalToLocal(Vector3D{-dir})` | `frameB.globalToLocalRelative(Vector3D{-dir})` | Relative (direction). See "Eigen Expression Type Deduction" note below. |
| 78 | `frameA.localToGlobal(supportALocal)` | `frameA.localToGlobalAbsolute(supportALocal)` | Absolute (position) |
| 83 | `frameB.localToGlobal(supportBLocal)` | `frameB.localToGlobalAbsolute(supportBLocal)` | Absolute (position) |

**Note on SupportFunction.cpp lines 42, 71**: `dir` is typed as `const msd_sim::Vector3D&` (the function parameter), so `globalToLocal(dir)` currently selects the `Vector3D` overload (rotation-only) correctly. The migration to `globalToLocalRelative(dir)` is for clarity but does not change behavior. Template argument deduction succeeds here because `dir` is a concrete `Vector3D`, not an Eigen expression.

**Note on SupportFunction.cpp lines 52, 81 (Eigen expression type deduction)**: `Vector3D{-dir}` must be retained. The expression `-dir` evaluates to an `Eigen::CwiseUnaryOp<...>` expression template, NOT a concrete `Vector3D`. Template argument deduction for `const T&` cannot deduce `T` from this expression type (the concept is never even checked -- deduction itself fails). The explicit `Vector3D{-dir}` construction materializes the expression into a concrete type that the template can deduce. The same applies to EPA.cpp line 530-532 where `-normal` (a `Coordinate`) produces an Eigen expression and must be wrapped as `Coordinate{-normal}`.

### GJK.cpp

| Line(s) | Current Code | New Code | Transform Type |
|---|---|---|---|
| 64 | `frame.localToGlobalBatch(corners)` | Unchanged (batch API) | N/A |
| 88 | `frameA.localToGlobal(hullA.getCentroid())` | `frameA.localToGlobalAbsolute(hullA.getCentroid())` | Absolute (position) |
| 89 | `frameB.localToGlobal(hullB.getCentroid())` | `frameB.localToGlobalAbsolute(hullB.getCentroid())` | Absolute (position) |

### MotionController.cpp

| Line(s) | Current Code | New Code | Transform Type |
|---|---|---|---|
| 38 | `frame.localToGlobal(Vector3D{0, 0, scaledMoveSpeed})` | `frame.localToGlobalRelative(Vector3D{0, 0, scaledMoveSpeed})` | Relative (direction) |
| 43 | `frame.localToGlobal(Vector3D{0, 0, scaledMoveSpeed})` | `frame.localToGlobalRelative(Vector3D{0, 0, scaledMoveSpeed})` | Relative (direction) |
| 48 | `frame.localToGlobal(Vector3D{scaledMoveSpeed, 0, 0})` | `frame.localToGlobalRelative(Vector3D{scaledMoveSpeed, 0, 0})` | Relative (direction) |
| 53 | `frame.localToGlobal(Vector3D{scaledMoveSpeed, 0, 0})` | `frame.localToGlobalRelative(Vector3D{scaledMoveSpeed, 0, 0})` | Relative (direction) |
| 58 | `frame.localToGlobal(Vector3D{0, scaledMoveSpeed, 0})` | `frame.localToGlobalRelative(Vector3D{0, scaledMoveSpeed, 0})` | Relative (direction) |
| 63 | `frame.localToGlobal(Vector3D{0, scaledMoveSpeed, 0})` | `frame.localToGlobalRelative(Vector3D{0, scaledMoveSpeed, 0})` | Relative (direction) |

**Rationale**: These are direction vectors representing local-frame movement directions. The result is added to `newOrigin`, which is a `Coordinate`. This is correct -- the `Vector3D` output of `localToGlobalRelative` is added to a position, yielding a new position (direction + point = point).

### ShaderTransformTest.cpp (msd-gui)

| Line(s) | Current Code | New Code | Transform Type |
|---|---|---|---|
| 608 | `frame.localToGlobal(localPoint)` | `frame.localToGlobalAbsolute(localPoint)` | Absolute (position) |
| 651 | `frame.localToGlobal(localApex)` | `frame.localToGlobalAbsolute(localApex)` | Absolute (position) |

### ReferenceFrameTest.cpp (msd-sim)

All calls in ReferenceFrameTest.cpp use `globalToLocal(Coordinate)` / `localToGlobal(Coordinate)` for point transforms (absolute), except for line 231 which uses `globalToLocal(Vector3D{worldVelocity})` for a direction transform (relative).

| Pattern | Count | Migration |
|---|---|---|
| `frame.globalToLocal(coordinate)` | ~20 | `frame.globalToLocalAbsolute(coordinate)` |
| `frame.localToGlobal(coordinate)` | ~25 | `frame.localToGlobalAbsolute(coordinate)` |
| `frame.globalToLocal(Vector3D{...})` | 1 | `frame.globalToLocalRelative(worldVelocity)` |

## Template Implementation Details

### Why Header-Only

The template functions must be defined in the header because:
1. Templates are instantiated at the call site, requiring the definition to be visible.
2. The function bodies are trivial (single-expression returns), so header-only incurs no code duplication concern.
3. The `[[nodiscard]]` attribute and inline definition enable the compiler to inline these small functions, achieving zero overhead over the current non-template implementations.

### Return Type Preservation

The expression `getRotation().transpose() * globalVector` returns an `Eigen::Product<...>` expression template, not a concrete `Eigen::Vector3d`. The explicit construction `T{expr}` converts this to the concrete type `T` (e.g., `Coordinate`, `Vector3D`, `AngularRate`, or `AngularCoordinate`). All four accepted types have converting constructors from `Eigen::MatrixBase<OtherDerived>`, making this safe:

```cpp
// In Vec3DBase<Derived>:
template <typename OtherDerived>
Vec3DBase(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Vector3d{other} {}

// In AngularRate:
template <typename OtherDerived>
AngularRate(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Vector3d{other} {}
```

### Concept vs. `static_assert` vs. SFINAE

| Approach | Error Quality | Overload Participation | C++20 Required |
|---|---|---|---|
| C++20 concept | Best (clear "constraint not satisfied") | Yes | Yes |
| `static_assert` | Good (custom message) | No (still participates) | No |
| SFINAE `enable_if` | Poor (template deduction failure) | Yes | No |

The project already requires C++20 (per CLAUDE.md), so concepts are the idiomatic choice. The concept is defined in `msd_sim::detail` namespace to keep it internal.

### `origin_` Subtraction in `globalToLocalAbsolute`

The expression `globalPoint - origin_` involves subtracting a `Coordinate` (the `origin_`) from the template parameter `T`. Since all target types inherit from `Eigen::Vector3d`, and `Eigen::Vector3d` subtraction is defined via `Eigen::MatrixBase`, this operation is valid for all accepted types. The result is an Eigen expression that is then multiplied by the rotation transpose.

For the `localToGlobalAbsolute` case, `getRotation() * localPoint + origin_` similarly works because addition between `Eigen::Vector3d`-derived types is defined through the Eigen expression template system.

### Interaction with Deprecated Overloads

The deprecated non-template overloads and the new template functions coexist without ambiguity:

- When calling `frame.globalToLocalAbsolute(coord)`, only the template matches (no non-template overload has that name).
- When calling `frame.globalToLocal(coord)`, only the deprecated non-template overloads match (the templates have different names).
- There is no overload resolution conflict because the function names are distinct.

### Eigen Expression Type Deduction

The template parameter `T` is deduced from the function argument type via `const T&`. This means the argument must be a **concrete type** that the compiler can deduce `T` from. Eigen expression templates (such as `Eigen::CwiseUnaryOp` produced by `-dir` or `-normal`) are NOT concrete `Eigen::Vector3d`-derived types, and template argument deduction will fail before the concept is even checked.

**Call sites affected**:
- `SupportFunction.cpp` lines 52, 81: `-dir` where `dir` is `const Vector3D&`
- `EPA.cpp` line 530-532: `-normal` where `normal` is a `Coordinate`

**Resolution**: These call sites must retain explicit type construction to materialize the Eigen expression into a concrete type:
- `globalToLocalRelative(Vector3D{-dir})` -- explicit `Vector3D` construction
- `globalToLocalRelative(Coordinate{-normal})` -- explicit `Coordinate` construction

Alternatively, callers could use explicit template argument specification (e.g., `globalToLocalRelative<Vector3D>(-dir)`), but explicit construction is preferred because it is consistent with the existing code style and avoids introducing a new pattern.

This is **not** a limitation of the concept design -- it is an inherent property of C++ template argument deduction. The concept correctly rejects Eigen expression types (they do not derive from `Eigen::Vector3d`), and even if the concept were loosened, the return type `T` could not be deduced from an expression type.

### Compiler and Eigen Compatibility

The `#include <concepts>` and `#include <type_traits>` headers are required for the concept definition. All target compilers in the project's build matrix fully support these headers:
- **GCC 11+**: Full `<concepts>` support since GCC 10.
- **Clang 14+**: Full `<concepts>` support since Clang 12.
- **MSVC 2019+**: Full `<concepts>` support since MSVC 19.26 (VS 2019 16.6).

No known interaction issues exist between `<concepts>` and Eigen headers. The concept uses only `std::derived_from` and `std::is_constructible_v`, which operate on type traits and do not interact with Eigen's template metaprogramming infrastructure. The concept is evaluated at instantiation time, after Eigen headers are fully parsed.

## Test Impact

### Existing Tests Affected

| Test File | Impact | Action Required |
|---|---|---|
| `msd-sim/test/Environment/ReferenceFrameTest.cpp` | ~46 calls to deprecated API | Update all `globalToLocal`/`localToGlobal` calls to explicit `Absolute`/`Relative` variants |
| `msd-gui/test/ShaderTransformTest.cpp` | 2 calls to deprecated API | Update to `localToGlobalAbsolute` |

All existing tests must continue to pass with identical numerical results after migration. The migration is functionally equivalent for all existing call sites -- the only behavioral difference is that the new template functions call `getRotation()` (triggering lazy rotation matrix update) whereas the old overloads accessed `rotation_` directly. This difference is a robustness improvement (see "`getRotation()` vs direct `rotation_` access" section above) and does not affect any existing call site since no current caller uses the mutable `getAngularCoordinate()` reference before calling a transform method.

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|---|---|---|
| `ReferenceFrame` | `globalToLocalRelative_Coordinate_RotationOnly` | Passing a `Coordinate` to `globalToLocalRelative` applies rotation only -- no translation. This directly tests the bug that motivated this refactor. |
| `ReferenceFrame` | `globalToLocalRelative_Vector3D_SameAsAbsolute_AtOrigin` | At identity origin, `Relative` and `Absolute` produce the same result for any vector. |
| `ReferenceFrame` | `globalToLocalAbsolute_Coordinate_IncludesTranslation` | Absolute transform includes origin subtraction/addition. |
| `ReferenceFrame` | `localToGlobalAbsolute_Coordinate_IncludesTranslation` | Reverse direction of absolute transform. |
| `ReferenceFrame` | `globalToLocalRelative_AngularRate` | `AngularRate` accepted by template, rotation-only applied. Validates R5 requirement. |
| `ReferenceFrame` | `localToGlobalRelative_AngularRate_MatchesExisting` | Template `localToGlobalRelative(AngularRate)` matches existing `localToGlobal(AngularRate)`. |
| `ReferenceFrame` | `Absolute_Relative_Roundtrip` | `localToGlobalAbsolute(globalToLocalAbsolute(p)) == p` and `localToGlobalRelative(globalToLocalRelative(v)) == v`. |
| `ReferenceFrame` | `TypeDeduction_AllTypes` | All three types (`Vector3D`, `Coordinate`, `AngularRate`) work with both `Absolute` and `Relative` variants without explicit template arguments (AC2). |
| `ReferenceFrame` | `Relative_Does_Not_Apply_Translation_Coordinate` | A `Coordinate` passed to `globalToLocalRelative` produces a different result than the same `Coordinate` passed to `globalToLocalAbsolute` when origin is non-zero. This is the critical regression test for AC1. |

#### Integration Tests

No new integration tests required. The existing collision detection and physics integration tests provide sufficient coverage of the transform pipeline. If all existing tests pass after migration, the refactor is behaviorally correct.

## Open Questions

### Design Decisions (Human Input Needed)

1. **Concept location**: Where should the `EigenVec3Type` concept be defined?
   - Option A: Inline in `ReferenceFrame.hpp` within a `detail` namespace block -- Pros: Self-contained, no new files. Cons: Concept definition in an unrelated header.
   - Option B: In a new `msd-sim/src/DataTypes/EigenConcepts.hpp` header -- Pros: Reusable by future templates. Cons: New file for a single concept.
   - Recommendation: Option A, since the concept is currently only used by ReferenceFrame. It can be extracted to its own header if future templates need it.

2. **`globalToLocalAbsolute` for `AngularRate` and `Vector3D`**: The `Absolute` variants apply translation. Calling `globalToLocalAbsolute(someAngularRate)` would subtract `origin_` from an angular rate vector, which is physically meaningless. Should the concept be split into two concepts (one for `Absolute` accepting only `Coordinate`, one for `Relative` accepting all types)?
   - Option A: Single concept for all four functions -- Pros: Simpler API, consistent template constraint. Cons: Allows semantically meaningless `globalToLocalAbsolute(angularRate)`.
   - Option B: `Absolute` constrained to `Coordinate` only, `Relative` accepts all types -- Pros: Type-safe. Cons: `Absolute` is no longer a template (just a named non-template function taking `Coordinate`), losing consistency.
   - Option C: Single concept, but document that `Absolute` is intended for positional types. Callers who pass `AngularRate` to `Absolute` get the correct mathematical result (it is well-defined, just unusual).
   - Recommendation: Option A. The ticket requirement (R1) specifies templates for all four functions. The mathematical operation is well-defined even if semantically unusual. Adding a code review convention ("use Relative for non-positional types") is simpler than splitting the concept. If stronger enforcement is desired later, Option B can be implemented as a non-breaking change (adding a concept to `Absolute` that restricts to `Coordinate` would cause a compile error for misuse, which is desirable).

### Prototype Required

None. The template approach is straightforward and the concept constraint is a well-known C++20 pattern. No prototyping is needed.

### Requirements Clarification

1. **Deprecated function removal timeline**: The ticket states deprecated overloads should be removed "in a future ticket." Is there a target timeline or should a follow-up ticket be created during implementation?

---

## Migration Strategy

### Implementation Order

1. **Step 1**: Add the `EigenVec3Type` concept and the four template functions to `ReferenceFrame.hpp`. Add the missing `globalToLocal(AngularRate)` implementation to `ReferenceFrame.cpp`. Build to verify the new functions compile.

2. **Step 2**: Add `[[deprecated]]` attributes to all six existing overloads. Build to see all deprecation warnings (this validates the migration list is complete).

3. **Step 3**: Migrate all callers in `msd-sim/src/` (EPA.cpp, SupportFunction.cpp, GJK.cpp, MotionController.cpp). Build to verify deprecation warnings are eliminated for production code.

4. **Step 4**: Migrate test callers in `msd-sim/test/` (ReferenceFrameTest.cpp) and `msd-gui/test/` (ShaderTransformTest.cpp). Build to verify zero deprecation warnings remain.

5. **Step 5**: Add new tests (the 9 unit tests listed above). Run full test suite to verify all 669+ existing tests pass.

6. **Step 6**: Update CLAUDE.md documentation (Environment/CLAUDE.md, msd-sim/CLAUDE.md) to reference the new API.

### Rollback Plan

If issues arise during implementation:
- The deprecated overloads remain fully functional. Reverting the migration is simply reverting the call-site changes.
- The template functions are additive and do not modify existing behavior.
- The concept is internal and has no downstream dependents.

---

## Architect Revision Notes

**Date**: 2026-02-07
**Responding to**: Design Review -- Initial Assessment

### Changes Made

| Issue ID | Original | Revised | Rationale |
|----------|----------|---------|-----------|
| I1 | PlantUML note defined concept as `EigenVector3Type` with `MatrixBase<T>` derivation and compile-time dimension checks | PlantUML note now uses `EigenVec3Type` with `Eigen::Vector3d` derivation and `is_constructible` check, matching the design text exactly | Diagram must be authoritative for implementers who reference only the diagram |
| I2 | Claimed "the templates use the same pattern as the existing non-template methods" and "migration is purely a rename" | Documented that new templates call `getRotation()` (lazy update) whereas old overloads access `rotation_` directly; noted this is a positive behavioral improvement; added caveat to "purely a rename" claim | Honest documentation of behavioral difference prevents implementer confusion |
| I3 | EPA line 530: proposed `globalToLocalRelative(-normal)`. SupportFunction lines 52, 81: proposed `globalToLocalRelative(-dir)` | EPA line 530: revised to `globalToLocalRelative(Coordinate{-normal})`. SupportFunction lines 52, 81: revised to `globalToLocalRelative(Vector3D{-dir})`. Added "Eigen Expression Type Deduction" subsection explaining the fundamental issue | `-expr` produces Eigen expression templates, not concrete types; template argument deduction for `const T&` fails on expression types before the concept is even checked |
| I4 | Type hierarchy listed only Vector3D, Coordinate, AngularRate; concept documented as accepting three types | Added AngularCoordinate to type hierarchy and concept acceptance documentation; noted explicit constructor does not affect `is_constructible` (tests direct init); documented as accepted but semantically unusual | AngularCoordinate inherits from Eigen::Vector3d and satisfies both concept predicates |
| I5 | No compiler compatibility analysis for `<concepts>` header | Added "Compiler and Eigen Compatibility" subsection confirming GCC 11+, Clang 14+, MSVC 2019+ all support `<concepts>`; noted no known Eigen interaction issues | Eliminates feasibility concern about header dependencies |

### Diagram Updates
- Updated PlantUML concept note: renamed `EigenVector3Type` to `EigenVec3Type`, replaced `MatrixBase<T>` derivation with `Eigen::Vector3d` derivation, replaced dimension checks with `is_constructible` check
- Added `AngularCoordinate` to the "Accepted types" list in the PlantUML note
- Added `AngularCoordinate` class to the PlantUML diagram as a context type
- Added `AngularCoordinate` inheritance arrow from `EigenVector3d`
- Updated the SupportFunction.cpp migration note in the diagram to show retained `Vector3D{-dir}` construction

### Unchanged (Per Reviewer Guidance)
- Problem analysis and CRTP type hierarchy correction
- API naming convention (Absolute/Relative)
- Concept approach and `msd_sim::detail` namespace placement
- Deprecation strategy (three-phase)
- Migration plan granularity (line-by-line tables)
- Test plan (9 unit tests)
- Header organization (templates between batch API and deprecated overloads)
- `[[nodiscard]]` attribute on all template functions
- Open Question 1 resolution (concept inline in ReferenceFrame.hpp)
- Open Question 2 resolution (single concept for all functions)
- Missing `globalToLocal(AngularRate)` identification
- Rollback plan

---

## Design Review -- Final Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-02-07
**Status**: APPROVED
**Iteration**: 1 of 1

### Issue Resolution Verification

| Issue ID | Status | Verification |
|----------|--------|--------------|
| I1 | RESOLVED | PlantUML note (lines 74-78) now uses `EigenVec3Type` with `std::derived_from<T, Eigen::Vector3d>` and `std::is_constructible_v<T, const Eigen::Vector3d&>`, matching design text exactly. Accepted types list includes `AngularCoordinate`. Eigen expression warning and `getRotation()` behavioral note added. |
| I2 | RESOLVED | Dedicated subsection "`getRotation()` vs direct `rotation_` access -- behavioral improvement" (lines 191-202) honestly documents that new templates call `getRotation()` (lazy update) while old overloads access `rotation_` directly. Improvement documented as intentional. Test Impact section (line 368) adds appropriate caveat to the "purely a rename" claim. |
| I3 | RESOLVED | Dedicated subsection "Eigen Expression Type Deduction" (lines 334-348) correctly explains that `-dir` and `-normal` produce `Eigen::CwiseUnaryOp` expression templates that fail template argument deduction. Migration tables corrected: EPA line 221 now shows `Coordinate{-normal}`, SupportFunction lines 237/241 now show `Vector3D{-dir}`. Explicit construction recommended over explicit template arguments for consistency. |
| I4 | RESOLVED | `AngularCoordinate` added to type hierarchy (line 23), concept acceptance documentation (lines 32-34, 40, 64), and PlantUML diagram (class at lines 116-120, inheritance at line 142). Correctly notes that `explicit` constructor satisfies `is_constructible_v` since that trait tests direct initialization. Documented as accepted but semantically unusual, consistent with `AngularRate` treatment. |
| I5 | RESOLVED | Dedicated subsection "Compiler and Eigen Compatibility" (lines 350-357) confirms GCC 11+ (since GCC 10), Clang 14+ (since Clang 12), and MSVC 2019+ (since 19.26) all support `<concepts>`. States no known interaction issues between `<concepts>` and Eigen headers. |

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | PASS | `globalToLocalAbsolute`, `globalToLocalRelative` follow project `camelCase` method naming. `EigenVec3Type` concept in `detail` namespace follows project patterns. |
| Namespace organization | PASS | Concept placed in `msd_sim::detail`, consistent with existing `Vec3DBase` placement. Template functions are members of `msd_sim::ReferenceFrame`. |
| File structure | PASS | No new files introduced. All changes to existing `ReferenceFrame.hpp` and `ReferenceFrame.cpp`. Consistent with project's one-class-per-header convention. |
| Dependency direction | PASS | Template functions are header-only additions to msd-sim. No new inter-library dependencies. `#include <concepts>` and `#include <type_traits>` are standard library headers. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | PASS | No resource management involved -- pure functional transforms. |
| Smart pointer appropriateness | PASS | No pointers used. Templates operate on value types passed by const reference. |
| Value/reference semantics | PASS | Functions take `const T&` and return `T` by value. Consistent with existing overloads and project preference for return values over output parameters. |
| Rule of 0/3/5 | PASS | No special member functions involved in this change. |
| Const correctness | PASS | All four template functions are `const` member functions. `[[nodiscard]]` applied to all return values. |
| Exception safety | PASS | Template functions are nothrow (Eigen matrix operations do not throw for fixed-size vectors). No dynamic allocation. |
| Brace initialization | PASS | Return statements use `T{expr}` brace initialization, consistent with CLAUDE.md coding standards. |
| C++20 concept usage | PASS | Idiomatic C++20 concept with `std::derived_from` and `std::is_constructible_v`. Correct choice over `static_assert` or SFINAE for this project. |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | PASS | `<concepts>` and `<type_traits>` supported by all target compilers (GCC 11+, Clang 14+, MSVC 2019+). No known Eigen interaction issues. |
| Template complexity | PASS | Single-expression template bodies with straightforward concept constraint. No SFINAE, no partial specialization, no recursive templates. |
| Memory strategy | PASS | No heap allocation. All operations on stack-allocated Eigen vectors. |
| Thread safety | PASS | Template functions are const methods calling `getRotation()` which is already thread-safe for reads (mutable lazy evaluation pattern). Same thread safety profile as existing API. |
| Build integration | PASS | Header-only template additions require no CMake changes. Deprecated overloads remain in `.cpp` file -- no ABI break. |
| Eigen expression deduction | PASS | Design correctly identifies and documents that Eigen expression types (e.g., `-dir`) fail template argument deduction, requiring explicit type construction at affected call sites. Migration tables updated accordingly. |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | PASS | `ReferenceFrame` can be instantiated standalone with `Coordinate` origin and rotation. No external dependencies needed for testing. |
| Mockable dependencies | PASS | No dependencies to mock. Templates operate directly on Eigen math. |
| Observable state | PASS | Transform results are returned by value and directly comparable with `EXPECT_DOUBLE_EQ` / `almostEqual`. |
| Test plan completeness | PASS | 9 new unit tests cover: the critical regression scenario (AC1/AC6), type deduction for all types (AC2), round-trip correctness, angular rate symmetry (R5), and the Absolute vs Relative behavioral difference. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | `AngularCoordinate` template constructor is `explicit` -- if a future caller passes an Eigen expression that evaluates to `AngularCoordinate`, the brace-init `T{expr}` in the template return could behave differently than implicit conversion | Maintenance | Low | Low | Design correctly documents that `T{expr}` uses direct initialization where `explicit` is permitted. Current code has no `AngularCoordinate` transform call sites. | No |
| R2 | Stale MEMORY.md entry stating "Coordinate inherits from Vector3D" could confuse future developers if not updated | Maintenance | Medium | Low | Design identifies this at line 27. Implementer should update MEMORY.md as part of Step 6 (documentation update). | No |
| R3 | Deprecation warnings may be treated as errors by CI if `-Werror` is enabled, potentially breaking the build during the intermediate state where deprecated functions are marked but not yet migrated | Integration | Low | Medium | Migration strategy mitigates by completing all caller migration (Steps 3-4) before running full CI. Step 2 deliberately builds to enumerate warnings. | No |

### Prototype Guidance

No prototypes required. The template approach is a well-established C++20 pattern. The concept constraint uses only standard library traits (`std::derived_from`, `std::is_constructible_v`). All Eigen expression interactions are documented with clear resolution strategies. The migration plan is deterministic (line-by-line tables with verified call sites).

### Requirements Coverage

| Requirement | Addressed | Design Reference |
|-------------|-----------|-----------------|
| R1: Explicit Absolute/Relative function names | Yes | Lines 88-136 (template definitions) |
| R2: Deprecate old overloads | Yes | Lines 138-158 (deprecation attributes) |
| R3: Migrate all internal callers | Yes | Lines 214-285 (migration tables) |
| R4: Keep InPlace and Batch variants | Yes | Lines 46-50 in PlantUML, unchanged in design |
| R5: Symmetric AngularRate support | Yes | Lines 160-167 (missing implementation), line 339 (test) |

### Acceptance Criteria Coverage

| Criterion | Addressed | Validation |
|-----------|-----------|------------|
| AC1: No overload ambiguity | Yes | Test `Relative_Does_Not_Apply_Translation_Coordinate` (line 343) |
| AC2: Template deduction works | Yes | Test `TypeDeduction_AllTypes` (line 342) |
| AC3: Workarounds eliminated | Yes | Migration tables show all `Vector3D{...}` workarounds removed or reduced to Eigen expression materialization |
| AC4: Deprecation warnings | Yes | Step 2 of migration strategy (line 380) |
| AC5: All existing tests pass | Yes | Step 5 explicitly requires all 669+ tests pass (line 386) |
| AC6: Test coverage for regression | Yes | Test `globalToLocalRelative_Coordinate_RotationOnly` (line 335) |

### Summary

The revised design fully resolves all 5 issues from the initial assessment. The architect's revisions are thorough and technically accurate:

1. The PlantUML diagram is now synchronized with the design text.
2. The `getRotation()` behavioral improvement is honestly documented with appropriate caveats.
3. The Eigen expression type deduction limitation is correctly analyzed, with migration tables updated to retain explicit type construction where needed.
4. `AngularCoordinate` is properly documented as accepted by the concept.
5. Compiler compatibility is confirmed for all target toolchains.

The design is well-structured, follows project coding standards, addresses a real and documented bug, and provides a safe migration path with full backward compatibility. The template approach is sound, the concept constraint is appropriate, and the test plan provides comprehensive coverage. No prototypes are needed. The design is ready for implementation.

---

## Design Review -- Initial Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-02-07
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Change |
|----|-------|----------|-----------------|
| I1 | PlantUML diagram concept definition inconsistent with design document text | Architectural | Synchronize the PlantUML note to match the actual concept definition in the design text |
| I2 | Design claims existing overloads use `rotation_` directly without lazy update, but does not note the behavioral improvement | C++ Quality | Document the behavioral difference between old and new API regarding `getRotation()` vs `rotation_` |
| I3 | SupportFunction.cpp migration entries incorrectly state `globalToLocalRelative(dir)` for lines 42/71 | Feasibility | Clarify that `-dir` (an Eigen expression, not a `Vector3D`) requires verification that the concept accepts Eigen expression types, or that the explicit `Vector3D{-dir}` construction must be retained |
| I4 | Missing `AngularCoordinate` from concept acceptance analysis | C++ Quality | Document why `AngularCoordinate` is excluded from the concept (or include it if it should be supported) |
| I5 | Missing header dependency analysis for `#include <concepts>` | Feasibility | Verify that all target compilers/toolchains in the build matrix support `<concepts>` and document any Eigen header interaction concerns |

### Revision Instructions for Architect

The following changes must be made before final review:

1. **Issue I1 -- PlantUML Diagram Inconsistency**: The PlantUML note on lines 74-78 defines the concept as:
   ```
   concept EigenVector3Type =
       std::derived_from<T, Eigen::MatrixBase<T>>
       && (T::RowsAtCompileTime == 3)
       && (T::ColsAtCompileTime == 1);
   ```
   But the design document text at lines 49-52 defines it as:
   ```cpp
   concept EigenVec3Type =
     std::derived_from<T, Eigen::Vector3d> &&
     std::is_constructible_v<T, const Eigen::Vector3d&>;
   ```
   These are fundamentally different constraints (name, base class check, additional predicates). The PlantUML diagram must be updated to match the authoritative concept definition in the design text, including the correct name `EigenVec3Type`. The diagram currently would mislead an implementer who references only the diagram.

2. **Issue I2 -- `getRotation()` vs Direct `rotation_` Access**: The existing non-template overloads in `ReferenceFrame.cpp` (lines 183-214) access the `rotation_` member directly without calling `getRotation()`, which means they do NOT trigger the lazy rotation matrix update. The new template functions call `getRotation()`, which DOES trigger the lazy update via `updateRotationMatrix()` when `updated_` is false. This is a subtle but positive behavioral difference -- the new API is more robust when `getAngularCoordinate()` (non-const, which sets `updated_ = false`) has been called. The design document at line 186 states "the templates use the same pattern as the existing non-template methods" -- this is incorrect. Add a note explicitly documenting this difference and confirming it is an intentional improvement, not an accidental divergence. This also means the claim at line 327 that "The migration is purely a rename -- no behavioral change for any existing call site" needs a caveat for the edge case where the mutable `getAngularCoordinate()` reference is used.

3. **Issue I3 -- Eigen Expression Type Compatibility with Concept**: On SupportFunction.cpp lines 52 and 81, the current code is `frameB.globalToLocal(msd_sim::Vector3D{-dir})` where `dir` is `const msd_sim::Vector3D&`. The design proposes replacing this with `frameB.globalToLocalRelative(-dir)`. However, `-dir` evaluates to an `Eigen::CwiseUnaryOp<...>` expression template, NOT a `Vector3D`. The concept `EigenVec3Type` requires `std::derived_from<T, Eigen::Vector3d>`, and an Eigen expression type does NOT derive from `Eigen::Vector3d`. Template argument deduction will fail because `T` cannot be deduced from an Eigen expression -- the concept is never even checked in this case; the deduction itself fails since the parameter type is `const T&` and the compiler must deduce `T` from the argument.

   The same issue affects the EPA.cpp migration at line 205 where `frame.globalToLocalRelative(-normal)` is proposed. The expression `-normal` (where `normal` is a `Coordinate`) returns an `Eigen::CwiseUnaryOp`, not a `Coordinate`.

   The design must either:
   - (a) Acknowledge that explicit type construction is still needed at these call sites (e.g., `globalToLocalRelative(Vector3D{-dir})` or `globalToLocalRelative(Coordinate{-normal})`), OR
   - (b) Add explicit template argument specification (e.g., `globalToLocalRelative<Vector3D>(-dir)`) at these sites, OR
   - (c) Add overloads that accept Eigen expression types (more complex, not recommended).

   Update the migration tables and the SupportFunction.cpp notes accordingly.

4. **Issue I4 -- `AngularCoordinate` Not Addressed**: The type hierarchy includes `AngularCoordinate` which also inherits from `Vec3DBase<AngularCoordinate>` and therefore from `Eigen::Vector3d`. The concept `EigenVec3Type` would accept `AngularCoordinate` since it satisfies both `std::derived_from<AngularCoordinate, Eigen::Vector3d>` and `std::is_constructible_v<AngularCoordinate, const Eigen::Vector3d&>`. While there may be no current call sites passing `AngularCoordinate` to these transforms, the design should explicitly document that `AngularCoordinate` is accepted by the concept and whether this is intentional or whether the concept should exclude it. (Recommendation: document it as accepted but semantically unusual, consistent with the Open Question 2 resolution for `AngularRate`.)

5. **Issue I5 -- Header Dependency Verification**: The design states that `#include <concepts>` and `#include <type_traits>` must be added to `ReferenceFrame.hpp`. Add a brief note confirming that the project's minimum compiler requirements (GCC 11+, Clang 14+, MSVC 2019+, per root CLAUDE.md) all support `<concepts>`. Also note whether `<concepts>` has any known interaction issues with Eigen headers (Eigen's heavy use of template metaprogramming can sometimes conflict with concept checks in certain compiler versions). A one-line statement such as "Verified: all target compilers support <concepts>; no known Eigen conflicts" is sufficient.

### Items Passing Review (No Changes Needed)

The following aspects of the design are well-executed and should not be modified:

- **Problem analysis**: The overload resolution bug is correctly identified, and the CRTP type hierarchy is accurately described. The correction of the MEMORY.md entry about Coordinate/Vector3D inheritance is valuable.
- **API naming convention**: `Absolute`/`Relative` naming is clear, self-documenting, and unambiguous. The naming choice is superior to alternatives like `Point`/`Direction` or `WithTranslation`/`RotationOnly`.
- **Concept approach**: Using C++20 concepts is the right choice for this project (C++20 required). The `msd_sim::detail` namespace placement is appropriate.
- **Deprecation strategy**: The three-phase approach (add new API, deprecate old API, migrate callers) is sound and maintains full backward compatibility.
- **Migration plan granularity**: The line-by-line migration tables for each source file are thorough and will significantly de-risk implementation.
- **Test plan**: The 9 proposed unit tests provide comprehensive coverage of the new API, including the critical regression test for AC1. The decision not to add integration tests is correct -- existing tests provide sufficient coverage.
- **Header organization**: Placing templates between the batch API and deprecated overloads creates clear visual separation.
- **`[[nodiscard]]` attribute**: Correctly applied to all template functions since discarding a transform result is almost certainly a bug.
- **Open Question 1 resolution**: Option A (concept inline in ReferenceFrame.hpp) is the right choice for a single-use concept.
- **Open Question 2 resolution**: Option A (single concept for all functions) correctly prioritizes API simplicity. The mathematical operations are well-defined even for unusual type combinations.
- **Missing `globalToLocal(AngularRate)` identification**: Catching the latent linker error is valuable -- implementing this before deprecation is the correct approach.
- **Rollback plan**: Sound and practical.

---

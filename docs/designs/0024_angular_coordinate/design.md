# Design: AngularCoordinate and AngularRate

## Summary

This design introduces two complementary classes for type-safe angular quantity representation in the simulation engine: **`AngularCoordinate`** for orientation angles with deferred normalization, and **`AngularRate`** for angular velocity/acceleration without normalization. Both classes inherit from `msd_sim::Vector3D` to provide full matrix operation support while adding semantic pitch/roll/yaw accessors. This replaces the current type-inconsistent approach where `EulerAngles` (struct of `Angle` objects) is used for orientation but `Coordinate` (generic 3D vector) is used for angular rates, preventing type confusion and providing clear semantic meaning to angular quantities throughout the codebase.

### Key Design Decisions (from Prototypes P1-P1e)

1. **Deferred Normalization** with 100π threshold — **10x faster** than eager-always normalization
2. **Override Compound Operators** (`+=`, `-=`, `*=`, `/=`) for complete normalization coverage
3. **Fast Accessors** — no normalization check on read (0.7 ns, same as raw Eigen)
4. **Accepted Limitation**: Direct `operator[]` access bypasses normalization (documented)

## Architecture Changes

### PlantUML Diagram
See: [`0024_angular_coordinate.puml`](./0024_angular_coordinate.puml)

### New Components

#### AngularCoordinate

- **Purpose**: Represents orientation angles with deferred normalization (threshold: 100π)
- **Header location**: `msd/msd-sim/src/Environment/AngularCoordinate.hpp`
- **Source location**: None (header-only, similar to `Coordinate`)
- **Normalization strategy**: Deferred with large threshold
  - Normalize only when |value| > 100π (~50 revolutions)
  - Check occurs in ALL modifying operations (assignment, `+=`, `-=`, `*=`, `/=`)
  - Accessors are fast (no check) — return raw value
  - **10x faster** than eager-always normalization for typical physics workloads
- **Key interfaces**:
  ```cpp
  namespace msd_sim {

  /**
   * @brief Orientation angles with deferred normalization
   *
   * Inherits from msd_sim::Vector3D for full matrix operation support.
   * Provides semantic pitch/roll/yaw accessors.
   *
   * Normalization strategy:
   * - Values are normalized to (-π, π] only when |value| exceeds kNormalizationThreshold
   * - Check occurs in assignment and compound operators (+=, -=, *=, /=)
   * - Accessors return raw values (fast, no normalization check)
   * - Use normalized() or normalize() for explicit normalization to (-π, π]
   *
   * LIMITATION: Direct access via operator[] bypasses normalization.
   * Use semantic accessors (pitch(), roll(), yaw()) and setters for normal use.
   *
   * Axis convention (ZYX intrinsic rotation):
   * - pitch: Rotation around Y-axis (component 0)
   * - roll:  Rotation around X-axis (component 1)
   * - yaw:   Rotation around Z-axis (component 2)
   *
   * Memory footprint: 24 bytes (same as msd_sim::Vector3D)
   */
  class AngularCoordinate : public msd_sim::Vector3D {
  public:
    /// Normalization threshold (~50 revolutions). Values exceeding this trigger normalization.
    static constexpr double kNormalizationThreshold = 100.0 * M_PI;

    // Default constructor - initializes to (0, 0, 0)
    AngularCoordinate() : msd_sim::Vector3D{0.0, 0.0, 0.0} {}

    // Constructor with pitch, roll, yaw values (in radians)
    AngularCoordinate(double pitch, double roll, double yaw)
      : msd_sim::Vector3D{pitch, roll, yaw} { normalizeIfNeeded(); }

    // Template constructor for Eigen expressions
    template <typename OtherDerived>
    AngularCoordinate(const Eigen::MatrixBase<OtherDerived>& other)
      : msd_sim::Vector3D{other} { normalizeIfNeeded(); }

    // Template assignment for Eigen expressions
    template <typename OtherDerived>
    AngularCoordinate& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
      this->msd_sim::Vector3D::operator=(other);
      normalizeIfNeeded();
      return *this;
    }

    // Override compound operators for complete normalization coverage
    template <typename OtherDerived>
    AngularCoordinate& operator+=(const Eigen::MatrixBase<OtherDerived>& other) {
      this->msd_sim::Vector3D::operator+=(other);
      normalizeIfNeeded();
      return *this;
    }

    template <typename OtherDerived>
    AngularCoordinate& operator-=(const Eigen::MatrixBase<OtherDerived>& other) {
      this->msd_sim::Vector3D::operator-=(other);
      normalizeIfNeeded();
      return *this;
    }

    AngularCoordinate& operator*=(double scalar) {
      this->msd_sim::Vector3D::operator*=(scalar);
      normalizeIfNeeded();
      return *this;
    }

    AngularCoordinate& operator/=(double scalar) {
      this->msd_sim::Vector3D::operator/=(scalar);
      normalizeIfNeeded();
      return *this;
    }

    // Fast accessors (no normalization check - returns raw value)
    double pitch() const { return (*this)[0]; }
    double roll() const { return (*this)[1]; }
    double yaw() const { return (*this)[2]; }

    // Degree accessors (convenience)
    double pitchDeg() const;
    double rollDeg() const;
    double yawDeg() const;

    // Setters with normalization check
    void setPitch(double radians);
    void setRoll(double radians);
    void setYaw(double radians);

    // Explicit full normalization to (-π, π]
    AngularCoordinate normalized() const;
    void normalize();

    // Rule of Zero
    AngularCoordinate(const AngularCoordinate&) = default;
    AngularCoordinate(AngularCoordinate&&) noexcept = default;
    AngularCoordinate& operator=(const AngularCoordinate&) = default;
    AngularCoordinate& operator=(AngularCoordinate&&) noexcept = default;
    ~AngularCoordinate() = default;

  private:
    /// Normalize only if any component exceeds threshold
    void normalizeIfNeeded() {
      if (std::abs((*this)[0]) > kNormalizationThreshold)
        (*this)[0] = normalizeAngle((*this)[0]);
      if (std::abs((*this)[1]) > kNormalizationThreshold)
        (*this)[1] = normalizeAngle((*this)[1]);
      if (std::abs((*this)[2]) > kNormalizationThreshold)
        (*this)[2] = normalizeAngle((*this)[2]);
    }

    static double normalizeAngle(double rad);  // Returns value in (-π, π]
  };

  }  // namespace msd_sim

  // std::format support (similar to Coordinate)
  template <>
  struct std::formatter<msd_sim::AngularCoordinate> {
    // Same format specification as Coordinate
    // "{:.2f}" → "(0.00, 0.00, 0.79)"
  };
  ```

- **Dependencies**:
  - `Eigen3` — Linear algebra base class
  - `Angle.hpp` — Reference normalization logic from existing `Angle::normalize()` method
  - `Coordinate.hpp` — Pattern to follow for Eigen inheritance and formatting

- **Thread safety**: Immutable after creation (value semantics)

- **Error handling**: No exceptions thrown; normalization handles all input ranges

#### AngularRate

- **Purpose**: Represents angular velocity or acceleration without normalization
- **Header location**: `msd/msd-sim/src/Environment/AngularRate.hpp`
- **Source location**: None (header-only, similar to `Coordinate`)
- **Key interfaces**:
  ```cpp
  namespace msd_sim {

  /**
   * @brief Angular rate vector (velocity or acceleration) without normalization
   *
   * Inherits from msd_sim::Vector3D for full matrix operation support.
   * Provides semantic pitch/roll/yaw accessors without any normalization.
   * Rates can exceed 2π rad/s and should not be normalized.
   *
   * Units:
   * - Angular velocity: rad/s
   * - Angular acceleration: rad/s²
   *
   * Memory footprint: 24 bytes (same as msd_sim::Vector3D)
   */
  class AngularRate : public msd_sim::Vector3D {
  public:
    // Default constructor - initializes to (0, 0, 0)
    AngularRate() : msd_sim::Vector3D{0.0, 0.0, 0.0} {}

    // Constructor with pitch, roll, yaw rates
    AngularRate(double pitchRate, double rollRate, double yawRate)
      : msd_sim::Vector3D{pitchRate, rollRate, yawRate} {}

    // Constructor from msd_sim::Vector3D
    AngularRate(const msd_sim::Vector3D& vec) : msd_sim::Vector3D{vec} {}

    // Template constructor for Eigen expressions
    template <typename OtherDerived>
    AngularRate(const Eigen::MatrixBase<OtherDerived>& other)
      : msd_sim::Vector3D{other} {}

    // Template assignment for Eigen expressions
    template <typename OtherDerived>
    AngularRate& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
      this->msd_sim::Vector3D::operator=(other);
      return *this;
    }

    // Semantic accessors (no normalization)
    double& pitch() { return (*this)[0]; }
    double& roll() { return (*this)[1]; }
    double& yaw() { return (*this)[2]; }
    double pitch() const { return (*this)[0]; }
    double roll() const { return (*this)[1]; }
    double yaw() const { return (*this)[2]; }

    // Rule of Zero
    AngularRate(const AngularRate&) = default;
    AngularRate(AngularRate&&) noexcept = default;
    AngularRate& operator=(const AngularRate&) = default;
    AngularRate& operator=(AngularRate&&) noexcept = default;
    ~AngularRate() = default;
  };

  }  // namespace msd_sim

  // std::format support (similar to Coordinate)
  template <>
  struct std::formatter<msd_sim::AngularRate> {
    // Same format specification as Coordinate
    // "{:.2f}" → "(12.57, 0.00, 0.00)"
  };
  ```

- **Dependencies**:
  - `Eigen3` — Linear algebra base class
  - `Coordinate.hpp` — Pattern to follow for Eigen inheritance and formatting

- **Thread safety**: Immutable after creation (value semantics)

- **Error handling**: No exceptions thrown; no normalization means no invalid input

---

### Modified Components

#### InertialState

- **Current location**: `msd/msd-sim/src/Physics/RigidBody/InertialState.hpp`
- **Changes required**:
  1. Replace `EulerAngles orientation` with `AngularCoordinate orientation`
  2. Replace `Coordinate angularVelocity` with `AngularRate angularVelocity`
  3. Replace `Coordinate angularAcceleration` with `AngularRate angularAcceleration`
  4. Add include for `AngularCoordinate.hpp` and `AngularRate.hpp`

- **Modified interface**:
  ```cpp
  #include "msd-sim/src/Environment/AngularCoordinate.hpp"
  #include "msd-sim/src/Environment/AngularRate.hpp"
  #include "msd-sim/src/Environment/Coordinate.hpp"

  struct InertialState {
    // Linear components (unchanged)
    Coordinate position;
    Coordinate velocity;
    Coordinate acceleration;

    // Angular components (CHANGED)
    AngularCoordinate orientation;       // Was: EulerAngles
    AngularRate angularVelocity;         // Was: Coordinate
    AngularRate angularAcceleration;     // Was: Coordinate
  };
  ```

- **Backward compatibility**: **BREAKING CHANGE**
  - Old code using `orientation.pitch.getRad()` must change to `orientation.pitch()`
  - Old code using `angularVelocity.x()` must change to `angularVelocity.pitch()`
  - Migration is straightforward: semantic accessors replace positional accessors

#### ReferenceFrame

- **Current location**: `msd/msd-sim/src/Environment/ReferenceFrame.hpp`, `ReferenceFrame.cpp`
- **Changes required**:
  1. Replace `EulerAngles euler_` with `AngularCoordinate angular_`
  2. Update `setRotation()` to accept only `AngularCoordinate`
  3. Update `getAngularCoordinate()` to return `AngularCoordinate`
  4. Remove all `EulerAngles` references
  5. Add include for `AngularCoordinate.hpp`
  6. Update constructors to accept `AngularCoordinate` instead of `EulerAngles`

- **Updated interface**:
  ```cpp
  class ReferenceFrame {
  public:
    // Constructors (updated)
    ReferenceFrame();
    ReferenceFrame(const Coordinate& origin);
    ReferenceFrame(const Coordinate& origin, const AngularCoordinate& angular);

    // Transform methods (unchanged)
    Coordinate globalToLocal(const Coordinate& coord) const;
    Coordinate localToGlobal(const Coordinate& coord) const;
    Coordinate globalToLocalRelative(const Coordinate& vec) const;
    Coordinate localToGlobalRelative(const Coordinate& vec) const;

    // Rotation setter
    /**
     * @brief Set the rotation using AngularCoordinate
     * @param angular Orientation angles (pitch, roll, yaw) in radians
     */
    void setRotation(const AngularCoordinate& angular);

    // Rotation getter
    /**
     * @brief Get the orientation as AngularCoordinate
     * @return AngularCoordinate representing current rotation
     */
    AngularCoordinate getAngularCoordinate() const;

  private:
    Coordinate origin_;
    AngularCoordinate angular_;  // Replaces EulerAngles euler_
    mutable Eigen::Matrix3d rotation_;
    mutable bool updated_;
  };
  ```

- **Breaking change**: All uses of `ReferenceFrame` must migrate to `AngularCoordinate`
  - No deprecation path - clean replacement
  - Constructor signature changes
  - Method signatures change

#### EulerAngles (REMOVED)

- **Current location**: `msd/msd-sim/src/Environment/EulerAngles.hpp`, `EulerAngles.cpp`
- **Changes required**:
  1. **DELETE** `EulerAngles.hpp`
  2. **DELETE** `EulerAngles.cpp`
  3. Remove from all CMakeLists.txt
  4. Replace all uses with `AngularCoordinate`

- **Migration required**:
  - All code using `EulerAngles` must convert to `AngularCoordinate`
  - Old: `euler.pitch.getRad()` → New: `angular.pitch()`
  - Old: `euler.roll.getRad()` → New: `angular.roll()`
  - Old: `euler.yaw.getRad()` → New: `angular.yaw()`
  - Old: `EulerAngles{Angle{p}, Angle{r}, Angle{y}}` → New: `AngularCoordinate{p, r, y}`

- **Breaking change**: No backward compatibility - clean removal
  - This consolidates testing effort into a single ticket
  - Avoids maintaining parallel implementations

---

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| AngularCoordinate | ReferenceFrame | Setter/Getter | `setRotation()` / `getAngularCoordinate()` |
| AngularCoordinate | InertialState | Member replacement | `orientation` field type change |
| AngularRate | InertialState | Member replacement | `angularVelocity`, `angularAcceleration` field type changes |

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `test/Environment/EnvironmentTest.cpp` | InertialState usage | Breaking change | Update to use `AngularCoordinate` and `AngularRate` accessors |
| `test/Environment/ReferenceFrameTest.cpp` | `setRotation()` tests | Deprecation warning | Add tests for new `AngularCoordinate` overload |
| `test/Physics/ForceApplicationScaffoldingTest.cpp` | InertialState angular fields | Breaking change | Update accessors (`pitch.getRad()` → `pitch()`) |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| AngularCoordinate | Default constructor | Initializes to (0, 0, 0) |
| AngularCoordinate | Value constructor | Stores pitch, roll, yaw correctly |
| AngularCoordinate | Eigen expression template | Constructs from Eigen operations |
| AngularCoordinate | `pitch()` normalization | Returns value normalized to (-π, π] |
| AngularCoordinate | `roll()` normalization | Returns value normalized to (-π, π] |
| AngularCoordinate | `yaw()` normalization | Returns value normalized to (-π, π] |
| AngularCoordinate | `pitchDeg()` conversion | Converts radians to degrees correctly |
| AngularCoordinate | Large angle normalization | 3π normalizes to π |
| AngularCoordinate | Negative angle normalization | -3π normalizes to -π |
| AngularCoordinate | `normalized()` method | Returns fully normalized copy |
| AngularCoordinate | `normalizeInPlace()` | Normalizes in-place |
| AngularCoordinate | Eigen operations | `+`, `-`, `*`, `cross()`, `dot()` work correctly |
| AngularCoordinate | std::format support | Formats as "(pitch, roll, yaw)" with precision |
| AngularRate | Default constructor | Initializes to (0, 0, 0) |
| AngularRate | Value constructor | Stores pitch, roll, yaw rates correctly |
| AngularRate | Eigen expression template | Constructs from Eigen operations |
| AngularRate | `pitch()` accessor | Returns raw value (no normalization) |
| AngularRate | `roll()` accessor | Returns raw value (no normalization) |
| AngularRate | `yaw()` accessor | Returns raw value (no normalization) |
| AngularRate | Large rates | 4π rad/s stored as 4π (not normalized) |
| AngularRate | Eigen operations | `+`, `-`, `*`, `cross()`, `dot()` work correctly |
| AngularRate | std::format support | Formats as "(pitchRate, rollRate, yawRate)" |
| InertialState | Angular fields type check | Compiles with new types |
| InertialState | Semantic accessors | `state.orientation.pitch()` works |
| InertialState | Type safety | Cannot assign AngularRate to orientation (compile error) |
| ReferenceFrame | `setRotation(AngularCoordinate)` | Sets rotation correctly |
| ReferenceFrame | `getAngularCoordinate()` | Returns correct orientation |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Physics integration | InertialState, AngularCoordinate, AngularRate | Angular velocity integrates to orientation correctly |
| Transform integration | ReferenceFrame, AngularCoordinate | Rotation matrix computed correctly from AngularCoordinate |

#### Benchmark Tests

**Performance is CRITICAL** for these classes as they will be called millions of times per simulation. Prototypes are REQUIRED before implementation.

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| AngularCoordinate | Lazy normalization overhead | Cost of `pitch()` call with normalization vs. raw access | < 10% overhead vs. raw double access |
| AngularCoordinate | Arithmetic chain | Cost of `(a + b) * 2 - c` with lazy normalization | Similar to `Coordinate` arithmetic |
| AngularCoordinate | `normalized()` method | Cost of explicit full normalization | ~3x cost of single `pitch()` call |
| AngularRate | Arithmetic operations | Cost of `(a + b) * 2 - c` (no normalization) | Same as `Coordinate` (baseline) |
| AngularRate | Cross product | Cost of `angularRate.cross(other)` | Same as `msd_sim::Vector3D::cross()` |
| InertialState | Angular field access | Cost of `state.orientation.pitch()` | Inline, negligible overhead |

---

## Prototype Results Summary

Nine prototypes (P1, P1b, P1c, P1d, P1e, P2, P3, P4, P5) were executed to validate design decisions. Full results in [`prototype-results.md`](./prototype-results.md).

### Resolved Design Decisions

1. **Normalization Strategy** — **RESOLVED: Deferred with 100π threshold**

   Prototypes P1-P1e explored lazy, eager, and deferred normalization strategies:

   | Strategy | Long Sim (1000 steps) | Decision |
   |----------|----------------------|----------|
   | Eager Always | 11,837 ns | ❌ Too slow |
   | Lazy (on access) | 7.91 ns per accessor | ❌ Slow reads |
   | **Deferred (100π threshold)** | **1,129 ns** | ✅ **10x faster** |

   **Decision**: Deferred normalization with 100π threshold, checks in all modifying operations.

2. **Internal Storage** — **RESOLVED: Raw double (msd_sim::Vector3D)**

   Prototype P2 compared Angle objects vs raw double:

   | Storage | Arithmetic | Memory |
   |---------|-----------|--------|
   | Angle objects | 29.5 ns | 48 bytes |
   | **Raw double** | **0.68 ns** | **24 bytes** |

   **Decision**: Inherit from msd_sim::Vector3D (43x faster, 50% smaller).

3. **Shared Interface Pattern** — **RESOLVED: Explicit duplication**

   Prototype P5 compared templates, CRTP, virtual, and duplication:

   | Approach | Memory | Accessor | Complexity |
   |----------|--------|----------|------------|
   | **No inheritance** | **24 B** | **8.59 ns** | Simple |
   | Virtual | 32 B | 10.3 ns | Simple |

   **Decision**: Explicit duplication — simplest, fastest, no memory overhead.

4. **Eigen Integration** — **RESOLVED: SIMD preserved**

   Prototype P4 confirmed zero overhead when inheriting from msd_sim::Vector3D.

5. **Normalization Coverage** — **RESOLVED: Override compound operators**

   Prototype P1e showed that overriding `+=`, `-=`, `*=`, `/=` provides complete coverage.

   **Accepted Limitation**: Direct `operator[]` access bypasses normalization. This is documented and acceptable since semantic accessors are the intended API.

### Prototype Artifacts

See `prototypes/0024_angular_coordinate/` for all benchmark code.

---

## Performance Considerations

### Critical Performance Requirements

These classes will be called **millions of times per frame** in physics simulation loops. Performance is NOT optional — it is a hard requirement.

### Expected Hot Paths

1. **Physics integration loop**:
   ```cpp
   // Called every frame for every dynamic object
   AngularRate deltaOrientation = angularVelocity * deltaTime;
   state.orientation = state.orientation + deltaOrientation.toAngularCoordinate();
   ```

2. **Torque calculation**:
   ```cpp
   // Called for every force application
   AngularRate torque = offsetPosition.cross(force);
   state.angularAcceleration = inverseMomentOfInertia * torque;
   ```

3. **Orientation queries**:
   ```cpp
   // Called for rendering, collision detection
   double currentYaw = state.orientation.yaw();  // Normalization happens here
   ```

### Performance Constraints (Validated by Prototypes)

| Operation | Target | Actual (Prototype) | Status |
|-----------|--------|-------------------|--------|
| `AngularCoordinate::pitch()` accessor | < 10% overhead vs raw | **0%** (0.69 ns) | ✅ PASS |
| `AngularCoordinate::operator+=` | Reasonable overhead | **3x** (3.0 ns vs 0.9 ns) | ✅ PASS |
| `AngularRate::pitch()` accessor | 0% overhead | **0%** (0.69 ns) | ✅ PASS |
| Long simulation (1000 steps) | < 5x vs Never | **1.9x** (999 ns vs 524 ns) | ✅ PASS |
| Eigen SIMD preservation | 0% overhead | **0%** | ✅ PASS |
| Memory footprint | 24 bytes | **24 bytes** | ✅ PASS |

### Prototype Validation Complete

All performance constraints validated by prototypes P1-P1e and P4. The deferred normalization strategy with 100π threshold achieves **10x speedup** over eager-always while maintaining correctness.

---

## Migration Strategy

### Single-Phase Clean Replacement (This Ticket)
1. Implement `AngularCoordinate` and `AngularRate` classes
2. **DELETE** `EulerAngles.hpp` and `EulerAngles.cpp`
3. Update `ReferenceFrame` to use `AngularCoordinate` (replace `EulerAngles euler_` with `AngularCoordinate angular_`)
4. Update `InertialState` field types to new classes
5. Update ALL code using `EulerAngles` or `InertialState` angular fields
6. Update all test files
7. Fix all compilation errors from accessor changes

**Rationale**: Human requested consolidation of testing effort into this single ticket rather than maintaining parallel implementations during deprecation

---

## Coding Standards Applied

### Initialization
- Brace initialization `{}` used throughout
- Default constructor initializes to `(0.0, 0.0, 0.0)` (not NaN, as these are rotational coordinates with valid zero)

### Rule of Zero
- Both classes use `= default` for all special member functions
- Compiler-generated implementations are correct (value semantics)

### Memory Management
- Pure value types — no dynamic allocation
- Inherit from `msd_sim::Vector3D` (24 bytes, stack-allocated)

### Naming
- Classes: `PascalCase` (`AngularCoordinate`, `AngularRate`)
- Methods: `camelCase` (`pitch()`, `pitchDeg()`, `toCoordinate()`)
- No member variables (inherited from `msd_sim::Vector3D`)

### Return Values
- Prefer returning values over output parameters
- Conversions return new instances, not references

---

## Alternatives Considered

### Alternative 1: Single `AngularVector` class with normalization flag

```cpp
class AngularVector {
public:
  AngularVector(bool normalize = true);
  double pitch() const;  // Normalizes if flag is true
private:
  msd_sim::Vector3D data_;
  bool shouldNormalize_;
};
```

**Rejected because**:
- No type safety — can accidentally assign rates to orientation
- Runtime overhead of checking flag on every access
- Unclear API — is this an orientation or a rate?

### Alternative 2: Keep `EulerAngles`, add Eigen operations

Extend `EulerAngles` with cross product, dot product, etc. instead of creating new classes.

**Rejected because**:
- `EulerAngles` has 3× `Angle` objects (48 bytes) — twice the memory
- Cannot inherit from `msd_sim::Vector3D` (already has 3 members)
- Doesn't solve the angular rate representation problem

### Alternative 3: Use `Coordinate` for everything, manual normalization

Keep using `Coordinate` for all angular quantities, normalize manually when needed.

**Rejected because**:
- No type safety
- No semantic accessors (must remember `x()` = pitch)
- Easy to forget normalization, leading to bugs

---

## Risks and Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Lazy normalization too slow | High | Medium | **P1 prototype** validates performance; if too slow, switch to eager normalization or cached normalized values |
| Eigen expression templates broken | High | Low | **P4 prototype** tests expression templates; fallback to explicit conversion if needed |
| Numerical drift from infrequent normalization | Medium | Medium | **P3 prototype** validates stability; may need periodic `normalizeInPlace()` calls |
| Migration introduces bugs | Medium | Medium | Comprehensive unit tests, semantic accessors make bugs obvious (compile errors) |
| Code duplication between classes | Low | High | Acceptable trade-off for type safety; **P5 prototype** evaluates if policy template reduces this |

---

## Future Enhancements (Out of Scope)

These are NOT part of this ticket but may be addressed in future work:

1. **Quaternion representation** — More efficient for 3D rotations, no gimbal lock
2. **Rotation matrix caching in `AngularCoordinate`** — Avoid recomputing in `ReferenceFrame`
3. **SIMD-optimized normalization** — Use Eigen's vectorization for batch normalization
4. **`AngularCoordinate` derivatives** — `d/dt AngularCoordinate` → `AngularRate` with automatic conversion

---

## References

### Existing Code Patterns
- [`Coordinate.hpp`](../../../msd/msd-sim/src/Environment/Coordinate.hpp) — Eigen inheritance pattern, formatting
- [`Angle.hpp`](../../../msd/msd-sim/src/Environment/Angle.hpp) — Normalization logic
- [`EulerAngles.hpp`](../../../msd/msd-sim/src/Environment/EulerAngles.hpp) — Current orientation representation
- [`InertialState.hpp`](../../../msd/msd-sim/src/Physics/RigidBody/InertialState.hpp) — Migration target

### Coding Standards
- [Root CLAUDE.md](../../../CLAUDE.md) — Project-wide standards
- [msd-sim/CLAUDE.md](../../../msd/msd-sim/CLAUDE.md) — Simulation library standards
- [Environment/CLAUDE.md](../../../msd/msd-sim/src/Environment/CLAUDE.md) — Environment module standards

### Related Tickets
- [0023a_force_application_scaffolding](../../../tickets/0023a_force_application_scaffolding.md) — Exposed the type inconsistency issue
- [0023_force_application_system](../../../tickets/0023_force_application_system.md) — Will benefit from type-safe angular quantities

---

## Design Complexity Sanity Check

### Red Flags Evaluation

**Red Flag 1: Combinatorial Overloads** — ✅ NOT PRESENT
- Only 2 new overloads added to `ReferenceFrame` (`setRotation`, `getAngularCoordinate`)
- No combinatorial explosion

**Red Flag 2: Optional Wrappers for Legacy Paths** — ✅ NOT PRESENT
- No `std::optional` used for backward compatibility
- Both APIs coexist via function overloading, not optional members

**Red Flag 3: Modified Components Outnumber New Components** — ✅ ACCEPTABLE
- New components: 2 (`AngularCoordinate`, `AngularRate`)
- Modified components: 3 (`InertialState`, `ReferenceFrame`, `EulerAngles`)
- Modification ratio is acceptable (not dominated by backward compatibility)
- `InertialState` change is a true design improvement (type safety), not just compatibility

**Red Flag 4: Conditional Logic Explosion** — ✅ NOT PRESENT
- No `if (usesNewType)` branches in the design
- Old and new APIs are separate overloads, not conditional paths

**Verdict**: Design complexity is acceptable. The breaking change to `InertialState` is justified by type safety improvement, and the deprecation path for `ReferenceFrame` is clean (no complex conditional logic).

---

---

## Revision History

### Revision 2 (2026-01-20)
**Status**: Design finalized based on prototype results (P1-P1e, P2, P3, P4, P5)

**Changes Made**:

1. **Changed normalization strategy from lazy to deferred**:
   - Old: Lazy normalization on accessor (slow: 7.91 ns per read)
   - New: Deferred normalization with 100π threshold (fast: 0.69 ns per read)
   - Rationale: Prototype P1d showed 10x speedup over eager-always

2. **Added compound operator overrides**:
   - Override `+=`, `-=`, `*=`, `/=` to include `normalizeIfNeeded()` check
   - Rationale: Prototype P1e showed this provides complete coverage for compound operations

3. **Documented accepted limitation**:
   - Direct `operator[]` access bypasses normalization
   - Rationale: Acceptable trade-off; semantic accessors are the intended API

4. **Updated performance constraints with actual measurements**:
   - All targets validated by prototypes
   - Long simulation: 1.9x overhead (target was < 5x)
   - Accessor: 0% overhead (target was < 10%)

5. **Resolved all open questions with prototype data**:
   - Normalization: Deferred with 100π threshold
   - Storage: Raw double (msd_sim::Vector3D)
   - Interface: Explicit duplication
   - SIMD: Preserved (0% overhead)

---

### Revision 1 (2026-01-20)
**Status**: Approved revisions applied based on human feedback

**Changes Made**:

1. **I1 - Removed Coordinate Conversions**:
   - Removed `toCoordinate()` and `fromCoordinate()` from `AngularCoordinate`
   - Removed `toCoordinate()` and `fromCoordinate()` from `AngularRate`
   - Removed from Integration Points table
   - Removed from test plan
   - Removed from acceptance criteria

2. **I2 - Removed Raw Accessors**:
   - Removed `pitchRaw()`, `rollRaw()`, `yawRaw()` from `AngularCoordinate` interface
   - Removed from test plan
   - Removed from acceptance criteria
   - Only normalized `pitch()`, `roll()`, `yaw()` remain

3. **I3 - Removed Cross-Type Conversions**:
   - Removed `AngularCoordinate::toRate()` method
   - Removed `AngularRate::toAngularCoordinate()` method
   - Removed from Integration Points table
   - Removed from test plan
   - Removed from acceptance criteria

4. **I4 - Removed EulerAngles Entirely**:
   - Changed from "deprecation path" to "clean removal"
   - Updated ReferenceFrame section: removed deprecated methods, changed internal storage to `AngularCoordinate`
   - Updated EulerAngles section: now marked as REMOVED with migration notes
   - Removed EulerAngles from Integration Points table
   - Removed EulerAngles conversion tests from test plan
   - Updated Migration Strategy to single-phase clean replacement
   - PlantUML diagram will be updated to remove EulerAngles

**Rationale**: Human approved automatic revisions to simplify API, remove semantic inconsistencies, and consolidate testing effort into a single clean implementation.

---

## Design Review — Initial Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-01-20
**Status**: REVISION_REQUESTED → REVISIONS_APPLIED
**Iteration**: 0 of 1

### Issues Requiring Revision (RESOLVED)

All four issues (I1, I2, I3, I4) have been addressed in Revision 1.

### Items Passing Review (No Changes Needed)

The following design elements are sound and should NOT be modified:

1. **Two-Class Design**: The separation of `AngularCoordinate` (normalizing) and `AngularRate` (non-normalizing) is excellent type safety
2. **Eigen Inheritance**: Inheriting from `msd_sim::Vector3D` is the right approach for performance
3. **Lazy Normalization**: Normalizing on access for `AngularCoordinate` is appropriate
4. **Semantic Accessors**: `pitch()`, `roll()`, `yaw()` provide clear semantic meaning
5. **InertialState Migration**: Breaking change is justified by type safety improvement
6. **Prototype Requirements**: P1-P5 prototypes are comprehensive and necessary
7. **Memory Layout**: 24-byte footprint matches `msd_sim::Vector3D` baseline
8. **Format Support**: `std::format` integration following `Coordinate` pattern is correct
9. **Rule of Zero**: Using `= default` for all special member functions is correct

---


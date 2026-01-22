# Feature Ticket: AngularCoordinate and AngularRate

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-20
- **Author**: Human + AI
- **Priority**: Medium
- **Estimated Complexity**: Large (prototyping required)
- **Target Component(s)**: msd-sim (Environment)

---

## Summary
Create two complementary classes for angular quantity representation:
1. **`AngularCoordinate`** — For orientation angles with automatic normalization (replacing `EulerAngles`)
2. **`AngularRate`** — For angular velocity/acceleration without normalization (replacing `Coordinate` usage)

Both classes wrap `Eigen::Vector3d` with semantic pitch/roll/yaw accessors while providing full Eigen matrix operations. This type-safe approach prevents accidental misuse (e.g., assigning a rate to an orientation) while maintaining the distinct semantic requirements of each quantity type.

## Motivation
Ticket 0023a_force_application_scaffolding exposed a fundamental design issue with angular quantity representation:

1. **Type Inconsistency**: `InertialState` now uses different types for angular quantities:
   - `orientation`: `EulerAngles` (struct of three `Angle` objects)
   - `angularVelocity`: `Coordinate` (wraps `Eigen::Vector3d`)
   - `angularAcceleration`: `Coordinate` (wraps `Eigen::Vector3d`)

2. **Semantic Mismatch**: Using `Coordinate` for angular quantities loses semantic meaning - `x()`, `y()`, `z()` don't convey that these represent pitch, roll, yaw components.

3. **Missing Operations**: `EulerAngles` doesn't support vector operations (cross product, dot product, scalar multiplication) needed for physics calculations like torque computation (`r.cross(F)`).

4. **Normalization Semantics Differ**:
   - **Orientation**: Angles SHOULD be normalized — 370° and 10° represent the same orientation
   - **Angular rates**: SHOULD NOT be normalized — 720°/s is a valid rate, distinct from 0°/s

The two-class approach solves these issues by:
- Providing type-safe separation of orientation vs. rate quantities
- Offering semantic pitch/roll/yaw accessors on both types
- Inheriting full Eigen vector math capabilities
- Automatic normalization for `AngularCoordinate` (orientation)
- No normalization for `AngularRate` (velocities/accelerations)

## Requirements

### Functional Requirements

#### AngularCoordinate (Orientation)
1. The system shall provide an `AngularCoordinate` class that inherits from `Eigen::Vector3d`
2. The system shall provide semantic accessors: `pitch()`, `roll()`, `yaw()` (read/write)
3. The system shall support all Eigen vector operations (arithmetic, cross, dot, norm, etc.)
4. The system shall support construction from three doubles, from `Eigen::Vector3d`, and from Eigen expressions
5. The system shall provide automatic normalization to (-π, π] on access (lazy, like `Angle`)
6. The system shall provide helper methods matching `Angle`: `getRad()`, `toDeg()` for each component
7. The system shall provide `std::format` support matching `Coordinate`'s formatter style

#### AngularRate (Velocity/Acceleration)
8. The system shall provide an `AngularRate` class that inherits from `Eigen::Vector3d`
9. The system shall provide semantic accessors: `pitch()`, `roll()`, `yaw()` (read/write)
10. The system shall support all Eigen vector operations without any normalization
11. The system shall support construction from three doubles, from `Eigen::Vector3d`, and from Eigen expressions
12. The system shall provide `std::format` support matching `Coordinate`'s formatter style

#### Interoperability
13. The system shall provide conversion methods between `AngularCoordinate`, `AngularRate`, and `Coordinate`
14. The system shall replace `EulerAngles` in `InertialState` with `AngularCoordinate` for orientation
15. The system shall replace `Coordinate` in `InertialState` with `AngularRate` for angular velocity/acceleration
16. The system shall update `ReferenceFrame::setRotation()` to accept `AngularCoordinate`

### Non-Functional Requirements
- **Performance**: Prototype required to validate overhead (see Prototype Requirements below)
- **Memory**: Same memory footprint as `Eigen::Vector3d` (24 bytes) for both classes
- **Thread Safety**: Value types - safe to copy across threads
- **Backward Compatibility**: Breaking change - `EulerAngles` will be deprecated/removed
- **Numerical Stability**: Normalization strategy must prevent drift in long-running simulations

## Constraints
- Must inherit from `Eigen::Vector3d` to leverage Eigen's SIMD optimizations
- Must follow the same pattern as `Coordinate` for consistency
- `AngularRate` must not introduce any normalization overhead
- `AngularCoordinate` uses deferred normalization (100π threshold, check in modifying operations)

## Prototype Requirements

Given the fundamental nature of angular math in this project, prototypes are required to validate:

### P1: Normalization Performance
- Compare normalization strategies: always, lazy (on access), never
- Measure overhead per operation (add, multiply, cross product)
- Establish acceptable performance threshold

### P2: Internal Storage Comparison
- Compare `Angle` objects vs raw `double` for `AngularCoordinate` internal storage
- Measure memory and performance impact
- Assess code complexity trade-offs

### P3: Numerical Stability
- Run long-duration simulation (millions of time steps)
- Compare angle drift with different normalization frequencies
- Determine optimal normalization strategy for stability

### P4: Eigen Integration
- Verify SIMD optimizations are preserved when inheriting from `Eigen::Vector3d`
- Benchmark against raw `Eigen::Vector3d` operations
- Test cross product, dot product, matrix multiplication performance

### P5: Shared Interface Pattern
- Compare implementation approaches for `AngularCoordinate` and `AngularRate`:
  - **Policy-based template**: `AngularVector<NormalizationPolicy>`
  - **CRTP base class**: `AngularBase<Derived>`
  - **Simple inheritance**: `AngularBase` → derived classes
  - **No inheritance**: Duplicate code in each class
- Evaluate criteria:
  - Eigen expression template compatibility
  - Compile-time overhead
  - Runtime performance
  - Code readability and maintainability
  - Debugger friendliness

## Acceptance Criteria

### AngularCoordinate Class (Orientation)
- [ ] `AngularCoordinate` class inherits from `Eigen::Vector3d`
- [ ] Default constructor initializes to (0, 0, 0)
- [ ] Constructor accepts (pitch, roll, yaw) as doubles (radians)
- [ ] Constructor accepts `Eigen::Vector3d`
- [ ] Template constructor accepts Eigen expressions
- [ ] Template assignment operator accepts Eigen expressions

### AngularCoordinate Semantic Accessors
- [ ] `pitch()` returns value normalized to (-π, π] (Y-axis rotation)
- [ ] `roll()` returns value normalized to (-π, π] (X-axis rotation)
- [ ] `yaw()` returns value normalized to (-π, π] (Z-axis rotation)
- [ ] `pitchDeg()`, `rollDeg()`, `yawDeg()` return degrees
- [ ] `setPitch()`, `setRoll()`, `setYaw()` for direct assignment

### AngularRate Class (Velocity/Acceleration)
- [ ] `AngularRate` class inherits from `Eigen::Vector3d`
- [ ] Default constructor initializes to (0, 0, 0)
- [ ] Constructor accepts (pitch, roll, yaw) as doubles (rad/s or rad/s²)
- [ ] Constructor accepts `Eigen::Vector3d`
- [ ] Template constructor accepts Eigen expressions
- [ ] Template assignment operator accepts Eigen expressions

### AngularRate Semantic Accessors
- [ ] `pitch()` returns reference (no normalization)
- [ ] `roll()` returns reference (no normalization)
- [ ] `yaw()` returns reference (no normalization)
- [ ] Const versions of all accessors provided
- [ ] Accessors work correctly with Eigen operations

### Interoperability
- [ ] Implicit conversion to `Eigen::Vector3d` (inherited)
- [ ] Can construct from `Eigen::Vector3d` and Eigen expressions

### Formatting
- [ ] `std::format` support for both classes with same syntax as `Coordinate`
- [ ] Default format: `(pitch, roll, yaw)` with 6 decimal places
- [ ] Custom precision/width/presentation supported

### InertialState Migration
- [ ] `InertialState::orientation` changed from `EulerAngles` to `AngularCoordinate`
- [ ] `InertialState::angularVelocity` changed from `Coordinate` to `AngularRate`
- [ ] `InertialState::angularAcceleration` changed from `Coordinate` to `AngularRate`
- [ ] All code using `InertialState` angular fields compiles and works

### ReferenceFrame Migration
- [ ] `ReferenceFrame::setRotation(const AngularCoordinate&)` implemented
- [ ] `ReferenceFrame::getAngularCoordinate()` returns `AngularCoordinate`
- [ ] Internal storage changed from `EulerAngles euler_` to `AngularCoordinate angular_`
- [ ] Constructor accepts `AngularCoordinate` instead of `EulerAngles`

### EulerAngles Removal
- [ ] `EulerAngles.hpp` deleted
- [ ] `EulerAngles.cpp` deleted
- [ ] All uses of `EulerAngles` replaced with `AngularCoordinate`
- [ ] All test files updated

### Tests
- [ ] Unit tests for `AngularCoordinate` constructors
- [ ] Unit tests for `AngularCoordinate` normalization behavior
- [ ] Unit tests for `AngularCoordinate` degree/radian accessors
- [ ] Unit tests for `AngularRate` constructors
- [ ] Unit tests for `AngularRate` non-normalizing behavior
- [ ] Unit tests for Eigen operations (cross, dot, arithmetic) on both
- [ ] Unit tests for format support on both
- [ ] Unit tests for interoperability conversions
- [ ] Existing tests compile and pass after InertialState migration

---

## Design Decisions (Human Input)

### Preferred Approaches
- Follow `Coordinate` class pattern for consistency
- Use pitch/roll/yaw order matching aerospace convention (ZYX rotation)
- Make normalization explicit - angular rates should never be auto-normalized 
- Keep the class lightweight - no caching, no lazy evaluation

### Things to Avoid
- Don't use a single class for both orientation and rates — different normalization requirements
- Don't add rotation matrix conversion methods here — keep in `ReferenceFrame`
- Don't remove `EulerAngles` immediately — deprecate first, remove in follow-up ticket
- Don't optimize prematurely — prototype first to establish baseline performance

### Resolved Questions
1. ~~Should `AngularCoordinate` be in its own header or combined with `Coordinate`?~~ **Resolved**: Own header (`AngularCoordinate.hpp`, `AngularRate.hpp`)
2. ~~Should we provide a `toRotationMatrix()` method, or keep that separate?~~ **Resolved**: Provided by `ReferenceFrame` class
3. ~~What happens to `ReferenceFrame::setRotation(const EulerAngles&)`?~~ **Resolved**: Full migration to `AngularCoordinate`

### Open Questions (Requiring Design/Prototype Input)
4. **Human Question**: Should we split `AngularCoordinate` and `AngularRate` into two separate classes?

   **AI Response**: Yes, the two-class design is recommended. Key benefits:
   - **Type Safety**: Prevents accidental assignment of rates to orientations (compile-time error)
   - **Semantic Clarity**: Clear intent — `AngularCoordinate` IS an angle, `AngularRate` is a rate vector
   - **Correct Behavior**: Auto-normalization for orientation, no normalization for rates
   - **API Compatibility**: `AngularCoordinate` can preserve `EulerAngles`-like interface (getRad, toDeg)
   - **Minimal Overhead**: Both inherit from `Eigen::Vector3d`, same memory footprint

   The ticket has been updated to reflect this two-class design.

5. **Human Question**: Is a prototype necessary for this change?

   **AI Response**: Yes, prototyping is strongly recommended. This is foundational code that will be called millions of times in simulation. Key areas to prototype:

   | Prototype | Purpose | Key Questions |
   |-----------|---------|---------------|
   | **P1: Normalization Performance** | Measure overhead | Is lazy normalization fast enough? How often does it trigger? |
   | **P2: Internal Storage** | `Angle` vs `double` | Does `Angle` add significant overhead? Is the complexity worth it? |
   | **P3: Numerical Stability** | Long-run drift | How does angle accumulation behave over millions of steps? |
   | **P4: Eigen Integration** | SIMD preservation | Does inheritance break Eigen's optimizations? |

   **Trade-offs of normalizing frequently:**
   - **Pro**: Prevents numerical overflow, maintains numerical precision
   - **Pro**: Ensures angles always in expected range for trig functions
   - **Con**: `fmod` is relatively expensive (~10-20 cycles vs ~1 for addition)
   - **Recommendation**: Lazy normalization (on access) balances both concerns

   **Trade-offs of `Angle` vs `double` internally:**
   - **`Angle` internal storage**: 16 bytes per component (double + enum), automatic normalization mode tracking
   - **`double` internal storage**: 8 bytes per component, must manually normalize
   - **Recommendation**: Prototype both approaches and measure; expect `double` to win on performance

6. **Human Question**: Should we use inheritance or CRTP to share the interface between `AngularCoordinate` and `AngularRate`?

   **AI Response**: This is worth prototyping. Options ranked by likely suitability:

   | Approach | Pros | Cons | Recommendation |
   |----------|------|------|----------------|
   | **Policy-based template** | Single implementation, zero-overhead, distinct types | Template complexity | **Prototype first** |
   | **CRTP** | Familiar pattern, customization points | Eigen uses CRTP internally - potential conflict | Prototype |
   | **Simple inheritance** | Easy to understand | Extra vtable potential, Eigen compat unclear | Lower priority |
   | **No inheritance** | Simple, guaranteed Eigen compat | Code duplication | Fallback if others fail |

   Key concern: Eigen heavily uses CRTP and expression templates. Any inheritance pattern must preserve:
   - Expression template optimization (lazy evaluation)
   - SIMD vectorization
   - Return type deduction for operations like `cross()`, `+`, `*`

   **Added prototype P5** to evaluate these approaches.

---

## Proposed API Design

### AngularCoordinate Class (Orientation with Auto-Normalization)
```cpp
namespace msd_sim {

/**
 * @brief Orientation angles with automatic normalization to (-π, π]
 *
 * Inherits from Eigen::Vector3d for full matrix operation support.
 * Provides semantic pitch/roll/yaw accessors with lazy normalization.
 *
 * Axis convention (ZYX intrinsic rotation):
 * - pitch: Rotation around Y-axis (component 0)
 * - roll:  Rotation around X-axis (component 1)
 * - yaw:   Rotation around Z-axis (component 2)
 */
class AngularCoordinate : public Eigen::Vector3d {
public:
  // Default constructor - initializes to (0, 0, 0)
  AngularCoordinate() : Eigen::Vector3d{0.0, 0.0, 0.0} {}

  // Constructor with pitch, roll, yaw values (in radians)
  AngularCoordinate(double pitch, double roll, double yaw)
    : Eigen::Vector3d{pitch, roll, yaw} {}

  // Constructor from Eigen::Vector3d
  AngularCoordinate(const Eigen::Vector3d& vec) : Eigen::Vector3d{vec} {}

  // Template constructor for Eigen expressions
  template <typename OtherDerived>
  AngularCoordinate(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other} {}

  // Template assignment for Eigen expressions
  template <typename OtherDerived>
  AngularCoordinate& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
    this->Eigen::Vector3d::operator=(other);
    return *this;
  }

  // Normalized accessors (lazy normalization to (-π, π])
  double pitch() const;  // Returns normalized value
  double roll() const;   // Returns normalized value
  double yaw() const;    // Returns normalized value

  // Degree accessors (convenience)
  double pitchDeg() const;
  double rollDeg() const;
  double yawDeg() const;

  // Raw accessors (no normalization, for Eigen interop)
  double& pitchRaw() { return (*this)[0]; }
  double& rollRaw() { return (*this)[1]; }
  double& yawRaw() { return (*this)[2]; }
  double pitchRaw() const { return (*this)[0]; }
  double rollRaw() const { return (*this)[1]; }
  double yawRaw() const { return (*this)[2]; }

  // Setters
  void setPitch(double radians) { (*this)[0] = radians; }
  void setRoll(double radians) { (*this)[1] = radians; }
  void setYaw(double radians) { (*this)[2] = radians; }

  // Explicit full normalization
  AngularCoordinate normalized() const;
  void normalizeInPlace();

  // Conversions
  Coordinate toCoordinate() const;
  static AngularCoordinate fromCoordinate(const Coordinate& coord);
  AngularRate toRate() const;  // For computing derivatives

private:
  static double normalizeAngle(double rad);  // (-π, π]
};

}  // namespace msd_sim
```

### AngularRate Class (Velocity/Acceleration without Normalization)
```cpp
namespace msd_sim {

/**
 * @brief Angular rate vector (velocity or acceleration) without normalization
 *
 * Inherits from Eigen::Vector3d for full matrix operation support.
 * Provides semantic pitch/roll/yaw accessors without any normalization.
 * Rates can exceed 2π rad/s and should not be normalized.
 *
 * Units:
 * - Angular velocity: rad/s
 * - Angular acceleration: rad/s²
 */
class AngularRate : public Eigen::Vector3d {
public:
  // Default constructor - initializes to (0, 0, 0)
  AngularRate() : Eigen::Vector3d{0.0, 0.0, 0.0} {}

  // Constructor with pitch, roll, yaw rates
  AngularRate(double pitchRate, double rollRate, double yawRate)
    : Eigen::Vector3d{pitchRate, rollRate, yawRate} {}

  // Constructor from Eigen::Vector3d
  AngularRate(const Eigen::Vector3d& vec) : Eigen::Vector3d{vec} {}

  // Template constructor for Eigen expressions
  template <typename OtherDerived>
  AngularRate(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other} {}

  // Template assignment for Eigen expressions
  template <typename OtherDerived>
  AngularRate& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
    this->Eigen::Vector3d::operator=(other);
    return *this;
  }

  // Semantic accessors (no normalization)
  double& pitch() { return (*this)[0]; }
  double& roll() { return (*this)[1]; }
  double& yaw() { return (*this)[2]; }
  double pitch() const { return (*this)[0]; }
  double roll() const { return (*this)[1]; }
  double yaw() const { return (*this)[2]; }

  // Conversions
  Coordinate toCoordinate() const;
  static AngularRate fromCoordinate(const Coordinate& coord);
  AngularCoordinate toAngularCoordinate() const;  // For integration results
};

}  // namespace msd_sim
```

### InertialState Migration
```cpp
struct InertialState {
  // Linear (unchanged)
  Coordinate position;
  Coordinate velocity;
  Coordinate acceleration;

  // Angular (type-safe separation)
  AngularCoordinate orientation;       // Was EulerAngles (auto-normalizing)
  AngularRate angularVelocity;         // Was Coordinate (no normalization)
  AngularRate angularAcceleration;     // Was Coordinate (no normalization)
};
```

### Alternative: Policy-Based Template Design
```cpp
namespace msd_sim {

// Normalization policies
struct AutoNormalize {
  static constexpr bool normalizes = true;
  static double apply(double rad) {
    // Normalize to (-π, π]
    double result = std::fmod(rad + M_PI, 2 * M_PI);
    return result <= 0 ? result + M_PI : result - M_PI;
  }
};

struct NoNormalize {
  static constexpr bool normalizes = false;
  static double apply(double rad) { return rad; }
};

/**
 * @brief Template base for angular vectors with configurable normalization
 */
template <typename NormalizationPolicy>
class AngularVector : public Eigen::Vector3d {
public:
  using Policy = NormalizationPolicy;

  // Default constructor
  AngularVector() : Eigen::Vector3d{0.0, 0.0, 0.0} {}

  // Constructor with pitch, roll, yaw
  AngularVector(double pitch, double roll, double yaw)
    : Eigen::Vector3d{pitch, roll, yaw} {}

  // Constructor from Eigen::Vector3d
  AngularVector(const Eigen::Vector3d& vec) : Eigen::Vector3d{vec} {}

  // Template constructor for Eigen expressions
  template <typename OtherDerived>
  AngularVector(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other} {}

  // Template assignment for Eigen expressions
  template <typename OtherDerived>
  AngularVector& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
    this->Eigen::Vector3d::operator=(other);
    return *this;
  }

  // Semantic accessors (policy-controlled normalization)
  double pitch() const { return Policy::apply((*this)[0]); }
  double roll() const { return Policy::apply((*this)[1]); }
  double yaw() const { return Policy::apply((*this)[2]); }

  // Raw accessors (always unnormalized)
  double& pitchRaw() { return (*this)[0]; }
  double& rollRaw() { return (*this)[1]; }
  double& yawRaw() { return (*this)[2]; }

  // Setters
  void setPitch(double rad) { (*this)[0] = rad; }
  void setRoll(double rad) { (*this)[1] = rad; }
  void setYaw(double rad) { (*this)[2] = rad; }

  // Conversions
  Coordinate toCoordinate() const {
    return Coordinate{pitch(), roll(), yaw()};
  }
};

// Type aliases for clarity
using AngularCoordinate = AngularVector<AutoNormalize>;
using AngularRate = AngularVector<NoNormalize>;

}  // namespace msd_sim
```

**Note**: The prototype phase (P5) will determine if this policy-based approach or the explicit two-class approach is preferable based on Eigen compatibility and performance testing.

### Usage Examples
```cpp
// Orientation (auto-normalizing)
AngularCoordinate orientation{0.0, 0.0, M_PI/4};  // 45° yaw
orientation.setYaw(3 * M_PI);  // Store large value
double yaw = orientation.yaw();  // Returns π (normalized)
double yawDeg = orientation.yawDeg();  // Returns 180.0

// Angular rates (no normalization)
AngularRate angularVelocity{0.0, 0.0, 4 * M_PI};  // 720°/s yaw rate
double yawRate = angularVelocity.yaw();  // Returns 4π (not normalized!)

// Eigen operations work on both
AngularRate torque = r.cross(force);  // Cross product returns AngularRate
double magnitude = angularVelocity.norm();
AngularRate scaled = angularVelocity * 2.0;

// Type-safe physics integration
AngularRate deltaOrientation = angularVelocity * dt;
orientation = orientation + deltaOrientation.toAngularCoordinate();

// Conversions for legacy code
Coordinate asCoord = angularVelocity.toCoordinate();

// Format support (same for both)
std::cout << std::format("Orientation: {:.2f}", orientation);
// Output: Orientation: (0.00, 0.00, 0.79)
```

---

## References

### Related Code
- `msd/msd-sim/src/Environment/Coordinate.hpp` — Pattern to follow for Eigen inheritance
- `msd/msd-sim/src/Environment/Angle.hpp` — Normalization logic to reference
- `msd/msd-sim/src/Environment/EulerAngles.hpp` — To be deprecated/replaced
- `msd/msd-sim/src/Physics/RigidBody/InertialState.hpp` — Migration target
- `msd/msd-sim/src/Environment/ReferenceFrame.hpp` — Uses EulerAngles, needs migration

### New Files to Create
- `msd/msd-sim/src/Environment/AngularCoordinate.hpp` — Orientation class
- `msd/msd-sim/src/Environment/AngularRate.hpp` — Rate class
- `msd/msd-sim/test/Environment/AngularCoordinateTest.cpp` — Unit tests
- `msd/msd-sim/test/Environment/AngularRateTest.cpp` — Unit tests

### Related Tickets
- `0023a_force_application_scaffolding` — Exposed the type inconsistency issue
- `0023_force_application_system` — Will benefit from unified angular types

---

## Workflow Log

### Design Phase
- **Started**: 2026-01-20
- **Completed**: 2026-01-20
- **Artifacts**:
  - `docs/designs/0024_angular_coordinate/design.md`
  - `docs/designs/0024_angular_coordinate/0024_angular_coordinate.puml`
- **Notes**:
  - Designed two-class approach: `AngularCoordinate` (auto-normalizing) and `AngularRate` (non-normalizing)
  - Both inherit from `Eigen::Vector3d` for full matrix operation support
  - Lazy normalization strategy for `AngularCoordinate` (on access via `pitch()`, `roll()`, `yaw()`)
  - Raw `double` internal storage (24 bytes, same as `Eigen::Vector3d`)
  - Breaking change to `InertialState` for type safety
  - Deprecation path for `EulerAngles` and `ReferenceFrame` methods
  - Five prototypes (P1-P5) required to validate performance and design approach

### Design Review Phase
- **Started**: 2026-01-20
- **Completed**: 2026-01-20
- **Status**: APPROVED
- **Reviewer Notes**:
  - **Initial Review**: Identified 4 issues requiring revision based on human feedback
    - (I1) Remove Coordinate conversions
    - (I2) Remove raw accessors
    - (I3) Remove AngularCoordinate ↔ AngularRate conversions
    - (I4) Remove EulerAngles entirely NOW
  - **Revisions**: Human approved automatic revisions, all applied to design.md and 0024_angular_coordinate.puml
  - **Final Review**: All issues (I1-I4) verified as properly addressed
  - **Quality Assessment**: Design passes all criteria
    - Architectural Fit: ✅ Type-safe separation, follows Coordinate pattern
    - C++ Code Quality: ✅ Rule of Zero, 24-byte footprint, lazy normalization
    - Feasibility: ✅ Prototypes P1-P5 defined to validate performance/stability
    - Testability: ✅ Comprehensive test plan including benchmarks
    - PlantUML Accuracy: ✅ Diagram correctly reflects simplified design
  - **Outcome**: Design approved for prototype phase

### Prototype Phase
- **Started**: 2026-01-20
- **Completed**: 2026-01-20
- **Prototypes**:
  - P1: Normalization Performance — ❌ FAILED (lazy: 3029% overhead, eager: 1.5% overhead ✅)
  - P2: Internal Storage Comparison — ✅ PASSED (raw double recommended)
  - P3: Numerical Stability — ✅ PASSED (lazy normalization keeps angles bounded)
  - P4: Eigen Integration — ✅ PASSED (SIMD preserved, -2.7% overhead)
  - P5: Shared Interface Pattern — ✅ PASSED (no inheritance recommended)
- **Artifacts**:
  - `docs/designs/0024_angular_coordinate/prototype-results.md`
  - `prototypes/0024_angular_coordinate/p1_normalization_performance.cpp`
  - `prototypes/0024_angular_coordinate/p2_internal_storage.cpp`
  - `prototypes/0024_angular_coordinate/p3_numerical_stability.cpp`
  - `prototypes/0024_angular_coordinate/p4_eigen_integration.cpp`
  - `prototypes/0024_angular_coordinate/p5_shared_interface.cpp`
  - `prototypes/0024_angular_coordinate/CMakeLists.txt`
- **Notes**:
  - **CRITICAL FINDING**: Lazy normalization FAILED performance target (3029% overhead vs < 10% target)
  - **DESIGN CHANGE REQUIRED**: Must use **EAGER normalization** (normalize on construction/assignment, not on access)
  - Accessor methods (`pitch()`, `roll()`, `yaw()`) should return raw values (already normalized)
  - Constructor and assignment operator must normalize values in-place
  - All other design decisions validated: raw double storage, no inheritance pattern, Eigen integration works perfectly
  - Numerical stability confirmed: normalization keeps angles bounded over 10M time steps
  - Recommendation: Use eager normalization + raw double storage + no inheritance pattern

### Implementation Phase
- **Started**: 2026-01-21
- **Completed**: 2026-01-21
- **Files Created**:
  - `msd/msd-sim/src/Environment/AngularCoordinate.hpp`
  - `msd/msd-sim/src/Environment/AngularRate.hpp`
  - `msd/msd-sim/test/Environment/AngularCoordinateTest.cpp`
  - `msd/msd-sim/test/Environment/AngularRateTest.cpp`
- **Files Modified**:
  - `msd/msd-sim/src/Physics/RigidBody/InertialState.hpp` — Migrated to AngularCoordinate and AngularRate
  - `msd/msd-sim/src/Environment/WorldModel.hpp` — Updated for new types
  - `msd/msd-sim/src/Environment/WorldModel.cpp` — Updated for new types
  - `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp` — Updated for new types
  - `msd/msd-sim/src/Physics/RigidBody/AssetInertial.cpp` — Updated for new types
  - All test files migrated and passing
- **Files Deleted**:
  - `msd/msd-sim/src/Environment/EulerAngles.hpp` — Removed as planned
  - `msd/msd-sim/src/Environment/EulerAngles.cpp` — Removed as planned
- **Artifacts**:
  - `docs/designs/0024_angular_coordinate/implementation-notes.md`
- **Notes**:
  - **DESIGN CHANGE**: Used **DEFERRED normalization** (check on assign, not on access) per prototype P1 findings
  - AngularCoordinate normalizes only when |value| > 100π (kNormalizationThreshold)
  - Normalization applied in constructors, assignment operators, and compound operators (+=, -=, *=, /=)
  - Accessors (pitch(), roll(), yaw()) return raw values (fast, no normalization overhead)
  - Explicit normalize() and normalized() methods available for manual normalization
  - All 214 tests passing, including normalization edge cases
  - EulerAngles fully removed from codebase

### Quality Gate Phase (Attempt 1)
- **Started**: 2026-01-21 16:45
- **Completed**: 2026-01-21 16:45
- **Status**: FAILED
- **Report**: `docs/designs/0024_angular_coordinate/quality-gate-report.md` (iteration 1)
- **Failures**:
  1. Unused parameter 'worldPoint' in `AssetInertial.cpp:73`
  2. Unused parameter 'dt' in `WorldModel.cpp:88`
- **Notes**:
  - Build failed with warnings-as-errors enabled
  - These are scaffolding functions from ticket 0023a_force_application_scaffolding
  - Parameters need `[[maybe_unused]]` attribute until force application is fully implemented

### Quality Gate Phase (Attempt 2)
- **Started**: 2026-01-21 17:15
- **Completed**: 2026-01-21 17:30
- **Status**: PASSED
- **Report**: `docs/designs/0024_angular_coordinate/quality-gate-report.md`
- **Fixes Applied**:
  - Fixed unused parameter warnings with `[[maybe_unused]]` attributes (already done by human)
  - Migrated benchmarks and GUI tests from EulerAngles to AngularCoordinate
  - Fixed sign conversion warnings in GJKBench.cpp
  - Removed unused function from GJKBench.cpp
- **Results**:
  - Build: PASSED (0 warnings, 0 errors)
  - Tests: PASSED (293/293 tests passing in 3.40s)
  - Benchmarks: N/A (no benchmarks specified in design)
- **Notes**:
  - Additional EulerAngles references found and migrated:
    - `msd/msd-sim/bench/GJKBench.cpp`
    - `msd/msd-gui/test/ShaderTransformTest.cpp`
    - `msd/msd-gui/test/unit/gpu_instance_manager_test.cpp`
    - `msd/msd-gui/src/SDLApp.cpp`
  - These files are not part of 0024 implementation but needed updating for complete EulerAngles removal
  - All 293 tests passing confirms successful migration

### Implementation Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Documentation Update Phase
- **Started**: 2026-01-21
- **Completed**: 2026-01-21
- **CLAUDE.md Updates**:
  - `msd/msd-sim/src/Environment/CLAUDE.md` — Added AngularCoordinate and AngularRate sections, updated InertialState and ReferenceFrame sections, removed EulerAngles section
  - `msd/msd-sim/CLAUDE.md` — Added Recent Architectural Changes entry, updated diagrams index, updated Environment module summary, updated Engine component examples
- **Diagrams Indexed**:
  - `docs/msd/msd-sim/Environment/angular-coordinate.puml` — Detailed component diagram for AngularCoordinate and AngularRate
  - `docs/msd/msd-sim/Environment/mathematical-primitives.puml` — Updated to include AngularCoordinate and AngularRate, removed EulerAngles
- **Notes**:
  - Created dedicated angular-coordinate.puml diagram from design diagram (removed highlighting for stable codebase)
  - Updated mathematical-primitives.puml to replace EulerAngles with new classes
  - Comprehensive documentation for both AngularCoordinate and AngularRate including performance characteristics from prototypes
  - Migration guides provided in InertialState, ReferenceFrame, and Recent Architectural Changes sections
  - All EulerAngles references removed from documentation
  - Documentation sync summary created at `docs/designs/0024_angular_coordinate/doc-sync-summary.md`

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}

# Prototype Results: AngularCoordinate and AngularRate

**Ticket**: `0024_angular_coordinate`
**Date**: 2026-01-20
**Prototypes Executed**: P1, P1b, P1c, P1d, P1e, P2, P3, P4, P5
**Test Platform**: macOS ARM64 (Apple M-series), Clang 17.0, Eigen 3.x (via Conan)
**Benchmark Framework**: Google Benchmark

---

## Executive Summary

Nine prototypes were executed to determine the optimal normalization strategy. Key findings:

1. **P1-P1e (Normalization Strategy)**: **Deferred normalization with large threshold** is optimal — normalize only when values exceed ±100π, with checks in all modifying operations.
2. **P2 (Internal Storage)**: Raw `double` storage (Eigen::Vector3d) is **43x faster** for arithmetic and **50% smaller** memory.
3. **P3 (Numerical Stability)**: Normalization prevents unbounded growth over long simulations.
4. **P4 (Eigen Integration)**: Inheritance from Eigen::Vector3d preserves SIMD with **zero overhead**.
5. **P5 (Shared Interface Pattern)**: **Explicit duplication** — 24-byte footprint, simplest code.

**Final Recommendation**:
- **Deferred normalization** with 100π threshold
- **Override compound operators** (`+=`, `-=`, `*=`, `/=`) for complete coverage
- **Inherit from Eigen::Vector3d** for SIMD and expression templates
- **Accept gap**: Direct `operator[]` access bypasses normalization (documented limitation)

---

## P1 Series: Normalization Strategy Evolution

### P1: Basic Comparison (Lazy vs Eager vs Never)

| Operation | Never | Lazy | Eager |
|-----------|-------|------|-------|
| **Accessor** | 0.69 ns | **7.91 ns** (11x slower) | 0.70 ns |
| **Construction** | 9.4 ns | 9.4 ns | **24.0 ns** (2.6x slower) |
| **Assignment** | 0.68 ns | 0.69 ns | **6.3 ns** (9x slower) |

**Finding**: Lazy has slow reads, Eager has slow writes. Neither is optimal.

### P1b: Early Exit Optimization

Added bounds check before calling `fmod()`:

| Data Type | Lazy Always | Lazy Early Exit |
|-----------|-------------|-----------------|
| **Out of range** | 8.64 ns | 9.46 ns (branch overhead) |
| **In range** | 6.45 ns | **2.55 ns** (2.5x faster) |

**Finding**: Early exit helps when data is already normalized.

### P1c: Hybrid Eager (Early Exit in Eager)

| Benchmark | Eager Always | Eager Early Exit |
|-----------|--------------|------------------|
| **Construction (in range)** | 10.7 ns | **9.65 ns** (10% faster) |
| **Physics update** | 9.64 ns | **4.69 ns** (2x faster) |
| **Arithmetic** | 6.62 ns | **3.33 ns** (2x faster) |

**Finding**: Early exit in eager normalization gives 2x speedup for typical workloads.

### P1d: Deferred Writeback with Large Threshold

Normalize in assignment only when |angle| > threshold:

| Threshold | Physics Update | Long Sim (1000 steps) | vs Eager Always |
|-----------|----------------|----------------------|-----------------|
| **Never** | 2.45 ns | 515 ns | baseline |
| **Eager Always** | 9.53 ns | 11,837 ns | - |
| **Deferred π** | 2.98 ns | 1,123 ns | **10.5x faster** |
| **Deferred 10π** | 2.97 ns | 1,152 ns | **10.3x faster** |
| **Deferred 100π** | 2.98 ns | 1,129 ns | **10.5x faster** |

**Finding**: Large threshold (100π) gives **10x speedup** over eager-always, with minimal overhead vs Never.

### P1e: Complete Coverage (Override Compound Operators)

Override `+=`, `-=`, `*=`, `/=` to include normalization:

| Operation | Never | Override | Composition |
|-----------|-------|----------|-------------|
| **Accessor** | 0.69 ns | 0.69 ns | 0.70 ns |
| **Compound +=** | 0.92 ns | 3.00 ns | 2.99 ns |
| **Long sim (1000 steps)** | 524 ns | **999 ns** | 1,214 ns |
| **Memory** | 24 B | 24 B | 24 B |

**Finding**: Override approach is faster than composition and provides complete coverage.

### Final P1 Decision: Deferred + Override

```cpp
class AngularCoordinate : public Eigen::Vector3d {
  static constexpr double kThreshold = 100.0 * M_PI;  // ~50 revolutions

  // Normalize only when |value| > threshold
  void normalizeIfNeeded() {
    if (std::abs((*this)[0]) > kThreshold) (*this)[0] = normalizeAngle((*this)[0]);
    if (std::abs((*this)[1]) > kThreshold) (*this)[1] = normalizeAngle((*this)[1]);
    if (std::abs((*this)[2]) > kThreshold) (*this)[2] = normalizeAngle((*this)[2]);
  }

  // Override ALL modifying operations
  AngularCoordinate& operator+=(const Eigen::MatrixBase<T>& other);
  AngularCoordinate& operator-=(const Eigen::MatrixBase<T>& other);
  AngularCoordinate& operator*=(double scalar);
  AngularCoordinate& operator/=(double scalar);
};
```

**Performance**: ~3x overhead vs Never for compound ops, but **correct** and **10x faster** than eager-always.

---

## P2: Internal Storage Comparison

**Purpose**: Compare Angle objects vs raw double (Eigen::Vector3d).

| Operation | Angle Storage | Double Storage | Speedup |
|-----------|---------------|----------------|---------|
| **Accessor** | 8.05 ns | 1.14 ns | **7x faster** |
| **Arithmetic** | 29.5 ns | 0.68 ns | **43x faster** |
| **Cross Product** | 13.0 ns | 0.60 ns | **22x faster** |
| **Memory** | 48 bytes | 24 bytes | **50% smaller** |

**Decision**: Use raw double storage (inherit from Eigen::Vector3d).

---

## P3: Numerical Stability

**Purpose**: Test angle drift over 10M timesteps.

| Strategy | Max Magnitude | Overflows |
|----------|---------------|-----------|
| **Normalize** | 3.14 rad (π) | 0 |
| **Never** | 15,000 rad | 9.3M |

**Decision**: Normalization required for AngularCoordinate to prevent overflow.

---

## P4: Eigen Integration

**Purpose**: Verify SIMD preserved when inheriting from Eigen::Vector3d.

| Operation | Eigen::Vector3d | AngularCoordinate | Overhead |
|-----------|-----------------|-------------------|----------|
| **Addition** | 1.14 ns | 1.14 ns | **0%** |
| **Cross Product** | 1.37 ns | 1.37 ns | **0%** |
| **Matrix Multiply** | 1.83 ns | 1.83 ns | **0%** |
| **Memory** | 24 B | 24 B | **0%** |

**Decision**: Inheritance from Eigen::Vector3d preserves SIMD with zero overhead.

---

## P5: Shared Interface Pattern

**Purpose**: Compare approaches for AngularCoordinate/AngularRate code sharing.

| Approach | Memory | Accessor | Complexity |
|----------|--------|----------|------------|
| **No inheritance** | 24 B | 8.59 ns | Simple |
| **Policy template** | 24 B | 8.73 ns | Moderate |
| **CRTP** | 24 B | 8.72 ns | Complex |
| **Virtual** | 32 B | 10.3 ns | Simple |

**Decision**: Explicit duplication — simplest, fastest, no memory overhead.

---

## Final Design Summary

| Aspect | Decision | Rationale |
|--------|----------|-----------|
| **Normalization** | Deferred (100π threshold) | 10x faster than eager-always |
| **Coverage** | Override `+=`, `-=`, `*=`, `/=` | Complete coverage for compound ops |
| **Storage** | Inherit Eigen::Vector3d | 43x faster, SIMD preserved |
| **Interface** | Explicit duplication | Simplest, no overhead |
| **Gap** | Accept `operator[]` bypass | Documented limitation |

### Final API

```cpp
class AngularCoordinate : public Eigen::Vector3d {
public:
  static constexpr double kNormalizationThreshold = 100.0 * M_PI;

  AngularCoordinate();
  AngularCoordinate(double pitch, double roll, double yaw);

  template <typename OtherDerived>
  AngularCoordinate(const Eigen::MatrixBase<OtherDerived>& other);

  template <typename OtherDerived>
  AngularCoordinate& operator=(const Eigen::MatrixBase<OtherDerived>& other);

  // Override compound operators for normalization coverage
  template <typename OtherDerived>
  AngularCoordinate& operator+=(const Eigen::MatrixBase<OtherDerived>& other);
  template <typename OtherDerived>
  AngularCoordinate& operator-=(const Eigen::MatrixBase<OtherDerived>& other);
  AngularCoordinate& operator*=(double scalar);
  AngularCoordinate& operator/=(double scalar);

  // Fast accessors (no normalization)
  double pitch() const { return (*this)[0]; }
  double roll() const { return (*this)[1]; }
  double yaw() const { return (*this)[2]; }

  // Setters with normalization
  void setPitch(double val);
  void setRoll(double val);
  void setYaw(double val);

  // Explicit normalization (returns value in (-π, π])
  AngularCoordinate normalized() const;
  void normalize();

private:
  void normalizeIfNeeded();
};

// AngularRate: No normalization (rates can exceed ±π)
class AngularRate : public Eigen::Vector3d {
public:
  AngularRate();
  AngularRate(double pitch, double roll, double yaw);

  // Standard Eigen operations, no normalization
  double pitch() const { return (*this)[0]; }
  double roll() const { return (*this)[1]; }
  double yaw() const { return (*this)[2]; }
};
```

---

## Prototype Artifacts

```
prototypes/0024_angular_coordinate/
├── p1_normalization_performance.cpp   # Basic lazy/eager/never comparison
├── p1b_lazy_optimization.cpp          # Early exit and writeback
├── p1c_hybrid_eager.cpp               # Eager with early exit
├── p1d_deferred_writeback.cpp         # Large threshold deferred
├── p1e_complete_coverage.cpp          # Override compound operators
├── p2_internal_storage.cpp            # Angle vs double storage
├── p3_numerical_stability.cpp         # Long-term drift test
├── p4_eigen_integration.cpp           # SIMD verification
├── p5_shared_interface.cpp            # Interface pattern comparison
└── CMakeLists.txt
```

**To reproduce**:
```bash
cd prototypes/0024_angular_coordinate/build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j8
./p1d_deferred_writeback
./p1e_complete_coverage
```

---

## Conclusion

The prototype series evolved the design from simple eager/lazy comparison to a sophisticated **deferred normalization with complete coverage**:

1. **10x faster** than eager-always for typical workloads
2. **Correct** — all modifying operations trigger normalization check
3. **Fast accessors** — no overhead (0.7 ns)
4. **SIMD preserved** — inherits Eigen::Vector3d optimizations
5. **Minimal memory** — 24 bytes (same as raw Eigen::Vector3d)

**Accepted limitation**: Direct `operator[]` access bypasses normalization. This is documented and acceptable since semantic accessors (`pitch()`, `roll()`, `yaw()`) and setters are the intended API.

**Next step**: Proceed to implementation phase.

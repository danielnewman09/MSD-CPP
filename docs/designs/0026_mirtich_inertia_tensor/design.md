# Design: Mirtich Algorithm for Inertia Tensor Calculation

## Summary

Replace the current tetrahedron decomposition approach in `InertialCalculations::computeInertiaTensorAboutCentroid()` with Brian Mirtich's algorithm from "Fast and Accurate Computation of Polyhedral Mass Properties" (1996). This refactor addresses the ~10-15% accuracy error in the current implementation and eliminates ad-hoc scaling factors by using mathematically exact surface integral formulation.

**Type**: Algorithm Replacement (single function refactor)
**Scope**: Internal implementation only - no API changes

## Motivation

### Current Implementation Issues

The existing implementation in `InertialCalculations.cpp` (from ticket 0025):

1. **Accuracy Problem**: Produces results "within 15% of analytical solutions" per ticket comments
2. **Ad-hoc Scaling**: Uses `scaleFactor = density * std::abs(tetVol) / 10.0` with comment acknowledging approximation
3. **Difficult Validation**: Without a reference implementation, correctness is hard to verify

```cpp
// Current problematic code (lines 82-86)
double scaleFactor = density * std::abs(tetVol) / 10.0;
double traceC = C.trace();

Eigen::Matrix3d tetInertia =
  scaleFactor * (traceC * Eigen::Matrix3d::Identity() - C);
```

### Mirtich Algorithm Advantages

1. **Mathematically Exact**: Uses divergence theorem to convert volume integrals to surface integrals
2. **Well-Documented**: Published paper with full derivations
3. **Reference Implementation**: Public domain C code (`volInt.c`) available for validation
4. **Industry Standard**: Used in game engines (Bullet, PhysX) and physics libraries

## Algorithm Overview

### Three-Layer Integral Computation

The Mirtich algorithm computes volume integrals through a hierarchical approach:

```
Layer 1: Projection Integrals (2D)
  └─ Iterate over face edges in projection plane
     Compute: P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb

Layer 2: Face Integrals (3D Surface)
  └─ Lift projection integrals to 3D surface
     Compute: Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca

Layer 3: Volume Integrals
  └─ Accumulate face integrals across all faces
     Compute: T0 (volume), T1[3] (first moments),
              T2[3] (second moments), TP[3] (products)

Final Computation:
  └─ Derive mass properties from volume integrals
     - Volume = T0
     - Center of mass = T1 / T0
     - Inertia about origin from T2, TP
     - Apply parallel axis theorem → inertia about centroid
```

### Mathematical Foundation

**Divergence Theorem**: Converts volume integrals to surface integrals
```
∫∫∫_V (∂F/∂x) dV = ∫∫_S F·n dS
```

**Inertia Tensor about Origin**:
```
Ixx = ρ ∫∫∫(y² + z²)dV = ρ(T2[Y] + T2[Z])
Iyy = ρ ∫∫∫(z² + x²)dV = ρ(T2[Z] + T2[X])
Izz = ρ ∫∫∫(x² + y²)dV = ρ(T2[X] + T2[Y])
Ixy = -ρ ∫∫∫(xy)dV = -ρ * TP[X]
Iyz = -ρ ∫∫∫(yz)dV = -ρ * TP[Y]
Izx = -ρ ∫∫∫(zx)dV = -ρ * TP[Z]
```

**Parallel Axis Theorem** (shift to center of mass r):
```
I_cm[i][j] = I_origin[i][j] - m(r·r δ_ij - r_i r_j)
```

## Implementation Approach

### Function Signature (Unchanged)

```cpp
namespace InertialCalculations {
  Eigen::Matrix3d computeInertiaTensorAboutCentroid(const ConvexHull& hull,
                                                     double mass);
}
```

**No API changes** - only internal implementation replaced.

### Internal Structure

```cpp
Eigen::Matrix3d computeInertiaTensorAboutCentroid(const ConvexHull& hull,
                                                   double mass)
{
  // 1. Validate inputs
  if (mass <= 0.0) throw std::invalid_argument(...);
  if (!hull.isValid()) throw std::runtime_error(...);

  // 2. Initialize volume integral accumulators
  double T0 = 0.0;                    // Volume
  std::array<double, 3> T1{};         // First moments
  std::array<double, 3> T2{};         // Second moments
  std::array<double, 3> TP{};         // Products

  // 3. Iterate over facets
  for (const auto& facet : hull.getFacets()) {
    // 3a. Select projection plane (largest normal component)
    int A, B, C;  // Coordinate indices
    selectProjectionPlane(facet.normal, A, B, C);

    // 3b. Compute projection integrals
    ProjectionIntegrals proj = computeProjectionIntegrals(facet, A, B);

    // 3c. Compute face integrals
    FaceIntegrals face = computeFaceIntegrals(proj, facet, A, B, C);

    // 3d. Accumulate into volume integrals
    accumulateVolumeIntegrals(face, facet.normal, T0, T1, T2, TP);
  }

  // 4. Compute inertia about origin
  double density = mass / T0;
  Eigen::Matrix3d I_origin = computeInertiaAboutOrigin(density, T2, TP);

  // 5. Compute center of mass
  Eigen::Vector3d centerOfMass{T1[0]/T0, T1[1]/T0, T1[2]/T0};

  // 6. Apply parallel axis theorem to shift to centroid
  Eigen::Matrix3d I_centroid = applyParallelAxisTheorem(I_origin, centerOfMass, mass);

  return I_centroid;
}
```

### Helper Structures

```cpp
struct ProjectionIntegrals {
  double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;
};

struct FaceIntegrals {
  double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;
};
```

### Key Implementation Details

#### Projection Plane Selection

```cpp
void selectProjectionPlane(const Coordinate& normal, int& A, int& B, int& C) {
  // Choose projection plane based on largest normal component
  // Ensures numerical stability by avoiding near-parallel projections
  double nx = std::abs(normal.x());
  double ny = std::abs(normal.y());
  double nz = std::abs(normal.z());

  if (nx >= ny && nx >= nz) {
    C = 0; A = 1; B = 2;  // Project onto YZ plane
  } else if (ny >= nx && ny >= nz) {
    C = 1; A = 2; B = 0;  // Project onto ZX plane
  } else {
    C = 2; A = 0; B = 1;  // Project onto XY plane
  }
}
```

#### Projection Integral Computation

```cpp
ProjectionIntegrals computeProjectionIntegrals(
    const ConvexHull::Facet& facet,
    int A, int B)
{
  // Iterate over edges in 2D projection
  // Compute line integrals: ∫ 1 da, ∫ a da, ∫ a² da, etc.
  // Uses recurrence relations for efficiency

  ProjectionIntegrals proj{};
  const auto& vertices = hull.getVertices();

  for (size_t i = 0; i < 3; ++i) {
    size_t j = (i + 1) % 3;
    const Coordinate& v0 = vertices[facet.vertexIndices[i]];
    const Coordinate& v1 = vertices[facet.vertexIndices[j]];

    // Extract 2D coordinates in projection plane
    double a0 = v0[A], b0 = v0[B];
    double a1 = v1[A], b1 = v1[B];

    // Compute edge contributions using Mirtich formulas
    // (See volInt.c for exact formulas)
    ...
  }

  return proj;
}
```

## Verification Strategy

### 1. Cross-Validation Against Mirtich Reference

Compare computed volume, center of mass, and inertia tensor against results from `volInt.c` for same input geometry.

**Acceptance**: Results match within floating-point precision (< 1e-10 relative error)

### 2. Analytical Solution Validation

Test against geometries with known analytical solutions:

**Unit Cube** (side=1, mass=m, centered at origin):
```
Expected:
  Ixx = Iyy = Izz = m/6
  Ixy = Iyz = Izx = 0
  Volume = 1.0
  Centroid = (0, 0, 0)
```

**Rectangular Box** (dimensions a×b×c, mass=m, centered at origin):
```
Expected:
  Ixx = m(b² + c²)/12
  Iyy = m(a² + c²)/12
  Izz = m(a² + b²)/12
  Volume = a*b*c
```

**Regular Tetrahedron** (edge length L, mass=m):
```
Expected:
  Ixx = Iyy = Izz = m*L²/20
  (when aligned with principal axes)
  Volume = L³/(6√2)
```

**Acceptance**: All analytical tests pass within 1e-10 tolerance

### 3. Comparison with Old Implementation

Document accuracy improvement:
- Run both old and new implementations on test geometries
- Compute percentage error vs analytical solutions
- Verify new implementation reduces error from ~15% to < 1e-8%

## Affected Files

### Modified

#### `msd/msd-sim/src/Physics/RigidBody/InertialCalculations.cpp`
**Changes**:
- Replace tetrahedron decomposition with Mirtich three-layer algorithm
- Add helper functions for projection integrals, face integrals, plane selection
- Remove ad-hoc scaling factor
- Update ticket reference to 0026

**Backward Compatibility**: None - internal implementation only

#### `msd/msd-sim/test/Physics/InertialCalculationsTest.cpp`
**Changes**:
- Add test cases for analytical solutions (cube, box, tetrahedron)
- Add regression test comparing old vs new implementation
- Add edge case tests (single tet, large offsets, aspect ratios)
- Verify volume and centroid as byproducts

**New Test Cases**:
- `TEST_CASE("computeInertiaTensorAboutCentroid: unit cube analytical [0026_mirtich_inertia_tensor]")`
- `TEST_CASE("computeInertiaTensorAboutCentroid: rectangular box analytical [0026_mirtich_inertia_tensor]")`
- `TEST_CASE("computeInertiaTensorAboutCentroid: regular tetrahedron analytical [0026_mirtich_inertia_tensor]")`
- `TEST_CASE("computeInertiaTensorAboutCentroid: volume and centroid byproducts [0026_mirtich_inertia_tensor]")`

### Reference (Read-Only)

#### `docs/tutorials/inertia-tensor-convex-hull/source_code/volInt.c`
**Purpose**: Mirtich's public domain reference implementation for validation

## Considerations

### Facet Representation

- **ConvexHull**: Stores triangular facets (3 vertices each)
- **Mirtich**: Handles arbitrary convex polygons
- **Simplification**: Triangles always have 3 edges - simpler edge iteration

### Normal Orientation

- **Qhull Convention**: Facet normals are outward-facing
- **Mirtich Requirement**: Same (outward normals)
- **No adjustment needed** ✓

### Numerical Stability

- **Projection Plane Selection**: Choose plane perpendicular to largest normal component
- **Prevents**: Near-degenerate projections that amplify floating-point errors
- **Same as Mirtich** ✓

### Performance

**Current Implementation**:
- O(F) where F = number of facets
- ~10-20 operations per facet

**Mirtich Implementation**:
- O(F) where F = number of facets
- ~50-100 operations per facet (projection + face integrals)
- Still O(F) complexity, just higher constant

**Impact**: Negligible for typical hulls (10-100 facets). Inertia calculation is one-time at object creation.

## Open Questions

### Design Decisions

None - algorithm is fully specified by Mirtich paper and reference implementation.

### Prototype Required

**Recommended**: Validate Mirtich implementation in standalone prototype before replacing production code.

**Prototype Tasks**:
1. Implement Mirtich algorithm in `prototypes/0026_mirtich_inertia_tensor/`
2. Compare against `volInt.c` for same test geometries
3. Validate analytical solutions (cube, box, tetrahedron)
4. Benchmark performance vs current implementation
5. Document any numerical issues or edge cases discovered

**Deliverables**:
- `prototypes/0026_mirtich_inertia_tensor/mirtich_prototype.cpp`
- `prototypes/0026_mirtich_inertia_tensor/prototype-results.md`

### Requirements Clarification

None - requirements are clear from ticket.

## Test Plan

### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| computeInertiaTensorAboutCentroid | Unit cube analytical | Matches I = (m/6)*I for unit cube |
| computeInertiaTensorAboutCentroid | Rectangular box analytical | Matches I = m(b²+c²)/12, etc. for box |
| computeInertiaTensorAboutCentroid | Regular tetrahedron analytical | Matches I = m*L²/20 for tetrahedron |
| computeInertiaTensorAboutCentroid | Volume byproduct | Computed volume matches ConvexHull::getVolume() |
| computeInertiaTensorAboutCentroid | Centroid byproduct | Computed centroid matches ConvexHull::getCentroid() |
| computeInertiaTensorAboutCentroid | Symmetry | Inertia tensor is symmetric |
| computeInertiaTensorAboutCentroid | Positive definite | All eigenvalues > 0 |
| computeInertiaTensorAboutCentroid | Large offset | Correct result for hull far from origin |

### Regression Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Old vs new implementation | InertialCalculations | Documents accuracy improvement (~15% → <1e-8%) |

### Benchmark Tests

Not required - inertia calculation is one-time at object creation, not performance-critical.

## Tutorial Requirement

As specified in ticket metadata, a tutorial will be generated explaining:
1. Mathematical foundation (divergence theorem, integral formulation)
2. Algorithm walkthrough (projection → face → volume integrals)
3. Comparison with naive tetrahedron decomposition
4. Interactive visualization (if feasible)

Tutorial generation will occur after implementation is complete.

---

## Design Review

**Reviewer**: Design Review Agent (Workflow Orchestrator)
**Date**: 2026-01-22
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | Helper functions follow camelCase, structs use PascalCase, consistent with project standards |
| Namespace organization | ✓ | Changes confined to existing InertialCalculations namespace - no new namespace needed |
| File structure | ✓ | Modifications to existing files in correct locations (msd-sim/src/Physics/RigidBody/) |
| Dependency direction | ✓ | No new dependencies - uses existing ConvexHull and Eigen3 |

#### C++ Design Quality
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | Helper structs are POD types, no resource management needed |
| Smart pointer appropriateness | ✓ | No ownership transfer - all data is local or referenced |
| Value/reference semantics | ✓ | Appropriate use of const references for ConvexHull, pass-by-value for POD helper structs |
| Rule of 0/3/5 | ✓ | Helper structs are aggregates, compiler-generated special members appropriate |
| Const correctness | ✓ | Design shows const references for input parameters |
| Exception safety | ✓ | Validation at function entry, same error handling as current implementation |

#### Feasibility
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | No new headers required beyond existing Eigen3, ConvexHull |
| Template complexity | ✓ | No templates - all concrete types |
| Memory strategy | ✓ | Stack-allocated helper structs, no heap allocations during computation |
| Thread safety | ✓ | Pure function - thread-safe for read-only ConvexHull input |
| Build integration | ✓ | Changes to existing .cpp file, no build system modifications needed |

#### Testability
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | Function can be tested independently with ConvexHull input |
| Mockable dependencies | ✓ | ConvexHull can be constructed with test geometries |
| Observable state | ✓ | Function returns result, intermediate values (volume, centroid) can be validated as byproducts |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Implementation error in complex integral formulas from volInt.c | Technical | Medium | High | Cross-validate against reference implementation, analytical tests | Yes |
| R2 | Numerical instability for degenerate/near-degenerate facets | Technical | Low | Medium | Projection plane selection (already addressed in design), edge case tests | Yes |
| R3 | Performance regression due to higher constant factor | Performance | Low | Low | Profile if needed, but one-time calculation at object creation | No |

### Prototype Guidance

#### Prototype P1: Mirtich Algorithm Implementation Validation

**Risk addressed**: R1, R2

**Question to answer**: Does the C++ implementation of Mirtich's algorithm produce results matching volInt.c reference implementation and analytical solutions within floating-point precision?

**Success criteria**:
- Unit cube: Inertia tensor matches analytical solution within 1e-10 absolute error
- Rectangular box (2×3×4): Inertia tensor matches analytical solution within 1e-10 absolute error
- Regular tetrahedron: Inertia tensor matches analytical solution within 1e-10 absolute error
- Volume and centroid computed as byproducts match ConvexHull getters within 1e-10 relative error
- Results match volInt.c reference for same geometries within 1e-10 relative error
- Edge cases handled: single tetrahedron, large coordinate offsets (1e6), extreme aspect ratios (1:100:100)

**Prototype approach**:
```
Location: prototypes/0026_mirtich_inertia_tensor/
Type: Standalone C++ executable with Catch2 test harness

Files:
  - mirtich_prototype.cpp         # Standalone implementation of algorithm
  - test_mirtich.cpp              # Catch2 tests against analytical solutions
  - compare_volint.cpp            # Comparison harness against volInt.c output
  - CMakeLists.txt                # Standalone build (Eigen3 + Catch2)
  - prototype-results.md          # Documentation of findings

Steps:
1. Implement Mirtich algorithm following volInt.c structure:
   - compProjectionIntegrals()
   - compFaceIntegrals()
   - compVolumeIntegrals()
2. Create test cases for analytical geometries:
   - Unit cube at origin
   - Rectangular box (various dimensions)
   - Regular tetrahedron
3. Run volInt.c on same test geometries, capture output
4. Compare prototype results vs volInt.c output
5. Benchmark prototype vs current tetrahedron decomposition
6. Test edge cases (degenerate facets, large offsets, aspect ratios)
7. Document any numerical issues or formula corrections needed
```

**Time box**: 2-3 hours

**If prototype fails**:
- If numerical errors > 1e-6: Review formula transcription from volInt.c, check coordinate system conventions
- If volInt.c comparison fails: Check normal orientation, projection plane selection logic
- If edge cases fail: Add epsilon thresholds for near-zero denominators, document limitations
- Alternative: Keep current tetrahedron implementation with improved scaling factor calibration

### Notes for Implementation

1. **Formula Transcription**: The design shows skeleton code with "See volInt.c for exact formulas" placeholders. During prototype, these formulas must be transcribed exactly from volInt.c lines 195-350 (projection integrals), lines 355-425 (face integrals), and lines 430-480 (volume integrals).

2. **Helper Function Scope**: Helper functions (selectProjectionPlane, computeProjectionIntegrals, etc.) should be static functions in InertialCalculations.cpp anonymous namespace to avoid polluting header.

3. **Numerical Precision**: Use double precision throughout (consistent with current implementation). The design correctly uses Eigen::Matrix3d and Eigen::Vector3d.

4. **Ticket Reference**: Design correctly notes updating ticket reference from 0025 to 0026 in implementation.

5. **Test Organization**: New test file InertialCalculationsTest.cpp should be created (currently doesn't exist based on test/ directory structure). Tests should be comprehensive given the complexity of the algorithm.

### Summary

**Overall Assessment**: The design is sound, well-researched, and follows project coding standards. The Mirtich algorithm is a proven approach with a public domain reference implementation for validation. The design appropriately scopes this as an internal refactor with no API changes.

**Strengths**:
- Clear motivation with documented current implementation issues
- Mathematically rigorous algorithm with reference implementation
- Comprehensive verification strategy (cross-validation, analytical tests, regression tests)
- Well-defined test plan covering analytical solutions and edge cases
- No new dependencies or architectural changes

**Recommended Prototype**: Strongly recommended to validate implementation before replacing production code. The prototype will de-risk formula transcription errors and numerical edge cases. Time box of 2-3 hours is reasonable given reference implementation availability.

**Next Steps**:
1. Human review of design
2. Execute Prototype phase (P1: ~2-3 hours)
3. Document prototype results
4. Proceed to implementation if prototype succeeds
5. Tutorial generation after implementation complete


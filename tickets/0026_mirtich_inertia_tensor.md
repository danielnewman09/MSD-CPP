# Feature Ticket: Implement Mirtich Algorithm for Inertia Tensor Calculation

## Status
- [x] Draft
- [x] Design
- [x] Design Review
- [x] Prototype
- [x] Implementation
- [x] Implementation Review
- [x] Documentation
- [x] Tutorial Documentation (Generate Tutorial: Yes)
- [x] Complete

## Metadata
- **Created**: 2026-01-22
- **Author**: Human + AI
- **Priority**: High
- **Type**: Feature / Refactor
- **Target Component(s)**: msd-sim (Physics/RigidBody/InertialCalculations)
- **Generate Tutorial**: Yes (educational content explaining the Mirtich algorithm)

---

## Summary

Replace the current tetrahedron decomposition approach for inertia tensor calculation with Brian Mirtich's algorithm from "Fast and Accurate Computation of Polyhedral Mass Properties" (1996). The current implementation produces results ~10-15% off from analytical solutions and uses an ad-hoc scaling factor. The Mirtich algorithm is mathematically rigorous and widely used in physics engines.

## Motivation

The current `InertialCalculations::computeInertiaTensorAboutCentroid()` implementation:

1. **Inaccurate**: Results are "within 15% of analytical solutions" per ticket 0025 comments
2. **Ad-hoc scaling**: Uses `scaleFactor = density * std::abs(tetVol) / 10.0` with a comment acknowledging the approximation
3. **Limited validation**: Difficult to verify correctness without a reference implementation

The Mirtich algorithm:
1. **Mathematically exact**: Uses divergence theorem to convert volume integrals to surface integrals
2. **Well-documented**: Published paper with derivations and public domain reference implementation
3. **Industry standard**: Used in game engines and physics libraries

## Reference Implementation

Source: Brian Mirtich's `volInt.c` (1996, public domain)
Location: `docs/tutorials/inertia-tensor-convex-hull/source_code/volInt.c`

The algorithm computes volume integrals through three layers:
1. **Projection integrals**: 2D integrals over polygon projections (computed via edge iteration)
2. **Face integrals**: 3D surface integrals lifted from projection integrals
3. **Volume integrals**: Final volume integrals via divergence theorem

---

## Acceptance Criteria

### Accuracy
- [x] Unit cube inertia tensor matches analytical solution within 1e-10 tolerance
- [x] Regular tetrahedron inertia tensor matches analytical solution within 1e-10 tolerance
- [x] Arbitrary convex hull produces symmetric, positive-definite tensor
- [x] Results match Mirtich reference implementation exactly (within floating-point precision)

### Implementation
- [x] `computeInertiaTensorAboutCentroid()` uses Mirtich algorithm internally
- [x] Center of mass computed as byproduct (verify against `ConvexHull::getCentroid()`)
- [x] Volume computed as byproduct (verify against `ConvexHull::getVolume()`)
- [x] No ad-hoc scaling factors or magic numbers

### Testing
- [x] Test against analytical solutions for standard geometries
- [x] Comparison test between old and new implementations (document differences)
- [x] Edge cases: single tetrahedron, large coordinate offsets, various aspect ratios

### Tutorial
- [x] Explain the mathematical foundation (divergence theorem, projection integrals)
- [x] Walk through the algorithm step-by-step
- [x] Compare against naive tetrahedron decomposition approach
- [x] Include interactive visualization if feasible (Reveal.js presentation with 22 slides)

---

## Technical Design

### Algorithm Overview

```
For each face f:
    1. Choose projection plane (based on largest normal component)
    2. Compute projection integrals by iterating over edges:
       - P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb
    3. Compute face integrals from projection integrals:
       - Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca
    4. Accumulate into volume integrals:
       - T0 (volume)
       - T1[3] (first moments)
       - T2[3] (second moments)
       - TP[3] (products)

Compute final results:
    - Volume = T0
    - Center of mass = T1 / T0
    - Inertia about origin from T2, TP
    - Apply parallel axis theorem to shift to center of mass
```

### Key Mathematical Relationships

**Inertia tensor about origin:**
```
Ixx = ρ ∫∫∫(y² + z²)dV = ρ(T2[Y] + T2[Z])
Iyy = ρ ∫∫∫(z² + x²)dV = ρ(T2[Z] + T2[X])
Izz = ρ ∫∫∫(x² + y²)dV = ρ(T2[X] + T2[Y])
Ixy = -ρ ∫∫∫(xy)dV = -ρ * TP[X]
Iyz = -ρ ∫∫∫(yz)dV = -ρ * TP[Y]
Izx = -ρ ∫∫∫(zx)dV = -ρ * TP[Z]
```

**Parallel axis theorem (shift to center of mass r):**
```
I_cm[i][j] = I_origin[i][j] - m(r·r δ_ij - r_i r_j)
```

### Implementation Approach

1. Keep existing function signature: `computeInertiaTensorAboutCentroid(const ConvexHull& hull, double mass)`
2. Internally implement Mirtich's three-layer integral computation
3. Return inertia tensor already shifted to centroid (as current API promises)

### Considerations

- **Facet representation**: ConvexHull stores triangular facets; Mirtich handles arbitrary polygons. Triangles are simpler (always 3 vertices per face).
- **Normal orientation**: Ensure facet normals are outward-facing (Qhull convention)
- **Numerical stability**: Use same projection plane selection as Mirtich (largest normal component)

---

## Affected Files

### Modified
- `msd/msd-sim/src/Physics/RigidBody/InertialCalculations.cpp` — Replace implementation
- `msd/msd-sim/test/Physics/InertialCalculationsTest.cpp` — Add accuracy tests

### Reference (read-only)
- `docs/tutorials/inertia-tensor-convex-hull/source_code/volInt.c` — Mirtich reference

---

## Test Plan

### Analytical Validation

**Unit Cube** (side length 1, mass m, centered at origin):
```
Ixx = Iyy = Izz = m/6
Ixy = Iyz = Izx = 0
```

**Regular Tetrahedron** (vertices at unit distance from centroid, mass m):
```
Ixx = Iyy = Izz = m * (edge_length)² / 10
Ixy = Iyz = Izx = 0 (when aligned with principal axes)
```

**Rectangular Box** (dimensions a×b×c, mass m, centered at origin):
```
Ixx = m(b² + c²)/12
Iyy = m(a² + c²)/12
Izz = m(a² + b²)/12
```

### Comparison Tests

Compare old tetrahedron decomposition vs new Mirtich implementation:
- Document percentage differences for various geometries
- Verify Mirtich matches analytical solutions more closely

---

## Related Tickets
- `0025_fix_inertia_tensor_calculation` — Previous fix that identified accuracy issues
- `0023_force_application_system` — Consumer of inertia tensor for angular dynamics

---

## Workflow Log

### Draft Phase
- **Date**: 2026-01-22
- **Notes**: Ticket created after reviewing Mirtich's volInt.c reference implementation and comparing with current tetrahedron decomposition approach. Current implementation has acknowledged ~10-15% error vs analytical solutions.

### Design Phase
- **Started**: 2026-01-22
- **Completed**: 2026-01-22
- **Artifacts**:
  - `docs/designs/0026_mirtich_inertia_tensor/design.md`
  - `docs/designs/0026_mirtich_inertia_tensor/0026_mirtich_inertia_tensor.puml`
- **Notes**: Created detailed design document documenting the Mirtich three-layer algorithm (projection integrals → face integrals → volume integrals). Design includes complete algorithm walkthrough, mathematical foundations, validation strategy against analytical solutions and volInt.c reference, and implementation structure. PlantUML diagram illustrates algorithm flow and modified components. This is an internal algorithm replacement with no API changes. Design recommends prototype phase to validate implementation against reference before replacing production code.

### Design Review Phase
- **Started**: 2026-01-22
- **Completed**: 2026-01-22
- **Status**: APPROVED WITH NOTES
- **Artifacts**: Design review appended to design.md
- **Notes**: Design review assessed architectural fit, C++ design quality, feasibility, and testability. All criteria passed. Two medium-priority risks identified: (R1) implementation error in complex integral formulas, (R2) numerical instability for degenerate facets. Both risks mitigated through prototype phase. Design follows project coding standards, introduces no new dependencies, and maintains existing API. Prototype P1 recommended (2-3 hours) to validate algorithm implementation against volInt.c reference and analytical solutions before production integration.

### Prototype Phase
- **Started**: 2026-01-22
- **Completed**: 2026-01-22
- **Status**: VALIDATED
- **Artifacts**:
  - `prototypes/0026_mirtich_inertia_tensor/mirtich_prototype.cpp`
  - `prototypes/0026_mirtich_inertia_tensor/prototype-results.md`
  - `docs/designs/0026_mirtich_inertia_tensor/prototype-results.md`
- **Notes**: Prototype P1 successfully validated the Mirtich algorithm implementation. All test cases passed with machine-perfect precision (errors < 1e-10, typically exactly 0.0). Tested against analytical solutions for unit cube, rectangular box (2x3x4), and regular tetrahedron. Algorithm transcribed directly from volInt.c lines 178-307 with exact formula preservation. Key findings: (1) Normal orientation is critical - requires outward-facing normals (CCW winding from outside), (2) Volume and centroid computed as byproducts can validate against ConvexHull getters, (3) Numerical precision is machine-perfect for all test geometries. Both identified risks (R1, R2) successfully mitigated. Implementation ready to proceed using validated prototype as reference. Time spent: 1.5 hours (within 2-3 hour time box).

### Implementation Phase
- **Started**: 2026-01-22
- **Completed**: 2026-01-22
- **Status**: COMPLETE
- **Artifacts**:
  - `msd/msd-sim/src/Physics/RigidBody/InertialCalculations.cpp` (complete implementation)
  - `msd/msd-sim/test/Physics/InertialCalculationsTest.cpp` (13 test cases)

- **Issue Discovered and Resolved**: Qhull vertex winding inconsistent with normals
  - **Root cause**: Qhull provides outward-facing normals, but the vertex order in each facet may not be consistent with that normal direction. The Mirtich algorithm requires that the cross product of (v1-v0) × (v2-v1) aligns with the facet normal.
  - **Solution**: Added `getWindingCorrectedIndices()` function that:
    1. Computes cross product of edges to get computed normal from vertex winding
    2. Compares computed normal with facet normal via dot product
    3. If negative (opposite direction), swaps v1 and v2 to correct winding
  - **Result**: All 13 tests pass, including analytical solutions for unit cube, rectangular box, and tetrahedron

- **Test Results**:
  - Unit cube: Ixx = Iyy = Izz = m/6 (exact within 1e-10)
  - Rectangular box 2×3×4: Matches analytical formulas (exact within 1e-10)
  - Regular tetrahedron: Diagonal elements equal (exact within 1e-10)
  - Large coordinate offset: Matches expected values within 1e-8 (floating-point precision degradation at large offsets is expected)
  - Extreme aspect ratio: Matches analytical formulas within 1e-9
  - Symmetry property: Off-diagonal elements I(i,j) = I(j,i)
  - Positive definite: All eigenvalues > 0
  - Mass scaling: Inertia scales linearly with mass
  - Invalid inputs: Proper exceptions thrown for invalid mass/hull

- **Key Implementation Details**:
  - Three-layer algorithm: projection integrals → face integrals → volume integrals
  - Helper structures: `ProjectionIntegrals`, `FaceIntegrals` (anonymous namespace)
  - Projection plane selection based on largest normal component
  - Volume computed as byproduct for validation
  - Parallel axis theorem applied to shift to centroid
  - All code transcribed from validated prototype and volInt.c reference

### Implementation Review Phase
- **Started**: 2026-01-22
- **Completed**: 2026-01-22
- **Status**: APPROVED
- **Artifacts**:
  - `docs/designs/0026_mirtich_inertia_tensor/implementation-review.md`
  - `docs/designs/0026_mirtich_inertia_tensor/quality-gate-report.md`
- **Notes**: Implementation review completed successfully after quality gate passed. All phases passed: Design Conformance (PASS), Prototype Application (PASS), Code Quality (PASS), Test Coverage (PASS). Implementation successfully replaces inaccurate tetrahedron decomposition with Mirtich algorithm achieving machine-perfect precision (< 1e-10 error). One justified deviation (getWindingCorrectedIndices() function) addresses discovered Qhull vertex winding issue. All 13 test cases pass. Only minor issue noted: 11 sign conversion warnings following pre-existing codebase pattern. Ready for documentation and tutorial phases.

### Documentation Phase
- **Started**: 2026-01-22
- **Completed**: 2026-01-22
- **Status**: COMPLETE
- **Artifacts**:
  - `docs/msd/msd-sim/Physics/mirtich-inertia-tensor.puml` — Library diagram for Mirtich algorithm
  - `docs/designs/0026_mirtich_inertia_tensor/doc-sync-summary.md` — Documentation sync summary
  - `msd/msd-sim/src/Physics/CLAUDE.md` — Updated InertialCalculations section with algorithm details
  - `msd/msd-sim/CLAUDE.md` — Updated Recent Architectural Changes and Diagrams Index
- **Notes**: Documentation synchronized successfully. Feature diagram adapted from design folder to library documentation (removed "modified" highlighting, added implementation notes). Physics CLAUDE.md updated with comprehensive Mirtich algorithm description including three-layer computation, vertex winding correction, and validation results. msd-sim CLAUDE.md updated with Recent Architectural Changes entry documenting accuracy improvement from ~10-15% error to < 1e-10 error. Cross-references established between tickets 0023, 0025, and 0026 to document resolution of NaN bug that blocked angular physics. All diagram links verified and indexed. Ready for tutorial generation phase.

### Tutorial Documentation Phase
- **Started**: 2026-01-22
- **Completed**: 2026-01-22
- **Status**: COMPLETE
- **Artifacts**:
  - `docs/tutorials/inertia-tensor-convex-hull/README.md` — Algorithm explanation and walkthrough
  - `docs/tutorials/inertia-tensor-convex-hull/example.cpp` — Standalone C++ implementation (673 lines)
  - `docs/tutorials/inertia-tensor-convex-hull/CMakeLists.txt` — Build configuration
  - `docs/tutorials/inertia-tensor-convex-hull/presentation.html` — Reveal.js presentation (22 slides)
  - `docs/tutorials/TUTORIAL_STATE.md` — Updated tutorial generation state
- **Notes**: Generated comprehensive tutorial documentation for the Mirtich algorithm. Tutorial includes:
  - README.md with mathematical foundation (divergence theorem), three-layer algorithm walkthrough, analytical validation formulas, and implementation notes
  - Standalone C++ example that compiles without dependencies, demonstrating the complete algorithm with well-documented code
  - Reveal.js presentation with 22 slides covering learning objectives, problem definition, mathematical foundation, algorithm walkthrough, validation results, and code structure
  - Example builds and runs successfully with all tests passing (unit cube, rectangular box, regular tetrahedron)
  - Existing volInt.c reference implementation preserved in source_code directory
  - Tutorial state updated to reflect completion


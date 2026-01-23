# Inertia Tensor Computation for Convex Hulls

## Overview

This tutorial explains how to compute the moment of inertia tensor for a convex polyhedron using Brian Mirtich's algorithm from "Fast and Accurate Computation of Polyhedral Mass Properties" (1996). The algorithm uses the divergence theorem to convert volume integrals into surface integrals, achieving machine-precision accuracy.

## Prerequisites

- Basic understanding of linear algebra (vectors, matrices, cross/dot products)
- Familiarity with C++ (functions, structs, loops)
- Knowledge of integrals (conceptual understanding is sufficient)

## The Problem

Given a convex polyhedron defined by its vertices and triangular facets, compute the **moment of inertia tensor** about its center of mass. The inertia tensor is a 3x3 symmetric matrix that describes how mass is distributed relative to rotation axes:

```
    | Ixx  Ixy  Ixz |
I = | Ixy  Iyy  Iyz |
    | Ixz  Iyz  Izz |
```

Where:
- **Diagonal elements** (Ixx, Iyy, Izz): Moments of inertia about x, y, z axes
- **Off-diagonal elements** (Ixy, Iyz, Ixz): Products of inertia

The inertia tensor is essential for rigid body dynamics: it determines how an object rotates in response to applied torques via `α = I⁻¹ * τ`.

## Mathematical Foundation

### Definitions

For a solid body with density ρ, the inertia tensor about the origin is:

```
Ixx = ρ ∫∫∫(y² + z²) dV
Iyy = ρ ∫∫∫(z² + x²) dV
Izz = ρ ∫∫∫(x² + y²) dV
Ixy = -ρ ∫∫∫(xy) dV
Iyz = -ρ ∫∫∫(yz) dV
Izx = -ρ ∫∫∫(zx) dV
```

### The Divergence Theorem Insight

Computing volume integrals directly is expensive. Mirtich's key insight is to use the **divergence theorem** to convert volume integrals to surface integrals:

```
∫∫∫ div(F) dV = ∫∫ F · n dA
```

For example, to compute `∫∫∫ x² dV`:
- Choose F = (x³/3, 0, 0), so div(F) = x²
- The volume integral becomes a surface integral over the polyhedron's faces

### Three-Layer Hierarchical Computation

Mirtich organizes the computation into three layers:

**Layer 1: Projection Integrals (2D)**
For each face, project onto a 2D plane and compute line integrals around the polygon edges:
- P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb

**Layer 2: Face Integrals (3D Surface)**
Lift the 2D projection integrals to 3D surface integrals over each face:
- Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca

**Layer 3: Volume Integrals (Accumulation)**
Sum face integrals across all faces to get volume integrals:
- T0 (volume)
- T1[3] (first moments: ∫x, ∫y, ∫z)
- T2[3] (second moments: ∫x², ∫y², ∫z²)
- TP[3] (products: ∫xy, ∫yz, ∫zx)

### Parallel Axis Theorem

The algorithm computes inertia about the origin. To shift to the center of mass:

```
I_cm = I_origin - m * (r·r * Identity - r ⊗ r)
```

Where r is the center of mass position vector.

## Algorithm Walkthrough

### Step 1: For Each Facet

```cpp
for (each triangular facet f with vertices v0, v1, v2 and normal n):

    // 1a. Select projection plane based on largest normal component
    //     If |nx| is largest, project onto YZ plane (A=Y, B=Z, C=X)
    //     If |ny| is largest, project onto XZ plane (A=Z, B=X, C=Y)
    //     If |nz| is largest, project onto XY plane (A=X, B=Y, C=Z)

    // 1b. Compute projection integrals (2D line integrals over edges)
    P = computeProjectionIntegrals(facet, A, B);

    // 1c. Lift to face integrals (3D surface)
    F = computeFaceIntegrals(facet, P, A, B, C);

    // 1d. Accumulate into volume integrals
    T0 += contribution to volume
    T1 += contribution to first moments
    T2 += contribution to second moments
    TP += contribution to products
```

### Step 2: Finalize Volume Integrals

```cpp
T1 /= 2.0;  // First moments
T2 /= 3.0;  // Second moments
TP /= 2.0;  // Products
```

### Step 3: Compute Inertia About Origin

```cpp
Ixx = density * (T2[Y] + T2[Z]);
Iyy = density * (T2[Z] + T2[X]);
Izz = density * (T2[X] + T2[Y]);
Ixy = -density * TP[X];
Iyz = -density * TP[Y];
Izx = -density * TP[Z];
```

### Step 4: Shift to Center of Mass

```cpp
r = T1 / T0;  // Center of mass

I_cm[xx] -= mass * (r.y² + r.z²);
I_cm[yy] -= mass * (r.z² + r.x²);
I_cm[zz] -= mass * (r.x² + r.y²);
I_cm[xy] += mass * r.x * r.y;
I_cm[yz] += mass * r.y * r.z;
I_cm[zx] += mass * r.z * r.x;
```

## Implementation Notes

### Tutorial vs Production Code

This tutorial provides a **standalone, readable implementation** of the Mirtich algorithm.

| Aspect | Tutorial (`example.cpp`) | Production (`InertialCalculations.cpp`) |
|--------|--------------------------|----------------------------------------|
| Dependencies | None (self-contained) | Eigen, ConvexHull, Qhull |
| Goal | Clarity and education | Performance and integration |
| Error handling | Minimal | Full validation |
| Vertex winding | Manual (CCW) | Auto-corrected |

### Vertex Winding Requirement

The algorithm requires that facet vertices are ordered **counter-clockwise when viewed from outside** the polyhedron. This ensures:
- The cross product of edges points outward (aligns with normal)
- Volume contributions have correct sign

The production implementation includes `getWindingCorrectedIndices()` to handle Qhull's vertex ordering, which may not match this convention.

### Numerical Considerations

- **Projection plane selection**: Choosing the plane where the normal has largest component avoids division by near-zero values
- **Large coordinates**: Floating-point precision degrades at large coordinate offsets (errors ~1e-8 at offset 1000+)
- **Degenerate facets**: Near-zero area facets can cause numerical instability

## Analytical Validation

The implementation is validated against known analytical solutions:

**Unit Cube** (1×1×1, mass m, centered at origin):
```
Ixx = Iyy = Izz = m/6 ≈ 0.166667
Ixy = Iyz = Izx = 0
```

**Rectangular Box** (a×b×c, mass m, centered at origin):
```
Ixx = m(b² + c²)/12
Iyy = m(a² + c²)/12
Izz = m(a² + b²)/12
```

**Regular Tetrahedron** (edge length L, mass m):
```
Ixx = Iyy = Izz = m*L²/20  (when centered and aligned)
```

## References

1. Mirtich, B. (1996). "Fast and Accurate Computation of Polyhedral Mass Properties." Journal of Graphics Tools, 1(2), 31-50.
   - Original paper describing the algorithm
   - Available at: https://people.eecs.berkeley.edu/~jfc/mirtich/massProps.html

2. `source_code/volInt.c` — Mirtich's public domain reference implementation

3. `source_code/README` — Original documentation from Mirtich

## See Also

- `msd/msd-sim/src/Physics/RigidBody/InertialCalculations.cpp` — Production implementation
- `msd/msd-sim/src/Physics/RigidBody/InertialCalculations.hpp` — API documentation
- `tickets/0026_mirtich_inertia_tensor.md` — Implementation ticket with full history

## Building and Running

```bash
cd docs/tutorials/inertia-tensor-convex-hull
mkdir -p build && cd build
cmake ..
make
./example
```

Expected output shows inertia tensors for unit cube, rectangular box, and tetrahedron with comparison to analytical solutions.

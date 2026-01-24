# Expanding Polytope Algorithm (EPA) for Collision Contact Information

## Overview

The Expanding Polytope Algorithm (EPA) computes detailed contact information—penetration depth, contact normal, and contact point—from two colliding convex shapes. It builds upon the GJK (Gilbert-Johnson-Keerthi) collision detection algorithm, taking the final simplex from GJK and expanding it to find the closest point on the Minkowski difference boundary.

## Prerequisites

- Basic linear algebra (vectors, dot products, cross products)
- Understanding of convex geometry
- Familiarity with GJK algorithm (or read the GJK primer below)
- C++ knowledge: classes, vectors, basic algorithms

## The Problem

When two convex objects collide, game engines and physics simulations need more than just "yes, they're colliding." They need:

1. **Penetration depth**: How far the objects overlap (to separate them)
2. **Contact normal**: Which direction to push them apart
3. **Contact point**: Where on the objects the collision occurs (for torque calculation)

GJK tells us *if* objects collide, but not *how much* or *where*. EPA fills this gap.

## Mathematical Foundation

### The Minkowski Difference

For two convex shapes A and B, their **Minkowski difference** is:

```
A ⊖ B = { a - b | a ∈ A, b ∈ B }
```

This is also a convex shape. The key insight:

> **Theorem**: Shapes A and B intersect if and only if their Minkowski difference contains the origin.

Why? If A and B share a point p (p ∈ A and p ∈ B), then p - p = 0 is in A ⊖ B.

### From GJK to EPA

GJK iteratively builds a simplex (up to 4 vertices in 3D) inside the Minkowski difference. If this simplex contains the origin, GJK reports collision and returns the simplex.

EPA takes this simplex and expands it outward until it approximates the Minkowski difference boundary near the origin. The closest point on this boundary to the origin gives us:

- **Penetration depth** = distance from origin to closest point
- **Contact normal** = direction from origin to closest point (normalized)

### The Support Function

Both GJK and EPA rely on the **support function**:

```
support(shape, direction) = point in shape furthest in the given direction
```

For the Minkowski difference:

```
support(A ⊖ B, d) = support(A, d) - support(B, -d)
```

This lets us query points on the Minkowski boundary without explicitly computing all A ⊖ B vertices.

## Algorithm Walkthrough

### Step 1: Initialize Polytope from GJK Simplex

GJK returns a tetrahedron (4 vertices) that contains the origin. We create 4 triangular faces:

```
Tetrahedron with vertices A, B, C, D:
  - Face ABC (away from D)
  - Face ACD (away from B)
  - Face ADB (away from C)
  - Face BDC (away from A)
```

For each face, compute:
- **Normal**: Cross product of two edges, pointing outward (away from origin)
- **Distance**: Distance from origin to the face plane

### Step 2: Find Closest Face

Iterate through all faces and find the one with minimum distance to the origin. This face represents our current best guess for the closest point on the Minkowski boundary.

### Step 3: Expand Polytope

Query a new support point in the direction of the closest face's normal:

```cpp
newPoint = supportMinkowski(closestFace.normal);
```

**Convergence check**: If the new point is within epsilon of the face distance, we've found the closest point—stop here.

Otherwise, the new point is further out, so we must expand the polytope:

1. **Find visible faces**: Faces where the new point is "in front of" (positive side of the plane)
2. **Build horizon edges**: Edges shared by exactly one visible face (the silhouette)
3. **Remove visible faces**: They're now inside the expanded polytope
4. **Add new faces**: Connect the new point to each horizon edge

### Step 4: Extract Contact Information

Once converged, the closest face gives us:

```cpp
penetrationDepth = closestFace.distance;
contactNormal = closestFace.normal;  // Points from A toward B
contactPoint = centroid(closestFace);  // Approximate contact location
```

## Implementation Notes

### Production vs Tutorial Code

| Production (`EPA.cpp`) | Tutorial (`example.cpp`) |
|------------------------|--------------------------|
| Uses `Coordinate`, `CoordinateRate` from Eigen | Simple `Vec3` struct |
| Integrates with `AssetPhysical`, `ReferenceFrame` | Standalone shapes |
| Handles world-space transforms | Local-space only |
| Robust simplex completion for edge cases | Assumes valid tetrahedron |

### Key Numerical Considerations

1. **Epsilon tolerance**: Used for convergence check and duplicate detection
2. **Normal orientation**: Must consistently point outward (away from origin)
3. **Degenerate faces**: Coplanar vertices produce zero-area faces—skip them
4. **Edge equality**: Compare edges order-independently (AB == BA)

### Complexity

- **Per iteration**: O(F) for finding closest face, O(F) for visibility check
- **Total**: O(I × F) where I = iterations (typically 4-20), F = faces (grows with I)
- **Memory**: Stores vertices and faces of expanding polytope

## Pseudocode

```
function EPA(simplex, shapeA, shapeB):
    polytope = createTetrahedronFromSimplex(simplex)

    for iteration = 1 to maxIterations:
        closestFace = findClosestFaceToOrigin(polytope)

        newPoint = supportMinkowski(shapeA, shapeB, closestFace.normal)

        // Convergence check
        if dot(newPoint, closestFace.normal) - closestFace.distance < epsilon:
            return CollisionResult(
                normal = closestFace.normal,
                depth = closestFace.distance,
                point = centroid(closestFace)
            )

        // Expand polytope
        visibleFaces = findVisibleFaces(polytope, newPoint)
        horizonEdges = buildHorizon(visibleFaces)
        removeFaces(polytope, visibleFaces)
        addVertex(polytope, newPoint)

        for each edge in horizonEdges:
            addFace(polytope, edge.v0, edge.v1, newPointIndex)

    error("EPA did not converge")
```

## Visualization

```
Initial Tetrahedron          After 1 Expansion           After 2 Expansions
      (4 faces)                 (6 faces)                   (8 faces)

       /\                        /|\                         /|\
      /  \                      / | \                       / | \
     /    \         →          /  |  \          →          /  *  \
    /  *   \                  / * |   \                   / * | * \
   /________\                /___ |____\                 /___ * ____\

   * = origin                New vertex added           Polytope closer
                             in closest face            to boundary
                             direction
```

## References

- Ericson, "Real-Time Collision Detection" (2004), Chapter 9.3
- van den Bergen, "Collision Detection in Interactive 3D Environments" (2003)
- Casey Muratori, "Implementing GJK/EPA" (Handmade Hero)

## See Also

- `msd-sim/src/Physics/EPA.hpp` — Production EPA implementation
- `msd-sim/src/Physics/GJK.hpp` — GJK collision detection
- `msd-sim/src/Physics/CollisionHandler.hpp` — GJK+EPA orchestration
- [Inertia Tensor Tutorial](../inertia-tensor-convex-hull/) — Related physics computation

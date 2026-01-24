# Tutorial Generation State

## Session Info
- Last updated: 2026-01-23
- Feature: 0027a_expanding_polytope_algorithm

## Completed Topics
- [x] Inertia Tensor for Convex Hulls (Mirtich Algorithm) — docs/tutorials/inertia-tensor-convex-hull/
- [x] Expanding Polytope Algorithm (EPA) — docs/tutorials/expanding-polytope-algorithm/

## Pending Topics
- [ ] None currently pending

## Codebase Mappings

### Inertia Tensor Tutorial
| Production | Tutorial | Status |
|------------|----------|--------|
| `InertialCalculations::computeInertiaTensorAboutCentroid()` | `computeInertiaTensorAboutCentroid()` | Complete |
| `msd_sim::ConvexHull` | `Mesh` | Complete |
| `msd_sim::ConvexHull::Facet` | `Facet` | Complete |
| `Eigen::Matrix3d` | `Mat3` | Complete |
| `msd_sim::Coordinate` | `Vec3` | Complete |

### EPA Tutorial
| Production | Tutorial | Status |
|------------|----------|--------|
| `msd_sim::EPA` | `epa()` function | Complete |
| `msd_sim::EPA::EPAFace` | `EPAFace` struct | Complete |
| `msd_sim::EPA::EPAEdge` | `EPAEdge` struct | Complete |
| `msd_sim::CollisionResult` | `CollisionResult` struct | Complete |
| `msd_sim::CollisionHandler` | `gjkIntersects()` + `epa()` | Complete |
| `msd_sim::Coordinate` | `Vec3` struct | Complete |
| `msd_sim::ConvexHull` | `ConvexShape` struct | Complete |
| `msd_sim::GJK` | `gjkIntersects()` (simplified) | Complete |

## Tutorial Structure
```
docs/tutorials/
├── TUTORIAL_STATE.md
├── inertia-tensor-convex-hull/
│   ├── README.md
│   ├── example.cpp
│   ├── CMakeLists.txt
│   ├── presentation.html
│   └── source_code/
└── expanding-polytope-algorithm/
    ├── README.md              # Algorithm explanation (Minkowski, GJK, EPA)
    ├── example.cpp            # Standalone C++ implementation (~500 lines)
    ├── CMakeLists.txt         # Build configuration (C++17, no deps)
    ├── presentation.html      # Reveal.js slides (22 slides)
    └── build/                 # CMake build directory
        └── example            # Compiled executable
```

## Presentation Progress
- Total slides created: 44 (22 for inertia tensor + 22 for EPA)
- Topics covered in EPA presentation:
  1. Title and learning objectives
  2. The problem (what EPA solves)
  3. Minkowski difference explanation
  4. From GJK to EPA
  5. Support function
  6. EPA algorithm overview
  7. Step 1: Initialize polytope
  8. Face data structure
  9. Step 2: Find closest face
  10. Step 3: Query support point
  11. Step 4: Convergence check
  12. Step 5: Expansion overview
  13. Visibility test
  14. Horizon edges
  15. Adding new faces
  16. Expansion visualization
  17. Contact extraction
  18. Performance characteristics
  19. Production vs tutorial code
  20. Key takeaways
  21. References
  22. Try it yourself

## Validation Results

### Inertia Tensor Tutorial
- Unit cube: PASS (Ixx = Iyy = Izz = m/6, error < 1e-10)
- Rectangular box 2×3×4: PASS (matches analytical, error < 1e-10)
- Regular tetrahedron: PASS (diagonal symmetry, error < 1e-10)

### EPA Tutorial
- Overlapping cubes (X-axis): PASS (depth = 0.3m, normal = (1,0,0), error < 1e-16)
- Overlapping cubes (Y-axis): PASS (depth = 0.4m, normal = (0,1,0))
- Non-overlapping cubes: PASS (correctly reports no collision)
- Convergence: 2-3 iterations for simple cases

## Notes for Next Session
- EPA tutorial covers complete algorithm: Minkowski difference → GJK → EPA → contact extraction
- Simplified GJK in tutorial builds tetrahedron directly (production uses iterative approach)
- Tutorial demonstrates horizon edge construction and polytope expansion
- Reveal.js presentation uses CDN-hosted libraries for portability
- Production implementation includes world-space transforms via ReferenceFrame
- Tutorial keeps everything in local coordinates for clarity

## Related Tickets
- 0026_mirtich_inertia_tensor — Inertia tensor tutorial
- 0027a_expanding_polytope_algorithm — EPA tutorial (this)
- 0027_collision_response_system — Will use EPA output for physics response

# Design: Vertex-Face Contact Manifold Generation

## Summary

This design adds multi-point contact manifold generation for vertex-face collision geometry to fix energy injection from single-point friction contacts. When a cube rests on a corner (vertex-face contact), the current system produces a single contact point at an offset lever arm. Friction forces at this point create uncompensated yaw torque, causing the contact to jump between corners and inject energy. The fix generates 3-4 contact points spanning the contact patch, ensuring friction torques cancel and eliminating energy injection.

## Root Cause Recap (from 0055b Investigation)

**Problem**: Tilted cube with compound tilt (tiltX=0.01, tiltY=π/3) exhibits:
- 89% single-point contacts (vs 0% for pure pitch)
- 4.8 million times more yaw rotation
- +7.7% kinetic energy injection (peak 134.7 J from initial 125 J)
- Spurious 20.7m cross-axis displacement from 0.01 rad perturbation

**Root cause chain**:
1. Compound tilt → cube rests on vertex
2. EPA/SAT produce 1 contact point (geometrically correct but physically inadequate)
3. Friction Jacobian angular coupling `rA × t` creates yaw torque from offset contact
4. Yaw rotates cube → contact jumps to new corner
5. Oscillating friction direction → net positive work → energy injection

**Complete investigation**: `docs/investigations/0055b_friction_direction_root_cause/investigation-log.md`

## Architecture Changes

### PlantUML Diagram
See: `./0055c_friction_direction_fix.puml`

### New Components

#### VertexFaceDetector

- **Purpose**: Classify contact geometry (face-face, edge-edge, vertex-face, vertex-vertex) by analyzing polygon sizes and face alignment
- **Header location**: `msd/msd-sim/src/Physics/Collision/VertexFaceDetector.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Collision/VertexFaceDetector.cpp`
- **Key interfaces**:
  ```cpp
  enum class ContactType {
    FaceFace,       // Both sides have >= 3 vertices
    EdgeEdge,       // Both sides have 2 vertices
    VertexFace,     // One side has 1 vertex, other has >= 3
    VertexVertex,   // Both sides have 1 vertex
    Unknown         // Fallback case
  };

  class VertexFaceDetector {
  public:
    VertexFaceDetector() = default;

    // Analyze polygon sizes and classify contact geometry
    [[nodiscard]] ContactType detectContactType(
      size_t refVertCount,
      size_t incVertCount) const;

    // Convenience method for vertex-face check
    [[nodiscard]] bool isVertexFaceContact(
      size_t refVertCount,
      size_t incVertCount) const;

    VertexFaceDetector(const VertexFaceDetector&) = default;
    VertexFaceDetector& operator=(const VertexFaceDetector&) = default;
    VertexFaceDetector(VertexFaceDetector&&) noexcept = default;
    VertexFaceDetector& operator=(VertexFaceDetector&&) noexcept = default;
    ~VertexFaceDetector() = default;
  };
  ```
- **Dependencies**: None (pure logic based on vertex counts)
- **Thread safety**: Stateless, safe for concurrent use
- **Error handling**: Returns `ContactType::Unknown` for unexpected cases

#### VertexFaceManifoldGenerator

- **Purpose**: Generate 3-4 contact points for vertex-face geometry by projecting reference face vertices onto contact plane around incident vertex
- **Header location**: `msd/msd-sim/src/Physics/Collision/VertexFaceManifoldGenerator.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Collision/VertexFaceManifoldGenerator.cpp`
- **Key interfaces**:
  ```cpp
  class VertexFaceManifoldGenerator {
  public:
    // Configuration for manifold generation
    struct Config {
      double depthTolerance{1e-6};    // Depth consistency tolerance [m]
      size_t maxContacts{4};          // Maximum contacts to generate
    };

    explicit VertexFaceManifoldGenerator(Config config = Config{});

    // Generate contact manifold for vertex-face geometry
    //
    // refFaceVertices: vertices of the reference face (3-4 points, world space)
    // incidentVertex: the single incident vertex (world space)
    // contactNormal: contact normal (from incident toward reference)
    // epaDepth: EPA penetration depth for consistency
    //
    // Returns: number of valid contacts generated (0-4)
    [[nodiscard]] size_t generate(
      const std::vector<Coordinate>& refFaceVertices,
      const Coordinate& incidentVertex,
      const Vector3D& contactNormal,
      double epaDepth,
      std::array<ContactPoint, 4>& contacts) const;

    VertexFaceManifoldGenerator(const VertexFaceManifoldGenerator&) = default;
    VertexFaceManifoldGenerator& operator=(const VertexFaceManifoldGenerator&) = default;
    VertexFaceManifoldGenerator(VertexFaceManifoldGenerator&&) noexcept = default;
    VertexFaceManifoldGenerator& operator=(VertexFaceManifoldGenerator&&) noexcept = default;
    ~VertexFaceManifoldGenerator() = default;

  private:
    Config config_;

    // Project reference face vertices onto contact plane
    [[nodiscard]] std::vector<Coordinate> projectFaceOntoPlane(
      const std::vector<Coordinate>& faceVertices,
      const Coordinate& planePoint,
      const Vector3D& planeNormal) const;

    // Compute per-point penetration depth from incident vertex to face
    [[nodiscard]] double computePointDepth(
      const Coordinate& refPoint,
      const Coordinate& incidentVertex,
      const Vector3D& contactNormal) const;
  };
  ```
- **Dependencies**:
  - `Coordinate` (Environment module)
  - `Vector3D` (DataTypes module)
  - `ContactPoint` (CollisionResult.hpp)
- **Thread safety**: Stateless (config is const), safe for concurrent use
- **Error handling**:
  - Returns 0 contacts for degenerate face geometry (< 3 vertices)
  - Validates projected points lie within depth tolerance of plane
  - Falls back to fewer contacts if projection produces invalid geometry

**Algorithm overview**:
1. **Project reference face**: Project each vertex of the reference face onto the contact plane (plane containing incident vertex, perpendicular to contact normal)
2. **Validate projection**: Ensure projected points form a valid convex polygon (3-4 vertices)
3. **Compute depths**: For each projected point, compute penetration depth = distance from incident vertex along contact normal
4. **Consistency check**: Verify depths are within tolerance of EPA depth
5. **Build contacts**: Create ContactPoint pairs (refPoint, incidentVertex, depth) for each projected vertex
6. **Limit count**: Return min(projectedPoints.size(), 4) contacts

**Geometric insight**: For a vertex-face contact, the reference face vertices naturally span the contact patch. Projecting them onto the contact plane gives us distributed contact points that ensure yaw torques from friction cancel.

### Modified Components

#### CollisionHandler

- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionHandler.hpp/.cpp`
- **Changes required**:
  1. Add new private method `generateVertexFaceManifold()`:
     ```cpp
     [[nodiscard]] CollisionResult generateVertexFaceManifold(
       const AssetPhysical& assetA,
       const AssetPhysical& assetB,
       const SATResult& sat) const;
     ```
  2. Modify `buildSATContact()` to detect vertex-face geometry and delegate to `generateVertexFaceManifold()`
  3. Add member instances:
     ```cpp
     VertexFaceDetector vertexFaceDetector_;
     VertexFaceManifoldGenerator vertexFaceManifoldGenerator_;
     ```
- **Backward compatibility**:
  - Existing API unchanged (still returns `std::optional<CollisionResult>`)
  - Existing behavior preserved for face-face and edge-edge contacts
  - Single-point fallback retained for vertex-vertex and unknown cases

**Integration logic**:
```cpp
CollisionResult CollisionHandler::buildSATContact(...) const {
  // Existing witness point computation...

  // Detect contact geometry
  auto refFaceVerts = extractFaceVertices(refAsset, sat.normal);
  auto incVerts = extractFaceVertices(incAsset, -sat.normal);

  if (vertexFaceDetector_.isVertexFaceContact(refFaceVerts.size(), incVerts.size())) {
    return generateVertexFaceManifold(assetA, assetB, sat);
  }

  // Existing single-point fallback...
}
```

#### EPA

- **Current location**: `msd/msd-sim/src/Physics/Collision/EPA.hpp/.cpp`
- **Changes required**:
  1. Add new private method `generateVertexFaceManifold()`:
     ```cpp
     [[nodiscard]] size_t generateVertexFaceManifold(
       const Facet& epaFace,
       const std::vector<Coordinate>& refVerts,
       const std::vector<Coordinate>& incVerts,
       bool refIsA,
       std::array<ContactPoint, 4>& contacts) const;
     ```
  2. Modify `extractContactManifold()` at degenerate case (line 574):
     ```cpp
     if (refVerts.size() < 3 || incidentPoly.size() < 3) {
       // Try vertex-face manifold generation
       if (vertexFaceDetector_.isVertexFaceContact(refVerts.size(), incidentPoly.size())) {
         size_t count = generateVertexFaceManifold(epaFace, refVerts, incidentPoly, refIsA, contacts);
         if (count >= 3) {
           return count;
         }
       }

       // Existing edge contact and single-point fallbacks...
     }
     ```
  3. Add member instances:
     ```cpp
     VertexFaceDetector vertexFaceDetector_;
     VertexFaceManifoldGenerator vertexFaceManifoldGenerator_;
     ```
- **Backward compatibility**:
  - Existing face-face clipping (Sutherland-Hodgman) unchanged
  - Edge contact generation (0040c) unchanged
  - Single-point fallback retained as last resort

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| VertexFaceDetector | CollisionHandler | Composition | Member instance, called in `buildSATContact()` |
| VertexFaceDetector | EPA | Composition | Member instance, called in `extractContactManifold()` |
| VertexFaceManifoldGenerator | CollisionHandler | Composition | Member instance, called in new `generateVertexFaceManifold()` method |
| VertexFaceManifoldGenerator | EPA | Composition | Member instance, called in new `generateVertexFaceManifold()` method |

**Key insight**: Both EPA and SAT fallback paths can produce vertex-face contacts. The fix applies at both integration points for consistency.

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `TiltedCubeTrajectoryTest.cpp` | All 19 tests | Expected to fix 3 failures | Verify all pass after implementation |
| `TiltedCubeTrajectoryTest.cpp` | `Diag_ContactCount_And_LeverArm` | Contact count distribution will change | Update assertions: expect >50% four-point contacts |
| `PhysicsTest.cpp` | All collision tests | No expected changes | Monitor for regressions |
| `CollisionTest.cpp` (if exists) | EPA/SAT tests | No expected changes | Monitor for regressions |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| VertexFaceDetector | `DetectContactType_FaceFace` | Correctly identifies face-face (>= 3 verts both sides) |
| VertexFaceDetector | `DetectContactType_EdgeEdge` | Correctly identifies edge-edge (2 verts both sides) |
| VertexFaceDetector | `DetectContactType_VertexFace` | Correctly identifies vertex-face (1 vert one side, >= 3 other) |
| VertexFaceDetector | `DetectContactType_VertexVertex` | Correctly identifies vertex-vertex (1 vert both sides) |
| VertexFaceManifoldGenerator | `Generate_CubeOnFloor_FourContacts` | Cube corner on flat floor produces 4 contacts |
| VertexFaceManifoldGenerator | `Generate_TriangleVertex_ThreeContacts` | Vertex on triangular face produces 3 contacts |
| VertexFaceManifoldGenerator | `Generate_DepthConsistency` | All contact depths within tolerance of EPA depth |
| VertexFaceManifoldGenerator | `Generate_DegenerateFace_ReturnsZero` | < 3 face vertices returns 0 contacts |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `CollisionHandler_VertexFaceSAT_MultiPoint` | CollisionHandler, VertexFaceManifoldGenerator, SAT | SAT fallback produces multi-point manifold for vertex-face |
| `EPA_VertexFaceFallback_MultiPoint` | EPA, VertexFaceManifoldGenerator | EPA degenerate case produces multi-point manifold for vertex-face |
| `TiltedCube_CompoundTilt_NoEnergyInjection` | Full collision pipeline | Compound tilt cube does not inject energy (KE decreases) |
| `TiltedCube_CompoundTilt_FourPointContacts` | Full collision pipeline | Compound tilt produces >50% four-point contacts |

#### Regression Tests (Existing Suite)

| Test Suite | Baseline | Expected After Fix | Action |
|-----------|----------|-------------------|--------|
| Full test suite | 693 passing | 693 passing (no regressions) | Run full suite and verify |
| TiltedCubeTrajectory | 16/19 passing | 19/19 passing (3 fixes) | Verify all pass |

## Open Questions

### Design Decisions (Human Input Needed)

1. **Depth assignment strategy for contact points**
   - Option A: Use EPA depth for all points — Pros: Simple, consistent with EPA result. Cons: May not reflect true per-point penetration for tilted faces.
   - Option B: Compute per-point depth via projection — Pros: Physically accurate per-point depth. Cons: Slightly more computation, potential inconsistency with EPA.
   - Recommendation: **Option A** for initial implementation. EPA depth is the global penetration depth along the contact normal, which is appropriate for all points in the manifold. Per-point depth (Option B) can be added later if constraint solver stability requires it.

2. **Member ownership vs function parameters**
   - Option A: CollisionHandler and EPA own detector/generator members — Pros: No parameter passing, single instantiation. Cons: Slight memory overhead.
   - Option B: Pass detector/generator as function parameters — Pros: More flexible, testable. Cons: Parameter proliferation.
   - Recommendation: **Option A** (member ownership). The detector is stateless and the generator is lightweight, so memory overhead is negligible. Member ownership simplifies call sites.

3. **Edge case: Degenerate vertex-face (parallel/near-parallel surfaces)**
   - Question: What if the reference face is nearly parallel to the contact normal (edge-on view)?
   - Option A: Project anyway, may produce degenerate line segment — Pros: Simple. Cons: May produce < 3 contacts.
   - Option B: Detect and fall back to single-point or edge contact — Pros: Robust. Cons: Added complexity.
   - Recommendation: **Option B**. Add alignment check: if `abs(refFaceNormal · contactNormal) < threshold`, fall back to existing edge/single-point logic. This prevents degenerate projections.

### Prototype Required

1. **Contact depth consistency validation**
   - Uncertainty: Will constraint solver remain stable if all contact points share the same EPA depth, or does it need per-point depth variation?
   - Validation: Prototype with Option A (uniform EPA depth) and run full test suite. If instability arises, implement Option B (per-point depth).

2. **Performance impact measurement**
   - Uncertainty: What is the actual performance overhead of vertex-face manifold generation vs single-point fallback?
   - Validation: Benchmark collision detection on tilted cube scenarios (vertex-face heavy) before and after. Target < 10% overhead for vertex-face cases, 0% for others.

### Requirements Clarification

1. **Single-point fallback retention**
   - Question: Should single-point contact fallback be completely removed, or retained for vertex-vertex and unknown cases?
   - Clarification needed: Confirm that vertex-vertex contacts (rare) can safely use single-point, or if they also need manifold generation.
   - Recommendation: Retain single-point fallback for vertex-vertex and unknown cases. Vertex-vertex is rare and does not suffer from yaw coupling (no lever arm asymmetry).

2. **Contact count expectations for test assertions**
   - Question: What percentage of four-point contacts is acceptable for compound tilt case?
   - Current state: 0% four-point, 89% single-point
   - Target: >50% four-point, <1% single-point (from 0055b acceptance criteria)
   - Clarification: Confirm these targets are appropriate, or if they should be adjusted based on prototype results.

## Performance Considerations

### Expected Overhead

| Operation | Current Cost (FLOPs) | After Fix (FLOPs) | Frequency | Impact |
|-----------|---------------------|------------------|-----------|--------|
| Face-face collision | ~2000-5000 (EPA + clip) | No change | ~80% of contacts | 0% overhead |
| Edge-edge collision | ~2000 (EPA + edge gen) | No change | ~10% of contacts | 0% overhead |
| Vertex-face collision | ~2000 (EPA) + 50 (single point) | ~2000 (EPA) + 300 (manifold gen) | ~10% of contacts | +250 FLOPs per vertex-face |
| Vertex-vertex collision | ~2000 (EPA) + 50 (single point) | No change | <1% of contacts | 0% overhead |

**Net impact**: < 5% overhead on vertex-face-heavy scenarios (tilted cubes), 0% overhead on typical scenarios (face-face dominant).

### Algorithmic Complexity

- **VertexFaceDetector::detectContactType()**: O(1) — simple vertex count comparison
- **VertexFaceManifoldGenerator::generate()**: O(n) where n = reference face vertex count (typically 3-4)
  - Project n vertices: O(n)
  - Compute n depths: O(n)
  - Build n contact pairs: O(n)
  - Total: O(n) ≈ O(1) for fixed n ≤ 4

**Conclusion**: Negligible asymptotic cost increase. Absolute cost increase is ~250 FLOPs per vertex-face contact, which is acceptable given the 10× improvement in physical accuracy.

## Implementation Notes

### Key Geometric Relationships

For a vertex-face contact:
- **Incident vertex**: The single vertex from object A (or B) that penetrates
- **Reference face**: The face from object B (or A) that is penetrated
- **Contact normal**: Points from incident vertex toward reference face
- **Contact plane**: Plane containing incident vertex, perpendicular to contact normal

**Manifold generation strategy**:
```
1. Project reference face vertices onto contact plane:
   projectedPoint = faceVertex - ((faceVertex - incidentVertex) · normal) * normal

2. Compute depth for each projected point:
   depth = (projectedPoint - incidentVertex) · normal

3. Create contact pairs:
   ContactPoint{projectedPoint, incidentVertex, depth}
```

**Yaw cancellation proof**: For a symmetric reference face (e.g., square) with vertices V1, V2, V3, V4 projected around incident vertex C:
```
Lever arms: r1 = V1 - C, r2 = V2 - C, r3 = V3 - C, r4 = V4 - C
Friction forces: F1, F2, F3, F4 (tangent direction t)
Yaw torques: τ1 = r1 × F1, τ2 = r2 × F2, τ3 = r3 × F3, τ4 = r4 × F4

For symmetric face: Σ(ri × Fi) ≈ 0 (opposing corners cancel)
```

This is the physical basis for why multi-point contacts eliminate yaw-driven energy injection.

### Coding Standards Applied

- **Initialization**: Brace initialization `{}` throughout, `NaN` for uninitialized floats if needed
- **Naming**: `PascalCase` for classes (`VertexFaceDetector`), `camelCase` for methods (`detectContactType`), `snake_case_` for members (`config_`)
- **Return Values**: `detectContactType()` returns enum by value, `generate()` returns count and fills output array (matches existing `extractContactManifold` pattern)
- **Memory**: Value semantics for config, no dynamic allocation in hot path
- **Error Handling**: Return 0 contacts for degenerate cases, validate inputs, no exceptions in collision path

## Rollout Plan

### Phase 1: Core Implementation
1. Implement `VertexFaceDetector` with unit tests
2. Implement `VertexFaceManifoldGenerator` with unit tests
3. Integrate into `CollisionHandler::buildSATContact()` with feature flag (disabled by default)
4. Integrate into `EPA::extractContactManifold()` with same feature flag
5. Run baseline tests (expect 693 passing, feature disabled)

### Phase 2: Validation
1. Enable feature flag
2. Run full test suite (expect 693 passing + 3 fixes in TiltedCubeTrajectory)
3. Run benchmarks (expect < 10% overhead on vertex-face cases)
4. Visual validation in GUI (drop tilted cube, verify trajectory looks correct)

### Phase 3: Finalization
1. Remove feature flag if validation passes
2. Update `TiltedCubeTrajectoryTest.cpp` assertions for new contact count distribution
3. Document findings in investigation log
4. Update ticket 0055c to "Complete"

## References

- **Investigation log**: `docs/investigations/0055b_friction_direction_root_cause/investigation-log.md`
- **Test suite**: Ticket 0055a (`tickets/0055a_tilted_cube_trajectory_test_suite.md`)
- **Edge contact manifold**: Ticket 0040c (`docs/designs/0040c-edge-contact-manifold/design.md`)
- **SAT fallback**: Ticket 0047 (`docs/designs/0047_face_contact_manifold_generation/design.md`)
- **Box2D manifold generation**: `b2CollidePolygons()` in `b2_collision.cpp`
- **Bullet manifold generation**: `btBoxBoxDetector::getClosestPoints()` in `btBoxBoxDetector.cpp`
- **Ericson (2004)**: "Real-Time Collision Detection" Chapter 5

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-11
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | Classes use PascalCase (VertexFaceDetector, VertexFaceManifoldGenerator), methods use camelCase (detectContactType, generate), members use snake_case_ (config_) |
| Namespace organization | ✓ | New components correctly placed in msd_sim namespace (implied by integration with CollisionHandler, EPA) |
| File structure | ✓ | Follows msd/msd-sim/src/Physics/Collision/ pattern, consistent with existing components |
| Dependency direction | ✓ | Dependencies flow correctly: new components depend on Coordinate/Vector3D (lower-level), CollisionHandler/EPA depend on new components (composition), no cycles introduced |

#### C++ Design Quality
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | No resources to manage, relies on value semantics and standard containers |
| Smart pointer appropriateness | ✓ | No smart pointers needed - components are stateless or hold simple config by value |
| Value/reference semantics | ✓ | Config held by value, assets passed by const reference, vectors passed by const reference - appropriate for each use case |
| Rule of 0/3/5 | ✓ | Both new classes explicitly default all special members (Rule of Zero), matches project pattern |
| Const correctness | ✓ | generate() and detectContactType() methods are const (stateless), input parameters are const references where appropriate |
| Exception safety | ✓ | No exceptions in collision path, error handling via return values (0 contacts for degenerate cases) |
| Initialization | ✓ | Brace initialization specified throughout, NaN for uninitialized floats mentioned |
| Return values | ✓ | generate() returns count + fills output array (matches existing extractContactManifold pattern), detectContactType() returns enum by value |

#### Feasibility
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | Dependencies on Coordinate, Vector3D, ContactPoint are all existing types, no circular dependencies |
| Template complexity | ✓ | No templates used, straightforward concrete classes |
| Memory strategy | ✓ | No dynamic allocation in hot path, uses stack-allocated std::array for output contacts |
| Thread safety | ✓ | Both components are stateless (config is const), safe for concurrent use as documented |
| Build integration | ✓ | New .hpp/.cpp files added to msd-sim, straightforward CMake integration |

#### Testability
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | Both components can be instantiated standalone with minimal dependencies (Coordinate, Vector3D) |
| Mockable dependencies | ✓ | Components have no external dependencies requiring mocking - pure computational logic |
| Observable state | ✓ | generate() returns contacts array that can be inspected, detectContactType() returns observable enum |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Contact depth consistency: uniform EPA depth vs per-point depth may affect constraint solver stability | Technical | Medium | Medium | Start with Option A (uniform EPA depth), measure stability in full test suite. If instability detected, implement Option B (per-point depth computation). Design already supports both approaches. | No |
| R2 | Degenerate vertex-face geometry (reference face nearly parallel to contact normal) may produce < 3 contacts | Technical | Low | Low | Design already recommends Option B (alignment check + fallback). Implementation should include `abs(refFaceNormal · contactNormal) < threshold` check before projection. | No |
| R3 | Performance overhead on vertex-face contacts (~250 FLOPs) may be higher than estimated in practice | Performance | Low | Low | Benchmark before/after on tilted cube scenarios. Target < 10% overhead for vertex-face-heavy cases, 0% for others. Abort if > 15% overhead. | Yes |
| R4 | EPA method signature shows public members (assetA_, assetB_, epsilon_, vertices_, faces_) - appears to violate encapsulation | Maintenance | High | Low | This is existing code pattern - EPA already exposes these as public members. New methods follow existing patterns. No change needed for this ticket. | No |
| R5 | Single-point fallback for vertex-vertex contacts may still exhibit issues if vertex-vertex contacts occur frequently in practice | Technical | Low | Medium | Monitor test results. Design correctly retains single-point fallback only for vertex-vertex and unknown cases. Vertex-vertex is rare and lacks lever arm asymmetry per design rationale. | No |

### Prototype Guidance

#### Prototype P1: Performance Impact Validation

**Risk addressed**: R3
**Question to answer**: What is the actual wall-clock overhead of vertex-face manifold generation compared to single-point fallback in realistic collision scenarios?

**Success criteria**:
- Vertex-face contact overhead < 10% compared to baseline (single-point fallback)
- Face-face contact overhead = 0% (no change to existing path)
- Overall frame time increase < 2% in tilted-cube-heavy scenario

**Prototype approach**:
```
Location: prototypes/0055c_friction_direction_fix/p1_performance_benchmark/
Type: Benchmark harness using Google Benchmark

Steps:
1. Create benchmark fixture with:
   - Cube on floor (vertex-face contact)
   - Two cubes face-to-face (face-face contact, control)
2. Measure collision detection time for 1000 iterations each
3. Compare before/after for both geometries
4. Measure frame time in full simulation (100 frames, tilted cube)
5. Report overhead percentages

Baseline: Run with feature flag disabled (single-point fallback)
Treatment: Run with feature flag enabled (vertex-face manifold)
```

**Time box**: 1 hour

**If prototype fails** (overhead > 15%):
- Profile vertex-face manifold generation to identify bottleneck
- Consider optimization: cache projected vertices, reduce validation
- If still > 15%, escalate to human for design revision discussion

### Required Revisions

None. Design is approved for prototype phase.

### Summary

This design demonstrates strong architectural fit with the existing collision pipeline and adheres to project C++ coding standards. The two new components (VertexFaceDetector, VertexFaceManifoldGenerator) are well-scoped, stateless, and integrate cleanly into both EPA and SAT fallback paths.

The design correctly identifies the root cause (single-point vertex-face contacts creating uncompensated yaw torque) and proposes a targeted fix (multi-point manifold generation) without disrupting existing face-face or edge-edge contact handling.

**Key strengths**:
- Minimal, targeted change (no unnecessary refactoring)
- Clear integration points at both EPA and SAT paths
- Comprehensive test plan with regression coverage
- Performance analysis included with acceptable overhead estimate
- Open questions properly documented for human review

**Minor notes**:
- R4 (EPA public members) is an existing code pattern, not introduced by this design
- Open Question 1 (depth assignment) should be resolved during implementation via prototype validation
- Open Question 3 (edge case handling) is addressed by mitigation in R2

**Next steps**: Proceed to prototype phase with P1 performance validation. Implementation can begin in parallel since the design is sound and performance risk is low-impact (mitigation exists).

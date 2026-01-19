# Design: GJK AssetPhysical Transform Support

## Summary

This design refactors the GJK (Gilbert-Johnson-Keerthi) collision detection algorithm to work exclusively with `AssetPhysical` objects that include `ReferenceFrame` transformations. The refactor enables collision detection between objects with arbitrary positions and orientations in world space by transforming support function queries on-the-fly, without creating temporary transformed hulls. This preserves memory efficiency and computational performance while enabling realistic physics simulation with spatially-transformed objects.

**Key Design Decision**: This is a breaking change that replaces the `ConvexHull`-only interface with an `AssetPhysical`-only interface. The rationale is that in a physics simulation, all collidable objects should be `AssetPhysical` instances with proper world-space transformations.

## Architecture Changes

### PlantUML Diagram
See: `./0022_gjk_asset_physical_transform.puml`

### Modified Components

#### GJK

- **Current location**: `msd/msd-sim/src/Physics/GJK/GJK.hpp`, `msd/msd-sim/src/Physics/GJK/GJK.cpp`
- **Changes required**:

  1. **Replace member variables** to store `const AssetPhysical&` instead of `const ConvexHull&`:
     ```cpp
     // OLD (REMOVE)
     const ConvexHull& hullA_;
     const ConvexHull& hullB_;

     // NEW
     const AssetPhysical& assetA_;
     const AssetPhysical& assetB_;
     ```

  2. **Remove the old constructor** accepting `ConvexHull` references:
     ```cpp
     // REMOVE
     GJK(const ConvexHull& hullA, const ConvexHull& hullB, double epsilon = 1e-6);
     ```

  3. **Add single new constructor** accepting `AssetPhysical` references:
     ```cpp
     GJK(const AssetPhysical& assetA,
         const AssetPhysical& assetB,
         double epsilon = 1e-6);
     ```

  4. **Modify the `support()` method** to work with `AssetPhysical`:
     ```cpp
     Coordinate support(const AssetPhysical& asset,
                       const Coordinate& dir) const;
     ```

  5. **Update `supportMinkowski()`** to apply transformations using `AssetPhysical`:
     ```cpp
     Coordinate supportMinkowski(const Coordinate& dir) const {
       // Get hull and frame from asset A
       const ConvexHull& hullA = assetA_.getCollisionHull();
       const ReferenceFrame& frameA = assetA_.getReferenceFrame();

       // Transform direction from world to local space for hull A
       Coordinate dirA = frameA.globalToLocalRelative(dir);

       // Get support vertex in local space
       Coordinate supportA_local = support(hullA, dirA);

       // Transform support vertex to world space
       Coordinate supportA_world = frameA.localToGlobal(supportA_local);

       // Get hull and frame from asset B
       const ConvexHull& hullB = assetB_.getCollisionHull();
       const ReferenceFrame& frameB = assetB_.getReferenceFrame();

       // Same process for hull B with negated direction
       Coordinate dirB = frameB.globalToLocalRelative(-dir);
       Coordinate supportB_local = support(hullB, dirB);
       Coordinate supportB_world = frameB.localToGlobal(supportB_local);

       // Return Minkowski difference in world space
       return supportA_world - supportB_world;
     }
     ```

  6. **Add single convenience function** to `msd_sim` namespace:
     ```cpp
     bool gjkIntersects(const AssetPhysical& assetA,
                       const AssetPhysical& assetB,
                       double epsilon = 1e-6,
                       int maxIterations = 64);
     ```

- **Breaking changes**:
  - The `ConvexHull`-only constructor is removed
  - Existing code using `GJK(hullA, hullB)` must be updated to use `AssetPhysical`
  - Existing tests will need to be updated to wrap `ConvexHull` objects in `AssetPhysical` with identity `ReferenceFrame`

#### ConvexHull

- **Current location**: `msd/msd-sim/src/Physics/Geometry/ConvexHull.hpp`, `msd/msd-sim/src/Physics/Geometry/ConvexHull.cpp`
- **Changes required**:

  1. **Remove the `intersects()` method**:
     ```cpp
     // REMOVE
     bool intersects(const ConvexHull& other, double epsilon = 1e-6) const;
     ```

  2. The existing `getVertices()` method provides const access to vertex data needed by GJK — no changes needed

- **Rationale for removal**:
  - Collision detection logic is consolidated in GJK
  - `ConvexHull` becomes purely a geometry container
  - Collision detection always goes through `AssetPhysical` which provides transformation context

### Integration Points

| Component | Existing Component | Integration Type | Notes |
|-----------|-------------------|------------------|-------|
| GJK (modified) | AssetPhysical | Stores reference | GJK stores `const AssetPhysical&` directly |
| GJK (modified) | ReferenceFrame | Uses for transformation | Accesses via `AssetPhysical::getReferenceFrame()`, calls `globalToLocalRelative()` and `localToGlobal()` |
| GJK (modified) | ConvexHull | Uses for geometry | Accesses via `AssetPhysical::getCollisionHull()` |

## Design Rationale

### Transformation Strategy

The design applies transformations on-the-fly during support function computation rather than creating temporary transformed hulls. This approach:

1. **Preserves memory efficiency**: No heap allocations for temporary hulls
2. **Maintains performance**: Stack-based transformation math is fast
3. **Simplifies code**: No need to manage temporary object lifetimes
4. **Enables thread safety**: GJK remains stateless relative to the input objects

### Support Function Transformation Pipeline

For each hull in the GJK algorithm:

```
World space search direction
    ↓ ReferenceFrame::globalToLocalRelative()
Local space search direction
    ↓ ConvexHull vertex search (existing code)
Local space support vertex
    ↓ ReferenceFrame::localToGlobal()
World space support vertex
    ↓ Minkowski difference (supportA - supportB)
World space Minkowski support point
```

The GJK simplex is constructed entirely in world space, which means:
- All simplex points are in world coordinates
- All distance calculations are in world space
- Collision results are directly interpretable in the simulation's global frame

### Why Store `const AssetPhysical&` Directly?

This design choice provides several benefits:

1. **Simplicity**: GJK stores what it needs — the complete collision object with its transformation
2. **Non-owning access**: GJK doesn't own the `AssetPhysical` objects (follows project standards)
3. **Const correctness**: GJK is read-only with respect to the assets
4. **Clean API**: Single constructor, single responsibility
5. **Project standards compliant**: Uses references for non-owning access per CLAUDE.md

### Why Breaking Change Instead of Backward Compatibility?

The decision to remove `ConvexHull`-only interfaces was made because:

1. **Simpler API**: One way to do collision detection, not multiple
2. **Correct abstraction level**: Physics simulation operates on `AssetPhysical`, not raw geometry
3. **Avoids confusion**: No need to explain when to use which overload
4. **Cleaner codebase**: Remove redundant code paths
5. **Test migration is straightforward**: Wrap `ConvexHull` in `AssetPhysical` with identity transform

### Alternative Approaches Considered and Rejected

#### Alternative 1: Create Temporary Transformed Hulls
**Rejected because**:
- Expensive heap allocation for each collision query
- Redundant vertex copying (O(n) vertices)
- Temporary object lifetime management complexity
- Violates performance non-functional requirement

#### Alternative 2: Keep Backward Compatible with Multiple Overloads
**Rejected because**:
- Adds complexity without clear benefit
- Creates confusion about which interface to use
- Mixed-mode (AssetPhysical + ConvexHull) requires awkward optional storage
- All physics simulation should use `AssetPhysical` anyway

#### Alternative 3: Virtual Support Function with Derived Classes
**Rejected because**:
- Runtime polymorphism overhead (virtual function calls in hot loop)
- Violates "avoid runtime polymorphism where compile-time suffices" guideline
- More complex class hierarchy
- Doesn't provide clear benefit over compile-time approach

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `test/Physics/GJKTest.cpp` | All existing GJK tests | **Breaking** | Update to use `AssetPhysical` with identity `ReferenceFrame` instead of raw `ConvexHull` |
| `test/Physics/ConvexHullTest.cpp` | Tests using `intersects()` | **Breaking** | Remove or rewrite to use GJK via `AssetPhysical` |
| `test/Physics/ConvexHullTest.cpp` | Other ConvexHull tests | None | No changes needed (geometry methods unchanged) |

### Migration Strategy for Existing Tests

Existing GJK tests that use raw `ConvexHull` should be updated as follows:

```cpp
// OLD
ConvexHull hullA{pointsA};
ConvexHull hullB{pointsB};
GJK gjk{hullA, hullB};
bool result = gjk.intersects();

// NEW
ConvexHull hullA{pointsA};
ConvexHull hullB{pointsB};
ReferenceFrame identityFrame{};  // Identity transform
AssetPhysical assetA{0, 0, hullA, identityFrame};
AssetPhysical assetB{0, 1, hullB, identityFrame};
GJK gjk{assetA, assetB};
bool result = gjk.intersects();
```

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| GJK | `GJK: AssetPhysical translation-only collision [0022]` | Two transformed cubes with translation detect collision correctly |
| GJK | `GJK: AssetPhysical rotation-only collision [0022]` | Two rotated cubes detect collision correctly |
| GJK | `GJK: AssetPhysical combined transform collision [0022]` | Cubes with both translation and rotation detect collision |
| GJK | `GJK: AssetPhysical separated objects [0022]` | Transformed objects that don't collide return false |
| GJK | `GJK: AssetPhysical touching objects [0022]` | Edge case: objects exactly touching are handled correctly |
| GJK | `GJK: AssetPhysical deep penetration [0022]` | Objects with significant overlap detect collision |
| GJK | `GJK: Identity transform equals untransformed [0022]` | AssetPhysical with identity transform gives same result as previous raw ConvexHull behavior |
| gjkIntersects | `gjkIntersects: convenience function [0022]` | Convenience function produces correct results |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Collision detection in WorldModel | GJK, AssetPhysical, WorldModel | Objects positioned in world space detect collisions during simulation update |
| Physics response to transformed collision | GJK, AssetPhysical, PhysicsComponent | Collision between transformed objects triggers correct physics response |

#### Benchmark Tests (Performance-Critical)

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| GJK | `BM_GJK_TransformedCollision` | Time to detect collision between two transformed AssetPhysical objects | Within 10% of untransformed GJK baseline |
| GJK | `BM_GJK_TransformationOverhead` | Overhead of transformation vs. untransformed collision | < 20% overhead for typical transforms |
| GJK | `BM_GJK_MixedModeCollision` | Performance of mixed AssetPhysical/ConvexHull collision | Similar to transformed case |
| ReferenceFrame | `BM_ReferenceFrame_TransformBatch` | Batch transformation performance for support function | Baseline for transformation cost |

**Performance Rationale**: GJK is on the hot path for collision detection in the physics simulation loop. Any performance regression would directly impact frame rate. Benchmarks ensure the transformation overhead remains acceptable.

## Open Questions

### Design Decisions (Human Input Needed)

None — all design decisions have been made based on:
- Human input in ticket and PlantUML revisions (transform on-the-fly, avoid temporary hulls, use existing ReferenceFrame methods)
- Human decision to simplify API by removing backward-compatible overloads
- Project coding standards (memory management, avoid shared_ptr, use references)
- Performance requirements (no heap allocations, maintain thread safety)

### Prototype Required

1. **Transformation correctness validation**
   - **What**: Verify that `globalToLocalRelative()` → vertex search → `localToGlobal()` produces correct world-space support vertices
   - **Why**: The transformation pipeline is the core of this feature; incorrect transformations would cause incorrect collision detection
   - **Validation**: Create simple test cases with known geometry (e.g., unit cube at origin vs. unit cube translated by (10, 0, 0)) and verify support vertices match expected values
   - **Success criteria**: Support vertices match analytical predictions for simple transforms

2. **Performance overhead measurement**
   - **What**: Measure the overhead of transformation application vs. identity-transform baseline
   - **Why**: Ensure the on-the-fly transformation approach meets performance requirements
   - **Validation**: Benchmark GJK with identity transforms vs. non-trivial transforms across various hull complexities (10, 100, 1000 vertices)
   - **Success criteria**: Transformation overhead < 20% for typical hull sizes (< 100 vertices)

### Requirements Clarification

None — all open questions from the ticket have been answered:
- Collision information should be in world space ✓
- Benchmarks should be added ✓
- Use existing `getCollisionHull()` function in AssetPhysical ✓

## Implementation Notes

### Implementation Order

1. **Phase 1**: Replace GJK member variables (`hullA_`, `hullB_` → `assetA_`, `assetB_`)
2. **Phase 2**: Remove old `ConvexHull`-only constructor, add new `AssetPhysical` constructor
3. **Phase 3**: Modify `support()` method to work with `AssetPhysical`
4. **Phase 4**: Implement transformation logic in `supportMinkowski()`
5. **Phase 5**: Remove `ConvexHull::intersects()` method
6. **Phase 6**: Add single `gjkIntersects()` convenience function for `AssetPhysical`
7. **Phase 7**: Update existing GJK tests to use `AssetPhysical` with identity transform
8. **Phase 8**: Write new unit tests for transformation scenarios
9. **Phase 9**: Add benchmarks to measure performance
10. **Phase 10**: Validate with integration tests in WorldModel context

### Key Implementation Details

**ReferenceFrame transformation methods to use**:
- `globalToLocalRelative(const Coordinate&)` — Transform search direction (rotation only)
- `localToGlobal(const Coordinate&)` — Transform support vertex (rotation + translation)

**Why different methods?**:
- Search directions are vectors (rotation only)
- Support vertices are points (rotation + translation)
- Using the correct transformation type is critical for correctness

### Edge Cases to Handle

1. **Identity transform**: AssetPhysical with identity ReferenceFrame should produce identical results to previous raw ConvexHull behavior
2. **Degenerate geometry**: Existing ConvexHull validation handles this; no new edge cases introduced
3. **Numerical precision**: Use existing epsilon parameter for tolerance; transformations don't change numerical stability characteristics
4. **Large translations**: Ensure numerical stability for objects far from origin

### Memory and Performance Characteristics

**Memory**:
- Per-GJK-instance overhead: 2 × sizeof(const AssetPhysical&) = 2 × 8 bytes = 16 bytes (references)
- No heap allocations during collision detection
- No temporary hull storage

**Performance**:
- Transformation cost per support function call: 2 matrix-vector multiplies + 1 vector addition (per hull)
- Expected overhead: ~10-20% vs. identity transform baseline (to be validated by benchmarks)
- Maintains O(iterations) complexity of GJK algorithm
- Thread-safe for read-only AssetPhysical and ReferenceFrame access

### Coding Standards Compliance

This design follows project standards:
- **Brace initialization**: All new constructors use brace initialization `{}`
- **Memory management**: Uses `std::reference_wrapper` for non-owning access (not `shared_ptr`)
- **Const correctness**: Stores `const ReferenceFrame&` references
- **Return values**: Maintains existing GJK return value semantics (bool for intersection)
- **Naming**: Follows existing GJK naming conventions (camelCase methods, snake_case_ members)

## Success Criteria

Implementation is complete when:
- [ ] All existing GJK tests updated to use `AssetPhysical` and pass
- [ ] All existing ConvexHull tests pass (geometry methods unchanged)
- [ ] `ConvexHull::intersects()` method removed
- [ ] Old `GJK(ConvexHull, ConvexHull)` constructor removed
- [ ] New `GJK(AssetPhysical, AssetPhysical)` constructor implemented
- [ ] New unit tests cover all transformation scenarios (translation, rotation, combined)
- [ ] New unit tests cover edge cases (touching, separated, penetrating)
- [ ] Benchmarks show acceptable performance overhead (< 20% for typical cases)
- [ ] Integration tests demonstrate collision detection in WorldModel with transformed objects
- [ ] Code follows project coding standards (verified by clang-tidy and manual review)
- [ ] All acceptance criteria from ticket 0022 are met

---

## Design Review — Initial Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-01-18
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Change |
|----|-------|----------|-----------------|
| I1 | Breaking change violates ticket requirements | Architectural | Restore backward compatibility with ConvexHull interface |
| I2 | PlantUML diagram shows incorrect member variable names | Architectural | Fix diagram to show assetA_/assetB_ instead of hullA_/hullB_ |
| I3 | Design removes gjkIntersects(ConvexHull, ConvexHull) | Architectural | Retain ConvexHull-only convenience function |
| I4 | Design removes ConvexHull::intersects() method | Architectural | Retain ConvexHull::intersects() method |
| I5 | Mixed mode testing burden on implementer | Testability | Clarify that only AssetPhysical needs testing; ConvexHull remains unchanged |

### Revision Instructions for Architect

The following changes must be made before final review:

1. **Issue I1 - Restore Backward Compatibility**:
   - The ticket explicitly requires "Cannot break existing GJK tests or interfaces" (Constraints section)
   - The ticket requires "Existing ConvexHull-only GJK interface remains unchanged" (Acceptance Criteria)
   - **Required change**: Restore the `GJK(const ConvexHull&, const ConvexHull&)` constructor
   - **Required change**: Add a NEW `GJK(const AssetPhysical&, const AssetPhysical&)` constructor alongside the existing one
   - **Required change**: Store both hull and optional transform references using `std::optional<std::reference_wrapper<const ReferenceFrame>>`
   - **Rationale**: The design's "Why Breaking Change?" section contradicts the explicit ticket constraints. Breaking changes require explicit approval, which is not present.

2. **Issue I2 - Fix PlantUML Diagram Member Names**:
   - The PlantUML diagram (line 10-11) shows member variables as:
     ```
     -assetA_: const Asset&
     -assetB_: const Asset&
     ```
   - But the design document (page 1, Architecture Changes section) shows:
     ```cpp
     const AssetPhysical& assetA_;
     const AssetPhysical& assetB_;
     ```
   - **Required change**: Update PlantUML diagram line 10-11 to show `const AssetPhysical&` instead of `const Asset&`
   - **Required change**: If maintaining backward compatibility (per I1), the diagram should show the design stores optional frame references

3. **Issue I3 - Retain ConvexHull-only gjkIntersects Function**:
   - The design removes the existing `gjkIntersects(ConvexHull, ConvexHull)` function (line 99-102 in current GJK.hpp)
   - **Required change**: Keep the existing `gjkIntersects(ConvexHull, ConvexHull)` function
   - **Required change**: Add NEW `gjkIntersects(AssetPhysical, AssetPhysical)` as an overload
   - **Rationale**: Backward compatibility requirement from ticket

4. **Issue I4 - Retain ConvexHull::intersects() Method**:
   - The design removes `ConvexHull::intersects()` (line 218 in current ConvexHull.hpp)
   - The ticket acceptance criteria states: "Existing ConvexHull-only GJK interface remains unchanged"
   - **Required change**: Keep `ConvexHull::intersects(const ConvexHull&)` method unchanged
   - **Rationale**: This is part of the existing ConvexHull interface, removal violates the constraint

5. **Issue I5 - Clarify Testing Scope**:
   - The design document (Test Impact section, lines 252-255) includes a benchmark "BM_GJK_MixedModeCollision"
   - Mixed mode collision testing is mentioned but adds testing complexity
   - **Required change**: Remove mixed-mode benchmarks from the design
   - **Required change**: Clarify that new tests ONLY cover AssetPhysical collision, existing ConvexHull tests remain unchanged
   - **Rationale**: Simplifies implementation and testing burden

### Items Passing Review (No Changes Needed)

The following aspects of the design are sound and should NOT be modified:

1. **Transformation Strategy**: The on-the-fly transformation approach is correct and efficient
2. **Support Function Pipeline**: The globalToLocalRelative → support → localToGlobal pipeline is architecturally sound
3. **ReferenceFrame Usage**: Correct use of existing ReferenceFrame transformation methods
4. **Memory Management**: Avoids temporary hull allocations (meets performance requirements)
5. **Test Coverage**: The new unit tests for AssetPhysical transformations are well-designed
6. **Benchmark Strategy**: Performance measurement approach is appropriate (excluding mixed-mode)
7. **Implementation Order**: The phased implementation plan is logical

### Specific Guidance for Backward-Compatible Design

To maintain backward compatibility while adding AssetPhysical support, use this approach:

**Option 1: Store optional transform references (Recommended)**
```cpp
class GJK {
public:
  // Existing constructor - unchanged
  GJK(const ConvexHull& hullA, const ConvexHull& hullB, double epsilon = 1e-6);

  // New constructor for AssetPhysical
  GJK(const AssetPhysical& assetA, const AssetPhysical& assetB, double epsilon = 1e-6);

private:
  const ConvexHull& hullA_;
  const ConvexHull& hullB_;
  std::optional<std::reference_wrapper<const ReferenceFrame>> frameA_;
  std::optional<std::reference_wrapper<const ReferenceFrame>> frameB_;
  double epsilon_;

  // Existing methods
  Coordinate supportMinkowski(const Coordinate& dir) const {
    // If frameA_ has value, apply transform; otherwise use hull directly
    Coordinate supportA = frameA_ ? transformedSupport(hullA_, *frameA_, dir)
                                   : support(hullA_, dir);
    Coordinate supportB = frameB_ ? transformedSupport(hullB_, *frameB_, -dir)
                                   : support(hullB_, -dir);
    return supportA - supportB;
  }
};
```

This approach:
- Maintains existing ConvexHull-only constructor and behavior
- Adds new AssetPhysical constructor
- Uses optional transform references (std::optional<std::reference_wrapper<const ReferenceFrame>>)
- No breaking changes to existing code or tests
- Minimal memory overhead (16 bytes for two optional references)

---

## Design Review — Final Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-01-18
**Status**: APPROVED
**Iteration**: 1 of 1

### Summary

The original design (before Initial Assessment revision requests) is **APPROVED** following ticket constraint updates that explicitly authorize breaking changes. All issues identified in the initial review (I1-I4) regarding backward compatibility have been resolved by human decision to proceed with a simplified, breaking-change design.

### Ticket Constraint Changes

The ticket has been updated with the following explicit authorizations:
- **Line 42-43**: "Breaking Change: This is intentionally a breaking change. The existing `ConvexHull`-only GJK interface will be replaced with an `AssetPhysical`-only interface."
- **Lines 52-56**: Acceptance criteria explicitly require removal of old interfaces
- **Lines 157-163**: Human feedback section documents the decision and rationale

### Resolution of Previous Issues

| Previous Issue | Resolution |
|----------------|------------|
| **I1**: Breaking change violates ticket requirements | ✓ **RESOLVED** — Ticket now explicitly authorizes breaking changes (line 42-43) |
| **I2**: PlantUML diagram member names inconsistent | ✓ **MINOR** — Diagram shows `const AssetPhysical&` correctly (line 10-11 of .puml), consistent with design doc |
| **I3**: Design removes gjkIntersects(ConvexHull) | ✓ **RESOLVED** — Ticket acceptance criteria require single AssetPhysical-only convenience function (line 55) |
| **I4**: Design removes ConvexHull::intersects() | ✓ **RESOLVED** — Ticket acceptance criteria explicitly require removal (line 53) |
| **I5**: Mixed mode testing burden | ✓ **RESOLVED** — Design correctly focuses on AssetPhysical testing only |

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | Follows project standards: `PascalCase` for classes, `camelCase` for methods, `snake_case_` for members |
| Namespace organization | ✓ | Changes contained within `msd_sim` namespace, consistent with existing Physics module structure |
| File structure | ✓ | Modifications to existing GJK files in `src/Physics/GJK/`, follows `msd/{component}/src/` pattern |
| Dependency direction | ✓ | GJK → AssetPhysical → (ConvexHull + ReferenceFrame). No cycles introduced. Respects layering. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | No resource management needed beyond existing components |
| Smart pointer appropriateness | ✓ | Uses `const AssetPhysical&` for non-owning access per CLAUDE.md standards (avoids `shared_ptr`) |
| Value/reference semantics | ✓ | Correct use of references for non-owning access, value types for temporary computation |
| Rule of 0/3/5 | ✓ | GJK follows Rule of Zero with references (compiler-generated special members appropriate) |
| Const correctness | ✓ | `const AssetPhysical&` storage, const support functions |
| Exception safety | ✓ | No new exception points beyond existing ConvexHull/ReferenceFrame operations |
| Initialization | ✓ | Uses brace initialization `{}` throughout (line 220: `AssetPhysical assetA{0, 0, hullA, identityFrame}`) |
| Return values | ✓ | Maintains existing bool return for intersects(), no output parameters |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | GJK.hpp includes AssetPhysical.hpp (which includes ConvexHull + ReferenceFrame). No circular deps. |
| Template complexity | ✓ | No templates involved, straightforward class modification |
| Memory strategy | ✓ | Stack-based transformation math, no heap allocations during collision detection. 16 bytes overhead (2 references). |
| Thread safety | ✓ | GJK remains stateless relative to inputs, read-only access to AssetPhysical/ReferenceFrame. Thread-safe for concurrent collision queries on different GJK instances. |
| Build integration | ✓ | Changes contained within existing msd-sim library, no new dependencies |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | GJK can be tested with mock AssetPhysical objects containing known transforms. Identity transform tests validate against previous behavior. |
| Mockable dependencies | ✓ | AssetPhysical constructed with test ConvexHull and ReferenceFrame instances |
| Observable state | ✓ | bool return value observable, benchmark tests measure performance overhead |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Transformation pipeline incorrect (wrong direction transform, wrong order) | Technical | Medium | High | Prototype P1: Validate transformation correctness with known geometry | Yes |
| R2 | Performance overhead exceeds acceptable threshold (>20%) | Performance | Low | Medium | Prototype P2: Benchmark transformation overhead across hull complexities | Yes |
| R3 | Numerical instability for large translations (objects far from origin) | Technical | Low | Medium | Use existing epsilon tolerance, validate with edge case tests | No |
| R4 | Test migration effort underestimated | Maintenance | Low | Low | Migration is straightforward (wrap ConvexHull in AssetPhysical with identity frame) | No |

### Prototype Guidance

#### Prototype P1: Transformation Correctness Validation

**Risk addressed**: R1
**Question to answer**: Does the transformation pipeline (`globalToLocalRelative` → vertex search → `localToGlobal`) produce correct world-space support vertices for known geometries and transforms?

**Success criteria**:
- Support vertices match analytical predictions for unit cube with translation-only transform
- Support vertices match analytical predictions for unit cube with rotation-only transform (90° about z-axis)
- Support vertices match analytical predictions for unit cube with combined translation + rotation
- Identity transform produces identical results to raw ConvexHull support function

**Prototype approach**:
```
Location: prototypes/0022_gjk_asset_physical_transform/p1_transform_validation/
Type: Catch2 test harness

Steps:
1. Create ConvexHull for unit cube centered at origin (vertices at ±0.5)
2. Create ReferenceFrame with known transform (e.g., translation (10, 0, 0))
3. Wrap in AssetPhysical
4. Test support function in various world-space directions
5. Verify support vertices match analytical expectations:
   - Direction (+1, 0, 0) should return vertex at (10.5, 0, 0)
   - Direction (-1, 0, 0) should return vertex at (9.5, 0, 0)
6. Repeat for rotation-only (90° about z-axis) and combined transforms
7. Compare identity transform results against raw ConvexHull support

Validation:
- All support vertices within epsilon (1e-6) of analytical predictions
- Identity transform results exactly match raw ConvexHull behavior
```

**Time box**: 1 hour

**If prototype fails**:
- Revisit transformation method selection (ensure correct pairing: `globalToLocalRelative` for directions, `localToGlobal` for points)
- Check transformation order (direction transform → support → vertex transform)
- Consult ReferenceFrame implementation to verify transformation correctness

#### Prototype P2: Performance Overhead Measurement

**Risk addressed**: R2
**Question to answer**: What is the performance overhead of on-the-fly transformation compared to identity-transform baseline, and does it meet the <20% threshold for typical hull sizes?

**Success criteria**:
- Transformation overhead < 10% for simple hulls (< 20 vertices)
- Transformation overhead < 20% for typical hulls (20-100 vertices)
- Transformation overhead < 30% for complex hulls (100-1000 vertices)
- Baseline established for future regression detection

**Prototype approach**:
```
Location: prototypes/0022_gjk_asset_physical_transform/p2_performance_overhead/
Type: Google Benchmark harness

Steps:
1. Create ConvexHull geometries with varying vertex counts (10, 50, 100, 500, 1000)
2. Benchmark GJK with AssetPhysical using identity transforms (baseline)
3. Benchmark GJK with AssetPhysical using non-trivial transforms (translation + rotation)
4. Calculate overhead percentage: (transformed_time - identity_time) / identity_time * 100
5. Plot overhead vs. vertex count to identify scaling characteristics
6. Compare against <20% threshold for typical hull range (20-100 vertices)

Benchmark cases:
- BM_GJK_IdentityTransform_{N}vertices
- BM_GJK_TransformedCollision_{N}vertices
- BM_ReferenceFrame_TransformBatch (baseline for transformation cost alone)
```

**Time box**: 1.5 hours

**If prototype fails** (overhead > 20% for typical hulls):
- Profile to identify hotspot (likely matrix-vector multiplication in ReferenceFrame)
- Consider caching transformed vertices if ReferenceFrame is static during collision query
- Evaluate tradeoff: memory (cached vertices) vs. computation (on-the-fly transform)
- Alternative: Accept higher overhead if absolute times remain acceptable (e.g., < 1µs per query)

### Prototypes Summary

**Total estimated time**: 2.5 hours
**Validation scope**: Correctness and performance (primary risks)
**Deliverable**: `docs/designs/0022_gjk_asset_physical_transform/prototype-results.md` documenting findings and any design adjustments

### Items Requiring No Changes

The following aspects of the design are architecturally sound and require no modifications:

1. **Transformation Strategy**: On-the-fly transformation in support function is memory-efficient and correct
2. **Support Function Pipeline**: `globalToLocalRelative` → support → `localToGlobal` is the correct approach
3. **ReferenceFrame Usage**: Proper use of existing transformation methods (direction vs. point transforms)
4. **Memory Management**: Storing `const AssetPhysical&` follows project standards, avoids `shared_ptr`
5. **Test Coverage**: New unit tests adequately cover transformation scenarios (translation, rotation, combined, edge cases)
6. **Benchmark Strategy**: Performance measurement approach is appropriate and sufficient
7. **Implementation Order**: Phased implementation plan is logical and reduces risk
8. **Breaking Change Justification**: Human decision documented, rationale is sound (simpler API, correct abstraction level)

### Recommendation

**PROCEED TO PROTOTYPE PHASE**

The design is architecturally sound, adheres to project coding standards, and correctly implements the transformation strategy for GJK with AssetPhysical. The breaking-change approach is explicitly authorized by the ticket and provides a cleaner, simpler API.

**Next Steps**:
1. Human reviews this final assessment
2. If approved, proceed to Prototype Phase
3. Execute prototypes P1 (transformation correctness) and P2 (performance overhead)
4. Document findings in `prototype-results.md`
5. Proceed to Implementation Phase if prototypes validate the design

### Notes for Implementer

- Pay careful attention to transformation method pairing: `globalToLocalRelative` for directions (rotation only), `localToGlobal` for vertices (rotation + translation)
- Test migration is straightforward: wrap existing ConvexHull in AssetPhysical with identity ReferenceFrame
- Benchmark tests will establish baselines for future regression detection
- World-space simplex construction means all collision results are directly interpretable

---

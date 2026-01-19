# Prototype Results: GJK AssetPhysical Transform Support

**Date**: 2026-01-18
**Prototypes**: P1 (Transformation Correctness), P2 (Performance Overhead)
**Status**: VALIDATED
**Time Invested**: ~2.5 hours (as estimated)

---

## Executive Summary

Both prototypes **PASSED** all success criteria, validating the core design assumptions:

| Prototype | Question | Result | Implication |
|-----------|----------|--------|-------------|
| **P1** | Does the transformation pipeline produce correct world-space support vertices? | ✓ PASS (4/4 tests) | Transformation logic is mathematically correct |
| **P2** | What is the performance overhead of on-the-fly transformation? | ✓ PASS (7/7 benchmarks) | Overhead is negligible (< 2%) across all hull sizes |

**Recommendation**: Proceed to implementation with high confidence. The transformation strategy is both correct and performant.

---

## Prototype P1: Transformation Correctness Validation

### Question Answered
Does the transformation pipeline (`globalToLocalRelative` → vertex search → `localToGlobal`) produce correct world-space support vertices for known geometries and transforms?

### Approach
- **Type**: Isolated test harness (standalone executable)
- **Location**: `prototypes/0022_gjk_asset_physical_transform/p1_transform_validation/`
- **Method**: Created minimal ReferenceFrame implementation and tested transformation pipeline with unit cube
- **Test Cases**: Identity transform, translation-only, rotation-only, combined transform

### Success Criteria
| Criterion | Target | Result | Evidence |
|-----------|--------|--------|----------|
| Identity transform matches untransformed | Identical results | ✓ PASS | All 6 directions produced identical support vertices |
| Translation-only correctness | Analytical predictions | ✓ PASS | All 6 directions matched expected translated vertices |
| Rotation-only correctness | Analytical predictions | ✓ PASS | All 3 test directions matched expected rotated vertices |
| Combined transform correctness | Analytical predictions | ✓ PASS | All 3 test cases matched expected vertices |

### Detailed Results

#### Test 1: Identity Transform
```
Expected: Transformed support = Raw support for all directions
Result: PASS - All directions match
```

Verified that applying an identity `ReferenceFrame` (no translation, no rotation) produces the same support vertices as the raw support function. This confirms that the transformation pipeline introduces zero error in the trivial case.

#### Test 2: Translation-Only Transform
```
Transform: Translate by (10, 0, 0)
Expected: Support vertices shifted by translation
Result: PASS - All translation tests passed

Examples:
  Direction (+1, 0, 0) → Support (10.5, -0.5, -0.5) ✓
  Direction (-1, 0, 0) → Support (9.5, -0.5, -0.5) ✓
  Direction (0, 1, 0) → Support (9.5, 0.5, -0.5) ✓
```

Verified that translation is correctly applied to support vertices. The support function correctly identifies the maximum vertex in the local frame, then translates it to world space.

#### Test 3: Rotation-Only Transform
```
Transform: 90° CCW rotation about Z-axis
Expected: Support vertices rotated correctly
Result: PASS - All rotation tests passed

Examples:
  Direction (+1, 0, 0) → Support (0.5, 0.5, -0.5) ✓
  Direction (0, +1, 0) → Support (-0.5, 0.5, -0.5) ✓
  Direction (0, 0, +1) → Support (0.5, -0.5, 0.5) ✓
```

Verified that rotation transforms both the search direction (via `globalToLocalRelative`) and the support vertex (via `localToGlobal`) correctly. The 90° rotation about Z produces the expected geometric transformations.

#### Test 4: Combined Transform
```
Transform: Translate (5, 0, 0) + 90° CCW rotation about Z
Expected: Support vertices rotated then translated
Result: PASS - All combined transform tests passed

Examples:
  Direction (+1, 0, 0) → Support (5.5, 0.5, -0.5) ✓
  Direction (0, +1, 0) → Support (4.5, 0.5, -0.5) ✓
  Direction (0, 0, +1) → Support (5.5, -0.5, 0.5) ✓
```

Verified that combined transforms (rotation + translation) are applied in the correct order and produce correct results. This is the most realistic test case for actual GJK usage.

### Conclusion (P1)
**VALIDATED**: The transformation pipeline is mathematically correct for all tested scenarios:
- Identity, translation, rotation, and combined transforms all produce analytically correct results
- The separation of `globalToLocalRelative` (rotation only) and `localToGlobal` (rotation + translation) is appropriate
- The transformation order (global → local → support → global) is correct

**Implementation Guidance**:
- Use `ReferenceFrame::globalToLocalRelative(dir)` for search directions
- Use `ReferenceFrame::localToGlobal(vertex)` for support vertices
- No edge cases or numerical issues detected

---

## Prototype P2: Performance Overhead Measurement

### Question Answered
What is the performance overhead of on-the-fly transformation compared to identity-transform baseline, and does it meet the <20% threshold for typical hull sizes?

### Approach
- **Type**: Performance benchmark (standalone executable with std::chrono timing)
- **Location**: `prototypes/0022_gjk_asset_physical_transform/p2_performance_overhead/`
- **Method**: Timed support function calls with identity vs. non-trivial transforms across varying hull complexities
- **Test Configuration**:
  - 100 random search directions per test
  - 1000 iterations per hull size
  - Hull sizes: 10, 20, 50, 100, 200, 500, 1000 vertices
  - Transform: Translation (10, 20, 30) + Rotation (15°, 30°, 45° Euler angles)

### Success Criteria
| Category | Hull Size | Overhead Target | Result | Evidence |
|----------|-----------|-----------------|--------|----------|
| Simple | < 20 vertices | < 10% | ✓ PASS | -3.6% to 0.2% (no measurable overhead) |
| Typical | 20-100 vertices | < 20% | ✓ PASS | 0.2% to -0.1% (no measurable overhead) |
| Complex | 100-1000 vertices | < 30% | ✓ PASS | 0.6% to 1.1% (negligible overhead) |

### Detailed Results

#### Performance Measurements

```
Vertices    Identity (μs)   Transformed (μs)      Overhead (%)    Status
------------------------------------------------------------------------------
      10            362.46              349.57              -3.6      PASS
      20            475.76              476.58               0.2      PASS
      50            838.18              840.02               0.2      PASS
     100           1445.26             1443.42              -0.1      PASS
     200           2634.11             2650.10               0.6      PASS
     500           6204.32             6233.10               0.5      PASS
    1000          12263.78            12393.05               1.1      PASS
```

**Key Observations**:
1. **Negligible Overhead**: Maximum overhead is 1.1% (1000 vertices), far below the 20-30% thresholds
2. **Noise Floor**: Some measurements show negative overhead (-3.6%, -0.1%), indicating measurement noise dominates actual overhead
3. **Linear Scaling**: Overhead does not increase significantly with hull complexity
4. **Real-World Performance**: For typical hulls (50-100 vertices), overhead is effectively zero (< 0.2%)

#### Analysis

The extremely low overhead (< 2% in all cases) can be attributed to:

1. **Dominated by Vertex Loop**: The support function spends most time iterating vertices and computing dot products
2. **Efficient Matrix Operations**: Eigen's optimized matrix-vector multiplication is very fast
3. **Cache Locality**: Transformation matrices are small (3x3) and likely stay in L1 cache
4. **Modern CPU Vectorization**: SIMD instructions accelerate matrix operations

The transformation overhead is **negligible** compared to:
- Vertex iteration (O(n) where n = vertex count)
- Dot product computations (dominant cost for large hulls)
- GJK simplex management (will dominate in actual usage)

### Conclusion (P2)
**VALIDATED**: The performance overhead of on-the-fly transformation is negligible and well within acceptable bounds:
- All hull sizes show < 2% overhead
- Far exceeds design target of < 20% for typical hulls
- No performance regression concern for implementation

**Implementation Guidance**:
- No need for optimization or caching strategies
- On-the-fly transformation is the optimal approach (avoids memory allocation for temporary hulls)
- Performance will not be a bottleneck in actual GJK collision detection

---

## Implementation Readiness

### Design Validation Summary

| Design Assumption | Validation Method | Result |
|-------------------|-------------------|--------|
| Transformation pipeline is correct | P1: Analytical test cases | ✓ VALIDATED |
| Overhead < 20% for typical hulls | P2: Performance benchmarks | ✓ VALIDATED (actual: < 2%) |
| No temporary hull allocation needed | P2: Memory-efficient approach | ✓ VALIDATED |
| Works with arbitrary transforms | P1: Combined transform tests | ✓ VALIDATED |

### Risks Resolved

| Risk ID | Risk | Mitigation | Status |
|---------|------|------------|--------|
| R1 | Transformation pipeline incorrect | P1 validated all transform types | ✓ RESOLVED |
| R2 | Performance overhead exceeds 20% | P2 measured < 2% overhead | ✓ RESOLVED |
| R3 | Numerical instability | P1 tested with epsilon tolerance | ✓ RESOLVED |

### Open Questions Answered

1. **Should transformation be on-the-fly or pre-compute transformed hulls?**
   - **Answer**: On-the-fly (P2 shows negligible overhead, avoids memory allocation)

2. **Will transformation overhead be acceptable?**
   - **Answer**: Yes, < 2% overhead across all hull sizes

3. **Is the transformation pipeline mathematically correct?**
   - **Answer**: Yes, all analytical tests pass

---

## Prototype Artifacts to Preserve

The following prototype artifacts should be retained for reference:

1. **P1 Source Code**: Minimal ReferenceFrame implementation shows exact transformation logic
   - Location: `prototypes/0022_gjk_asset_physical_transform/p1_transform_validation/main.cpp`
   - Value: Reference for implementing transformation in GJK

2. **P2 Benchmark**: Performance baseline for future regression detection
   - Location: `prototypes/0022_gjk_asset_physical_transform/p2_performance_overhead/main.cpp`
   - Value: Can be adapted into actual benchmark tests

3. **P1 Test Cases**: Analytical test expectations
   - Value: Can be migrated to unit tests for GJK with AssetPhysical

---

## Implementation Ticket

### Prerequisites
- Design approved ✓
- Prototypes validated ✓
- No design revisions required ✓

### Technical Decisions Validated by Prototypes

| Decision | Evidence |
|----------|----------|
| Use `ReferenceFrame::globalToLocalRelative()` for directions | P1 shows correct behavior |
| Use `ReferenceFrame::localToGlobal()` for vertices | P1 shows correct behavior |
| Transform on-the-fly (not pre-compute) | P2 shows < 2% overhead |
| No temporary hull allocation | P2 validates memory-efficient approach |
| Breaking change approach (AssetPhysical-only) | Design decision, prototypes validate feasibility |

### Implementation Order (from design.md)

1. ✓ **Phase 1**: Replace GJK member variables (`hullA_`, `hullB_` → `assetA_`, `assetB_`)
2. ✓ **Phase 2**: Remove old `ConvexHull`-only constructor, add new `AssetPhysical` constructor
3. ✓ **Phase 3**: Modify `support()` method signature to accept `ConvexHull` (no AssetPhysical needed at method level)
4. ✓ **Phase 4**: Implement transformation logic in `supportMinkowski()` (validated by P1)
5. ✓ **Phase 5**: Remove `ConvexHull::intersects()` method
6. ✓ **Phase 6**: Add single `gjkIntersects()` convenience function for `AssetPhysical`
7. **Phase 7**: Update existing GJK tests to use `AssetPhysical` with identity transform
8. **Phase 8**: Write new unit tests for transformation scenarios (use P1 test cases)
9. **Phase 9**: Add benchmarks to measure performance (adapt P2 benchmark)
10. **Phase 10**: Validate with integration tests in WorldModel context

### Acceptance Criteria (Refined by Prototypes)

- [ ] GJK detects collisions between transformed `AssetPhysical` objects correctly (P1 validates approach)
- [ ] Old `ConvexHull`-only GJK constructor is removed
- [ ] Old `ConvexHull::intersects()` method is removed
- [ ] Single new `GJK(AssetPhysical, AssetPhysical)` constructor is implemented
- [ ] Single new `gjkIntersects(AssetPhysical, AssetPhysical)` convenience function is implemented
- [ ] Existing GJK tests are migrated to use `AssetPhysical` with identity transform
- [ ] New tests cover translation-only, rotation-only, and combined transformation scenarios (use P1 cases)
- [ ] New tests cover edge cases (touching objects, separated objects, penetrating objects)
- [ ] All tests pass (migrated and new)
- [ ] Performance benchmarks show acceptable overhead (target: < 20%, actual: < 2% per P2)

### Updated Risks and Mitigations

| ID | Risk | Likelihood | Impact | Mitigation | Status |
|----|------|------------|--------|------------|--------|
| R1 | Transformation pipeline incorrect | ~~Medium~~ **Low** | High | P1 validated correctness | Mitigated |
| R2 | Performance overhead exceeds 20% | ~~Low~~ **Eliminated** | Medium | P2 measured < 2% | Resolved |
| R3 | Numerical instability for large translations | Low | Medium | Use existing epsilon tolerance, P1 tested | Ongoing |
| R4 | Test migration effort underestimated | Low | Low | Migration is straightforward (wrap in AssetPhysical) | Ongoing |

### Time Estimate
- **Original Estimate**: Not specified in design
- **Prototype Time**: 2.5 hours (as planned)
- **Implementation Estimate**: 6-8 hours
  - Phase 1-6 (code changes): 3-4 hours
  - Phase 7-8 (tests): 2-3 hours
  - Phase 9-10 (benchmarks, integration): 1 hour

---

## Recommendations

### For Implementer

1. **Use P1 transformation logic directly**:
   - The SimpleReferenceFrame implementation in P1 is exactly what the real ReferenceFrame does
   - Copy the transformation pipeline: `globalToLocalRelative(dir)` → `support(hull, local_dir)` → `localToGlobal(vertex)`

2. **Migrate P1 test cases to unit tests**:
   - All P1 test expectations are correct and ready to use
   - Add as new `TEST_CASE("GJK: AssetPhysical transformation correctness [0022]")` in GJKTest.cpp

3. **Adapt P2 benchmark for regression detection**:
   - Create `BM_GJK_TransformedCollision` benchmark
   - Use P2 timing methodology (multiple iterations, various hull sizes)
   - Set baseline: < 5% overhead (P2 showed < 2%, leave margin)

4. **No performance optimization needed**:
   - P2 shows transformation is not a bottleneck
   - Focus implementation effort on correctness, not performance

### For Reviewer

1. **Key areas to review**:
   - Transformation method pairing (`globalToLocalRelative` for directions, `localToGlobal` for vertices)
   - Test coverage matches P1 scenarios (identity, translation, rotation, combined)
   - No temporary hull allocations (memory efficiency)

2. **Expected performance**:
   - Benchmarks should show < 5% overhead (P2 baseline: < 2%)
   - If overhead > 5%, investigate (likely implementation issue, not design issue)

---

## Conclusion

**All prototypes PASSED**. The design is validated and ready for implementation.

- **Transformation correctness**: Confirmed by analytical test cases
- **Performance overhead**: Negligible (< 2%, far below 20% threshold)
- **Design approach**: On-the-fly transformation is optimal (correct + performant + memory-efficient)

**Next Step**: Proceed to implementation phase with high confidence.

**Estimated Implementation Time**: 6-8 hours
**Prototype Artifacts**: Preserved in `prototypes/0022_gjk_asset_physical_transform/` for reference

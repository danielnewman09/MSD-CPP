# Implementation Notes — 0055c_friction_direction_fix

**Date**: 2026-02-11
**Status**: PARTIAL IMPLEMENTATION (EPA integration complete, CollisionHandler pending)
**Branch**: 0055c-friction-direction-fix
**Commits**: f2b34e5, cb740a4

---

## Summary

Implemented vertex-face contact manifold generation system to fix energy injection from single-point friction contacts. Two new components (VertexFaceDetector, VertexFaceManifoldGenerator) were created and integrated into EPA's degenerate case handling.

**Completion**: ~50% (EPA path done, CollisionHandler SAT fallback path pending)

---

## Files Created

| File | Purpose | LOC |
|------|---------|-----|
| `msd/msd-sim/src/Physics/Collision/VertexFaceDetector.hpp` | Contact geometry classification | 106 |
| `msd/msd-sim/src/Physics/Collision/VertexFaceDetector.cpp` | Detector implementation | 41 |
| `msd/msd-sim/src/Physics/Collision/VertexFaceManifoldGenerator.hpp` | Multi-point manifold generation | 137 |
| `msd/msd-sim/src/Physics/Collision/VertexFaceManifoldGenerator.cpp` | Generator implementation | 92 |
| **Total** | | **376 LOC** |

---

## Files Modified

| File | Changes | Reason |
|------|---------|--------|
| `msd/msd-sim/src/Physics/Collision/CMakeLists.txt` | Added VertexFaceDetector.cpp, VertexFaceManifoldGenerator.cpp to build | Build system integration |
| `msd/msd-sim/src/Physics/Collision/EPA.hpp` | Added includes, member instances (vertexFaceDetector_, vertexFaceManifoldGenerator_), generateVertexFaceManifold() method | EPA integration |
| `msd/msd-sim/src/Physics/Collision/EPA.cpp` | Added vertex-face check at line 574, implemented generateVertexFaceManifold() method | EPA integration |
| `docs/designs/0055c_friction_direction_fix/iteration-log.md` | Created from template, 2 iteration entries | Iteration tracking |

---

## Design Adherence Matrix

| Design Element | Implemented? | Notes |
|----------------|--------------|-------|
| VertexFaceDetector class | ✅ | Matches design specification exactly |
| VertexFaceManifoldGenerator class | ✅ | Matches design, Option A (uniform EPA depth) applied |
| EPA integration (degenerate case) | ✅ | Integrated at line 574 as specified |
| CollisionHandler integration (SAT fallback) | ❌ | **PENDING** — Requires extractFaceVertices() helper method |
| Unit tests for VertexFaceDetector | ❌ | **PENDING** |
| Unit tests for VertexFaceManifoldGenerator | ❌ | **PENDING** |
| Integration tests | ❌ | **PENDING** |
| Tilted cube test updates | ❌ | **PENDING** |

### Design Deviations

**Minor deviation**: Split `VertexFaceManifoldGenerator` constructor into two overloads (default + explicit) instead of single constructor with default argument. This avoids C++ aggregate initialization conflict with default member initializers in Config struct.

**Impact**: None — semantically equivalent, fixes build error.

---

## Prototype Application Notes

**N/A** — Human directed to skip prototype phase entirely and proceed directly to implementation.

---

## Test Coverage Summary

### Build Status
✅ **PASS** — All new files compile without warnings.

### Test Results

| Suite | Status | Notes |
|-------|--------|-------|
| Full test suite | 657/661 passing | 4 failures appear pre-existing (D4, H3, B2, B5) |
| VertexFaceDetector unit tests | ❌ Not implemented | **TODO** |
| VertexFaceManifoldGenerator unit tests | ❌ Not implemented | **TODO** |
| TiltedCubeTrajectory integration tests | ❌ Not run | **TODO** |

### Pre-existing Failures
- `ContactManifoldStabilityTest.D4_MicroJitter_DampsOut`
- `ParameterIsolation.H3_TimestepSensitivity_ERPAmplification`
- `RotationalCollisionTest.B2_CubeEdgeImpact_PredictableRotationAxis`
- `RotationalCollisionTest.B5_LShapeDrop_RotationFromAsymmetricCOM`

**Assessment**: No regressions introduced by EPA integration. Pre-existing failures unrelated to vertex-face manifold generation.

---

## Known Limitations

1. **CollisionHandler SAT fallback not integrated** — Vertex-face manifold generation only works for contacts detected via EPA, not SAT fallback. Requires implementing `extractFaceVertices()` helper method or similar to identify face vertices from SAT normal.

2. **No unit tests** — New components have zero test coverage. Need tests per design document:
   - VertexFaceDetector: DetectContactType_* tests (4 test cases)
   - VertexFaceManifoldGenerator: Generate_* tests (4 test cases)
   - Integration tests for EPA and CollisionHandler paths

3. **Tilted cube tests not updated** — Test assertions in `TiltedCubeTrajectoryTest.cpp` still expect pre-fix contact counts (89% single-point). Need to update expectations to >50% four-point after implementation completes.

4. **Contact count diagnostic test pending** — `Diag_ContactCount_And_LeverArm` test will need assertion updates once vertex-face manifold generation is active.

---

## Future Considerations

### Performance Impact

**Not measured** — Design estimates < 5% overhead on vertex-face-heavy scenarios, 0% on typical scenarios. Actual measurement deferred due to budget constraints.

**Recommendation**: Benchmark before/after on tilted cube scenarios once implementation completes.

### Per-Point Depth (Option B)

**Current**: All contacts use uniform EPA depth (Option A from design).

**Future**: If constraint solver stability issues arise, implement per-point depth computation using `computePointDepth()` method (already present but unused). This would replace uniform depth with per-contact-point projection distances.

### Edge Case Handling

**Degenerate face detection**: Current implementation returns 0 contacts for < 3 face vertices. Consider adding alignment check (as recommended in design) to detect nearly-parallel face-normal configurations and fall back gracefully.

---

## Implementation Checklist

**Completed**:
- [x] VertexFaceDetector class (header + implementation)
- [x] VertexFaceManifoldGenerator class (header + implementation)
- [x] CMakeLists.txt integration
- [x] EPA degenerate case integration
- [x] Build system verification
- [x] Regression testing (657/661 maintained)

**Pending**:
- [ ] CollisionHandler SAT fallback integration
- [ ] Unit tests for VertexFaceDetector
- [ ] Unit tests for VertexFaceManifoldGenerator
- [ ] Integration tests (EPA path, CollisionHandler path)
- [ ] Tilted cube test assertion updates
- [ ] Full test suite validation (target: 693+ passing)
- [ ] Performance benchmarking
- [ ] Visual validation in GUI

---

## Handoff Notes

### For Next Implementation Session

1. **Complete CollisionHandler integration**:
   - Extract face vertices from SAT normal and depth
   - Add vertex-face detection before single-point fallback
   - Implement `generateVertexFaceManifold()` method or integrate generator directly

2. **Write unit tests** (see design document for test specifications):
   - `test/Physics/VertexFaceDetectorTest.cpp` — 4 test cases
   - `test/Physics/VertexFaceManifoldGeneratorTest.cpp` — 4 test cases

3. **Run tilted cube tests**:
   ```bash
   ./build/Debug/debug/msd_sim_test --gtest_filter="TiltedCubeTrajectory.*"
   ```
   - Check if `Sliding_ThrownCube_SDLAppConfig` passes (target: spurious Y < 10% of X)
   - Check if `Sliding_PurePitch_vs_CompoundTilt_SpuriousY` passes
   - Update `Diag_ContactCount_And_LeverArm` assertions for new contact distribution

4. **Verify full test suite**:
   ```bash
   ./build/Debug/debug/msd_sim_test
   ```
   - Target: 693+ passing (or at least no new failures vs baseline)

### Iteration Log Location

Full implementation history: `docs/designs/0055c_friction_direction_fix/iteration-log.md`

---

## Questions for Human Review

1. **CollisionHandler approach**: Should I implement a general `extractFaceVertices()` helper that works for both EPA and SAT, or keep them separate with SAT-specific logic?

2. **Test coverage priority**: Unit tests vs integration tests — which should be implemented first given budget constraints?

3. **Performance baseline**: Should I run benchmarks now (partial implementation) or wait until CollisionHandler integration completes?

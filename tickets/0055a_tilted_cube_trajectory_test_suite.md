# Ticket 0055a: Tilted Cube Trajectory Test Suite

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Quality Gate Passed — Awaiting Review
**Type**: Test Suite
**Priority**: High
**Assignee**: cpp-implementer → Human Review
**Created**: 2026-02-10
**Branch**: 0055a-tilted-cube-trajectory-test-suite
**GitHub Issue**: #37
**GitHub PR**: #38
**Parent Ticket**: [0055_tilted_cube_friction_direction](0055_tilted_cube_friction_direction.md)
**Dependencies**: None
**Generate Tutorial**: No

---

## Objective

Create a comprehensive test suite that drops tilted cubes onto a horizontal floor from multiple tilt orientations and asserts that the resulting trajectory (lateral displacement direction) is physically consistent with the tilt geometry. These tests serve as the diagnostic foundation for the friction direction investigation.

---

## Test Matrix

### Tilt Angles

Use a moderate tilt angle `θ = 0.1 rad (~5.7°)` — large enough to produce visible trajectory differences, small enough to avoid multi-bounce complexity.

| Test ID | Tilt (radians) | Description |
|---------|----------------|-------------|
| T1 | `(+θ, 0, 0)` | Pure positive X tilt |
| T2 | `(-θ, 0, 0)` | Pure negative X tilt |
| T3 | `(0, +θ, 0)` | Pure positive Y tilt |
| T4 | `(0, -θ, 0)` | Pure negative Y tilt |
| T5 | `(+θ, +θ, 0)` | Compound X+Y tilt |
| T6 | `(+θ, -θ, 0)` | Compound X-Y tilt |
| T7 | `(-θ, +θ, 0)` | Compound -X+Y tilt |
| T8 | `(-θ, -θ, 0)` | Compound -X-Y tilt |

### Test Setup (Common to All)

- **Floor**: `createCubePoints(100.0)` centered at `z = -50` (top face at `z = 0`)
- **Cube**: `createCubePoints(1.0)`, mass = 10.0 kg
- **Initial position**: Center of mass at height such that lowest corner is at `z = 0.01` (just above floor)
- **Initial velocity**: `(0, 0, 0)` — dropped from rest
- **Friction coefficient**: 0.5 (both body and floor)
- **Restitution**: 0.3 (low, to minimize multi-bounce complexity)
- **Simulation duration**: 200 frames at 16ms timestep (~3.2 seconds)

### Assertions

For each test, after simulation:

1. **Direction assertion**: The cube's final lateral displacement (X, Y) from origin must be in the direction consistent with the tilt geometry. Specifically:
   - A tilt that lowers the +Y edge of the cube should produce displacement with a positive Y component (the cube tips/slides toward the lowered edge).
   - A tilt about +X axis rotates the cube such that the -Y edge goes down → cube tips toward -Y... (exact sign mapping to be determined empirically by running a known-correct configuration first, then asserting the pattern holds across all orientations).

2. **Symmetry assertion**: Pairs of tests with negated tilt must produce mirrored trajectories:
   - `T1` vs `T2`: X displacement equal, Y displacement opposite (or vice versa depending on axis convention)
   - `T3` vs `T4`: Same pattern
   - `T5` vs `T8`: Full diagonal mirror
   - `T6` vs `T7`: Full diagonal mirror

3. **Magnitude assertion**: All tests should produce comparable displacement magnitudes (within 2x of each other), since the tilt angle magnitude is the same.

4. **No NaN/divergence**: Position remains finite throughout simulation.

---

## Implementation Plan

### New Test File

Create `msd/msd-sim/test/Physics/Collision/TiltedCubeTrajectoryTest.cpp`

### Test Structure

```
TEST_SUITE: TiltedCubeTrajectory

Helper function:
  runTiltedCubeSimulation(double tiltX, double tiltY, int frames)
    → returns { finalPosition, finalOrientation, maxLateralDisplacement }

Individual tests:
  T1_PurePositiveXTilt
  T2_PureNegativeXTilt
  T3_PurePositiveYTilt
  T4_PureNegativeYTilt
  T5_CompoundXPlusYTilt
  T6_CompoundXMinusYTilt
  T7_CompoundNegXPlusYTilt
  T8_CompoundNegXMinusYTilt

Symmetry tests:
  Symmetry_XTilt_MirrorsYDisplacement
  Symmetry_YTilt_MirrorsXDisplacement
  Symmetry_CompoundTilt_DiagonalMirror
```

### Helper Function Design

```cpp
struct TiltResult {
  Coordinate finalPosition;
  QuaternionD finalOrientation;
  double lateralDisplacementX;
  double lateralDisplacementY;
  double finalZ;
  bool nanDetected;
};

TiltResult runTiltedCubeSimulation(double tiltX, double tiltY,
                                    double frictionCoeff = 0.5,
                                    double restitution = 0.3,
                                    int frames = 200);
```

### CMake Integration

Add `TiltedCubeTrajectoryTest.cpp` to existing test target in `msd/msd-sim/test/CMakeLists.txt`.

---

## Acceptance Criteria

1. All 8 tilt orientation tests implemented and named consistently
2. Symmetry tests verify mirrored trajectories for negated tilts
3. Helper function encapsulates common setup to avoid test duplication
4. Tests are initially expected to **expose failures** — this is a diagnostic test suite
5. Test file compiles and links with existing test infrastructure
6. Tests use the ticket reference: `// Ticket: 0055a_tilted_cube_trajectory_test_suite`

---

## Notes

- The exact sign mapping between tilt axis and displacement direction depends on the coordinate system convention (right-handed, Z-up). The first test (T1) should be verified manually or against a known-correct reference before asserting the pattern.
- If friction is not yet wired for the floor environment object, these tests may need to explicitly set `setFrictionCoefficient()` on both the inertial object and the environment object.
- Existing tilted cube tests (H8, C2, C3) only check energy/stability, not trajectory direction. This test suite fills that gap.

---

## Workflow Log

### Draft → Ready for Implementation Phase
- **Started**: 2026-02-10 15:10
- **Completed**: 2026-02-10 15:12
- **Branch**: 0055a-tilted-cube-trajectory-test-suite
- **PR**: N/A (will be created after implementation)
- **Artifacts**:
  - Created branch `0055a-tilted-cube-trajectory-test-suite`
  - Created GitHub issue #37
- **Notes**: Test suite implementation ticket with clear requirements. Skipping design phase as this is straightforward test code following existing patterns.

### Implementation Phase
- **Started**: 2026-02-10 15:15
- **Completed**: 2026-02-10 15:45
- **Branch**: 0055a-tilted-cube-trajectory-test-suite
- **PR**: #38
- **Artifacts**:
  - `msd/msd-sim/test/Physics/Collision/TiltedCubeTrajectoryTest.cpp` (554 lines)
  - Updated `msd/msd-sim/test/Physics/Collision/CMakeLists.txt`
- **Test Results**:
  - 12 tests total: 8 PASS (no NaN), 4 FAIL (zero displacement, expected diagnostic failure)
  - All tilted cubes end at (0, 0, Z) with zero X/Y motion
  - Confirms friction direction bug as hypothesized in parent ticket
- **Notes**: Tests successfully expose the bug — zero lateral displacement for all tilt configurations. This is the expected failure mode for a diagnostic test suite.

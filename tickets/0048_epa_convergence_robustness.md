# Ticket 0048: EPA Convergence Robustness

## Status: Complete

## Type: Bug Fix

## Priority: High

## Failing Tests
- H6_ZeroGravity_RestingContact_Stable (ParameterIsolation)

## Problem Statement
EPA throws `std::runtime_error("EPA failed to converge within max iterations")` during
simulation of two cubes in resting contact with zero gravity. The exception occurs on
frame 2 of the simulation, after the PositionCorrector has applied position and
orientation corrections from frame 1.

## Root Cause Analysis

### Confirmed Root Cause: EPA cannot converge when both objects have identical micro-rotation

The PositionCorrector applies angular pseudo-impulses at corner contact points, which
introduces a tiny rotation (~1.4e-7 rad around X-axis) to both cubes. When both cubes
share the same micro-rotation, the Minkowski difference geometry becomes nearly identical
to the axis-aligned case but with faces tilted by ~2.8e-7 rad. EPA's expansion algorithm
cannot make sufficient progress per iteration (improvement < epsilon=1e-6), causing it to
cycle and hit max iterations.

### Evidence

1. **Static collision at same positions works fine**: Fresh AssetInertial objects at the
   exact post-PositionCorrector positions (-0.0005, 0.9905) with identity quaternions
   produce correct collision results (4 contacts, correct depth).

2. **Individual rotation of either cube works fine**: Rotating only cube A or only cube B
   by the simulation quaternion still produces correct results.

3. **Both cubes with same quaternion triggers EPA exception**: When both cubes have
   `qx = -1.40625e-07` (same rotation direction/magnitude), EPA fails to converge.

4. **The rotation comes from PositionCorrector**: Angular pseudo-impulses at corner
   contact points (r x n != 0 for corners at +-0.5 Y/Z) produce rotational corrections
   even for face-face contacts.

### Simulation Quaternions at Failure
```
qA = (1, -1.40625e-07,  5.27344e-11, -1.0842e-19)
qB = (1, -1.40625e-07, -5.27344e-11,  1.0842e-19)
```

Both share the same X rotation component. The Y/Z components are opposite-signed and
much smaller.

### Additional Finding: EPA returns incorrect depth at certain rotations
The rotation sensitivity sweep also revealed that at certain rotation values (5e-8,
1e-7, 1e-4 rad), EPA returns penetration depth of 1.0 instead of the correct 0.01.
This indicates EPA is finding a wrong face of the Minkowski difference polytope, not
just failing to converge.

## Proposed Fix Approaches

### Approach A: Graceful EPA degradation (recommended)
Instead of throwing an exception when EPA reaches max iterations, return the best
available result (closest face found so far). This matches standard practice in
production physics engines.

### Approach B: PositionCorrector angular threshold
Increase the angular correction threshold in PositionCorrector from 1e-12 to a
value that prevents sub-epsilon rotations (e.g., 1e-6). This prevents the
PositionCorrector from introducing rotations that are too small to be meaningful
but large enough to break EPA.

### Approach C: EPA convergence criterion improvement
Modify the EPA convergence check to detect stalling (same closest face for N
iterations) and terminate early with the current best result.

## Fix Applied: Approach A — Graceful EPA Degradation

When `expandPolytope()` reaches max iterations without converging, EPA now returns
the best result from the closest polytope face found so far instead of throwing
`std::runtime_error`. This matches standard practice in production physics engines
(Bullet, PhysX, Box2D all use best-effort EPA results).

### Changes
- `EPA::computeContactInfo()`: Removed exception throw on `expandPolytope()` failure.
  The function now falls through to extract contact info from the best face regardless.
- `EPA.hpp`: Updated `@throws` documentation to reflect the change.

### Test Results
- **Before**: 681/688 passing (7 failing)
- **After**: 683/688 passing (5 failing)
- **Net**: +2 tests fixed (H6_SimulationFrameByFrame, H6_ZeroGravity_RestingContact_Stable)
- **Regressions**: 0

## Dependencies
- **Ticket 0047** (face contact manifold generation): May depend on this fix.
  The D1/D4/H1 failures from 0047 may share the same EPA convergence root cause
  at shallow penetration depths with micro-rotations.

## Diagnostic Test Files
- `msd/msd-sim/test/Physics/Collision/EPAConvergenceDiagnosticTest.cpp`
  - Phase 1: Static H6 reproduction (passes — confirms static case works)
  - Phase 2: Penetration depth sweeps (cube-on-floor, cube-cube)
  - Phase 3: Frame-by-frame simulation (identifies frame 2 as failure point)
  - Phase 4a: Rotation sensitivity sweep
  - Phase 4b: Exact quaternion reproduction (confirms root cause)
  - Phase 4: Position probe (confirms positions alone are not the issue)

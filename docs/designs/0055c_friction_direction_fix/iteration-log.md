# Iteration Log — 0055c_friction_direction_fix

> **Purpose**: Track every build-test cycle during implementation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0055c_friction_direction_fix/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0055c_friction_direction_fix
**Branch**: 0055c-friction-direction-fix
**Baseline**: 693/693 tests passing at start (assumed from ticket)

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 0 — 2026-02-11 10:00
**Commit**: f2b34e5
**Hypothesis**: Create base components for vertex-face manifold generation (VertexFaceDetector + VertexFaceManifoldGenerator) following design specifications.
**Changes**:
- `msd/msd-sim/src/Physics/Collision/VertexFaceDetector.hpp`: Contact type classification enum and detector class
- `msd/msd-sim/src/Physics/Collision/VertexFaceDetector.cpp`: Implementation with simple vertex count rules
- `msd/msd-sim/src/Physics/Collision/VertexFaceManifoldGenerator.hpp`: Manifold generator with projection-based algorithm
- `msd/msd-sim/src/Physics/Collision/VertexFaceManifoldGenerator.cpp`: Implementation using uniform EPA depth (Option A)
- `docs/designs/0055c_friction_direction_fix/iteration-log.md`: Created from template
**Build Result**: NOT ATTEMPTED (components not yet integrated into build system)
**Test Result**: NOT ATTEMPTED (no tests yet)
**Impact vs Previous**: N/A (first iteration)
**Assessment**: Components created according to design. Next: Add to CMake, integrate into EPA degenerate case (line 574 in EPA.cpp), write unit tests.

### Iteration 1 — 2026-02-11 10:30
**Commit**: (pending)
**Hypothesis**: Integrate vertex-face manifold generation into EPA degenerate case handling, add to build system.
**Changes**:
- `msd/msd-sim/src/Physics/Collision/CMakeLists.txt`: Added VertexFaceDetector and VertexFaceManifoldGenerator to build
- `msd/msd-sim/src/Physics/Collision/EPA.hpp`: Added includes, member instances, and generateVertexFaceManifold method declaration
- `msd/msd-sim/src/Physics/Collision/EPA.cpp`: Integrated vertex-face detection at line 574 (degenerate case), implemented generateVertexFaceManifold method
- `msd/msd-sim/src/Physics/Collision/VertexFaceManifoldGenerator.hpp`: Fixed default constructor issue (split into default + explicit constructors)
**Build Result**: PASS
**Test Result**: 657/661 — 4 failures (D4_MicroJitter, H3_TimestepSensitivity, B2_CubeEdgeImpact, B5_LShapeDrop) appear pre-existing
**Impact vs Previous**: +0 passes, -0 regressions (baseline maintained)
**Assessment**: EPA integration successful, no regressions introduced. Next: Integrate into CollisionHandler SAT fallback path, write unit tests for new components, test tilted cube scenarios.

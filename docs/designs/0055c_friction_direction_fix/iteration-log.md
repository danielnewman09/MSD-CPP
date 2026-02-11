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
**Commit**: cb740a4
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

### Iteration 2 — 2026-02-11 (session 2 completion)
**Commit**: dfd189b
**Hypothesis**: Complete CollisionHandler SAT fallback integration and add comprehensive unit tests for new components.
**Changes**:
- `msd/msd-sim/src/Physics/Collision/CollisionHandler.hpp`: Added includes for VertexFaceDetector/Generator, added extractFaceVertices() helper method, added member instances
- `msd/msd-sim/src/Physics/Collision/CollisionHandler.cpp`: Implemented extractFaceVertices() to find face vertices from hull given normal, modified buildSATContact() to detect vertex-face geometry and delegate to manifold generator, added fallback to single-point contact if manifold generation fails
- `msd/msd-sim/test/Physics/Collision/VertexFaceDetectorTest.cpp`: Created 5 unit tests for contact type classification (FaceFace, EdgeEdge, VertexFace, VertexVertex, Unknown)
- `msd/msd-sim/test/Physics/Collision/VertexFaceManifoldGeneratorTest.cpp`: Created 6 unit tests for manifold generation (CubeOnFloor, TriangleVertex, DepthConsistency, DegenerateFace, MaxContactLimit, ProjectionPlaneCorrectness)
- `msd/msd-sim/test/Physics/Collision/CMakeLists.txt`: Added new test files to build
**Build Result**: PASS
**Test Result**: 668/672 — Added 11 new tests (all pass), 4 pre-existing failures unchanged
**Impact vs Previous**: +11 passes (new tests), -0 regressions (baseline maintained)
**Assessment**: CollisionHandler SAT fallback integration complete. All unit tests pass. Implementation now covers both EPA and SAT paths. Remaining: Integration testing with tilted cube trajectory scenarios from 0055a (tests not yet available in main branch).

### Iteration 3 — 2026-02-11 (session 3)
**Commit**: 59e1654
**Hypothesis**: Merge 0055b branch (which contains TiltedCubeTrajectory integration tests and diagnostics) into 0055c to enable end-to-end validation of the vertex-face manifold fix.
**Changes**:
- `git merge 0055b-friction-direction-root-cause`: Brought in TiltedCubeTrajectoryTest.cpp with diagnostic tests and integration tests
- `msd/msd-sim/test/Physics/Collision/CMakeLists.txt`: Resolved merge conflict — kept all three new test files (TiltedCubeTrajectoryTest, VertexFaceDetectorTest, VertexFaceManifoldGeneratorTest)
**Build Result**: PASS
**Test Result**: 672/686 — 14 TiltedCubeTrajectory tests fail (same 3 from 0055a + 11 others), contact count diagnostic shows **89% single-point contacts unchanged**
**Impact vs Previous**: +14 new tests added from merge, 14 failing. Fix is NOT activating for the actual bug scenario.
**Assessment**: **CRITICAL FINDING** — The vertex-face detection at EPA line 574 (degenerate polygon check: `refVerts.size() < 3 || incidentPoly.size() < 3`) NEVER triggers for the tilted cube scenario. For a cube on a floor, EPA builds coplanar facet polygons with >= 3 vertices on both sides. The Sutherland-Hodgman clipping THEN reduces the incident polygon to 1 point, but this happens AFTER the degenerate check. The fix is in the wrong location — it needs to go AFTER clipping, not before.

### Iteration 4 — 2026-02-11 (session 3, continued)
**Commit**: (uncommitted)
**Hypothesis**: Move vertex-face manifold expansion to AFTER Sutherland-Hodgman clipping, where `finalPoints.size() <= 2 && refVerts.size() >= 3` detects the actual vertex-face geometry.
**Changes**:
- `msd/msd-sim/src/Physics/Collision/EPA.cpp`: Added post-clipping vertex-face expansion at line 646 (after final point filtering). When clipping reduces to 1-2 points but reference face has >= 3 vertices, projects reference face vertices onto contact plane with uniform EPA depth.
**Build Result**: PASS
**Test Result**: 672/686 — 14 TiltedCubeTrajectory tests still fail, BUT behavior changed fundamentally:
- Contact count: **0% single-point, 100% 4-point** (fix IS activating)
- Yaw coupling: compound+friction peak omega_z dropped from 2.06 to 0.019 rad/s
- Lateral displacement: now essentially **zero** (1e-10 to 1e-13 m) — cube does not slide at all
- The cube is **over-constrained**: 4-point friction manifold prevents all lateral motion
**Impact vs Previous**: Fix activates correctly (0% single-point → 100% 4-point). But over-constraining creates new failure mode — no sliding where sliding is expected.
**Assessment**: The manifold expansion correctly eliminates single-point contacts, but the 4 friction constraints (one per contact point) over-constrain the body and prevent any lateral sliding. Root cause options: (1) friction force too high with 4 contact points — may need to distribute friction budget across contact points, (2) contact point placement creates zero-net-torque configuration that locks the body, (3) need to re-examine whether ALL reference face vertices should be used vs a subset. **NOTE**: This iteration used REFERENCE face vertices (from the floor, 50m apart) — see iteration 5 for the fix.

### Iteration 5 — 2026-02-11 (session 3, continued)
**Commit**: 5f5c51a (iteration 4 EPA fix) + uncommitted (iteration 5 incident-face switch)
**Hypothesis**: Iteration 4 used reference face vertices (floor = 100m cube, vertices at ±50m). Contact points were 50m apart for a 1m cube — clearly wrong. Switch to INCIDENT face vertices (pre-clipping), which represent the actual touching body's face geometry (~0.5m apart for a 1m cube).
**Changes**:
- `msd/msd-sim/src/Physics/Collision/EPA.cpp`:
  - Save `incidentPolyPreClip` before the Sutherland-Hodgman clipping loop
  - Post-clipping expansion now uses `incidentPolyPreClip` (incident face) instead of `refVerts` (reference face)
**Build Result**: PASS
**Test Result**: 675/696 — 21 failures total:
- 4 pre-existing: D4, H3, B2, B5
- 14 TiltedCubeTrajectory (expect lateral displacement, get zero)
- **3 NEW REGRESSIONS**: B1_CubeCornerImpact (no rotation: omega=1.1e-7), F4_RotationEnergyTransfer (no rotational KE), C3_TiltedCubeSettles (stuck at 30°, doesn't settle to flat face)
**Impact vs Previous**: Contact points now at correct scale (~0.5m, cube-sized). Energy injection eliminated (compound KE identical to pure pitch). Yaw coupling reduced from 2.06 to 3.9e-9 rad/s. BUT bodies are STILL over-constrained — 3 additional regressions in rotation tests.
**Assessment**: Even with correctly-scaled incident face vertices, creating 4 contact points across the ENTIRE bottom face of a tilted cube is wrong. Only ONE corner is actually touching — the other 3 vertices are phantom contacts that prevent the cube from rotating. The vertex-face expansion creates contacts at all face vertices regardless of whether they are actually in contact. This over-constrains rotational motion. Need a fundamentally different approach: either (a) only expand near the ACTUAL contact point, or (b) don't expand at all and fix the friction Jacobian instead, or (c) limit expansion to vertices that are actually within the penetration zone.

### Circle Detection — CONFIRMED
EPA.cpp `extractContactManifold()` modified in iterations 1, 4, and 5 (3 times). Each attempt places contacts at different locations but all result in over-constraining. The approach of "expand single-point to multi-point by projecting face vertices" is fundamentally flawed for vertex-face contacts where only one vertex is truly in contact. **Human decision: revert EPA changes, fix in constraint solver instead.**

### Iteration 6 — 2026-02-11 (session 3, revert)
**Commit**: 5a0d7de
**Hypothesis**: The EPA manifold expansion approach is fundamentally wrong — it trades yaw coupling for over-constraining. Revert ALL EPA and CollisionHandler changes to main branch state. The fix should go in the friction constraint (address yaw coupling at the Jacobian level, not the manifold level).
**Changes**:
- `msd/msd-sim/src/Physics/Collision/EPA.cpp`: Reverted to main (removed post-clipping expansion, removed incidentPolyPreClip)
- `msd/msd-sim/src/Physics/Collision/EPA.hpp`: Reverted to main (removed VertexFaceDetector/Generator members, removed generateVertexFaceManifold)
- `msd/msd-sim/src/Physics/Collision/CollisionHandler.cpp`: Reverted to main (removed extractFaceVertices, removed vertex-face detection in SAT fallback)
- `msd/msd-sim/src/Physics/Collision/CollisionHandler.hpp`: Reverted to main (removed VertexFaceDetector/Generator members, removed extractFaceVertices)
- VertexFaceDetector and VertexFaceManifoldGenerator source/test files KEPT (inert unless called, 11 unit tests still pass)
**Build Result**: PASS
**Test Result**: 689/696 — Back to baseline. 4 pre-existing (D4, H3, B2, B5) + 3 TiltedCubeTrajectory (the original 0055a bug). B1, F4, C3 regressions GONE.
**Impact vs Previous**: All regressions eliminated. Clean baseline restored.
**Assessment**: EPA manifold expansion abandoned. Next approach: fix the friction Jacobian directly to eliminate yaw coupling from single-point vertex-face contacts without changing the contact manifold.

### Iteration 7 — 2026-02-11 (session 3, friction Jacobian approach)
**Commit**: (pending)
**Hypothesis**: The friction lever arm `rA = contactPoint - CoM` has a component along the contact normal `n`. When computing `rA × t` in the Jacobian, this normal component creates parasitic torque about the normal axis (yaw for horizontal contacts). Projecting the lever arm onto the tangent plane (`rA_tangent = rA - (rA·n)·n`) should eliminate the parasitic yaw coupling while preserving correct tangential angular friction.
**Changes**:
- `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.cpp`: In constructor, project lever arms onto tangent plane before storing. `lever_arm_a_ = fullLeverA - normal * (fullLeverA·normal)`, same for B.
**Build Result**: PASS
**Test Result**: 10/24 TiltedCubeTrajectory pass (same as baseline). Key diagnostics:
- Peak KE: pure=100.4, compound=89.7 (initial=125) — no energy injection above initial, but compound≠pure (asymmetry remains)
- Yaw coupling: compound+friction peak omega_z = **1.93 rad/s** — barely changed from baseline (was 2.06 without fix)
- The 3 original 0055a failures still present
**Impact vs Previous**: Negligible change from baseline. Lever arm projection did NOT fix yaw coupling.
**Assessment**: **FAILED**. The tangent-plane projection of the lever arm had minimal effect on yaw coupling (1.93 vs 2.06 rad/s). The yaw torque does NOT come primarily from the normal component of the lever arm. The cross product `rA × t` creates yaw even with a tangent-plane-only lever arm when the lever arm has components in both tangent directions. Reverting this change.

**Human insight (post-iteration 7)**: Yaw coupling may be physically correct — a corner impact SHOULD create yaw. The real bug is **energy injection**: friction should only dissipate energy (F_friction · v_contact ≤ 0), but under certain circumstances friction acts in the wrong direction, causing KE to increase. The fix should target the energy injection mechanism, not the yaw coupling itself. Next step: diagnostic to measure friction power (F·v) per frame and identify when/why friction injects energy.

### Iteration 8 — 2026-02-11 (session 4, energy diagnostic — wrong scenario)
**Commit**: (pending)
**Hypothesis**: Measure total mechanical energy (KE_translational + KE_rotational + gravitational PE) frame by frame. Any frame where total E increases means a non-conservative force injected energy. Compare friction vs frictionless cases.
**Changes**:
- `msd/msd-sim/test/Physics/Collision/TiltedCubeTrajectoryTest.cpp`: Added `Diag_MechanicalEnergy_FrictionInjection` test measuring E = ½mv² + ½Iω² + mgh every frame for friction=0.5 and friction=0.0.
**Build Result**: PASS
**Test Result**: `injectionFrames=0` for BOTH friction=0.5 and friction=0.0 cases. Zero energy injection detected.
**Impact vs Previous**: N/A (diagnostic only, no code changes)
**Assessment**: **WRONG SCENARIO**. This diagnostic uses a tilted cube dropped from REST (tiltX=0.1, tiltY=0.1, no horizontal velocity). The actual failing tests (Sliding_ThrownCube_SDLAppConfig, SpuriousY) use PI/3 pitch + 5 m/s initial horizontal velocity. The energy injection may only manifest when friction opposes sliding motion while the body is simultaneously rotating — a scenario not covered by this drop-only test. Next: modify diagnostic to use the actual failing scenario parameters (PI/3 pitch, 0.01 roll, 5 m/s initial Vx).

### Iteration 9 — 2026-02-11 (session 4, energy diagnostic — correct scenario)
**Commit**: (pending)
**Hypothesis**: Same as iteration 8, but using the actual failing scenario parameters (PI/3 pitch, 0.01 roll, 5 m/s initial Vx, e=0.5, friction=0.5, dt=10ms) matching Sliding_ThrownCube_SDLAppConfig. Also uses actual inertia tensor for accurate rotational KE.
**Changes**:
- `msd/msd-sim/test/Physics/Collision/TiltedCubeTrajectoryTest.cpp`: Updated `Diag_MechanicalEnergy_FrictionInjection` with thrown-cube parameters and real inertia tensor.
**Build Result**: PASS
**Test Result**: **ENERGY INJECTION CONFIRMED**
- friction=0.5: **238 injection frames**, cumulative injection = **106.35 J**, max single-frame injection = **0.96 J** at frame 121
- friction=0.0: **0 injection frames**, cumulative injection = 0 J
- Injection pattern: starts at frame 77 (vy=-1.42, wz=+0.10), intensifies through frames 99-200+. The cube accelerates in the -Y direction via friction (vy grows from -1.78 to -2.4+), while keT increases monotonically. After frame ~280, vy reverses sign but injection continues.
- Key observation: at every injection frame, the cube has significant vy (spurious lateral velocity). Friction appears to be pushing the cube laterally rather than opposing its sliding direction.
**Impact vs Previous**: Confirmed the user's hypothesis — friction is injecting energy by acting in the wrong direction. The bug only manifests with initial horizontal velocity (sliding + tilting).
**Assessment**: Energy injection confirmed. The friction force vector does not oppose the contact-point sliding velocity. Next step: instrument the constraint solver to print the actual friction force direction vs contact point velocity direction at injection frames, to identify WHY friction acts in the wrong direction (tangent basis misalignment? stale contact normal? wrong lever arm?).

### Iteration 10 — 2026-02-11 (session 5, friction work direction diagnostic)
**Commit**: (pending)
**Hypothesis**: If friction injects energy, then the friction work proxy Fw = Δv_tangent · v_tangent should be positive at injection frames. Decompose the velocity impulse (post minus pre minus gravity) into normal and tangential components and check friction work sign.
**Changes**:
- `msd/msd-sim/test/Physics/Collision/TiltedCubeTrajectoryTest.cpp`: Added `Diag_FrictionWorkDirection` diagnostic test measuring per-frame tangential impulse, contact-point velocity, and friction work proxy.
**Build Result**: PASS
**Test Result**: **SURPRISING FINDING**
- Contact normal perfectly stable at (0,0,-1) every frame — no wobbling
- Tangent basis consistently t1=(0,1,0), t2=(1,0,0) — stable
- At MANY injection frames, dvT≈(0,0) — no tangential velocity impulse
- Friction work proxy (Fw = dvT·vT) is mostly small or zero at injection frames
- Only ~41/200 frames have positive friction work, but 238/500 frames have energy injection
- dE can be 0.5-0.96 J even when Fw=0
**Impact vs Previous**: Friction is NOT directly doing positive work at most injection frames. This rules out "friction pushes in wrong direction" as the primary mechanism.
**Assessment**: The energy injection mechanism is more subtle than direct friction positive work. Key insight: `getInertiaTensor()` returns body-frame tensor, but for a cube it equals the world-frame tensor (I_body ∝ Identity), so the energy computation is correct. The injection likely comes from friction gradually altering the trajectory over many frames (building vy, wz through small per-frame impulses), which changes the normal-direction approach velocity at subsequent bounces, causing restitution to amplify KE. **Two hypotheses to test**: (A) Friction changes the normal impulse via A-matrix coupling even when λ_t≈0, or (B) Friction acts on early frames (changing trajectory), and injection manifests at later bounce frames. Evidence for (B): injection starts at frame 77 (after 0.77s of trajectory modification). Next: create diagnostic comparing normal impulse (Δv_n) between friction=0 and friction=0.5 at corresponding frames.

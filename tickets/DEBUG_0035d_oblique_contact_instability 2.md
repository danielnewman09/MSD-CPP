# DEBUG TICKET: Oblique Contact Energy Injection and Direction Reversal

## Status
- [x] Phase 1: Reproduce and Characterize
- [x] Phase 2: Isolate Energy Source
- [x] Phase 3: Root Cause Identification
- [ ] Phase 4: Fix and Validate

**Current Phase**: Root causes identified. Fix tickets created.
**Created**: 2026-02-02
**Parent Ticket**: [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)
**Related Debug Ticket**: [DEBUG_0035d_friction_energy_injection](DEBUG_0035d_friction_energy_injection.md)
**Trigger**: `FrictionValidationTest::oblique_contact` — tilted box exhibits direction reversal, energy injection, and near-rest instability

### Fix Tickets (created from this investigation)

| Ticket | Priority | Root Cause | Status |
|--------|----------|------------|--------|
| [0035da_post_impulse_energy_clamping](0035da_post_impulse_energy_clamping.md) | CRITICAL | Normal impulse injects rotational energy at bounces | Ready for Implementation |
| [0035db_friction_direction_clamping](0035db_friction_direction_clamping.md) | HIGH | Friction solver reverses lateral direction | Ready for Implementation |
| [0035dc_angular_velocity_rest_threshold](0035dc_angular_velocity_rest_threshold.md) | MEDIUM | Angular velocity threshold creates energy pump | Ready for Implementation |

---

## Problem Statement

A slightly rotated box dropped onto a frictional surface with lateral + downward velocity exhibits three physics violations:

1. **Direction reversal**: Initial x-velocity is +0.5 m/s, but after first contact it reverses to -3.36 m/s
2. **Energy injection at near-rest**: Box nearly comes to rest (KE ≈ 0.001 J at step 50), then KE explodes exponentially from 0.001 → 10.9 J over steps 51–70
3. **Step-to-step energy growth**: 32 instances of KE increasing between consecutive steps (2% tolerance exceeded)

### How This Differs from DEBUG_0035d_friction_energy_injection

The existing debug ticket covers a **flat box with pure horizontal velocity** where the ECOS solver produces wrong impulses on the first timestep. This ticket covers a **tilted box with oblique velocity** where:
- The collision response reverses lateral direction on first contact (not just wrong magnitude)
- The system reaches near-rest, then becomes **unstable** and re-injects energy exponentially
- The instability is cyclic — the box goes through multiple settle/explode cycles before eventually coming to rest

This suggests additional failure modes beyond the ECOS formulation bug, potentially in:
- Contact normal computation for tilted geometry
- Impulse direction calculation when contact normals are not axis-aligned
- Overlap-based contact detection oscillating between contact/no-contact at small penetrations

---

## Reproduction

**Test**: `FrictionValidationTest::oblique_contact`
**File**: `msd/msd-sim/test/Physics/FrictionValidationTest.cpp:257`

```bash
cmake --build --preset conan-debug --target msd_sim_test
./build/Debug/debug/msd_sim_test --gtest_filter="FrictionValidationTest.oblique_contact"
```

### Test Setup
- **Floor**: 10m cube at z=-5.0 (top face at z=0.0)
- **Box**: 1m cube at z=0.49 (bottom at z=-0.01, 0.01m overlap with floor)
- **Box rotation**: `AngularCoordinate{0.1, 0., 0.}` (slight tilt about x-axis)
- **Initial velocity**: `(0.5, 0.0, -10.0)` m/s (lateral + downward)
- **Friction**: μ = 0.6
- **Restitution**: e = 0.5
- **Timestep**: 16 ms cumulative
- **Steps**: 1000

### Observed Behavior

**Phase 1: First contact (steps 0–18)** — Direction reversal
```
Step 0:  vel_x goes from +0.5 to -3.36  ← WRONG: reversed direction
         vel_y ≈ 0 (correct)
Steps 1-18: vel_x remains negative, slowly decelerating
```

**Phase 2: First settling (steps 19–50)** — KE injection during bouncing
```
Step 19: KE spikes from 43.0 to 44.0 (should only decrease)
Steps 26-33: KE ramps from 31.1 → 44.1 (sustained energy injection during bounce)
Step 50: KE drops to 0.001 J (nearly at rest)
```

**Phase 3: Near-rest instability (steps 51–70)** — Exponential energy explosion
```
Step 50: KE ≈ 0.001 J (box appears at rest)
Step 51: KE → 0.022 J  (22× increase)
Step 52: KE → 0.077 J
Step 53: KE → 0.175 J
  ...
Step 70: KE → 10.94 J  (10,000× increase from near-rest)
```

**Phase 4: Eventual damping (steps 70–100)** — KE eventually decreases to rest

### Expected Behavior

1. KE should monotonically decrease (within numerical tolerance) at every step
2. x-velocity should remain ≥ 0 (friction opposes motion, never reverses it)
3. Once at rest, the object should stay at rest

---

## Investigation Plan

### Phase 1: Reproduce and Characterize
- [ ] Add diagnostic logging to print per-step: position, velocity, angular velocity, KE, contact count, contact normals, applied impulses
- [ ] Determine if direction reversal happens during normal-only phase or friction phase
- [ ] Determine if step-51 instability coincides with contact detection toggling (contact appears/disappears)
- [ ] Compare behavior with rotation=0 (flat box) to isolate tilt contribution

### Phase 2: Isolate Energy Source
- [ ] Run with μ=0, e=0.5 (no friction, same restitution) — does direction reversal still occur?
- [ ] Run with μ=0.6, e=0 (friction, no bounce) — does near-rest instability still occur?
- [ ] Run with tilt but no friction — does the tilted geometry alone cause direction reversal?
- [ ] Instrument `ConstraintSolver::solveActiveSet()` to log impulse magnitudes/directions per contact
- [ ] Track contact normal directions at each step — are they stable or oscillating?

### Phase 3: Root Cause Identification

Hypotheses to test (ordered by likelihood):

**H1: Contact normal oscillation for tilted geometry** (HIGH)
- A tilted box near a flat surface can have the GJK/EPA contact normal flip between face-face and edge-face configurations
- If the normal oscillates between steps, the impulse direction oscillates, potentially injecting energy
- Test: Log contact normals at steps 49-52 and check for discontinuities

**H2: Overlap-based contact triggers impulse at small penetration** (HIGH)
- At near-rest, the box oscillates around the contact surface with tiny penetration
- Each time penetration > 0, a full contact impulse is applied
- If the impulse overshoots (pushes box above surface), next step has no contact, box falls back, repeat with growing amplitude
- This is the classic "contact jitter" problem — requires position-level stabilization (Baumgarte or split impulse)
- Test: Log penetration depth at steps 49-52

**H3: Friction impulse direction wrong for non-axis-aligned contact** (MEDIUM)
- When the box is tilted, the contact tangent directions are not axis-aligned
- If the friction impulse is computed in the wrong frame or with incorrect tangent vectors, it can add lateral velocity instead of opposing it
- Test: Compare tangent vectors used by friction solver with actual sliding direction

**H4: Restitution amplifying small velocities** (MEDIUM)
- With e=0.5, a small approach velocity gets 50% returned
- Combined with gravity pulling the box down each step, the bounce height can grow if the effective restitution exceeds 1.0 for the tilted case
- Test: Run with e=0 and check if instability disappears

**H5: Cumulative timestep scheme interaction** (LOW)
- `world.update(std::chrono::milliseconds{16 * (i + 1)})` passes cumulative time
- If WorldModel computes dt from consecutive calls correctly, this is fine
- If there's an off-by-one or dt accumulation error, impulses could be scaled wrong
- Test: Verify dt is consistently 16ms in each update

### Phase 4: Fix and Validate
- [ ] Implement fix based on identified root cause
- [ ] All 3 criteria in `oblique_contact` test pass:
  - (1) KE monotonically non-increasing step-to-step (2% tolerance)
  - (2) Object at rest after full simulation window
  - (3) Zero lateral direction reversals
- [ ] No regressions in existing tests

---

## Diagnostic Test Suggestions

1. **Flat vs tilted comparison**: Same initial conditions but rotation=0 — isolate geometric contribution
2. **Friction-off tilted**: μ=0, same tilt — isolate normal contact behavior for tilted bodies
3. **No-restitution tilted**: e=0, same tilt — isolate restitution contribution to instability
4. **Contact normal logging**: Per-step contact normal directions to detect oscillation
5. **Penetration depth logging**: Per-step penetration to detect contact toggling

---

## Key Files for Investigation

| File | Relevance |
|------|-----------|
| `msd-sim/src/Environment/WorldModel.cpp` | Two-phase solve, contact detection, force application |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | ASM solver, ECOS integration, impulse extraction |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` | Contact normal/tangent computation |
| `msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.cpp` | ECOS formulation (already known buggy) |
| `msd-sim/test/Physics/FrictionValidationTest.cpp:257` | Reproducing test case |

---

## Success Criteria

1. Root cause identified with diagnostic evidence
2. `FrictionValidationTest::oblique_contact` passes all 3 criteria
3. No regressions in existing test suite
4. Fix is minimal and targeted

---

## References

- **Related debug ticket**: [DEBUG_0035d_friction_energy_injection](DEBUG_0035d_friction_energy_injection.md) — ECOS formulation bugs (flat geometry)
- **Parent ticket**: [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)
- **Physics integration failures**: [0035d8_physics_integration_test_failures](0035d8_physics_integration_test_failures.md)
- **Two-phase solve**: [0035d5_two_phase_friction_solve](0035d5_two_phase_friction_solve.md)
- **Contact constraint factory**: [0035c_friction_pipeline_integration](0035c_friction_pipeline_integration.md)

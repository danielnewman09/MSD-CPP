# Ticket 0055d: Friction Solver Energy Conservation

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Type**: Implementation
**Priority**: High
**Assignee**: cpp-implementer
**Created**: 2026-02-11
**Branch**: 0055c-friction-direction-fix
**Parent Ticket**: [0055_tilted_cube_friction_direction](0055_tilted_cube_friction_direction.md)
**Dependencies**: [0055c](0055c_friction_direction_fix.md)
**Prototype**: No
**Generate Tutorial**: No

---

## Objective

Refine the capped coupled solve from 0055c iteration 12 so that friction solver energy conservation is maintained for elastic and inelastic bounces while still preventing the 3.75× normal inflation identified in iteration 11.

The current capped coupled solve clamps ANY normal inflation > 1e-10, which is too aggressive — it removes legitimate normal impulse increases that the coupled QP correctly produces for elastic collisions with friction. This causes A3, A4, F2, F3 test regressions (energy loss in bounces).

---

## Problem Statement

When the friction cone QP (FrictionConeSolver) solves the coupled normal+friction system, the A-matrix off-diagonal terms can inflate the normal impulse λ_n beyond the friction-free value. This inflation is:

- **Legitimate** for elastic bounces: the coupled solver correctly accounts for friction drag and compensates with slightly higher normal impulse
- **Pathological** for tilted cube corner contacts: EPA produces a single off-center contact point, and the A-matrix coupling inflates λ_n by 3.75× (iteration 11 finding)

The challenge is distinguishing these two cases.

---

## Approach Options (try in order)

### Option 1: Multiplicative threshold clamping
Only clamp when `coupledN > k * cleanN` where `k` is a threshold (e.g., 1.5× or 2.0×). This allows moderate inflation for elastic bounces while catching the extreme 3.75× case.

**Pros**: Simple, predictable, low risk
**Cons**: Threshold is somewhat arbitrary; intermediate inflation ratios may still cause issues

### Option 2: Energy-based friction scaling
After the coupled solve, compute the kinetic energy change (ΔKE) that the total impulse would produce. If ΔKE > 0 (energy injection), scale down friction lambda until ΔKE ≤ 0.

**Pros**: Physically motivated — friction should never inject energy; no arbitrary threshold
**Cons**: More complex to implement; requires computing KE change from impulse

### Option 3: Multi-iteration sequential impulse
Implement proper sequential impulse with multiple iterations (standard Box2D/Bullet approach): solve normal → solve friction → repeat N times with warm-starting between iterations.

**Pros**: Industry-standard approach; well-understood convergence properties
**Cons**: Larger architectural change; the single-pass decoupled solve (iteration 12a) already showed over-dissipation, though multiple iterations may converge to a better solution

---

## Evidence from 0055c

Key findings from the iteration log:

- **Iteration 11**: Normal impulse 3.75× larger with friction at first contact (dvN=0.686 vs 0.183). 238 energy injection frames in thrown-cube scenario.
- **Iteration 12a (decoupled)**: 694/699. A4 regressed (24% energy loss from over-dissipation), D4 regressed (resting stability degraded).
- **Iteration 12b (capped coupled)**: 690/699. A3, A4, F2, F3 regressed (elastic bounce energy conservation lost from over-aggressive clamping). Visually improved behavior confirmed.

Bookmark: `git tag bookmark/0055c-iter12-visual-improvement` at commit `ee80766`.

---

## Testing Plan

### Primary Validation
1. A3_PerfectlyElastic_EnergyConserved — max height > 1.0 for e=1.0 bounce
2. A4_EqualMassElastic_VelocitySwap — KE conserved within 10% for elastic head-on collision
3. F2_ElasticBounce_KEConserved — post-bounce KE ratio > 0.35
4. F3_InelasticBounce_KEReducedByESquared — post-bounce KE ratio > 0.125

### Regression Testing
1. D4_MicroJitter_DampsOut — resting contact stability preserved
2. Full test suite: no regressions vs main branch baseline (689/693)
3. Tilted cube energy diagnostic: zero or near-zero energy injection frames

### Energy Injection Diagnostic
Use the `Diag_MechanicalEnergy_FrictionInjection` test from iteration 9 to verify that the fix eliminates energy injection in the thrown-cube scenario (PI/3 pitch, 5 m/s Vx).

---

## Acceptance Criteria

1. A3, A4, F2, F3 tests pass (elastic bounce energy conservation)
2. D4 passes (resting contact stability)
3. No regressions vs main branch baseline
4. Energy injection frames reduced significantly vs 238 baseline
5. Tilted cube visual behavior remains improved (no regression from iteration 12 bookmark)

---

## Workflow Log

### Ready for Implementation Phase
- **Started**: 2026-02-11 13:40
- **Branch**: 0055c-friction-direction-fix
- **PR**: #41 (draft)
- **Notes**: Ticket created to refine the capped coupled solve from 0055c iteration 12. Current state bookmarked at git tag `bookmark/0055c-iter12-visual-improvement`. Will maintain separate iteration log at `docs/designs/0055d_friction_solver_energy_conservation/iteration-log.md`.

### Implementation Phase (In Progress)
- **Completed**: 2026-02-11 15:00
- **Iterations**: 4
- **Artifacts**:
  - `docs/designs/0055d_friction_solver_energy_conservation/iteration-log.md`
- **Status**: **BLOCKED** — Option 1 (multiplicative threshold) exhausted after 4 iterations
- **Findings**:
  1. **A4 fixed at all thresholds ≤ 2.0×** — Equal-mass elastic collision energy conservation works
  2. **A3, F2, F3 fail even with NO clamping for elastic contacts (e >= 0.8)** — This means the coupled solver itself produces incorrect results for these tests, not the clamping
  3. **Test oscillation**: Sliding_PurePitch and Compound_NoSpuriousYaw swap success/failure across threshold values (1.5-2.0× range)
  4. **D4, H3, B2, B5 unaffected** — Resting stability and rotational tests unchanged by threshold tuning
- **Recommendation**: **Human decision needed**. The multiplicative threshold approach cannot fix A3, F2, F3 because the problem lies in the FrictionConeSolver itself, not the post-solve clamping. Next steps:
  1. Investigate WHY the coupled solver fails A3, F2, F3 (possibly ECOS solver issues?)
  2. Try Option 2 (energy-based scaling) — more complex but addresses root cause
  3. Try Option 3 (multi-iteration sequential impulse) — architectural change
  4. Accept current state (A4 fixed, 691/699) and address A3/F2/F3 in a separate ticket

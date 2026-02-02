# Ticket 0035d9: Two-Phase Friction Solve Pipeline Fix

## Status
- [x] Draft
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Assignee**: N/A
**Created**: 2026-02-01
**Type**: Bug Fix / Investigation
**Requires Math Design**: No
**Generate Tutorial**: No
**Parent Ticket**: [0035d_friction_hardening_and_validation](0035d_friction_hardening_and_validation.md)
**Debug Ticket**: [DEBUG_0035d_friction_energy_injection](DEBUG_0035d_friction_energy_injection.md)
**Predecessor**: [0035d8_physics_integration_test_failures](0035d8_physics_integration_test_failures.md) — Verification that identified this ticket's scope

---

## Summary

The two-phase friction solve pipeline (implemented in 0035d5) combines ASM normal forces with ECOS friction forces, but the integration produces incorrect physics. The ECOS SOCP formulation is mathematically correct in isolation (ECOSSolveTest suite passes 12/12), but the 4 physics integration tests remain failing after all upstream fixes (0035d1, 0035d2, 0035d7). This ticket investigates and fixes the WorldModel-level integration bugs.

---

## Problem Statement

### Symptoms (from 0035d8 verification)

| # | Test | Symptom | Expected |
|---|------|---------|----------|
| 1 | MultipleSimultaneousContacts | Box penetrates wall (x = -35.04) | x >= -0.5 |
| 2 | EnergyMonotonicDecrease | Spurious vertical velocity (vz = -5.02 m/s) | vz ≈ 0 |
| 3 | KineticFrictionInclinedPlane | Friction barely slows motion (0.8% reduction) | >= 10% reduction |
| 4 | StickSlipTransition | Excessive oscillation (42 vs < 20 threshold) | < 20 oscillations |

### Pipeline Architecture (0035d5)

```
Phase 2: Create constraints
  normalConstraints:  [CC₀, CC₁, CC₂, CC₃]       ← N normals per collision pair
  frictionPairs:      [CentroidCC, CentroidFC]     ← 1 centroid normal + 1 centroid friction per pair

Phase 4a: ASM solves normalConstraints → normalResult.bodyForces
Phase 4b: ECOS solves frictionPairs → frictionResult.lambdas
          extractFrictionOnlyBodyForces() zeros CentroidCC lambdas → frictionBodyForces

Phase 5: Apply combined forces
  totalForce = normalResult.bodyForces[k] + frictionBodyForces[k]
```

### Key Evidence

- **ECOS in isolation**: ECOSSolveTest suite passes (12/12) — SOCP formulation correct
- **ASM in isolation**: Normal-only tests pass (friction off, μ=0) — ASM path correct
- **Combined**: All 4 physics tests fail — integration layer is the problem

---

## Root Cause Hypotheses

### H1: ECOS Friction Solve Receives Incorrect Input (HIGHEST PRIORITY)

The ECOS solver in Phase 4b receives `frictionPairs` containing 1 CentroidCC + 1 CentroidFC per pair. The `solveWithContacts()` method:
1. Assembles the effective mass matrix `A = J·M⁻¹·Jᵀ` (3×3 for 1 contact)
2. Assembles the RHS `b` with restitution/Baumgarte terms
3. Dispatches to `solveWithECOS(A, b, coneSpec, numContacts)`

**Potential issue**: The centroid normal in `frictionPairs` is a **different constraint** from the per-point normals in `normalConstraints`. The centroid normal uses the averaged contact point (centroid of manifold), while per-point normals use individual witness points. This means:
- The centroid normal Jacobian differs from any individual per-point normal Jacobian
- The effective mass `A` is computed for the centroid geometry, not the per-point geometry
- The RHS `b` includes Baumgarte/restitution terms for the centroid penetration, not individual penetrations

If the centroid penetration depth or contact normal differs significantly from the per-point values, the ECOS solve could produce incorrect friction bounds (μ·λ_n_centroid ≠ μ·Σλ_n_points).

**Verification**: Log centroid vs per-point normals, penetration depths, and effective mass matrices.

### H2: extractFrictionOnlyBodyForces() Filtering Bug

`extractFrictionOnlyBodyForces()` (ConstraintSolver.cpp:737-771) clones the lambda vector and zeros ContactConstraint entries before calling `extractContactBodyForces()`. But `extractContactBodyForces()` (line 699) also has filtering logic:

```cpp
if (dim_i == 1) {
  if (lambda_i(0) <= 0.0) { continue; }  // Skip separating normals
}
```

After zeroing the centroid normal lambda to 0.0, this check `lambda_i(0) <= 0.0` evaluates to **true** (0.0 <= 0.0), causing the loop to `continue` and skip this constraint entirely. This is the intended behavior for the centroid normal.

**However**: The friction constraint (dim=2) is NOT zeroed. Its lambda values pass through the `squaredNorm < 1e-30` check. If the ECOS solver returns very small but non-zero friction lambdas, they should still be applied. **This path appears correct.**

**Verification**: Log the lambda vector before and after zeroing, confirm friction entries are preserved.

### H3: Force Combination Double-Counting or Sign Error

WorldModel Phase 5 adds ASM and ECOS forces:
```cpp
totalLinear += normalResult.bodyForces[k].linearForce;
totalLinear += frictionBodyForces[k].linearForce;
```

**Potential issue**: The ECOS solve operates on the centroid constraints independently. The ECOS solver builds its own `A = J·M⁻¹·Jᵀ` matrix. If the centroid Jacobian includes a normal component (it does — CentroidCC is in frictionPairs), the ECOS solution's friction lambdas are influenced by the centroid normal. After zeroing the centroid normal lambda, the remaining friction lambda was solved in the context of that normal. If the centroid normal lambda differed from the ASM normal forces, the friction is bounded by the wrong μ·λ_n.

**Key insight**: The ECOS friction cone constraint is `||λ_t|| ≤ μ·λ_n_centroid`. But the actual normal force applied is the ASM result (sum of per-point normals). If `λ_n_centroid ≠ Σλ_n_points`, then friction is bounded by the wrong value.

**Verification**: Compare centroid λ_n from ECOS with sum of per-point λ_n from ASM.

### H4: Constraint Ordering Mismatch

`buildFrictionConeSpec()` maps FrictionConstraint instances to their paired ContactConstraint via position in the constraint list. If the ordering is [CentroidCC, CentroidFC] per pair, the cone spec correctly maps `normalIdx=0` for each pair.

**Potential issue**: For multiple collision pairs, the interleaving must be exact:
```
frictionPairs = [CentroidCC₀, CentroidFC₀, CentroidCC₁, CentroidFC₁, ...]
```

If the ordering is wrong, friction cones reference the wrong normal lambda.

**Verification**: Log constraint list ordering and cone spec mapping for multi-contact scenarios.

---

## Technical Approach

### Phase 1: Diagnostic Instrumentation

Add targeted logging (spdlog::debug) to the two-phase pipeline to trace intermediate values for a single-contact scenario (1 box on 1 floor):

**File: WorldModel.cpp**
1. After Phase 2: Log constraint counts, centroid vs per-point normals/penetrations
2. After Phase 4a: Log ASM normal lambdas and resulting body forces
3. After Phase 4b: Log ECOS full lambda vector, filtered lambda vector, friction body forces
4. Phase 5: Log combined forces per body

**File: ConstraintSolver.cpp**
1. In `extractFrictionOnlyBodyForces()`: Log original and modified lambda vectors
2. In `solveWithContacts()`: Log effective mass matrix A, RHS b, and solution lambda

### Phase 2: Minimal Reproduction Test

Create a diagnostic test (not for merge — temporary):
- Single 10kg box at z=0.49 on floor at z=-5.0
- Initial velocity: (10, 0, 0) m/s
- μ=0.5, e=0.0
- Run 1 timestep
- Log ALL intermediate values from Phase 1 instrumentation
- Compare against analytical expectations:
  - Normal impulse: cancels penetration velocity + Baumgarte correction
  - Friction impulse: opposes tangential velocity, bounded by μ·λ_n
  - Net force: friction decelerates x, normal supports z, no y component

### Phase 3: Fix

Based on diagnostic findings, implement the targeted fix. Most likely candidates:

**If H1 (incorrect ECOS input)**:
- Option A: Pass the ASM-computed normal lambda to the ECOS friction solve via `setNormalLambda()` on FrictionConstraint, so ECOS uses the correct friction bound
- Option B: Reformulate the two-phase pipeline so ECOS receives pre-computed normal forces as fixed input rather than solving its own centroid normal

**If H3 (force combination mismatch)**:
- Ensure ECOS friction is bounded by the ASM normal force magnitude, not the centroid normal

**If H4 (ordering)**:
- Add assertions or deterministic sorting to guarantee constraint ordering

### Phase 4: Validate

Run all 4 target tests plus full regression suite (538 tests).

---

## Files

### Production Files (Investigation Targets)

| File | Lines | Relevance |
|------|-------|-----------|
| `msd-sim/src/Environment/WorldModel.cpp` | 257-419 | Two-phase constraint creation, solving, force combination |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | 253-341, 668-771 | `solveWithContacts()` dispatch, `extractContactBodyForces()`, `extractFrictionOnlyBodyForces()` |
| `msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.cpp` | 233-378 | SOCP problem construction with Cholesky-lifted formulation |
| `msd-sim/src/Physics/Constraints/ContactConstraintFactory.cpp` | — | Centroid constraint creation, contact ordering |

### Test Files (Failing — Acceptance Criteria)

| File | Test | Status |
|------|------|--------|
| `msd-sim/test/Environment/WorldModelContactIntegrationTest.cpp:347` | MultipleSimultaneousContacts | FAIL |
| `msd-sim/test/Physics/FrictionEnergyTest.cpp` | EnergyMonotonicDecrease | FAIL |
| `msd-sim/test/Physics/FrictionValidationTest.cpp:152` | KineticFrictionInclinedPlane | FAIL |
| `msd-sim/test/Physics/FrictionStabilityTest.cpp:270` | StickSlipTransition | FAIL |

---

## Acceptance Criteria

- [ ] **AC1**: `WorldModelContactIntegrationTest.MultipleSimultaneousContacts_ResolvedCorrectly` passes
- [ ] **AC2**: `FrictionEnergyTest.EnergyMonotonicDecreaseForSliding` passes — no spurious vertical velocity
- [ ] **AC3**: `FrictionValidationTest.KineticFrictionInclinedPlane_SlowsMotion` passes — friction reduces speed by >= 10%
- [ ] **AC4**: `FrictionStabilityTest.StickSlipTransitionSmoothness` passes — oscillation count < 20
- [ ] **AC5**: All 538 existing tests pass (zero regressions)
- [ ] **AC6**: Diagnostic logging removed or gated behind spdlog::debug before merge

---

## Dependencies

- **Requires**: [0035d1](0035d1_ecos_socp_reformulation.md) — SOCP reformulation (complete)
- **Requires**: [0035d2](0035d2_friction_lambda_filter_fix.md) — Lambda filter fix (complete)
- **Requires**: [0035d5](0035d5_two_phase_friction_solve.md) — Two-phase pipeline (complete, under investigation)
- **Blocks**: [0035d8](0035d8_physics_integration_test_failures.md) — Physics integration test verification
- **Blocks**: [0035d](0035d_friction_hardening_and_validation.md) — Parent ticket acceptance criteria

---

## Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Multiple interacting bugs in pipeline | Medium | High | Systematic instrumentation to isolate each stage |
| Fix requires architectural change to two-phase approach | Medium | High | H1 fix (pass ASM λ_n to ECOS) is contained; full rearchitecture unlikely |
| StickSlipTransition needs additional work beyond pipeline fix | Medium | Medium | May need separate follow-on if stick-slip threshold logic is required |
| Diagnostic logging introduces performance regression | Low | Low | Remove/gate behind spdlog::debug before merge |

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-01
- **Notes**: Follow-on from 0035d8 verification which confirmed all 4 physics integration tests remain failing despite upstream ECOS solver fixes (0035d1, 0035d2, 0035d7). ECOS solver tests pass in isolation, narrowing the root cause to the two-phase solve pipeline integration layer in WorldModel and ConstraintSolver. Four hypotheses identified, prioritized by likelihood. Technical approach uses diagnostic instrumentation → minimal reproduction → targeted fix → validation.

### Ready for Implementation Phase
- **Started**: 2026-02-01
- **Notes**: Bug fix/investigation ticket advanced directly to implementation (skips math design, architectural design, and prototype phases). All dependencies complete (0035d1, 0035d2, 0035d5). Implementer will follow 4-phase approach: (1) Add diagnostic logging to WorldModel and ConstraintSolver, (2) Create minimal reproduction test, (3) Identify and fix root cause based on hypotheses H1-H4, (4) Validate with 4 failing physics tests plus full regression suite.

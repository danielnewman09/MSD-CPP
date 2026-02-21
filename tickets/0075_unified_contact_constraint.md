# Ticket 0075: Unified Contact Constraint (Block PGS)

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Design Complete — Awaiting Review
**Type**: Investigation / Architecture
**Priority**: High
**Created**: 2026-02-21
**Generate Tutorial**: No
**Parent Ticket**: None
**Related Tickets**: [0032](0032_contact_constraint_refactor.md) (contact constraint), [0035a](0035a_tangent_basis_and_friction_constraint.md) (friction constraint), [0043](0043_constraint_hierarchy_refactor.md) (constraint hierarchy), [0067](0067_contact_phase_energy_injection.md) (energy injection), [0069](0069_friction_velocity_reversal.md) (sliding mode), [0070](0070_nlopt_convergence_energy_injection.md) (decoupled solve), [0073](0073_hybrid_pgs_large_islands.md) (hybrid PGS)

---

## Summary

Merge the separate `ContactConstraint` (dimension=1, normal) and `FrictionConstraint` (dimension=2, tangent) into a single unified `ContactConstraint` with dimension=3. Replace the decoupled normal-then-friction solve with a Block Projected Gauss-Seidel (Block PGS) algorithm that solves the full 3x3 effective mass matrix per contact, naturally coupling normal and friction impulses.

---

## Problem

### Unstable Sliding Dynamics

The current decoupled solve architecture (ticket 0070) computes normal impulses blind to friction's mechanical effect. During sliding, friction produces torques via lever arms that press or lift the contact point. Because the normal solve happens first and independently, the resulting normal impulse is correct for the pre-friction state but insufficient for the post-friction state.

This manifests as:
1. **Insufficient normal impulse during sliding**: The normal solve doesn't anticipate friction-induced torque, producing lower lambda_n than physically required
2. **Tight friction bounds**: Since friction is bounded by mu * lambda_n, an underestimated lambda_n restricts friction, reducing deceleration
3. **Warm-start lag at sliding onset**: Transitioning from ballistic flight to sliding contact starts from lambda=0, taking several frames to ramp up to the correct coupled equilibrium
4. **Restitution threshold oscillation**: Micro-bouncing from insufficient normal force causes intermittent restitution suppression, creating inconsistent RHS values frame-to-frame

### Root Cause: Missing Cross-Coupling Terms

In the decoupled approach, the solver treats the effective mass as block-diagonal — it solves K_nn for normal, then K_tt for tangent independently. The off-diagonal terms K_nt and K_tn (which represent how friction impulses change normal velocity and vice versa) are ignored. These coupling terms are physically significant when:
- Lever arms are large relative to body size
- The body is rotating (tumbling contact)
- Friction is near or at the Coulomb cone surface (sliding)

### Architectural Overhead

The current two-class approach also imposes structural costs:
- Interleaved storage pattern `[CC, FC, CC, FC, ...]` with stride logic throughout `CollisionPipeline`
- `dynamic_cast<FrictionConstraint*>` in the solver dispatch loop
- Cross-wiring of lambda_n from `ContactConstraint` to `FrictionConstraint` via `setNormalLambda()`
- Duplicate contact geometry (normal, lever arms) stored in both constraint objects
- Coordinating sliding mode (ticket 0069) across two separate objects

---

## Proposed Solution

### Phase 1: Unified ContactConstraint (Data Structure)

Merge `FrictionConstraint` into `ContactConstraint`:

```
ContactConstraint (dimension = 1 or 3)
  Row 0: Normal direction       bounds: [0, +inf)      Baumgarte: alpha=ERP, beta=0
  Row 1: Tangent U direction    bounds: Coulomb cone    Baumgarte: alpha=0, beta=0
  Row 2: Tangent V direction    bounds: Coulomb cone    Baumgarte: alpha=0, beta=0
```

- `dimension()` returns 1 (frictionless, mu=0) or 3 (with friction)
- Owns complete contact frame: normal + tangent basis (computed via TangentBasis)
- Single set of lever arms (no duplication)
- Sliding mode (ticket 0069 logic) adjusts tangent basis internally
- `ContactCache` stores `Vec3 accumulatedImpulse` per contact point

### Phase 2: Block PGS Solver

Replace the decoupled normal-then-friction solve with a per-contact 3x3 block solve:

1. **Build effective mass**: K = J M^{-1} J^T (3x3 symmetric positive-definite per contact)
2. **Add CFM regularization**: K_diag += epsilon (prevents singularity at extreme mass ratios)
3. **Iteration loop** (per sweep, per contact):
   a. Compute velocity error: v_err = J * v_current + bias
   b. Solve unconstrained: delta_lambda = K^{-1} * (-v_err)
   c. Accumulate: lambda_temp = lambda_acc + delta_lambda
   d. **Project onto Coulomb cone**:
      - If lambda_n < 0: set lambda = (0, 0, 0) — separating contact
      - If ||lambda_t|| > mu * lambda_n: scale tangent to cone surface
   e. Compute actual delta: delta = lambda_projected - lambda_acc
   f. Apply velocity change: v += M^{-1} * J^T * delta
   g. Update accumulator: lambda_acc = lambda_projected

### Why This Works Without Heuristics

The off-diagonal terms K_nt in the 3x3 matrix capture the lever-arm coupling between friction and normal directions. When the block solve computes lambda_n, it inherently accounts for the effect that lambda_t will have on normal velocity — and vice versa. No artificial "friction anticipation bias" or "gravity floor" is needed. The math produces the correct coupled result.

### Phase 3 (if needed): Hold-and-Resolve Projection

If simple cone projection leaves residual penetration when friction is saturated (sliding at cone surface), implement a refined projection:
1. Detect friction binding (||lambda_t|| clamped to mu * lambda_n)
2. Treat clamped tangent impulses as known constants
3. Re-solve normal row: lambda_n = (1/K_nn) * (-v_bias_n - K_nt1 * lambda_t1 - K_nt2 * lambda_t2)
4. Ensures non-penetration is strictly satisfied even when friction is at the cone limit

---

## Investigation Questions

### I1: Quantify the Coupling Effect

Before implementation, measure the magnitude of the K_nt terms in existing recordings to confirm they are significant enough to explain the observed instability.

**Method**: For each contact point in a sliding recording, compute the full 3x3 K matrix and compare:
- K_nt / K_nn ratio (coupling strength relative to normal stiffness)
- Predicted normal impulse from coupled 3x3 vs decoupled scalar solve
- Correlation between K_nt magnitude and observed instability

### I2: Energy Conservation Verification

The previous coupled solve (NLopt QP, ticket 0070) caused energy injection. Verify that Block PGS with cone projection is energy-safe.

**Method**: Analytical argument + prototype validation
- The projection onto the Coulomb cone only reduces impulse magnitude — it never increases it
- Normal clamping (lambda_n >= 0) prevents attractive forces
- Combined: the projected impulse is always "inside" the unconstrained solution, so KE change is bounded

### I3: Solver Convergence Comparison

Compare convergence characteristics of Block PGS vs current decoupled approach.

**Metrics**:
- Iterations to converge (velocity residual < tolerance)
- Warm-start effectiveness (cold vs warm iteration count)
- Behavior at extreme mass ratios (heavy-on-light stacking)

### I4: Sliding Stability Test Cases

Define concrete test scenarios to validate the fix:

| Test | Setup | Success Criterion |
|------|-------|-------------------|
| Ramp slide | Box on 30-degree slope, mu=0.3 | Constant slide speed, lambda_n = mg*cos(30)*dt ± 5% |
| Flight-to-slide | Box dropped onto moving surface | Smooth transition, no bounce at contact onset |
| Tumbling slide | Tilted cube on floor (F4 variant) | No energy injection during contact episodes |
| Stack stability | 4-box stack at rest | No drift, warm-start converges in < 5 iterations |
| Edge slide | Cube sliding on edge contact | Stable 2-point manifold, consistent friction |

---

## Scope

### In Scope
- Merge `FrictionConstraint` into `ContactConstraint` (dim=3)
- Block PGS solver for per-contact coupled solve
- Update `CollisionPipeline` constraint creation (eliminate interleaving)
- Update `ContactCache` for 3-component impulse storage
- Update `ConstraintSolver` dispatch (eliminate `hasFriction` branching)
- Update `ProjectedGaussSeidel` for block iteration
- Update recording/transfer records for unified constraint
- Sliding mode integration (ticket 0069 logic moves into unified constraint)

### Out of Scope
- ASM solver changes (small-island path — can remain decoupled initially, or convert separately)
- New contact detection or manifold generation
- Broad-phase acceleration
- Contact sleeping/deactivation

---

## Risks

### R1: Energy Injection from Coupling
**Risk**: The previous coupled solver (NLopt) injected energy. Could Block PGS do the same?
**Mitigation**: Block PGS with cone projection is fundamentally different from the NLopt QP approach. The projection only reduces impulse magnitude. Validate with energy tracking in prototype.

### R2: Singularity at Extreme Mass Ratios
**Risk**: The 3x3 K matrix may become ill-conditioned with large mass ratios or long lever arms.
**Mitigation**: Add CFM (Constraint Force Mixing) regularization to K diagonal. Implement fallback to decoupled scalar solve if determinant is near zero.

### R3: Warm-Start Basis Rotation
**Risk**: When sliding mode rotates the tangent basis frame-to-frame, cached impulse vectors may be misaligned, causing a directional "kick."
**Mitigation**: Project cached impulse onto new basis vectors before applying warm-start. If normal similarity check fails (>15 degree rotation), invalidate cache entry.

### R4: Regression in Resting Contact Stability
**Risk**: The current decoupled solve is well-validated for resting contacts. Coupling could regress stacking behavior.
**Mitigation**: Run full test suite including StackCollapse benchmarks. Keep A/B toggle (`use_block_solver` flag) during development.

---

## Success Criteria

1. **Sliding stability**: Ramp-slide test produces constant normal impulse within 5% of analytical value (mg*cos(theta)*dt)
2. **Energy conservation**: No energy injection during sustained sliding contact (F4 tumbling test, episodes identified in ticket 0067)
3. **Resting contact**: No regression in stack stability tests (StackCollapse/4, StackCollapse/16)
4. **Convergence**: Block PGS converges in equal or fewer iterations than current approach with warm-starting
5. **Performance**: No regression in solver throughput for ClusterDrop/32 benchmark
6. **Code simplification**: Elimination of `FrictionConstraint` class, interleaving stride logic, and `dynamic_cast` dispatch

---

## References

- Erin Catto, "Sequential Impulses" (GDC 2006) — Block PGS formulation
- Tonge (2012), "Mass Splitting for Jitter-Free Parallel Rigid Body Simulation"
- Bullet Physics, btSequentialImpulseConstraintSolver — coupled normal+friction block solve
- Box2D, b2ContactSolver — 2-constraint block solve with manifold projection
- Jolt Physics, ContactConstraintManager — modern coupled contact solver

---

## Workflow Log

### Design Phase
- **Started**: 2026-02-21 00:00
- **Completed**: 2026-02-21 00:00
- **Branch**: 0075-unified-contact-constraint
- **PR**: N/A (draft PR to be created)
- **Artifacts**:
  - `docs/designs/0075_unified_contact_constraint/design.md`
  - `docs/designs/0075_unified_contact_constraint/0075_unified_contact_constraint.puml`
- **Notes**: Initial architecture design completed. Key design decisions documented:
  1. Unified ContactConstraint absorbs FrictionConstraint — single class with dimension=1 (mu=0) or dimension=3 (mu>0)
  2. BlockPGSSolver introduced — per-contact 3x3 block PGS with Coulomb cone projection
  3. NLopt retained behind flag during transition period; removed after Block PGS validation
  4. ContactCache updated to store Vec3 impulses (was flat scalar vector)
  5. UnifiedContactConstraintRecord replaces separate ContactConstraintRecord + FrictionConstraintRecord
  6. ASM path for frictionless contacts left unchanged to minimize regression risk
  7. Three open design questions require human decision before implementation: transfer record migration strategy, NLopt removal timing, and ASM threshold units

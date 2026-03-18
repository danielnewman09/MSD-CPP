# Pre-Implementation Review: 0075 Unified Contact Constraint

> **Date**: 2026-02-22
> **Reviewers**: Claude (traceability-assisted review), Gemini Pro (independent cross-review)
> **Status**: Review complete — findings incorporated into design revision
> **Purpose**: Document regression risks and behavioral guarantees before committing to Block PGS unification
>
> **Resolution (2026-02-22)**: The critical finding from Section 7.4 (Gemini) — that Block PGS with `(1+e)` in the coupled RHS can reproduce the DD-0070-H2 energy injection mechanism — has been addressed. The design (`design.md`) has been revised to implement **Gemini Recommendation 1**: separate restitution from the PGS loop via a two-phase approach (Phase A: normal-only restitution pre-solve, Phase B: purely dissipative Block PGS). This eliminates the regression risk by ensuring `(1+e)` never enters the coupled 3x3 system.

---

## 1. Pipeline Evolution Context

The collision pipeline has gone through three major architectural eras across ~55 tickets:

1. **Impulse-based** (0027-0029): Standalone `CollisionResponse` with ad-hoc impulse math
2. **Constraint-based with decoupled solve** (0031-0073): Lagrange multipliers, ASM/PGS, separate normal and friction constraints
3. **Proposed: Unified Block PGS** (0075): Single `ContactConstraint` with dim=3, coupled solve

Each transition was motivated by concrete failure modes discovered in the previous architecture.

---

## 2. Hard-Won Behavioral Guarantees at Risk

### 2.1 Energy Injection Prevention (0039, 0040b, 0067, 0070) -- HIGHEST RISK

Three separate energy injection mechanisms were identified and fixed over the pipeline's lifetime:

#### 2.1.1 Baumgarte ERP Velocity Amplification (0039e, 0040b)

- **Root cause** (DD-0040b-H2): Unclamped `(ERP/dt) * penetration` injected correction velocity into real kinetic energy.
- **Fix progression**: 5.0 m/s clamp (0039e, DD-0040b-H1 found this insufficient), then full replacement with **split impulse position correction** (0040b) decoupling position and velocity correction entirely.
- **0075 impact**: PositionCorrector is unchanged, but Block PGS's velocity residual workspace (`vRes_`) must remain completely separate from position correction. Any leakage would reintroduce the Baumgarte problem.

#### 2.1.2 FrictionConeSolver Convergence Failure (0067)

- **Root cause** (DD-0067-H2): Custom projected Newton solver failed to converge when friction was saturated at the cone boundary (rank-deficient projection Jacobian). Non-converged lambda did net positive work at 0.03-0.07 J/frame.
- **Fix**: Replaced with NLopt SLSQP solver (0068).
- **0075 impact**: Block PGS uses a fundamentally different iteration (Gauss-Seidel sweeps with cone projection) that should not exhibit the same rank-deficiency issue. However, the cone projection step itself is new code that must be validated for the saturated-friction case.

#### 2.1.3 Coupled QP Objective vs Physics Mismatch (0070) -- CRITICAL

This is the design decision most directly in tension with 0075. The traced decisions state:

- **DD-0070-H2** (Why coupled solve injects energy): "When the friction cone constraint `||lambda_t|| <= mu*lambda_n` is active AND A has off-diagonal normal-tangent coupling (from rotational lever arms), the cone pushes lambda away from the unconstrained optimum. The mismatch between what the QP optimizes (velocity error with `(1+e)` factor) and what physics requires (energy conservation against `Jv`) means the cone-constrained solution can inject energy."

- **DD-0070-H4** (Why decoupled is energy-safe): "Normal solve: energy-correct by the math shown above. Friction solve: `b_t = -Jv_t` with no restitution factor. The unconstrained friction optimum zeros the tangent velocity (maximum dissipation). The ball projection can only reduce the friction magnitude -> less dissipation, never injection. No cross-coupling: the `(1+e)` restitution factor lives entirely in the normal solve, isolated from the friction cone constraint."

**The 0075 design proposes to re-couple these solves.** The key difference from the original coupled approach:
- 0070's coupled solve: Full system QP with `(1+e)` restitution in the combined RHS
- 0075's Block PGS: Per-contact 3x3 block with accumulated projection

The argument is that Block PGS avoids the problem because each contact is solved independently with its own cone projection, rather than a global QP. This is the approach used by Box2D, Bullet, and Jolt. However, this project's specific combination of split impulse position correction, EPA-generated contact manifolds, and sliding friction mode heuristics has not been tested with a Block PGS formulation.

**Key question**: In Block PGS step 2 (`delta_lambda = K_inv * (-v_err - b_portion)`), the normal row's `(1+e)` restitution term is present in `b_portion`, and `K_inv` couples all three rows through the off-diagonal K_nt terms. Does the per-contact cone projection after this step prevent the energy injection mechanism identified in DD-0070-H2?

### 2.2 Friction Velocity Reversal (0069) -- MODERATE RISK

- **Root cause** (DD-0069-H1): Friction produced impulses reversing sliding direction. Post-solve clamping was insufficient because pre-solve velocity was already corrupted by accumulated reversals from prior frames.
- **Fix**: Sliding friction mode -- when `ContactCache` detects sustained sliding, tangent basis aligns with sliding direction and bounds become unilateral.
- **0075 impact**: Design claims Block PGS "naturally handles" this via cone projection. Open Question #2 in the design asks whether `setSlidingMode()` is still needed. **This is not settled.** The claim needs prototype confirmation, especially for edge contacts where the tangent basis can rotate frame-to-frame.

### 2.3 Per-Contact Penetration Depth (0040a) -- LOW RISK

- Previously all contacts shared a single `penetrationDepth`, causing N-times overcorrection with multi-point manifolds.
- 0075 preserves individual depth in the unified constructor. Should be safe.

### 2.4 Split Impulse Position Correction (0040b) -- MODERATE RISK

- Baumgarte stabilization was replaced with split impulse precisely because mixing position correction into the velocity solve injected energy.
- 0075 doesn't change `PositionCorrector`, but Block PGS's `vRes_` workspace must be verified as completely separate. Any leakage of position correction into the Block PGS RHS reintroduces the Baumgarte problem.

### 2.5 Warm-Start Cache Invalidation (0040d) -- MODERATE RISK

- Contact normal rotation >15 degrees invalidates the cache.
- 0075 changes cache format from `vector<double>` to `vector<Eigen::Vector3d>`. Invalidation logic preserved, but warm-start projection onto new contact frames (R3 in design risk table) is new code. Basis rotation of cached tangent impulses when contact normal shifts between frames could introduce subtle bugs.

### 2.6 Edge Contact Manifold (0040c) -- LOW RISK

- Edge-edge contacts produce 2-point manifolds. This is upstream of the solver (manifold generation), unaffected by 0075.

### 2.7 ASM Exactness for Small Systems (0034) -- LOW RISK

- Design explicitly preserves ASM for frictionless contacts (Open Question #3, Option A). Correct decision.

---

## 3. Specific Regression Concerns

### 3.1 Re-coupling Normal and Friction Solves

The decoupled solve (0070) was adopted **specifically** because the coupled solve injected energy when the cone constraint was active. The fundamental question is whether Block PGS's per-contact structure avoids the global QP's energy injection mechanism. This is plausible (industry standard engines do this) but unproven in this codebase's specific context.

### 3.2 Restitution Leakage Through K_nt Coupling

DD-0070-H4 proved energy safety requires `(1+e)` isolation from the friction cone constraint. In Block PGS, the 3x3 `K_inv` matrix couples normal and tangent rows. If the normal row's `(1+e)` term propagates through K_nt into the tangent solution, and the cone projection then shifts the result, the same energy injection mechanism from DD-0070-H2 could reappear at the per-contact level.

### 3.3 Sliding Mode Necessity Under Block PGS

The 0069 sliding friction mode was a targeted fix for a real physical violation. Removing it (or assuming Block PGS makes it unnecessary) without prototype evidence is risky. Recommend keeping `setSlidingMode()` as a safety net during validation, with a metric to measure whether it ever activates under Block PGS.

---

## 4. Recommended Prototype Validation Plan

These tests should be **hard gates** before proceeding from Phase 1 to Phase 2:

| # | Test | What It Validates | Pass Criteria | Source Ticket |
|---|------|-------------------|---------------|---------------|
| P1 | K_nt magnitude analysis (I1) | Is coupling significant enough to justify re-coupling? | K_nt/K_nn > 5% for sliding cases | Design I1 |
| P2 | 60-frame resting contact | No energy injection from Block PGS | Cumulative delta KE < 1e-6 J | 0039 regression |
| P3 | F4 tumbling slide reproduction | The exact failure mode from 0067/0070 | No energy injection > 0.01 J/frame | 0067, 0070 |
| P4 | Ramp slide steady state | Normal impulse accuracy with coupling | lambda_n within 5% of mg*cos(theta)*dt | Design test |
| P5 | Sliding direction preservation | Friction never reverses sliding direction | Post-solve tangent velocity same sign as pre-solve | 0069 regression |
| P6 | 4-body stack at rest | Resting contact stability | All body velocities < 0.01 m/s after 120 frames | Design test |
| P7 | Extreme mass ratio (1000:1) | CFM regularization sufficient | No NaN, correct collision response | 0034 guarantee |
| P8 | Restitution energy accounting | `(1+e)` factor doesn't inject energy through K_nt | For e=1: delta KE = 0; for e<1: delta KE < 0 | DD-0070-H1 |

### Additional Recommendations

1. **Runtime toggle, not compile-time**: Keep the decoupled solve path as a runtime toggle during prototype, not just `#ifdef`. This enables A/B comparison on the exact same simulation state.

2. **Preserve sliding mode as safety net**: Keep `setSlidingMode()` functional in the unified constraint. Add telemetry to measure activation frequency. Only remove it if Block PGS produces zero activations across all test scenarios.

3. **Energy tracking per-frame**: Add per-frame energy delta logging to the Block PGS path from day one. The 0067 energy injection was only 0.03-0.07 J/frame -- subtle enough to miss without instrumentation.

---

## 5. Existing Test Coverage to Preserve

These test guarantees were established through prior tickets and must not regress:

| Guarantee | Tolerance | Source |
|-----------|-----------|--------|
| Momentum conservation | 1e-6 | 0027, 0032a |
| EPA penetration depth accuracy | 1e-6 (unit cube) | 0027a |
| Witness point surface accuracy | 1e-6 | 0028 |
| Jacobian finite-difference match | 1e-5 | 0032a |
| Complementarity: lambda >= 0 | Exact | 0034 |
| Mass ratio robustness | Up to ~1e12 | 0034 |
| Restitution: e=1.0 velocity swap | Exact for equal mass | 0027, 0032 |
| Rest velocity threshold | 0.5 m/s | 0032a |
| No friction velocity reversal | Directional | 0069 |
| Per-contact penetration depth | Individual, no N-times overcorrection | 0040a |
| Edge contact torque generation | 2-point manifold | 0040c |
| Warm-start invalidation on normal rotation | >15 degrees | 0040d |

---

## 6. Conclusion (Claude)

The 0075 design is architecturally sound and aligns with industry-standard approaches (Box2D, Bullet, Jolt). The phased implementation order and risk mitigations are well-considered. However, the re-coupling of normal and friction solves directly reverses a decision (0070) that was made to fix a concrete energy injection bug. The prototype validation plan (especially P2, P3, and P8) must be treated as a hard gate before Phase 2 implementation proceeds.

---

## 7. Independent Cross-Review (Gemini Pro)

> The following assessment was produced by Gemini Pro as an independent review of this document and the 0075 design.

### 7.1 Risk Assessment: Completeness Gaps

The original risk analysis is **incomplete**. The following high-priority risks were not covered:

#### Solver Drift / Bias from Constraint Ordering
Block PGS (and all Gauss-Seidel variants) is sensitive to constraint processing order. If constraints are processed in a fixed order every frame, objects will tend to drift in that direction. **Mitigation**: Requires random constraint permutation (shuffling) every frame or alternating sweep direction.

#### Tangent Basis Instability Near Zero Sliding Velocity
In 3D, constructing the tangent plane for friction rows (t1, t2) is non-trivial when relative tangential velocity is near zero (static friction). The basis vectors can spin rapidly between frames, preventing the solver from effectively "locking" the contact, leading to jitter or creep on slopes.

#### Effective Mass Singularity
The 3x3 K matrix can be singular or near-singular (degenerate geometry or infinite mass bodies). NLopt likely handled rank-deficient cases internally; the Block PGS implementation must handle this explicitly. The design's CFM regularization (kCFMEpsilon = 1e-8) addresses this partially but should be validated.

### 7.2 Prototype Validation Gaps

The proposed validation plan focuses on steady-state behaviors (sliding/resting) and neglects **high-energy transient behaviors** where coupled injection occurs:

| Missing Test | Description | Why It Matters |
|-------------|-------------|----------------|
| **Oblique Drop** (high K_nt impact) | High rotational inertia body with offset CoM hitting surface at shallow angle with high friction | Maximizes K_nt terms; rebound KE must not exceed pre-impact KE |
| **Iteration Count Sensitivity** | Validate behavior at low (4), medium (10), and high (30) PGS iterations | Energy behavior may change dramatically at low iteration counts |
| **3x3 Block Condition Number** | Measure condition number of K matrices, not just K_nt/K_nn ratio | High condition number -> numerically unstable friction solution |

### 7.3 Additional Failure Modes

#### Warm-Start / Stiction Slip-Stick
Block PGS relies heavily on warm-starting lambda to simulate static friction (stiction). The current >15-degree cache invalidation could cause objects to slip briefly before re-gripping, creating a slip-stick vibration artifact. This threshold may need adjustment for the coupled solver.

#### Split Impulse "Fighting"
If the velocity solve (Block PGS) and position solve (Split Impulse / PositionCorrector) use different effective mass calculations or constraint directions due to integration timing, they can fight each other, injecting energy. The 3x3 block solve must use the **exact same geometric data** (Jacobians) as the position correction step.

### 7.4 Critical Assessment: Block PGS Does NOT Inherently Fix DD-0070-H2

> **Gemini's verdict: The proposal is mathematically incorrect in its assumption that Block PGS with cone projection inherently avoids the energy injection mechanism identified in DD-0070-H2.**

The energy injection stems from a fundamental conflict:

1. **Restitution** demands a specific normal velocity (v_n+)
2. **Coupling (K_nt)** means tangent impulse (lambda_t) changes normal velocity
3. **Cone constraint** limits lambda_t

In Block PGS, solving the 3x3 system for a target v (which includes the restitution bias) yields a candidate impulse. When lambda_t is clamped to satisfy the cone, the resulting v_n no longer matches the restitution target. Depending on the sign of K_nt, this projection can produce a v_n **higher in magnitude** than the restitution target, injecting energy.

**Why industry engines tolerate this**: The iterative nature of PGS distributes this error over multiple contacts and frames, usually resulting in net dissipation. But strictly, it does **not** eliminate the DD-0070-H2 physics mismatch -- it just makes it less likely to accumulate catastrophically.

### 7.5 Gemini Implementation Recommendations

#### Recommendation 1: Separate Restitution from the Solver Loop
Do not put the `(1+e)` term into the `b` vector of the Block PGS solve. Apply restitution as a separate, instantaneous impulse *before* the PGS loop. In the PGS loop, solve only for `v_n = 0` (pure non-penetration) and friction. This guarantees the PGS solver is purely dissipative.

*Trade-off*: Slightly less physically accurate friction-during-bounce behavior, but guarantees energy stability.

#### Recommendation 2: Safe 3x3 Inversion
Use LDLT decomposition (already proposed in design) with validated regularization. Do not fall back to LU -- use epsilon-regularized LDLT consistently. Validate that the regularization epsilon (1e-8) is sufficient for the condition numbers observed in production scenarios.

#### Recommendation 3: Manifold Batching
If collision detection generates manifolds (e.g., 4 points for box-box), ensure tangent basis vectors are aligned for all points in the manifold. Solving them individually with independently computed bases creates a "twisting" torque artifact.

#### Recommendation 4: Lock Tangent Basis During Iteration
Do not rotate t1, t2 vectors during PGS iterations within a single timestep. Updating basis vectors while iterating is a known source of energy generation (non-conservative field). Compute basis once at timestep start and hold fixed through all sweeps.

### 7.6 Gemini Summary Verdict

The move to Block PGS is standard industry practice and generally robust, but the specific claim that it "fixes" the QP energy injection bug (DD-0070-H2) is flawed. Without separating the restitution impulse from the coupled solve, the **regression risk for DD-0070 is HIGH**. The recommended approach is to apply restitution as a pre-solve impulse, then run Block PGS as a purely dissipative non-penetration + friction solver.

---

## 8. Combined Reviewer Consensus

Both reviewers agree on the following:

1. **The 0075 design is architecturally sound** in its goal of unifying constraints and using Block PGS (industry standard).
2. **The claim that Block PGS avoids DD-0070-H2 energy injection is unproven** and potentially incorrect. The `(1+e)` restitution term in the coupled 3x3 RHS, combined with K_nt coupling and cone projection, can theoretically reproduce the same mechanism.
3. **The prototype validation plan must be a hard gate** before Phase 2, with additional tests for oblique impacts, iteration sensitivity, and K matrix conditioning.
4. **Separating restitution from the PGS loop** (Gemini Recommendation 1) is the safest approach and should be seriously considered as a design revision.
5. **Runtime toggle** (not `#ifdef`) for A/B comparison during validation is essential.
6. **Sliding mode should be preserved** as a safety net until prototype evidence shows it is unnecessary under Block PGS.

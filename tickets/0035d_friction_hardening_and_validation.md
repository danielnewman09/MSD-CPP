# Ticket 0035d: Friction Hardening and Validation

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Assignee**: N/A
**Created**: 2026-01-31
**Generate Tutorial**: Yes
**Parent Ticket**: [0035_friction_constraints](0035_friction_constraints.md)

---

## Summary

Harden the friction constraint system with energy monitoring, stick-slip velocity thresholds, and regularization fallbacks. Run the full validation suite (all 6 M8 numerical examples) as integration tests. Verify performance (friction < 2x wall time of normal-only). This subtask ensures the friction system is robust, physically correct, and production-ready.

---

## Motivation

After 0035a-c, friction works end-to-end but may exhibit:
- **Energy injection**: Numerical artifacts where friction adds energy instead of dissipating it
- **Stick-slip jitter**: Oscillation between stick and slip at near-zero tangential velocity
- **Solver instability**: LLT decomposition failure for extreme mass ratios or degenerate contacts
- **Performance regression**: 3x constraint growth may slow the solver unacceptably

This subtask addresses these cross-cutting concerns through:
1. Energy monitoring (M6 validation)
2. Velocity threshold for stick-slip transition (M7 mitigation)
3. Regularization fallback (M7 mitigation)
4. Complete validation suite (M8)
5. Performance benchmarking

---

## Mathematical Foundation

This subtask validates:
- **[M6: Energy Dissipation](../docs/designs/0035_friction_constraints/M6-energy-dissipation.md)** — verify friction never injects energy
- **[M7: Numerical Stability](../docs/designs/0035_friction_constraints/M7-numerical-stability.md)** — implement velocity threshold, regularization
- **[M8: Numerical Examples](../docs/designs/0035_friction_constraints/M8-numerical-examples.md)** — all 6 examples as automated tests

---

## Technical Approach

### Energy Monitoring

Add energy tracking to friction-only test scenarios:
- Compute total kinetic energy $E_k = \frac{1}{2}\sum_i m_i v_i^2 + \frac{1}{2}\sum_i \omega_i^\top I_i \omega_i$ each timestep
- Assert $E_k(t+1) \leq E_k(t) + \epsilon$ for friction-only scenarios (no external forces)
- Tolerance $\epsilon = 10^{-6}$ J per timestep (accounts for floating-point accumulation)

### Velocity Threshold

Implement stick-slip jitter mitigation from M7:
- If $\|\mathbf{v}_t\| < v_{\text{rest}} = 0.01$ m/s after solve, clamp to $\mathbf{v}_t = \mathbf{0}$
- Use same threshold as restitution rest velocity (Ticket 0032)
- Post-processing step after LCP solve (does not affect solver internals)

### Regularization Fallback

If LLT decomposition fails (non-positive-definite $\mathbf{A}$):
- Add diagonal regularization $\mathbf{A}_{\text{reg}} = \mathbf{A} + \epsilon \mathbf{I}$ with $\epsilon = 10^{-10}$
- Log warning when regularization is triggered
- This should be rare — only needed for extreme mass ratios ($> 10^6:1$) with degenerate contact geometry

### Performance Validation

Benchmark friction solve vs. normal-only solve:
- 10 contacts with friction vs. 10 contacts normal-only
- Target: friction < 2x wall time (AC10 of parent ticket)
- Use existing Google Benchmark infrastructure

---

## Requirements

### Functional Requirements

1. **FR-1**: Energy monitoring available for test assertions
2. **FR-2**: Velocity threshold prevents stick-slip jitter for resting objects
3. **FR-3**: Regularization fallback prevents solver crashes for degenerate cases
4. **FR-4**: All 6 M8 numerical examples pass as automated tests
5. **FR-5**: Performance within 2x of normal-only for equivalent contact count

### Non-Functional Requirements

1. **NFR-1**: Velocity threshold is configurable (default 0.01 m/s)
2. **NFR-2**: Regularization parameter is configurable (default $10^{-10}$)
3. **NFR-3**: Energy monitoring adds negligible overhead (test-only code path)
4. **NFR-4**: All existing tests pass (zero regressions)

---

## Acceptance Criteria

- [ ] **AC1**: Energy monotonically non-increasing for sliding deceleration scenario (1000 timesteps, no external forces) — validates M6
- [ ] **AC2**: Block on inclined plane (static friction) shows zero velocity jitter over 1000 timesteps — validates M7 Case 4
- [ ] **AC3**: Friction cone saturation test (M8 Example 5): stick-to-slip transition at correct force threshold
- [ ] **AC4**: Two-body friction (M8 Example 6): Newton's third law verified ($\lambda_{t,A} + \lambda_{t,B} = 0$)
- [ ] **AC5**: Regularization triggered gracefully for mass ratio $10^6:1$ (no crash, result within 10% of expected)
- [ ] **AC6**: Performance: 10-contact friction solve < 2x wall time of 10-contact normal-only
- [ ] **AC7**: All 6 M8 numerical examples pass within specified tolerances
- [ ] **AC8**: All existing constraint tests pass (zero regressions)

---

## Dependencies

- **Ticket 0035a**: FrictionConstraint class (prerequisite)
- **Ticket 0035b**: Box-constrained ASM solver (prerequisite)
- **Ticket 0035c**: Friction pipeline integration (prerequisite — provides working end-to-end friction)
- **Blocks**: None (terminal subtask)

---

## Files

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Add velocity threshold post-processing, regularization fallback |
| `msd-sim/src/Physics/CollisionResponse.cpp` | Apply velocity threshold after friction solve |

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/FrictionValidationTest.cpp` | All 6 M8 examples as GTest cases |
| `msd-sim/test/Physics/FrictionEnergyTest.cpp` | Energy monotonicity tests |
| `msd-sim/test/Physics/FrictionStabilityTest.cpp` | Degenerate cases, mass ratio sensitivity, jitter tests |
| `msd-sim/benchmark/FrictionBenchmark.cpp` | Performance comparison (friction vs normal-only) |

---

## Validation Matrix

Maps M8 examples to test cases and acceptance criteria:

| M8 Example | Test Name | Key Assertion | AC |
|------------|-----------|---------------|-----|
| 1: Static friction (inclined plane) | `StaticFrictionInclinedPlane` | $v_t = 0$, $\lambda_t < \mu \lambda_n$ | AC7 |
| 2: Kinetic friction (inclined plane) | `KineticFrictionInclinedPlane` | $a = g(\sin\theta - \mu\cos\theta) \pm 5\%$ | AC7 |
| 3: Sliding deceleration | `SlidingDeceleration` | Stops at $v_0^2 / (2\mu g) \pm 5\%$ | AC1, AC7 |
| 4: Glancing collision with spin | `GlancingCollisionSpin` | $\omega = 16.90$ rad/s $\pm 0.5$ | AC7 |
| 5: Cone saturation | `FrictionConeSaturation` | Transition at $F = 23.54$ N | AC3, AC7 |
| 6: Two-body Newton's third law | `TwoBodyNewtonsThirdLaw` | $\lambda_{t,A} = -\lambda_{t,B}$ | AC4, AC7 |

---

## References

- **Math formulation**: [M6-energy-dissipation.md](../docs/designs/0035_friction_constraints/M6-energy-dissipation.md), [M7-numerical-stability.md](../docs/designs/0035_friction_constraints/M7-numerical-stability.md), [M8-numerical-examples.md](../docs/designs/0035_friction_constraints/M8-numerical-examples.md)
- **Benchmarking guide**: `docs/benchmarking.md`
- **Existing energy tests**: Ticket 0032 restitution energy validation pattern

---

## Workflow Log

### Draft Phase
- **Created**: 2026-01-31
- **Notes**: Terminal subtask for hardening and validation. Math formulation for M6, M7, M8 already complete. This subtask skips the math formulation and design phases — it's primarily implementation and testing of cross-cutting concerns identified in the math analysis.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

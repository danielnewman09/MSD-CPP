# Ticket 0035d: Friction Hardening and Validation

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Quality Gate
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

### Status Advancement (Draft → Ready for Design)
- **Updated**: 2026-02-01
- **Notes**: Ticket does not require Math Design phase (no "Requires Math Design: Yes" flag). Math formulation complete in parent ticket 0035. Advanced to Ready for Design phase. Architectural design is next step to structure the hardening features (energy monitoring, velocity threshold, regularization) and validation test suite.

### Design Phase
- **Started**: 2026-02-01
- **Completed**: 2026-02-01
- **Artifacts**:
  - `docs/designs/0035d_friction_hardening_and_validation/design.md` — Comprehensive design document
  - `docs/designs/0035d_friction_hardening_and_validation/0035d_friction_hardening_and_validation.puml` — PlantUML architecture diagram
- **Notes**: Design is primarily test-focused with minimal production code changes (80 LOC modified, 1560 LOC new test infrastructure). Three main components: (1) Energy monitoring infrastructure for M6 validation, (2) Velocity threshold post-processing for M7 stick-slip jitter mitigation, (3) Regularization fallback for M7 degenerate cases. All 6 M8 numerical examples mapped to automated test cases with traceability matrix. Performance benchmarks compare friction vs normal-only solver. Design ready for review.

### Design Review Phase (Initial Assessment)
- **Reviewed**: 2026-02-01
- **Reviewer**: Design Review Agent
- **Status**: REVISION_REQUESTED (Iteration 0 of 1)
- **Issues Found**: 2 architectural fit issues
  - **I1**: CollisionResponse component does not exist in codebase (removed in ticket 0032d). Replace with WorldModel which handles collision integration.
  - **I2**: PlantUML diagram shows non-existent CollisionResponse class. Update to show WorldModel as modified component.
- **Items Passing**: Test infrastructure design, ConstraintSolver modifications, test coverage, performance benchmarking, C++ design quality, naming conventions, memory management, error handling all approved.
- **Notes**: Minor architectural issue. Design correctly identifies where velocity threshold post-processing should occur (collision response integration layer), but references the wrong component name. Velocity threshold should be in WorldModel::updateCollisions(), not a non-existent CollisionResponse class. Awaiting architect revision to correct component references.

### Architect Revision Phase
- **Revised**: 2026-02-01
- **Architect**: cpp-architect agent (autonomous revision)
- **Changes Made**:
  - Fixed PlantUML diagram: Replaced `CollisionResponse <<modified>>` with `WorldModel <<modified>>` in "msd-sim::Environment" package
  - Updated diagram relationships: Changed to `WorldModel ..> ConstraintSolver : uses`
  - Verified design document already correctly referenced WorldModel in Modified Components section
- **Artifacts Updated**:
  - `docs/designs/0035d_friction_hardening_and_validation/0035d_friction_hardening_and_validation.puml` — Corrected component references
  - `docs/designs/0035d_friction_hardening_and_validation/design.md` — Appended architect revision notes
- **Notes**: Issues I1 and I2 resolved. PlantUML diagram now consistent with design document text. Ready for final design review.

### Design Review Phase (Final Assessment)
- **Reviewed**: 2026-02-01
- **Reviewer**: Design Review Agent
- **Status**: APPROVED (Iteration 1 of 1)
- **Verification**: All issues from initial assessment resolved. PlantUML diagram corrected, WorldModel properly shown as modified component in msd-sim::Environment package.
- **Criteria**: All architectural fit, C++ design quality, feasibility, and testability criteria pass.
- **Risks**: 3 low-likelihood/low-impact risks identified, all with clear mitigation strategies. No prototypes required.
- **Notes**: Design approved. Comprehensive test coverage for M6, M7, M8 validation. Ready for human review and advancement to Ready for Implementation phase.

### Human Approval Phase
- **Approved**: 2026-02-01
- **Approver**: Human operator
- **Status**: Design approved for implementation
- **Notes**: No open questions, no prototype required (math formulation M6, M7, M8 complete in parent ticket). Advanced to Ready for Implementation.

### Status Advancement (Design → Ready for Implementation)
- **Advanced**: 2026-02-01
- **Notes**: Design approved by autonomous review cycle and human. No prototyping phase needed. Proceeding directly to implementation of test infrastructure (1560 LOC) and production hardening features (80 LOC).

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

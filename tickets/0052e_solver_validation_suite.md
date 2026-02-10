# Ticket 0052e: Solver Validation Suite

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Draft
**Assignee**: N/A
**Created**: 2026-02-10
**Generate Tutorial**: No
**Parent Ticket**: [0052_custom_friction_cone_solver](0052_custom_friction_cone_solver.md)

---

## Summary

Comprehensive validation suite for the custom friction cone solver. This goes beyond the unit tests in 0052b/0052c to include end-to-end physics scenarios, energy monotonicity verification, convergence diagnostics, comparison against analytical solutions, and performance benchmarks.

---

## Motivation

The solver unit tests (0052c) verify the solver in isolation with synthetic A, b, μ inputs. This validation suite tests the solver in the full simulation context:
- Does it produce physically correct results when driven by actual collision detection?
- Does it maintain energy monotonicity across multi-frame simulations?
- Does it converge reliably across the range of contact configurations the simulation produces?
- Does it meet performance targets?

---

## Technical Approach

### Test Categories

#### 1. Known Analytical Solutions

Tests where the exact answer can be computed by hand, verifying solver correctness end-to-end through the simulation pipeline.

| Test | Scenario | Key Assertion |
|------|----------|---------------|
| Static friction inclined plane | Block on 20° slope, μ = 0.6 | Block stationary for 1000 frames, friction < μN |
| Kinetic friction inclined plane | Block on 45° slope, μ = 0.3 | Acceleration = g(sin45° - μcos45°) ± 5% |
| Sliding deceleration | Horizontal v₀ on flat surface, μ = 0.5 | Deceleration = μg, correct stopping distance |
| Glancing collision with spin | Off-center impact | Angular velocity from tangential impulse |
| Friction cone saturation | Increasing tangential load | Stick-to-slip transition at correct threshold |
| Two-body friction (Newton's 3rd law) | Both bodies dynamic | λ_t,A = -λ_t,B |

#### 2. Energy Monotonicity

Tests that friction never injects energy:
- Sliding deceleration with no external forces: KE monotonically decreasing over 1000 frames
- Resting contact with friction: KE stays at zero (no jitter)
- Multi-body friction chain: total energy non-increasing

#### 3. Convergence Diagnostics

Tests that verify solver convergence behavior:
- Cold start iteration count ≤ 8 for standard scenarios
- Warm start iteration count ≤ 3 for consecutive frames
- Residual decreases monotonically (quadratic convergence curve)
- Regularization triggers gracefully for mass ratio 10⁶:1

#### 4. Stress Tests

Tests at the boundary of the solver's operating envelope:
- 50 simultaneous contacts with friction
- Extreme μ values: μ = 0.01 (ice), μ = 2.0 (rubber)
- Mass ratio 10⁶:1 with friction
- Rapidly changing contact configurations (new contacts every frame)

#### 5. Performance Benchmarks

Using Google Benchmark infrastructure:
- Single-contact friction solve (target: < 10 μs)
- 10-contact friction solve (target: < 100 μs)
- Friction vs normal-only comparison (target: < 2× wall time)
- Warm start vs cold start timing comparison

---

## Acceptance Criteria

- [ ] **AC1**: All 6 analytical solution tests pass within specified tolerances
- [ ] **AC2**: Energy monotonicity verified for all friction-only scenarios (1000 frames, no energy increase > 10⁻⁶ J per frame)
- [ ] **AC3**: Convergence diagnostics confirm ≤ 8 iterations cold, ≤ 3 warm
- [ ] **AC4**: Stress tests pass without crashes or NaN (regularization acceptable)
- [ ] **AC5**: Performance benchmarks meet targets (< 2× normal-only wall time)
- [ ] **AC6**: All existing tests pass (zero regressions)
- [ ] **AC7**: Test report documents actual iteration counts, residuals, and timings for all scenarios

---

## Dependencies

- **Requires**: 0052c (Newton solver core), 0052d (solver integration)
- **Blocks**: None (terminal subtask)

---

## Files

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/FrictionConeSolverValidationTest.cpp` | Analytical solution tests (inclined plane, deceleration, etc.) |
| `msd-sim/test/Physics/FrictionEnergyMonotonicityTest.cpp` | Energy monotonicity tests |
| `msd-sim/test/Physics/FrictionConvergenceDiagnosticTest.cpp` | Convergence behavior tests |
| `msd-sim/test/Physics/FrictionStressTest.cpp` | Boundary condition and stress tests |
| `msd-sim/benchmark/FrictionConeSolverBenchmark.cpp` | Performance benchmarks |

---

## Validation Matrix

Maps parent ticket acceptance criteria to specific tests:

| Parent AC | Test File | Test Name | Assertion |
|-----------|-----------|-----------|-----------|
| AC1 (cone projection) | ConeProjectionTest | All cases | Exact values |
| AC2 (≤ 8 iterations cold) | ConvergenceDiagnosticTest | ColdStartIterations | iterations ≤ 8 |
| AC3 (≤ 3 iterations warm) | ConvergenceDiagnosticTest | WarmStartIterations | iterations ≤ 3 |
| AC4 (KKT residual) | FrictionConeSolverTest | All M8 examples | ‖Aλ-b‖ < 1e-8 |
| AC5 (cone feasibility) | FrictionConeSolverTest | All M8 examples | ‖λ_t‖ ≤ μλ_n + 1e-8 |
| AC6 (energy monotonicity) | FrictionEnergyMonotonicityTest | SlidingDeceleration | KE non-increasing |
| AC7 (mass ratio 10⁶) | FrictionStressTest | HighMassRatio | No crash, result within 10% |
| AC8 (zero regressions) | Existing test suite | All | Pass count ≥ baseline |
| AC9 (performance) | FrictionConeSolverBenchmark | FrictionVsNormalOnly | < 2× wall time |
| AC10 (ECOS removed) | Build system | cmake --build | No ECOS references |

---

## References

- 0052a math formulation, Section M8 (numerical examples)
- Parent ticket 0052 acceptance criteria
- 0035d (friction hardening and validation) — similar validation approach adapted for custom solver
- Existing energy test patterns from Tickets 0039a, 0042a

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-10
- **Notes**: Terminal validation subtask. Builds on the validation approach from 0035d but adapted for the custom solver instead of ECOS. Includes performance benchmarks that were deferred from 0035d.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

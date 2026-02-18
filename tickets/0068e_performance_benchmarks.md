# Ticket 0068e: NLopt Friction Solver Performance Benchmarks

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Type**: Test
**Priority**: Medium
**Created**: 2026-02-16
**Generate Tutorial**: No
**Parent Ticket**: [0068_nlopt_friction_cone_solver](0068_nlopt_friction_cone_solver.md)
**Depends On**: [0068c](0068c_constraint_solver_integration.md)

---

## Overview

Google Benchmark performance tests for `NLoptFrictionSolver` to validate the P2 prototype criterion: solve time within 2x of the custom solver baseline. This ticket is lower priority since the human approved 2x slowdown for correctness, but benchmarks provide a baseline for future optimization.

---

## Requirements

### R1: Benchmark Cases

Create `msd/msd-sim/test/Benchmark/FrictionSolverBenchmark.cpp` (requires `ENABLE_BENCHMARKS=ON`):

| Benchmark | Description | Target |
|-----------|-------------|--------|
| `BM_NLoptSLSQP_1Contact` | 1-contact (3 vars) cold start | < 50 μs |
| `BM_NLoptSLSQP_4Contacts` | 4-contact (12 vars) cold start | < 200 μs |
| `BM_NLoptSLSQP_1Contact_WarmStart` | 1-contact with warm-start | < 30 μs |
| `BM_NLoptCOBYLA_1Contact` | COBYLA 1-contact for comparison | Informational |

### R2: Benchmark Setup

- Generate realistic A/b/mu from a representative physics scenario
- Use `benchmark::DoNotOptimize()` to prevent dead code elimination
- Run 1000+ iterations per case for stable timing
- Build: `conan install . --build=missing -s build_type=Release -o "&:enable_benchmarks=True"`
- Run: `cmake --build --preset release-sim-only && ./build/Release/release/friction_solver_benchmark`

### R3: Baseline Recording

- Save benchmark results to `analysis/benchmark_baselines/friction_solver_nlopt.json`
- Document: solver algorithm, contact count, mean/median/stddev timing

---

## Acceptance Criteria

1. Benchmarks compile and run with `ENABLE_BENCHMARKS=ON`
2. 1-contact solve time < 50 μs (SLSQP)
3. 4-contact solve time < 200 μs (SLSQP)
4. Baseline JSON saved for future regression detection

---

## Files to Create

| File | Purpose |
|------|---------|
| `msd/msd-sim/test/Benchmark/FrictionSolverBenchmark.cpp` | Google Benchmark test |
| `analysis/benchmark_baselines/friction_solver_nlopt.json` | Baseline results |

---

## Notes

- This is optional for initial merge — can be deferred if benchmarks show acceptable performance during manual testing
- If timing exceeds 2x, consider COBYLA or algorithm tuning before raising concern
- Release build required for meaningful timing (Debug build has NLopt asserts enabled)

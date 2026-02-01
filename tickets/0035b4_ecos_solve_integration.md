# Ticket 0035b4: ECOS Solve Integration

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Documentation Complete
- [x] Merged / Complete

**Current Phase**: Merged / Complete
**Assignee**: N/A
**Created**: 2026-01-31
**Generate Tutorial**: No
**Parent Ticket**: [0035b_box_constrained_asm_solver](0035b_box_constrained_asm_solver.md)

---

## Summary

Integrate the ECOS solver into `ConstraintSolver` by implementing `solveWithECOS()` and the friction detection/dispatch logic in `solveWithContacts()`. This connects the problem construction (0035b3) to the ECOS solve call and routes friction constraints to ECOS while preserving the ASM path for normal-only contacts.

---

## Motivation

The previous tickets built the data pipeline (sparse matrices, cone specs, problem construction). This ticket connects that pipeline to the actual ECOS solver call and integrates it into the existing `ConstraintSolver` class, completing the solver implementation.

---

## Technical Approach

### Method: solveWithECOS()

**Location**: `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` / `ConstraintSolver.cpp`

**Responsibilities**:
1. Build ECOS problem from A, b, cone spec (using ECOSProblemBuilder from 0035b3)
2. Call `ECOSData::setup()` to create ECOS workspace
3. Configure ECOS solver settings (tolerance, max iterations)
4. Call `ECOS_solve()` and check exit code
5. Extract solution lambda from workspace->x
6. Extract diagnostics (iterations, residuals, gap)
7. Return `ActiveSetResult` with ECOS-specific fields

**Interface**:
```cpp
/// Solve friction LCP using ECOS SOCP solver
/// @param A Effective mass matrix (3C x 3C)
/// @param b RHS vector (3C x 1)
/// @param coneSpec Friction cone specification
/// @param numContacts Number of contacts
/// @return ActiveSetResult with lambda and ECOS diagnostics
ActiveSetResult solveWithECOS(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const FrictionConeSpec& coneSpec,
    int numContacts) const;
```

### Dispatch Logic in solveWithContacts()

**Modification to existing method**:
1. Scan input constraints for `FrictionConstraint` instances (via `dynamic_cast`)
2. If friction detected: call `buildFrictionConeSpec()` then `solveWithECOS()`
3. If no friction: use existing `solveActiveSet()` (zero-regression path)

### ActiveSetResult Extensions

**New fields** (from design.md):
```cpp
std::string solver_type{"ASM"};  // "ASM" or "ECOS"
int ecos_exit_flag{0};
double primal_residual{std::numeric_limits<double>::quiet_NaN()};
double dual_residual{std::numeric_limits<double>::quiet_NaN()};
double gap{std::numeric_limits<double>::quiet_NaN()};
```

### ECOS Configuration

**New member variables on ConstraintSolver**:
```cpp
double ecos_abs_tol_{1e-6};
double ecos_rel_tol_{1e-6};
int ecos_max_iters_{100};
```

**Setter/getter methods** for runtime configuration.

---

## Requirements

### Functional Requirements

1. **FR-1**: solveWithECOS calls ECOS_solve() and returns solution
2. **FR-2**: Solution lambda extracted correctly from ECOS workspace
3. **FR-3**: ECOS exit code mapped to converged flag (ECOS_OPTIMAL → true)
4. **FR-4**: ECOS diagnostics (iterations, residuals, gap) populated in result
5. **FR-5**: solveWithContacts dispatches to ECOS when FrictionConstraint detected
6. **FR-6**: solveWithContacts uses ASM when no FrictionConstraint present
7. **FR-7**: ECOS tolerance and max iterations configurable
8. **FR-8**: ECOS_MAXIT returns converged=false with last iterate

### Non-Functional Requirements

1. **NFR-1**: Zero regression — all existing tests pass with unchanged ASM path
2. **NFR-2**: ECOS solve completes in ≤ 30 iterations for all test cases
3. **NFR-3**: Clear error messages on ECOS failure (exit code, contact count, mu values)

---

## Acceptance Criteria

- [ ] **AC1**: solveWithECOS returns correct lambda for single-contact stick regime (interior solution)
- [ ] **AC2**: solveWithECOS returns correct lambda for single-contact slip regime (cone boundary)
- [ ] **AC3**: ECOS converges (ECOS_OPTIMAL) for well-conditioned problems
- [ ] **AC4**: ECOS diagnostics populated in ActiveSetResult
- [ ] **AC5**: Dispatch logic: no friction → ASM, friction → ECOS
- [ ] **AC6**: ECOS tolerance and max iterations configurable at runtime
- [ ] **AC7**: ECOS_MAXIT handled gracefully (converged=false, last iterate returned)
- [ ] **AC8**: All existing tests pass (zero regressions)

---

## Test Plan

| Test Case | What It Validates |
|-----------|-------------------|
| Single contact stick (force < mu*N) | ECOS returns interior solution, v_t = 0 |
| Single contact slip (force > mu*N) | ECOS returns cone boundary, v_t != 0 |
| ECOS convergence | Exit flag = ECOS_OPTIMAL, iterations < 30 |
| ECOS diagnostics | primal_residual, dual_residual, gap populated |
| Dispatch: no friction | solveWithContacts uses ASM path |
| Dispatch: with friction | solveWithContacts uses ECOS path |
| solver_type field | ASM result has "ASM", ECOS result has "ECOS" |
| Tolerance configuration | setECOSTolerance affects solve accuracy |
| Max iterations configuration | setECOSMaxIterations caps iterations |
| Zero friction (mu=0) | lambda_t = 0 (degenerate cone) |
| High mass ratio (1000:1) | ECOS converges without ECOS_NUMERICS |

---

## Files

### New Files

None — all changes are in existing files plus the existing ECOS/ directory.

### Modified Files

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Add solveWithECOS(), ECOS config members, ActiveSetResult ECOS fields |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Implement solveWithECOS(), extend solveWithContacts() dispatch |
| `msd-sim/test/Physics/Constraints/ECOS/CMakeLists.txt` | Add solve integration tests if separate file |

### Possibly New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/Constraints/ECOS/ECOSSolveTest.cpp` | ECOS solve integration tests |

---

## Dependencies

- **Requires**: [0035b2](0035b2_ecos_data_wrapper.md) (ECOSData for workspace management)
- **Requires**: [0035b3](0035b3_ecos_problem_construction.md) (ECOSProblemBuilder for problem construction)
- **Blocks**: [0035b5](0035b5_ecos_validation_tests.md) (validation needs working solver)

---

## References

- **Design document**: [design.md](../docs/designs/0035b_box_constrained_asm_solver/design.md) — "ECOS API Workflow", "Modified Components: ConstraintSolver"
- **ECOS API**: ECOS_setup(), ECOS_solve(), ECOS_cleanup(), exit codes
- **ECOS settings**: workspace->stgs->maxit, abstol, reltol, feastol

---

## Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| ECOS formulation doesn't produce correct friction forces | High | Validate against hand-computed M8 numerical examples |
| ECOS doesn't converge for some configurations | Medium | Return converged=false, log warning, cap at 100 iterations |
| Dispatch logic breaks existing ASM path | High | Run full regression suite (all existing tests) |
| ECOS numerical issues with high mass ratios | Medium | Test 1000:1 mass ratio, handle ECOS_NUMERICS gracefully |

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-01-31
- **Completed**: 2026-01-31
- **Artifacts**:
  - `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` — Added solveWithECOS(), ECOS config members, ActiveSetResult ECOS fields
  - `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` — Implemented solveWithECOS(), extended solveWithContacts() dispatch
  - `msd-sim/test/Physics/Constraints/ECOS/ECOSSolveTest.cpp` — ECOS solve integration tests
- **Notes**: Implementation complete per design specification

### Quality Gate Phase (Iteration 1)
- **Started**: 2026-02-01
- **Completed**: 2026-02-01
- **Status**: FAILED
- **Artifacts**:
  - `docs/designs/0035b_box_constrained_asm_solver/quality-gate-report.md` — Quality gate report
- **Notes**: Build failed with 10 sign-conversion warnings in ECOSProblemBuilderTest.cpp. Using signed `idxint` type to index unsigned `std::vector` containers. Requires explicit casts to satisfy Release build's `-Werror -Wsign-conversion` flags.
- **Required Changes**:
  1. Add static_cast<size_t> when using idxint values to index into std::vector in ECOSProblemBuilderTest.cpp
  2. Pattern: `data.G_.row_indices[static_cast<size_t>(idx)]`
  3. Apply to all 10 locations identified in quality gate report

### Quality Gate Phase (Iteration 2)
- **Started**: 2026-02-01
- **Completed**: 2026-02-01
- **Status**: PASSED
- **Artifacts**:
  - `docs/designs/0035b_box_constrained_asm_solver/quality-gate-report.md` — Updated quality gate report
- **Notes**: All sign-conversion warnings fixed. Clean build with zero warnings in Release mode. All 60 ECOS tests pass (100% success rate). One pre-existing test failure in GeometryDatabaseTest unrelated to this ticket (verified on parent commit). Ready for implementation review.

### Implementation Review Phase
- **Started**: 2026-02-01
- **Completed**: 2026-02-01
- **Status**: APPROVED
- **Artifacts**:
  - `docs/designs/0035b_box_constrained_asm_solver/implementation-review.md` — Implementation review report
- **Notes**: Implementation fully conforms to design specification (100% component conformance). All prototype learnings applied correctly. Code quality excellent (RAII, error handling, thread safety, documentation). Test coverage comprehensive (60 ECOS tests, 100% pass rate, zero regressions). All 8 acceptance criteria validated. Ready for documentation update.

### Documentation Phase
- **Started**: 2026-02-01
- **Completed**: 2026-02-01
- **Artifacts**:
  - `msd/msd-sim/src/Physics/Constraints/CLAUDE.md` — Updated with ECOS solver path, solveWithECOS() method, ECOS configuration options, usage examples
  - `msd/msd-sim/src/Physics/Constraints/ECOS/CLAUDE.md` — Updated integration status to "completed"
  - `docs/designs/0035b_box_constrained_asm_solver/doc-sync-summary.md` — Documentation sync summary
- **Notes**: Documentation updates completed. No new diagrams required - existing component diagrams (ecos-data.puml, ecos-problem-builder.puml) and design document diagram (0035b_box_constrained_asm_solver.puml) adequately document the integration. CLAUDE.md files updated with algorithm overview, interfaces, and usage examples. Tutorial generation not requested (Generate Tutorial: No). Ready to advance to Merged / Complete.

---

## Human Feedback

### Quality Gate Failure - Sign Conversion (2026-02-01)

**Status**: ✓ Resolved

The quality gate detected sign-conversion warnings in test code. These have been fixed:

**Issue**: Test code in `ECOSProblemBuilderTest.cpp` uses ECOS `idxint` (signed long) to index into `std::vector` containers (which use unsigned `size_type`). Release build has `-Werror` enabled, so warnings block compilation.

**Solution**: Added explicit `static_cast<size_t>(...)` when indexing vectors with `idxint` values.

**Resolution**: All 10 sign-conversion warnings fixed. Quality gate iteration 2 passed with clean build and all tests passing.

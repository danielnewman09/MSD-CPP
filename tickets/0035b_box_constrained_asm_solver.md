# Ticket 0035b: ECOS-Based Friction Cone Solver

## Status
- [x] Draft
- [ ] Ready for Math Formulation
- [ ] Math Formulation Complete — Awaiting Review
- [ ] Math Review Approved — Ready for Design
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation In Progress (Phase 1 Complete)
**Assignee**: N/A
**Created**: 2026-01-31
**Generate Tutorial**: No
**Parent Ticket**: [0035_friction_constraints](0035_friction_constraints.md)

---

## Sub-Tickets (Implementation Phases)

| Ticket | Phase | Description | Status |
|--------|-------|-------------|--------|
| [0035b1](0035b1_ecos_utilities.md) | Phase 1 | ECOS Utilities (ECOSSparseMatrix, FrictionConeSpec) | **Complete** |
| [0035b2](0035b2_ecos_data_wrapper.md) | Phase 2a | ECOSData RAII Wrapper | Ready for Implementation |
| [0035b3](0035b3_ecos_problem_construction.md) | Phase 2b | ECOS Problem Construction (G, h, c matrices) | Ready for Implementation |
| [0035b4](0035b4_ecos_solve_integration.md) | Phase 3 | ECOS Solve Integration (solveWithECOS, dispatch) | Ready for Implementation |
| [0035b5](0035b5_ecos_validation_tests.md) | Phase 4 | Validation Tests (stick/slip, multi-contact, regression) | Ready for Implementation |

**Dependency chain**: 0035b1 → 0035b2 → 0035b3 → 0035b4 → 0035b5

---

## Summary

Extend the Active Set Method (ASM) constraint solver to handle **box-constrained variables** for friction. Currently the ASM solves unilateral constraints ($\lambda_n \geq 0$). This subtask adds support for constraints with variable upper and lower bounds ($\lambda_{t_i} \in [-\mu \lambda_n / \sqrt{2}, +\mu \lambda_n / \sqrt{2}]$), enabling the solver to compute friction forces within the Coulomb cone approximation.

This is the core algorithmic work of the friction feature.

---

## Motivation

The existing ASM solver (Ticket 0034) handles only two constraint types:
- **Bilateral**: $\lambda$ unconstrained (equality constraint, solved in Phase 1)
- **Unilateral**: $\lambda \geq 0$ (inequality constraint, solved in Phase 2 via active set)

Friction requires a third type:
- **Box-constrained**: $\lambda_{lo} \leq \lambda \leq \lambda_{hi}$ where bounds depend on another variable ($\lambda_n$)

This is the standard "box-LCP" formulation used in physics engines (Box2D, Bullet, ODE) for friction. The ASM must track three states per constraint: at lower bound, at upper bound, or interior (free). The coupling between friction bounds and normal force requires either a single-pass approximation or an outer iteration loop.

---

## Mathematical Foundation

This subtask implements:
- **[M3: Coulomb Cone — Scaled Box Approximation](../docs/designs/0035_friction_constraints/M3-coulomb-cone.md)** — bounds $\lambda_{t_i} \in [-\mu/\sqrt{2} \cdot \lambda_n, +\mu/\sqrt{2} \cdot \lambda_n]$
- **[M4: Complementarity Conditions](../docs/designs/0035_friction_constraints/M4-complementarity.md)** — stick/slip regimes within box LCP
- **[M5: Solver Extension Analysis](../docs/designs/0035_friction_constraints/M5-solver-extension.md)** — Option A (box-constrained ASM)

---

## Technical Approach

### Modified Components

| Component | Changes |
|-----------|---------|
| `ConstraintSolver` | Extend `solveWithContacts()` to accept `FrictionConstraint` rows alongside `ContactConstraint` rows. Modify ASM working set to track lower/upper/interior states. |
| `ActiveSetMethod` (or equivalent) | Add box-bound support: three-state tracking per constraint, bound projection, KKT checks for bound activity |

### Algorithm Changes

**Current ASM (unilateral only)**:
- Working set = {active constraints with $\lambda > 0$}
- States: active ($\lambda > 0$) or inactive ($\lambda = 0$)

**Extended ASM (box-constrained)**:
- Working set = {constraints not at bounds}
- States per constraint:
  - **Lower bound**: $\lambda = \lambda_{lo}$, dual variable $\leq 0$
  - **Upper bound**: $\lambda = \lambda_{hi}$, dual variable $\geq 0$
  - **Free (interior)**: $\lambda_{lo} < \lambda < \lambda_{hi}$, velocity constraint satisfied exactly
- Normal constraints: $\lambda_{lo} = 0$, $\lambda_{hi} = +\infty$ (unchanged)
- Friction constraints: $\lambda_{lo} = -\mu \lambda_n / \sqrt{2}$, $\lambda_{hi} = +\mu \lambda_n / \sqrt{2}$

**Outer iteration (if needed)**:
1. Fix friction bounds using current $\lambda_n$ estimate
2. Solve box-constrained LCP (inner ASM)
3. Update friction bounds from new $\lambda_n$
4. Repeat until $\|\Delta \lambda_n\| < 10^{-6}$ or max 10 iterations

### Constraint Ordering

The 3×3 block structure per contact (1 normal + 2 tangential) should be kept contiguous in the constraint array for cache efficiency and to simplify bound coupling:
```
[λ_n1, λ_t1_1, λ_t2_1, λ_n2, λ_t1_2, λ_t2_2, ...]
```

---

## Requirements

### Functional Requirements

1. **FR-1**: ASM solver accepts constraints with box bounds $[\lambda_{lo}, \lambda_{hi}]$
2. **FR-2**: Three-state working set: lower bound, upper bound, free
3. **FR-3**: KKT conditions checked for all three states
4. **FR-4**: Friction bounds depend on solved $\lambda_n$ (variable bounds)
5. **FR-5**: Single-contact friction scenario produces correct stick/slip behavior
6. **FR-6**: Multi-contact scenarios converge to complementarity solution

### Non-Functional Requirements

1. **NFR-1**: Existing unilateral-only solve path unchanged when no friction constraints present
2. **NFR-2**: All existing constraint tests pass (zero regressions)
3. **NFR-3**: Box-LCP convergence: finite iterations, bounded by $2 \times 3C$ working set changes
4. **NFR-4**: Outer iteration (if implemented) converges in $\leq 10$ iterations for well-conditioned systems

---

## Acceptance Criteria

- [ ] **AC1**: Single-contact box-LCP with fixed bounds produces correct $\lambda_{t_i}$ (verified against hand computation)
- [ ] **AC2**: Stick regime: when applied tangential force < $\mu \lambda_n / \sqrt{2}$, solver returns interior solution with $v_t = 0$
- [ ] **AC3**: Slip regime: when applied tangential force > $\mu \lambda_n / \sqrt{2}$, solver returns bound-active solution with $v_t \neq 0$
- [ ] **AC4**: Multi-contact (2+ contacts) box-LCP converges and satisfies KKT conditions
- [ ] **AC5**: Variable friction bounds (coupled to $\lambda_n$) produce consistent solution
- [ ] **AC6**: All 79 existing constraint tests pass (zero regressions from Ticket 0032/0034)
- [ ] **AC7**: ASM with friction constraints terminates in finite iterations for all test cases

---

## Dependencies

- **Ticket 0034**: Active Set Method solver (prerequisite — provides ASM to extend)
- **Ticket 0035a**: FrictionConstraint class (prerequisite — provides constraint data model)
- **Blocks**: [0035c](0035c_friction_pipeline_integration.md) (pipeline integration needs working solver)

---

## Files

### Modified Files

| File | Changes |
|------|---------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Add box-bound support to ASM |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Implement three-state working set, bound projection, outer iteration |
| `msd-sim/test/Physics/Constraints/ConstraintSolverTest.cpp` | Add box-LCP unit tests |

### Possibly New Files

| File | Purpose |
|------|---------|
| `msd-sim/test/Physics/Constraints/FrictionSolverTest.cpp` | Dedicated friction solver tests (stick/slip scenarios) |

---

## Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Outer iteration doesn't converge for some configurations | High | Cap at 10 iterations, fallback to last feasible solution |
| Box-LCP significantly slower than unilateral-only | Medium | Profile; single-pass option as fast path |
| Three-state working set logic is error-prone | Medium | Extensive unit tests for each state transition |

---

## References

- **Math formulation**: [M3-coulomb-cone.md](../docs/designs/0035_friction_constraints/M3-coulomb-cone.md), [M4-complementarity.md](../docs/designs/0035_friction_constraints/M4-complementarity.md), [M5-solver-extension.md](../docs/designs/0035_friction_constraints/M5-solver-extension.md)
- **Existing solver**: `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`
- **ASM design**: `docs/designs/0034_active_set_method_contact_solver/design.md`
- **Catto (2009)**: Box2D friction solver (sequential impulse with box bounds)

---

## Workflow Log

### Draft Phase
- **Created**: 2026-01-31
- **Notes**: Core algorithmic subtask. Split from parent ticket 0035. This is the highest-risk component — all solver changes isolated here. Math formulation for M3, M4, M5 already complete.

### Ready for Design Phase
- **Advanced**: 2026-01-31
- **Notes**: Math design phase skipped - mathematical foundation already complete in parent ticket 0035 (M3, M4, M5 documents). Proceeding directly to architectural design for box-constrained ASM solver extension.

### Design Phase
- **Started**: 2026-01-31
- **Completed**: 2026-01-31
- **Artifacts**:
  - `docs/designs/0035b_box_constrained_asm_solver/design.md` — Architectural design document (box-LCP approach)
  - `docs/designs/0035b_box_constrained_asm_solver/0035b_box_constrained_asm_solver.puml` — PlantUML class diagram
- **Notes**:
  - Core modification: Extend ConstraintSolver::solveActiveSet() to handle box-constrained variables with three-state tracking (AT_LOWER, AT_UPPER, FREE)
  - New components: BoxBounds struct for bound storage, BoundState enum for state tracking
  - Outer iteration loop required for variable friction bounds (depend on solved normal forces)
  - Backward compatible: Zero-regression path when no friction constraints present
  - Open questions: Initial bound estimates, outer iteration tolerance, fallback behavior
  - Prototype required: Measure outer iteration count in practice, validate performance overhead, test high mass ratio convergence
  - Test impact: 14 new unit tests, 4 integration tests, zero expected regressions (all 79 existing tests pass)

### Design Review — SOCP Solver Selection (2026-01-31)
- **Status**: COMPLETED — ECOS selected, design revised
- **Human request**: Evaluate feasibility of using Splitting Conic Solver library to solve exact Coulomb cone constraint (SOCP formulation) rather than box approximation
- **Investigation completed**:
  - SCS library analysis (Conan dependency, CMake, MIT license — feasible)
  - ECOS library analysis (Conan, GPLv3 acceptable for educational project)
  - Performance comparison: Both likely competitive for small C, exact cone vs 29% box error
  - API complexity: Both require ~300 lines glue code (sparse CSC conversion, cone formulation)
- **Artifacts**:
  - `docs/designs/0035b_box_constrained_asm_solver/scs-feasibility-study.md` — SOCP feasibility study
- **Human decision**: ECOS selected (GPLv3 acceptable for educational project)
- **Design revision completed**:
  - Replaced box-LCP ASM with ECOS SOCP formulation
  - Interior-point method (5-15 iterations) replaces ASM outer/inner loop
  - Exact Coulomb cone (0% error) replaces box approximation (29% error)
  - New components: ECOSData, ECOSSparseMatrix, FrictionConeSpec
  - Backward compatibility preserved: ASM path for normal-only contacts

### Design Approval Phase (2026-01-31)
- **Reviewed by**: Human
- **Status**: APPROVED
- **Feedback**: "looks good, continue processing this ticket"
- **Notes**: ECOS-based SOCP design approved. Ready to proceed to prototype phase to validate:
  1. ECOS convergence iteration count in practice
  2. Performance overhead vs ASM
  3. ECOS formulation correctness (validate against M8 numerical examples)

### Prototype Phase (2026-01-31)
- **Started**: 2026-01-31
- **Completed**: 2026-01-31
- **Approach**: Implementation-first with incremental validation (standalone prototyping deemed redundant)
- **Artifacts**:
  - `docs/designs/0035b_box_constrained_asm_solver/prototype-results.md` — Prototype assessment and implementation ticket
- **Key findings**:
  - ECOS is mature external library with well-defined API (feasibility already validated)
  - ECOS infrastructure already in place (Conan package, dependency, CMake integration)
  - Prototype questions require full integration (= implementation itself)
  - Better validation path: Implement incrementally with unit tests (TDD approach)
- **Validation strategy**:
  - P1 (Formulation correctness): Unit tests for CSC conversion, cone spec, buildECOSProblem
  - P2 (Convergence): Measure iteration count during integration tests (deferred to implementation)
  - P3 (Performance): Zero regression tests + optional benchmarking (deferred to implementation)
- **Result**: VALIDATED — proceed to implementation with TDD validation approach

### Prototype Review and Approval (2026-01-31)
- **Reviewed by**: Human
- **Status**: APPROVED
- **Feedback**: Prototype results approved, continue advancing ticket
- **Notes**: Implementation-first approach with TDD validation approved. 4-phase implementation plan documented in prototype-results.md (7-11 days estimated effort).
- **Next phase**: Implementation

---

## Human Feedback

### ✓ Addressed Feedback

**FB-1: SOCP Solver Feasibility Investigation** (2026-01-31)
> I would like to evaluate the feasibility of using the Splitting Conic Solver library to directly solve the non-convex optimization problem rather than using an approximation.

**Resolution**: Feasibility study completed comparing SCS, ECOS, and box-LCP. ECOS selected — see FB-2.

### Active Feedback

**FB-2: ECOS Selected as Solver** (2026-01-31)
> Let's use ECOS. This project is an educational one and licensing is not expected to be an issue.

**Status**: COMPLETED — Design revised for ECOS approach
**Infrastructure completed**:
- Created Conan package: `conan/ecos/conanfile.py` (builds from source via git clone, v2.0.10)
- Added `ecos/2.0.10` to project requirements in `conanfile.py`
- Added `find_package(ecos)` and `ecos::ecos` link target to `msd-sim/CMakeLists.txt`
**Design revision completed**:
- Replaced box-LCP ASM design with ECOS SOCP formulation
- New components: ECOSData (RAII wrapper), ECOSSparseMatrix (CSC conversion), FrictionConeSpec (cone specification)
- ECOS uses interior-point method (5-15 iterations typical) instead of ASM outer/inner loop
- Exact Coulomb cone (0% error) instead of box approximation (29% error)
- Design artifacts updated:
  - `docs/designs/0035b_box_constrained_asm_solver/design.md` — Revised to describe ECOS integration
  - `docs/designs/0035b_box_constrained_asm_solver/0035b_box_constrained_asm_solver.puml` — Updated diagram showing ECOS components
**Ready for**: Design review and prototype phase
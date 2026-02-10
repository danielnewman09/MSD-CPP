# Ticket 0052d: Solver Integration

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
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

Integrate the `FrictionConeSolver` into the existing `ConstraintSolver` dispatch architecture. Replace the ECOS dispatch path with the custom solver. Remove the ECOS dependency from the build system.

---

## Motivation

The custom solver (0052c) is self-contained and tested in isolation. This subtask wires it into the simulation pipeline:
- `ConstraintSolver::solveWithContacts()` dispatches to `FrictionConeSolver` when contacts have nonzero friction
- The 3C×3C effective mass matrix and RHS are assembled from normal + tangential constraint Jacobians
- Warm starting uses ContactCache from the previous frame
- The ECOS code and Conan dependency are removed

---

## Technical Approach

### Dispatch Logic

In `ConstraintSolver::solveWithContacts()`:
1. If any contact has μ > 0: build 3C×3C system (normal + 2 tangential per contact), dispatch to `FrictionConeSolver`
2. If all contacts have μ = 0: use existing ASM path (normal-only, C×C system)

### Effective Mass Matrix Assembly

Extend the existing `A = J M⁻¹ Jᵀ` assembly to include friction Jacobian rows:
- Each contact contributes 3 rows to J: `[J_n; J_t1; J_t2]`
- The 3C×3C matrix A has 3×3 block structure per contact pair
- RHS `b` includes restitution terms for normal rows and zero for tangential rows (friction opposes relative motion, not a target velocity)

### Warm Starting Pipeline

1. ContactCache provides previous frame's impulse map (keyed by contact pair)
2. For each current contact, look up corresponding previous contact
3. Build `lambda0` vector from matched impulses; use zero for new contacts
4. Pass to `FrictionConeSolver::solve()` as initial guess

### ECOS Removal

- Remove `msd-sim/src/Physics/Constraints/ECOS/` directory
- Remove ECOS from `conanfile.py` dependencies
- Remove ECOS-related build targets from `CMakeLists.txt`
- Verify clean build with no ECOS references

---

## Acceptance Criteria

- [ ] **AC1**: Friction contacts dispatch to `FrictionConeSolver` (not ASM or ECOS)
- [ ] **AC2**: Normal-only contacts still dispatch to ASM (no regression)
- [ ] **AC3**: Warm starting active when ContactCache has matching contacts from previous frame
- [ ] **AC4**: 3C×3C effective mass matrix correctly assembled (verified by existing test infrastructure)
- [ ] **AC5**: ECOS dependency fully removed (build succeeds without ECOS, no ECOS references in source)
- [ ] **AC6**: All existing normal-contact tests pass (zero regressions)
- [ ] **AC7**: End-to-end friction scenario (inclined plane) produces physically correct result

---

## Dependencies

- **Requires**: 0052c (Newton solver core)
- **Blocks**: 0052e (validation suite — needs integration for end-to-end tests)

---

## Files

### Modified Files

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` | Add FrictionConeSolver dispatch |
| `msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` | Implement friction dispatch, warm start assembly |
| `msd-sim/CMakeLists.txt` | Add new files, remove ECOS files |
| `conanfile.py` | Remove ECOS dependency |

### Removed Files

| File | Reason |
|------|--------|
| `msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp` | ECOS replaced |
| `msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.cpp` | ECOS replaced |
| `msd-sim/src/Physics/Constraints/ECOS/ECOSSolver.hpp` | ECOS replaced |
| `msd-sim/src/Physics/Constraints/ECOS/ECOSSolver.cpp` | ECOS replaced |
| `msd-sim/src/Physics/Constraints/ECOS/ECOSTypes.hpp` | ECOS replaced |
| `msd-sim/test/Physics/Constraints/ECOS/*` | ECOS tests replaced by FrictionConeSolver tests |

---

## References

- 0052c (Newton solver core)
- 0040d (ContactCache warm starting)
- Existing `ConstraintSolver` dispatch architecture (Tickets 0031, 0034)

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-10
- **Notes**: Integration subtask. Depends on solver core being complete and tested in isolation.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

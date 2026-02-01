# Ticket 0035e: ECOS SOCP Mathematical Derivation

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Draft
**Assignee**: N/A
**Created**: 2026-02-01
**Generate Tutorial**: No
**Parent Ticket**: [0035_friction_constraints](0035_friction_constraints.md)

---

## Summary

Create a consolidated mathematical derivation document that traces the complete path from the Coulomb friction cone constraint to the ECOS SOCP standard form matrices (G, h, c). Currently the mathematical justification for the ECOS problem construction is fragmented across the design document, ticket 0035b3, code comments, and external references. This ticket produces a single, self-contained derivation that a reader can follow end-to-end without consulting multiple sources.

---

## Motivation

### Current State

The mathematical basis for the G matrix, h vector, and c vector construction in `ECOSProblemBuilder` is spread across:

1. **Design document** ([design.md](../docs/designs/0035b_box_constrained_asm_solver/design.md), lines ~415-486) — High-level ECOS standard form and cone constraint overview
2. **Ticket 0035b3** ([0035b3_ecos_problem_construction.md](0035b3_ecos_problem_construction.md), lines ~73-93) — Practical G matrix construction per contact
3. **Code comments** ([ECOSProblemBuilder.hpp](../msd/msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp), lines 37-51) — Inline derivation of slack variable algebra
4. **Friction cone math** ([M3-coulomb-cone.md](../docs/designs/0035_friction_constraints/M3-coulomb-cone.md)) — Coulomb cone formulation and approximation analysis
5. **External references** — ECOS paper, Boyd & Vandenberghe Ch. 4, Drake/MuJoCo implementations

The design document explicitly acknowledges this gap (lines ~476-485), stating the exact formulation would be detailed in the implementation with references to external sources.

### Problem

A developer or reviewer reading `ECOSProblemBuilder.cpp` cannot currently answer "why does G have this structure?" without assembling pieces from 4+ documents and external references. The derivation chain is:

1. Coulomb friction cone: $\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n$
2. SOCP standard form: $\min c^T x$ s.t. $Gx + s = h$, $s \in \mathcal{K}$
3. Mapping friction to SOCP: defining slack variables, choosing sign conventions
4. Resulting matrix entries: why $G_{3i,3i} = -\mu_i$, $G_{3i+1,3i+1} = -1$, $G_{3i+2,3i+2} = -1$
5. Why $h = 0$ and $c = 0$

None of this is documented as a single coherent chain.

### What This Enables

- Reviewers can validate the ECOS formulation correctness without reverse-engineering from code
- Future modifications (e.g., adding equality constraints in 0035b4) have a clear mathematical reference
- The derivation serves as a bridge between the physics-level math (M3-M5) and the implementation-level code

---

## Technical Approach

### Output

A single document at `docs/designs/0035b_box_constrained_asm_solver/ecos-socp-derivation.md` containing:

### Required Sections

1. **Problem Statement** — State the friction contact problem in physics notation: given effective mass matrix $A$, RHS vector $b$, and friction coefficients $\mu_i$, find impulse vector $\lambda$ satisfying $A\lambda = b$ subject to Coulomb cone constraints $\|\boldsymbol{\lambda}_{t_i}\| \leq \mu_i \lambda_{n_i}$ for each contact $i$.

2. **ECOS Standard Form** — Define the SOCP standard form that ECOS solves: $\min c^T x$ s.t. $Ax = b$, $Gx + s = h$, $s \in \mathcal{K}$, where $\mathcal{K}$ is a product of second-order cones. Cite the ECOS paper (Domahidi et al., 2013) for conventions.

3. **Variable Identification** — Map our physics variables to ECOS variables: decision variable $x = \lambda$ (the 3C impulse vector), cone $\mathcal{K} = \mathcal{Q}_3 \times \mathcal{Q}_3 \times \cdots$ (C copies of 3D second-order cones).

4. **Cone Constraint Derivation** — For a single contact $i$ with variables $[\lambda_{n_i}, \lambda_{t1_i}, \lambda_{t2_i}]$:
   - State the Coulomb constraint: $\|[\lambda_{t1}, \lambda_{t2}]\|_2 \leq \mu_i \lambda_n$
   - Rewrite as SOC: $[\mu_i \lambda_n, \lambda_{t1}, \lambda_{t2}] \in \mathcal{Q}_3$
   - Define slack variables: $s = h - G\lambda$
   - Derive $s_0 = \mu_i \lambda_n$ requires $G_{3i,3i} = -\mu_i$ (since $s_0 = 0 - (-\mu_i)\lambda_n$)
   - Derive $s_1 = \lambda_{t1}$ requires $G_{3i+1,3i+1} = -1$
   - Derive $s_2 = \lambda_{t2}$ requires $G_{3i+2,3i+2} = -1$
   - Conclude $h = 0$ (no offset in friction cone)

5. **Multi-Contact Extension** — Show the block-diagonal structure of G for C contacts and the cone_sizes array $[3, 3, \ldots, 3]$.

6. **Equality Constraints** — Explain how $A\lambda = b$ maps to ECOS equality constraints (deferred to 0035b4 for implementation, but derive the formulation here).

7. **Objective Function** — Derive why $c = 0$: the problem is a feasibility problem (find any $\lambda$ satisfying both the cone and equality constraints), not an optimization.

8. **Concrete Numerical Example** — Work through a single-contact case ($\mu = 0.5$, specific $A$ and $b$) showing the exact G, h, c, cone_sizes values and verifying the constraint is correctly encoded. Cross-reference with `ECOSProblemBuilderTest.SingleContactGMatrix`.

9. **Sign Convention Summary** — Table summarizing ECOS sign conventions and how they map to our physics sign conventions.

---

## Acceptance Criteria

- [ ] **AC1**: Document traces complete derivation from Coulomb cone to G matrix entries without gaps
- [ ] **AC2**: Each matrix entry ($G_{3i,3i} = -\mu_i$, $G_{3i+k,3i+k} = -1$) is individually derived with explicit algebra
- [ ] **AC3**: $h = 0$ and $c = 0$ are justified (not just stated)
- [ ] **AC4**: At least one concrete numerical example is worked through end-to-end
- [ ] **AC5**: ECOS sign conventions ($s = h - Gx$ vs $Gx + s = h$) are explicitly stated and reconciled
- [ ] **AC6**: Document is self-contained (reader does not need to consult external sources to follow the derivation)
- [ ] **AC7**: Cross-references to existing code (`ECOSProblemBuilder.hpp`, `ECOSProblemBuilder.cpp`) with line numbers

---

## Dependencies

- **Requires**: [0035b3](0035b3_ecos_problem_construction.md) (implementation must exist to cross-reference)
- **Requires**: [0035b design](../docs/designs/0035b_box_constrained_asm_solver/design.md) (ECOS standard form section)
- **Requires**: [M3-coulomb-cone.md](../docs/designs/0035_friction_constraints/M3-coulomb-cone.md) (Coulomb friction cone formulation)
- **Blocks**: None (documentation-only ticket)

---

## Files

### New Files

| File | Purpose |
|------|---------|
| `docs/designs/0035b_box_constrained_asm_solver/ecos-socp-derivation.md` | Consolidated SOCP derivation document |

### Modified Files

| File | Change |
|------|--------|
| `docs/designs/0035b_box_constrained_asm_solver/design.md` | Add cross-reference to derivation document in "ECOS Problem Construction" section |

---

## Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Derivation reveals a sign convention error in implementation | High | Validates correctness — fix implementation if needed |
| Document becomes stale if 0035b4 changes formulation | Low | 0035b4 adds equality constraints, does not change cone formulation |

---

## References

- Domahidi, A., Chu, E., & Boyd, S. (2013). "ECOS: An SOCP Solver for Embedded Systems." European Control Conference.
- Boyd, S. & Vandenberghe, L. (2004). *Convex Optimization*, Chapter 4 (Second-Order Cone Programming).
- [M3-coulomb-cone.md](../docs/designs/0035_friction_constraints/M3-coulomb-cone.md) — Coulomb friction cone formulation
- [ECOSProblemBuilder.hpp](../msd/msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp) — Implementation with inline derivation
- [design.md](../docs/designs/0035b_box_constrained_asm_solver/design.md) — ECOS problem construction section

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-01
- **Notes**: Created to address documentation gap identified during 0035b3 implementation review. The mathematical justification for the ECOS G, h, c matrices is currently fragmented across 4+ documents and code comments. This ticket consolidates it into a single self-contained derivation.

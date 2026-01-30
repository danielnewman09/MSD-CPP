# Design Review Context for Ticket 0032

## Current Situation

The design phase for ticket 0032 (Contact Constraint Refactor) has been completed. The design documents exist at:
- `docs/designs/0032_contact_constraint_refactor/math-formulation.md`
- `docs/designs/0032_contact_constraint_refactor/design.md`
- `docs/designs/0032_contact_constraint_refactor/0032_contact_constraint_refactor.puml`

## Human Feedback on Design (from ticket)

The human has provided the following feedback that needs to be addressed in the design review:

### Question 1: Multiple Contact Points
> In the math design section 2.2, you have one contact point per object. Is there a reason to constrain this to one contact point compared to if we have multiple contact points (e.g. two faces colliding)? Or does this generalize to multiple contact points?

### Question 2: Jacobian Derivation
> In 3.3, can you elaborate on how ∂C/∂X_A = -n^T? I think I can get there, but the intuition isn't immediately clear to me.

### Question 3: Angular Position vs Rates
> Can you elaborate on section 3.4? It's unclear to me where ∂ω comes from, considering we're using the angular positions, not rates.

### Question 4: Environment Mass
> Noting the special case with the environment, I suspect we'll want to slightly modify the AssetEnvironment such that it has a mass that can drive the inverse mass to zero.

## Prototype Results (Already Completed)

Unusually, three prototypes have already been run BEFORE design review:

### P1: PGS Convergence (Stacked Objects)
- **Result**: PASS 4/4 criteria
- **Key Finding**: Design document specifies Baumgarte parameter as `alpha=100` [1/s²] (acceleration-level), but PGS solver implementation uses Error Reduction Parameter (ERP) formulation. Correct conversion: `ERP = alpha * dt²`, so `alpha=100` → `ERP=0.0256`. Prototype found optimal range is ERP=0.2-1.0.
- **Design Impact**: Design should specify parameters in BOTH formulations (acceleration-level for theory, ERP for implementation)
- **Location**: `prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/`
- **Debug Findings**: `prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/Debug_Findings.md`

### P2: Energy Conservation (Bouncing Ball)
- **Result**: Originally FAIL 0/5, but DEBUG ANALYSIS found single-line bug in PROTOTYPE CODE
- **Key Finding**: Prototype confused constraint RHS `b = -(1+e)·Jq̇⁻` with target velocity `v_target = -e·v_pre`. The bug was in the prototype, NOT the design. After fixing prototype code, all 5 criteria PASS.
- **Design Impact**: Design formulation is CORRECT. Math document should clarify distinction between constraint RHS and target velocity.
- **P2 "DO NOT PROCEED" Recommendation**: INVALID - should be withdrawn after fix
- **Location**: `prototypes/0032_contact_constraint_refactor/p2_energy_conservation/`
- **Debug Findings**: `prototypes/0032_contact_constraint_refactor/p2_energy_conservation/Debug_Findings.md`

### P3: Performance Comparison
- **Result**: PARTIAL PASS 3/4 criteria
- **Key Finding**: PGS is 11.2x slower for N=1 (fails 5x target), but N=20 at 3.71µs is well under 1ms budget. Absolute overhead is negligible (< 4µs even for N=20).
- **Design Impact**: Performance is acceptable for real-time use. Consider hybrid approach (direct impulse for N=1, PGS for N≥2) as future optimization.
- **Location**: `prototypes/0032_contact_constraint_refactor/p3_performance/`
- **Results**: `prototypes/0032_contact_constraint_refactor/p3_performance/README.md`

## Implications for Design Review

1. The design has been VALIDATED by prototypes (with prototype code corrections, not design corrections)
2. Human feedback questions should be answered by reviewing the design documents
3. P1 finding suggests design document should add ERP parameter specification
4. P2 finding suggests math document should clarify constraint RHS vs target velocity
5. Performance is acceptable per P3
6. The review should determine if the design is ready to proceed to implementation given:
   - Human questions need answers
   - Minor documentation clarifications needed (ERP parameters, constraint RHS clarification)
   - Prototypes validate the approach

## Next Steps After Review

Based on review outcome:
- If APPROVED WITH NOTES: Update ticket, proceed to implementation (prototypes already done)
- If NEEDS REVISION: Address issues, re-review
- If BLOCKED: Escalate to human

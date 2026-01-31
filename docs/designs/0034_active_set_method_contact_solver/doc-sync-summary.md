# Documentation Sync Summary

## Feature: Active Set Method Contact Solver
**Date**: 2026-01-31
**Target Library**: msd-sim
**Ticket**: [0034_active_set_method_contact_solver](../../../tickets/0034_active_set_method_contact_solver.md)

## Diagrams Synchronized

### Referenced (No Copy Required)
This feature uses a design diagram located in the design folder rather than copying to library documentation. The design diagram is referenced from library documentation.

| Source | Referenced By | Notes |
|--------|---------------|-------|
| `docs/designs/0034_active_set_method_contact_solver/0034_active_set_method_contact_solver.puml` | `msd/msd-sim/CLAUDE.md` Diagrams Index | Design diagram remains in design folder per project convention for internal algorithm replacements |

## CLAUDE.md Updates

### Files Modified
- `msd/msd-sim/CLAUDE.md` — Added Recent Architectural Changes entry, updated Diagrams Index
- `msd/msd-sim/src/Physics/CLAUDE.md` — Updated Constraint System overview
- `msd/msd-sim/src/Physics/Constraints/CLAUDE.md` — Updated ConstraintSolver documentation with Active Set Method algorithm, updated UnilateralConstraint implementation status, updated Current Limitations section

### Sections Added

#### msd/msd-sim/CLAUDE.md
- **Recent Architectural Changes**: New entry "Active Set Method Contact Solver — 2026-01-31"
  - Documents replacement of PGS with ASM
  - Algorithm overview with KKT conditions
  - Benefits over PGS (exact solution, mass ratio robustness, determinism, no iteration tuning)
  - Performance characteristics
  - Public interface changes (backward compatible with semantic changes)
  - Breaking changes (internal only)
  - Key files modified
  - Thread safety, memory management, error handling notes

#### msd/msd-sim/src/Physics/CLAUDE.md
- **Constraint System overview**: Updated to mention Active Set Method for contact constraints

#### msd/msd-sim/src/Physics/Constraints/CLAUDE.md
- **ConstraintSolver Purpose**: Updated to document two solver methods (bilateral via LLT, contact via ASM)
- **Algorithm Overview (Contact Constraints - Active Set Method)**: New subsection documenting ASM algorithm
- **Active Set Method Benefits**: New subsection explaining exact solution, mass ratio robustness, determinism
- **Key Interfaces**: Updated with `MultiBodySolveResult`, `BodyForces`, `solveWithContacts()`, `setMaxIterations()`, `setConvergenceTolerance()`

### Sections Modified

#### msd/msd-sim/CLAUDE.md
- **Diagrams Index**: Added entry for `0034_active_set_method_contact_solver.puml`

#### msd/msd-sim/src/Physics/Constraints/CLAUDE.md
- **UnilateralConstraint Implementation Status**: Changed from "Interface defined, solver not yet implemented" to "Partially implemented" with note about Active Set Method for contact constraints
- **Current Limitations**: Updated to reflect two-body contact constraints implemented, bilateral solver distinction

### Diagrams Index
- Added: `0034_active_set_method_contact_solver.puml` in Diagrams Index table

## Verification

- [x] All diagram links verified (design diagram referenced correctly)
- [x] CLAUDE.md formatting consistent with existing entries
- [x] No broken references
- [x] Library documentation structure complete (Constraints sub-module CLAUDE.md updated)

## Notes

### Design Decision: No Library Diagram Copy
Unlike feature additions (e.g., collision-response.puml copied from design to library), this internal algorithm replacement keeps the design diagram in the design folder only. The diagram shows implementation details (PGSResult removal, ActiveSetResult addition, internal method changes) that are not part of the stable library architecture. The library documentation cross-references the design diagram for detailed algorithm flow.

### Rationale for Documentation Structure
- **msd-sim/CLAUDE.md**: High-level Recent Architectural Changes entry for visibility
- **Physics/CLAUDE.md**: Brief mention in Constraint System overview (defers to Constraints/CLAUDE.md)
- **Constraints/CLAUDE.md**: Detailed algorithm documentation with comparison to previous PGS approach

### Cross-References Established
- msd-sim CLAUDE.md → design diagram (Diagrams Index)
- msd-sim CLAUDE.md → ticket (Recent Architectural Changes)
- Physics CLAUDE.md → Constraints/CLAUDE.md (sub-module delegation)
- Constraints CLAUDE.md → detailed algorithm and interface documentation

### Semantic Changes Documented
The documentation explicitly notes the semantic changes to public methods:
- `setMaxIterations()`: Now sets safety cap (default 100) vs iteration budget (was 10)
- `setConvergenceTolerance()`: Now sets violation threshold (default 1e-6) vs PGS convergence (was 1e-4)
- `MultiBodySolveResult::iterations`: Now counts active set changes vs PGS iterations
- `MultiBodySolveResult::residual`: Now dual residual norm (complementarity measure) vs PGS convergence metric

These changes maintain backward compatibility (same signatures) but have different semantics, which is now clearly documented.

# Documentation Sync Summary

## Feature: 0035b3_ecos_problem_construction
**Date**: 2026-02-01
**Target Library**: msd-sim
**Sub-Module**: Physics/Constraints/ECOS

---

## Diagrams Synchronized

### Created
| Destination | Purpose | Changes |
|-------------|---------|---------|
| `docs/msd/msd-sim/Physics/Constraints/ecos-problem-builder.puml` | ECOSProblemBuilder component diagram | New diagram showing contact constraint to ECOS SOCP conversion workflow, G matrix construction, and mathematical formulation |

### Not Copied from Design
This ticket (0035b3) is a sub-ticket of the parent feature 0035b_box_constrained_asm_solver. The parent design diagram at `docs/designs/0035b_box_constrained_asm_solver/0035b_box_constrained_asm_solver.puml` shows the overall ECOS integration architecture. Since ECOSProblemBuilder is an implementation detail within the ECOS module, a new specialized diagram was created directly in the library documentation rather than copying from the design folder.

**Rationale**: The parent design shows `buildECOSProblem()` as a method in `ConstraintSolver`, but the implementation created a separate `ECOSProblemBuilder` utility class. The new diagram accurately reflects this implementation decision and provides detailed documentation of the G matrix construction algorithm.

---

## CLAUDE.md Updates

### Files Modified
| File | Changes |
|------|---------|
| `msd/msd-sim/src/Physics/Constraints/ECOS/CLAUDE.md` | Added ECOSProblemBuilder and FrictionConeSpec component documentation |

### Sections Added

#### Core Components Table
- Added `FrictionConeSpec` entry
- Added `ECOSProblemBuilder` entry

#### FrictionConeSpec Section (New)
- **Location**: After ECOSSparseMatrix, before ECOSData
- **Content**: Purpose, key interfaces, usage example, thread safety, error handling, memory management
- **Key details**: Friction cone specification for SOCP, getters with bounds checking, Rule of Zero semantics

#### ECOSProblemBuilder Section (New)
- **Location**: After ECOSData, before ECOS Exit Codes
- **Content**: Purpose, key interfaces, mathematical formulation, usage example, thread safety, error handling, memory management, performance
- **Key details**:
  - Converts contact constraint system to ECOS conic form
  - Documents G matrix block-diagonal structure with -μ_i and -1 entries
  - Explains why negative signs produce correct cone constraints
  - Includes h=0 and c=0 vector assignments
  - Notes that equality constraints (A_eq, b_eq) will be added in ticket 0035b4

### Sections Modified

#### Integration with ConstraintSolver
- **Before**: "Future integration (ticket 0035b3)"
- **After**: "Current integration status (as of ticket 0035b3)"
- Updated workflow to show ECOSProblemBuilder::build() as implemented
- Added "Current capabilities" checklist showing 0035b1, 0035b2, 0035b3 complete
- Added "Remaining work" section for ticket 0035b4

#### Test Organization
- Added `FrictionConeSpecTest.cpp` — 8 tests
- Added `ECOSProblemBuilderTest.cpp` — 14 tests
- Updated test coverage section with detailed breakdown of ECOSProblemBuilder tests

#### References - Tickets
- Added link to `0035b3_ecos_problem_construction.md`

### Diagrams Index
No changes to main msd-sim CLAUDE.md diagrams index. The ECOS module is documented in its own CLAUDE.md file, which now references the new `ecos-problem-builder.puml` diagram.

---

## Verification

- [x] All diagram links verified
- [x] CLAUDE.md formatting consistent
- [x] No broken references
- [x] ECOS module documentation structure complete
- [x] Diagram path uses relative links: `../../../../../../../../docs/msd/msd-sim/Physics/Constraints/ecos-problem-builder.puml`

---

## Notes

### Implementation vs Design
The design document (parent ticket 0035b) showed `buildECOSProblem()` as a private method in `ConstraintSolver`. The implementation created a separate `ECOSProblemBuilder` utility class with static methods instead. This is a better design because:
- Separation of concerns: Problem construction is isolated from solver logic
- Testability: ECOSProblemBuilder can be unit tested independently
- Reusability: Could be used by other solvers if needed

The new `ecos-problem-builder.puml` diagram documents this implementation decision.

### FrictionConeSpec Extension
During implementation, `FrictionConeSpec` was extended with getter methods:
- `getNumContacts()` — Returns number of contacts
- `getFrictionCoefficient(int contactIndex)` — Returns μ for contact with bounds checking

These additions were necessary for ECOSProblemBuilder to read friction coefficients. The CLAUDE.md now documents these getters.

### Documentation Structure
The ECOS module follows the project's hierarchical documentation pattern:
- `msd/msd-sim/CLAUDE.md` — High-level library architecture (no ECOS details)
- `msd/msd-sim/src/Physics/CLAUDE.md` — Physics module overview (references Constraints)
- `msd/msd-sim/src/Physics/Constraints/CLAUDE.md` — Constraints module (references ECOS)
- `msd/msd-sim/src/Physics/Constraints/ECOS/CLAUDE.md` — **Detailed ECOS component documentation** (updated in this sync)

This keeps the main library documentation focused while providing detailed component documentation where needed.

### No Design Diagram Copy Required
Unlike typical feature documentation syncs that copy diagrams from `docs/designs/{feature}/` to `docs/msd/{library}/`, this ticket created a new diagram directly in the library documentation. This is appropriate because:
1. The parent ticket (0035b) provides the overall architecture diagram
2. ECOSProblemBuilder is an implementation detail, not a top-level feature
3. A specialized diagram better documents the complex G matrix construction algorithm

### Equality Constraints Deferred
The CLAUDE.md notes that equality constraints (A_eq, b_eq encoding A·λ = b) will be added in ticket 0035b4 (ECOS solve integration). The current implementation (0035b3) focuses on cone constraints (G, h, cone_sizes).

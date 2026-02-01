# Documentation Sync Summary

## Feature: 0035b2_ecos_data_wrapper
**Date**: 2026-02-01
**Target Library**: msd-sim (Physics/Constraints/ECOS sub-module)

## Diagrams Synchronized

### Copied/Created
| Source | Destination | Changes |
|--------|-------------|---------|
| `docs/designs/0035b2_ecos_data_wrapper/0035b2_ecos_data_wrapper.puml` | `docs/msd/msd-sim/Physics/Constraints/ecos-data.puml` | Renamed file to match component name, no structural changes (diagram is feature-agnostic) |

### Updated
| File | Changes |
|------|---------|
| N/A | No existing library diagrams required updates (new sub-module) |

## CLAUDE.md Updates

### Sections Added
- `msd/msd-sim/src/Physics/Constraints/ECOS/CLAUDE.md` — Complete ECOS sub-module documentation
  - Module Overview
  - Architecture Overview
  - Component Details (ECOSSparseMatrix, ECOSData, ECOSWorkspaceDeleter)
  - ECOS Exit Codes
  - Integration with ConstraintSolver (future)
  - Design Rationale
  - Testing
  - Coding Standards
  - References

### Sections Modified
- `msd/msd-sim/src/Physics/Constraints/CLAUDE.md`:
  - Added ECOS module to Core Components table
  - Added Sub-Modules table with link to ECOS/CLAUDE.md

## Diagrams Index

New diagram entries:
- `docs/msd/msd-sim/Physics/Constraints/ecos-data.puml` — ECOSData RAII wrapper architecture

## Verification

- [x] All diagram links verified
- [x] CLAUDE.md formatting consistent
- [x] No broken references
- [x] Library documentation structure complete

## Notes

### Integration Context
The ECOS module is a utilities sub-module under Constraints that provides integration with the ECOS C library. It is not currently integrated into the active ConstraintSolver but will be used by the future box-constrained Active Set Method solver (ticket 0035b3).

### Documentation Structure
Created new sub-module documentation at `msd/msd-sim/src/Physics/Constraints/ECOS/CLAUDE.md` following the established pattern:
- Module overview with parent context
- Architecture overview with component table
- Detailed component documentation (ECOSSparseMatrix, ECOSData)
- Critical design constraints (ECOS Equilibration Constraint)
- Integration points (future ConstraintSolver usage)
- Testing and coding standards

### Diagram Treatment
The feature diagram was copied without modification to the library docs directory. The diagram is already written in a library-agnostic style (no "new/modified" highlighting, focused on component structure rather than change context).

### Cross-References
- Links to parent ticket (0035b_box_constrained_asm_solver)
- Links to sibling ticket (0035b1_ecos_utilities for ECOSSparseMatrix)
- Links to design documents for both components
- Links to ECOS external documentation

### Key Design Decisions Documented
1. **ECOS Equilibration Constraint**: Critical member ordering and custom move assignment requirement
2. **std::unique_ptr usage**: Rationale for using unique_ptr with custom deleter instead of raw pointer
3. **Move-only semantics**: Why `= default` is unsafe for move assignment
4. **Two-phase construction**: setup() separate from constructor to allow data population

### Future Integration
Documented that ECOSData will be used by the box-constrained ASM solver in ticket 0035b3 for solving contact LCPs with friction as SOCPs.

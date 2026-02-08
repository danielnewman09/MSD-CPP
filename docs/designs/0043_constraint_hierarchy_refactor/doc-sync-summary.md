# Documentation Sync Summary

## Feature: 0043_constraint_hierarchy_refactor
**Date**: 2026-02-08
**Target Library**: msd-sim (Constraints subsystem)

## Diagrams Synchronized

### Existing (Referenced, Not Copied)
| Source | Status | Notes |
|--------|--------|-------|
| `docs/designs/0043_constraint_hierarchy_refactor/0043_constraint_hierarchy_refactor.puml` | Already exists | Design diagram showing flattened 2-level hierarchy |

**Note**: This refactor does not require copying diagrams from design to library documentation. The existing design diagram at `docs/designs/0043_constraint_hierarchy_refactor/0043_constraint_hierarchy_refactor.puml` is referenced directly from all updated documentation, as this is a refactoring ticket documenting an architectural change rather than a new feature requiring library-level diagrams.

## CLAUDE.md Updates

### Sections Added

**msd/msd-sim/src/Physics/Constraints/CLAUDE.md**:
- Updated "Constraint Hierarchy" section with new flat 2-level hierarchy diagram and description
- Added "LambdaBounds Value Type" subsection documenting multiplier bounds encoding
- Updated "Implementing Custom Constraints" section with:
  - Single-body constraint example showing unified `evaluate(stateA, stateB, time)` signature
  - Two-body constraint example showing `bodyCount()` override and 12-DOF Jacobian
  - `LambdaBounds` factory method usage in examples

**msd/msd-sim/src/Physics/CLAUDE.md**:
- Updated "Constraint System" section replacing references to removed intermediate classes
- Added `LambdaBounds` to key components list
- Updated concrete implementations list to include `FrictionConstraint`
- Updated "Adding Custom Constraints" example to use new unified signatures

**msd/msd-sim/CLAUDE.md**:
- Added "Constraint Hierarchy Refactor — 2026-02-08" entry to "Recent Architectural Changes" section
- Detailed architectural changes: removed classes, new components, method renames, performance characteristics
- Listed all affected files (headers, sources, tests)

### Sections Modified

**msd/msd-sim/src/Physics/Constraints/CLAUDE.md**:
- Replaced old 3-level hierarchy diagram with flat 2-level hierarchy
- Removed references to `BilateralConstraint` and `UnilateralConstraint` as constraint base types
- Updated custom constraint implementation pattern to inherit directly from `Constraint`

**msd/msd-sim/src/Physics/CLAUDE.md**:
- Replaced "Abstract `Constraint` interface with `BilateralConstraint` and `UnilateralConstraint` specializations" with flat hierarchy description
- Updated example code from `BilateralConstraint` base to `Constraint` base with `lambdaBounds()` override

**msd/msd-sim/CLAUDE.md**:
- Added entry to Diagrams Index table referencing `0043_constraint_hierarchy_refactor.puml`

## Verification

- [x] All diagram links verified (single diagram exists at design path)
- [x] CLAUDE.md formatting consistent
- [x] No broken references
- [x] Constraint hierarchy documentation complete across all three CLAUDE.md files
- [x] Recent Architectural Changes entry added with full context
- [x] Diagrams Index updated

## Notes

**Design diagram location**: The refactor is documented via a single PlantUML diagram at `docs/designs/0043_constraint_hierarchy_refactor/0043_constraint_hierarchy_refactor.puml`. This diagram is referenced from:
- `msd/msd-sim/src/Physics/Constraints/CLAUDE.md` (Constraint Hierarchy section)
- `msd/msd-sim/src/Physics/CLAUDE.md` (Constraint System section)
- `msd/msd-sim/CLAUDE.md` (Recent Architectural Changes entry and Diagrams Index)

**Historical documentation preservation**: References to `BilateralConstraint`, `UnilateralConstraint`, and `TwoBodyConstraint` in the "Recent Architectural Changes" section of `msd/msd-sim/CLAUDE.md` were intentionally preserved. These entries document historical tickets (0031, 0032a) that introduced those classes before they were removed in ticket 0043. The historical record remains intact.

**No library-level diagrams created**: Unlike feature tickets that add new components requiring library documentation diagrams (e.g., `docs/msd/msd-sim/Physics/`), this refactoring ticket documents an architectural simplification. The design diagram is sufficient and is referenced directly from all documentation locations.

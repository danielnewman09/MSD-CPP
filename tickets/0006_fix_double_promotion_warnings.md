# Feature Ticket: Fix Double-Promotion Warnings

## Status
- [x] Draft
- [x] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-07
- **Author**: Claude Code
- **Priority**: Medium
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-assets, msd-sim, msd-gui

---

## Summary
Fix 217 compiler warnings triggered by `-Wdouble-promotion` flag across multiple files. The warnings occur when `float` values are implicitly converted to `double`, typically when constructing `Eigen::Vector3d` objects or calling functions that expect `double` parameters.

## Motivation
The `-Wdouble-promotion` warning catches implicit float-to-double conversions that may indicate:
- Unintentional precision changes
- Performance overhead from mixed float/double arithmetic
- Type mismatches between GPU data (typically float) and CPU simulation (typically double)

Fixing these warnings ensures intentional type usage and prevents subtle precision bugs.

## Requirements

### Functional Requirements
1. The system shall compile `GeometryFactory.cpp` without `-Wdouble-promotion` warnings
2. The system shall use consistent numeric precision for vertex data construction

### Non-Functional Requirements
- **Performance**: No runtime overhead from unnecessary type conversions
- **Memory**: N/A
- **Thread Safety**: N/A
- **Backward Compatibility**: No API changes

## Constraints
- `Eigen::Vector3d` uses `double` as its scalar type
- Current code passes `float` values (e.g., `half` variable) to `Vector3d` constructor

## Acceptance Criteria
- [ ] All source files compile without `-Wdouble-promotion` warnings
- [ ] All test files compile without `-Wdouble-promotion` warnings
- [ ] All existing tests pass
- [ ] Solution uses explicit casts or changes variable types appropriately

---

## Design Decisions (Human Input)

### Preferred Approaches
- Option A: Change `half` variable from `float` to `double` (aligns with Eigen's Vector3d)
- Option B: Use `static_cast<double>()` for each float value passed to Vector3d

### Things to Avoid
- Do not use C-style casts
- Do not suppress warning with pragmas

### Open Questions
1. Should `GeometryFactory` use `double` throughout for consistency with Eigen, or maintain `float` for GPU compatibility?

---

## References

### Related Code

**Source files (59 warnings):**
- `msd/msd-assets/src/GeometryFactory.cpp` — 39 warnings (cube vertex construction)
- `msd/msd-gui/src/SDLApp.cpp` — 9 warnings
- `msd/msd-sim/src/Environment/MotionController.cpp` — 7 warnings
- `msd/msd-gui/src/Camera3D.cpp` — 2 warnings

**Test files (158 warnings):**
- `msd/msd-sim/test/Physics/ConvexHullTest.cpp` — 119 warnings
- `msd/msd-assets/test/GeometryDatabaseTest.cpp` — 39 warnings

### Related Documentation
- N/A

### Related Tickets
- 0007_fix_sign_conversion_warnings
- 0008_fix_shadow_warnings
- 0009_fix_shorten_64_to_32_warnings
- 0010_fix_implicit_float_conversion_warnings

---

## Workflow Log

### Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0006_fix_double_promotion_warnings/design.md`
  - `docs/designs/0006_fix_double_promotion_warnings/0006_fix_double_promotion_warnings.puml`
- **Notes**:

### Design Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Prototype Phase
- **Started**:
- **Completed**:
- **Prototypes**:
- **Artifacts**:
- **Notes**:

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
- **Artifacts**:
- **Notes**:

### Implementation Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Documentation Update Phase
- **Started**:
- **Completed**:
- **CLAUDE.md Updates**:
- **Diagrams Indexed**:
- **Notes**:

---

## Human Feedback

### Feedback on Design
{Your comments on the design}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}

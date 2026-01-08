# Feature Ticket: Fix Implicit-Float-Conversion Warnings

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review (SKIPPED - no architectural changes)
- [x] Design Approved — Ready for Prototype (SKIPPED - no architectural changes)
- [x] Prototype Complete — Awaiting Review (SKIPPED - no architectural changes)
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-07
- **Author**: Claude Code
- **Priority**: Medium
- **Estimated Complexity**: Small
- **Target Component(s)**: msd-sim, msd-gui

---

## Summary
Fix compiler warnings triggered by `-Wimplicit-float-conversion` flag. This warning occurs when floating-point values are implicitly converted between types (e.g., `double` to `float`), which can cause precision loss.

## Motivation
Implicit float conversions can cause:
- Precision loss when converting `double` to `float`
- Unexpected results in calculations
- Inconsistent behavior between different parts of the codebase

Fixing these warnings ensures intentional type usage and prevents precision-related bugs.

## Requirements

### Functional Requirements
1. The system shall compile without `-Wimplicit-float-conversion` warnings
2. Float/double conversions shall be explicit where precision loss is acceptable

### Non-Functional Requirements
- **Performance**: No impact (may improve if unnecessary conversions are eliminated)
- **Memory**: No impact
- **Thread Safety**: N/A
- **Backward Compatibility**: No API changes

## Constraints
- Some APIs (SDL, GPU) require `float` types
- Eigen uses `double` for `Vector3d`

## Acceptance Criteria
- [x] All files compile without `-Wimplicit-float-conversion` warnings
- [x] All existing tests pass
- [x] Conversions use explicit `static_cast<float>()` or `static_cast<double>()`

---

## Design Decisions (Human Input)

### Preferred Approaches
- Use `static_cast<float>()` when intentionally converting double to float
- Consider using `float` literals (e.g., `1.0f` instead of `1.0`) where appropriate
- Document why precision loss is acceptable in comments if non-obvious

### Things to Avoid
- Do not use C-style casts
- Do not blindly add casts without understanding why the conversion exists

### Open Questions
1. Should test files use `double` throughout to avoid conversions, or use `float` to match GPU data types?

---

## References

### Related Code
- `msd/msd-gui/test/ShaderTransformTest.cpp` — 1 implicit-float-conversion warning (fixed)

**Note**: The ticket originally listed 9 warnings in `ConvexHullTest.cpp`, but investigation showed 0 warnings in that file with `-Wimplicit-float-conversion` enabled. Only 1 warning was found in `ShaderTransformTest.cpp` at line 1087.

### Related Documentation
- N/A

### Related Tickets
- 0006_fix_double_promotion_warnings — Related (opposite direction conversion)
- 0007_fix_sign_conversion_warnings
- 0008_fix_shadow_warnings
- 0009_fix_shorten_64_to_32_warnings

---

## Workflow Log

### Design Phase
- **Started**: SKIPPED
- **Completed**: SKIPPED
- **Artifacts**: None (no architectural changes required)
- **Notes**: This ticket involves simple warning fixes in test files only. No new classes, interfaces, or architectural changes are needed. Following the pattern of ticket 0006.

### Design Review Phase
- **Started**: SKIPPED
- **Completed**: SKIPPED
- **Status**: SKIPPED
- **Reviewer Notes**: No design document to review.

### Prototype Phase
- **Started**: SKIPPED
- **Completed**: SKIPPED
- **Prototypes**: None
- **Artifacts**: None
- **Notes**: No prototype needed for warning fixes.

### Implementation Phase
- **Started**: 2026-01-08
- **Completed**: 2026-01-08
- **Files Created**: None
- **Files Modified**:
  - `msd/msd-gui/test/ShaderTransformTest.cpp` (line 1087)
  - `CMakeLists.txt` (added `-Wimplicit-float-conversion` to compiler flags)
- **Artifacts**: None
- **Notes**:
  - Added explicit `static_cast<float>(M_PI)` at line 1087 to match the pattern used elsewhere in the file
  - Added `-Wimplicit-float-conversion` flag to CMakeLists.txt to catch future occurrences
  - The ticket originally listed 9 warnings in ConvexHullTest.cpp but investigation found 0 warnings
  - Only 1 actual warning was found and fixed

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

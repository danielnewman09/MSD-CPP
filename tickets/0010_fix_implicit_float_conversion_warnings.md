# Feature Ticket: Fix Implicit-Float-Conversion Warnings

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
- [ ] All files compile without `-Wimplicit-float-conversion` warnings
- [ ] All existing tests pass
- [ ] Conversions use explicit `static_cast<float>()` or `static_cast<double>()`

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
- `msd/msd-sim/test/Physics/ConvexHullTest.cpp` — 9 implicit-float-conversion warnings
- `msd/msd-gui/test/ShaderTransformTest.cpp` — 1 implicit-float-conversion warning

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
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0010_fix_implicit_float_conversion_warnings/design.md`
  - `docs/designs/0010_fix_implicit_float_conversion_warnings/0010_fix_implicit_float_conversion_warnings.puml`
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

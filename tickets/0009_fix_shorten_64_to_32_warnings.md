# Feature Ticket: Fix Shorten-64-to-32 Warnings

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
- **Target Component(s)**: msd-gui

---

## Summary
Fix compiler warnings triggered by `-Wshorten-64-to-32` flag. This warning occurs when a 64-bit integer is implicitly converted to a 32-bit integer, which can cause data loss on platforms where sizes differ.

## Motivation
Implicit 64-to-32 bit conversions can cause:
- Data truncation when values exceed 32-bit range
- Platform-specific bugs (code works on 32-bit but fails on 64-bit)
- Undefined behavior in extreme cases

Fixing these warnings ensures portable, correct code across platforms.

## Requirements

### Functional Requirements
1. The system shall compile without `-Wshorten-64-to-32` warnings
2. Integer types shall be explicitly sized where precision matters

### Non-Functional Requirements
- **Performance**: No significant impact
- **Memory**: May slightly increase memory if upgrading to 64-bit types
- **Thread Safety**: N/A
- **Backward Compatibility**: No API changes

## Constraints
- SDL3 API may dictate certain integer sizes
- Graphics APIs often use 32-bit integers for counts/sizes

## Acceptance Criteria
- [ ] All files compile without `-Wshorten-64-to-32` warnings
- [ ] All existing tests pass
- [ ] Solution uses appropriate integer types or explicit casts with bounds checking

---

## Design Decisions (Human Input)

### Preferred Approaches
- Use `static_cast<uint32_t>()` with appropriate bounds checking if values could exceed 32-bit range
- Consider using `gsl::narrow_cast` or similar for checked narrowing
- If the value is guaranteed to fit, document the assumption

### Things to Avoid
- Do not blindly cast without considering overflow
- Do not use C-style casts

### Open Questions
1. Should we add a utility function for checked narrowing conversions?

---

## References

### Related Code
- `msd/msd-gui/src/SDLGPUManager.hpp` — 1 shorten-64-to-32 warning

### Related Documentation
- N/A

### Related Tickets
- 0006_fix_double_promotion_warnings
- 0007_fix_sign_conversion_warnings
- 0008_fix_shadow_warnings
- 0010_fix_implicit_float_conversion_warnings

---

## Workflow Log

### Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0009_fix_shorten_64_to_32_warnings/design.md`
  - `docs/designs/0009_fix_shorten_64_to_32_warnings/0009_fix_shorten_64_to_32_warnings.puml`
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

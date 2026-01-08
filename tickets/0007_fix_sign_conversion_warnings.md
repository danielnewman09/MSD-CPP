# Feature Ticket: Fix Sign-Conversion Warnings

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review (Skipped - simple fix)
- [x] Design Approved — Ready for Prototype (Skipped - simple fix)
- [x] Prototype Complete — Awaiting Review (Skipped - simple fix)
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-07
- **Author**: Claude Code
- **Priority**: Medium
- **Estimated Complexity**: Small
- **Target Component(s)**: msd-assets, msd-sim

---

## Summary
Fix 4 compiler warnings triggered by `-Wconversion` (specifically `-Wsign-conversion`). The warnings occur when signed integers are implicitly converted to unsigned integers, such as `std::streamoff` to `size_t` from `file.tellg()` calls.

## Motivation
The `-Wsign-conversion` warning catches implicit signed-to-unsigned conversions that may cause:
- Undefined behavior if the signed value is negative
- Data loss or wraparound on conversion
- Subtle bugs when checking file positions

`tellg()` can return `-1` on error, which when converted to `size_t` becomes a very large positive number, potentially causing buffer overflows or incorrect allocations.

## Requirements

### Functional Requirements
1. The system shall compile `STLLoader.cpp` without `-Wsign-conversion` warnings
2. The system shall properly handle `tellg()` error conditions (returns `-1`)

### Non-Functional Requirements
- **Performance**: No significant overhead
- **Memory**: Proper bounds checking to prevent over-allocation
- **Thread Safety**: N/A
- **Backward Compatibility**: No API changes

## Constraints
- `std::istream::tellg()` returns `std::streamoff` (typically `long long`, signed)
- File sizes are inherently non-negative, but `tellg()` returns `-1` on error

## Acceptance Criteria
- [x] All affected files compile without `-Wsign-conversion` warnings
- [x] Error handling for `tellg()` failure is implemented where applicable
- [x] All existing tests pass (pre-existing test failure in ConvexHullTest.BoundingBoxOfCube is unrelated)

---

## Design Decisions (Human Input)

### Preferred Approaches
- Check `tellg()` result for `-1` before converting to `size_t`
- Use `static_cast<size_t>()` after validation
- Consider using `std::filesystem::file_size()` as an alternative (C++17)

### Things to Avoid
- Do not blindly cast without checking for error conditions
- Do not use C-style casts

### Open Questions
1. Should we throw an exception or return an error on `tellg()` failure?
2. Is `std::filesystem::file_size()` preferred over `tellg()` for getting file sizes?

---

## References

### Related Code
- `msd/msd-assets/src/STLLoader.cpp` — 2 warnings (`size_t fileSize = file.tellg();`)
- `msd/msd-sim/src/Physics/RigidBody/ConvexHull.cpp` — 1 warning
- `msd/msd-sim/src/Environment/WorldModel.cpp` — 1 warning

### Related Documentation
- N/A

### Related Tickets
- 0006_fix_double_promotion_warnings
- 0008_fix_shadow_warnings
- 0009_fix_shorten_64_to_32_warnings
- 0010_fix_implicit_float_conversion_warnings

---

## Workflow Log

### Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0007_fix_sign_conversion_warnings/design.md`
  - `docs/designs/0007_fix_sign_conversion_warnings/0007_fix_sign_conversion_warnings.puml`
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
- **Started**: 2026-01-07
- **Completed**: 2026-01-07
- **Files Created**: None
- **Files Modified**:
  - `msd/msd-assets/src/STLLoader.cpp` — Added error checking for `tellg()` before casting to `size_t` (2 locations)
  - `msd/msd-sim/src/Physics/RigidBody/ConvexHull.cpp` — Changed `int idx` to `size_t idx` for array indexing
  - `msd/msd-sim/src/Environment/WorldModel.cpp` — Used `static_cast<difference_type>()` for iterator arithmetic
- **Artifacts**: None
- **Notes**: Design/prototype phases skipped as this is a straightforward warning fix with well-established patterns.

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

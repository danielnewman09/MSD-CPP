# Feature Ticket: Fix Shadow Warnings

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
- **Target Component(s)**: msd-sim

---

## Summary
Fix compiler warnings triggered by `-Wshadow` flag. The warning occurs when a local variable shadows a variable in an outer scope, which can lead to confusing bugs where the wrong variable is accidentally used.

## Motivation
Variable shadowing is a common source of bugs:
- Developer intends to use outer variable but accidentally uses shadowed local
- Code becomes harder to reason about when same name means different things
- Refactoring becomes error-prone when shadowed names exist

Fixing these warnings improves code clarity and prevents subtle bugs.

## Requirements

### Functional Requirements
1. The system shall compile without `-Wshadow` warnings
2. Variable names shall be unique within their scope hierarchy

### Non-Functional Requirements
- **Performance**: No impact
- **Memory**: No impact
- **Thread Safety**: N/A
- **Backward Compatibility**: No API changes

## Constraints
- Rename variables to avoid shadowing, not suppress warnings
- Choose meaningful names that reflect the variable's purpose

## Acceptance Criteria
- [ ] All files compile without `-Wshadow` warnings
- [ ] All existing tests pass
- [ ] Renamed variables have clear, descriptive names

---

## Design Decisions (Human Input)

### Preferred Approaches
- Rename the inner (shadowing) variable to be more specific
- Use prefixes/suffixes that indicate scope or purpose (e.g., `localResult` vs `result`)

### Things to Avoid
- Do not rename outer variables if they have well-established names
- Do not use cryptic abbreviations

### Open Questions
- None

---

## References

### Related Code
- `msd/msd-sim/src/Physics/RigidBody/ConvexHull.cpp` — 1 shadow warning

### Related Documentation
- N/A

### Related Tickets
- 0006_fix_double_promotion_warnings
- 0007_fix_sign_conversion_warnings
- 0009_fix_shorten_64_to_32_warnings
- 0010_fix_implicit_float_conversion_warnings

---

## Workflow Log

### Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0008_fix_shadow_warnings/design.md`
  - `docs/designs/0008_fix_shadow_warnings/0008_fix_shadow_warnings.puml`
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

# Ticket 0032d: CollisionResponse Cleanup

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete — Awaiting Tutorial
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Assignee**: cpp-implementer
**Created**: 2026-01-29
**Generate Tutorial**: No
**Parent Ticket**: [0032_contact_constraint_refactor](0032_contact_constraint_refactor.md)

---

## Summary

Remove the standalone `CollisionResponse` namespace (files and tests) now that contact response is handled through the constraint framework. Update `CMakeLists.txt` to remove the old sources and verify no remaining references exist.

---

## Motivation

After 0032c migrates `WorldModel` to the constraint-based pipeline, the `CollisionResponse` namespace is dead code. Removing it:
1. Eliminates the parallel force-calculation system that motivated the refactor
2. Reduces maintenance burden and confusion about which collision path is active
3. Matches parent ticket FR-6 and AC8

---

## Technical Approach

### Files to Delete

| File | Reason |
|------|--------|
| `msd-sim/src/Physics/CollisionResponse.hpp` | Replaced by ContactConstraint + ContactConstraintFactory |
| `msd-sim/src/Physics/CollisionResponse.cpp` | Replaced by ContactConstraint + ContactConstraintFactory |
| `msd-sim/test/Physics/CollisionResponseTest.cpp` | Test coverage migrated to ContactConstraintTest + integration tests |

### Files to Modify

| File | Changes |
|------|---------|
| `msd-sim/CMakeLists.txt` | Remove `CollisionResponse.cpp` from sources, remove `CollisionResponseTest.cpp` from tests |

### Verification Steps

1. `grep -r "CollisionResponse" msd/` returns zero matches (no remaining references)
2. Full build succeeds (`cmake --build --preset conan-debug`)
3. All tests pass (`ctest --preset conan-debug`)
4. No header includes `CollisionResponse.hpp`

---

## Requirements

### Functional Requirements

1. **FR-1**: `CollisionResponse.hpp` and `CollisionResponse.cpp` deleted
2. **FR-2**: `CollisionResponseTest.cpp` deleted
3. **FR-3**: No remaining references to `CollisionResponse` in the codebase
4. **FR-4**: CMakeLists.txt updated to remove old sources

### Non-Functional Requirements

1. **NFR-1**: All existing tests pass without modification (no regressions)
2. **NFR-2**: Clean build with no warnings about missing files

---

## Acceptance Criteria

1. [ ] AC1: `CollisionResponse.hpp` and `CollisionResponse.cpp` no longer exist on disk
2. [ ] AC2: `CollisionResponseTest.cpp` no longer exists on disk
3. [ ] AC3: `grep -r "CollisionResponse" msd/` returns no matches
4. [ ] AC4: Full project builds successfully
5. [ ] AC5: All tests pass (parent AC9: existing constraint tests unaffected)

---

## Dependencies

- **Ticket 0032c**: WorldModel Contact Integration (must be complete — WorldModel no longer references CollisionResponse)
- **Ticket 0027**: Collision Response System (original ticket that introduced the code being removed)

---

## Risks

1. **Low risk**: Some test file or comment may reference `CollisionResponse` by name. Mitigation: grep verification step.

---

## Workflow Log

### Implementation Phase
- **Started**:
- **Completed**:
- **Notes**:

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

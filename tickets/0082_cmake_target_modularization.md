# Ticket 0082: CMake Target Modularization

## Status
- [x] Draft
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Refactoring / Build Infrastructure
**Priority**: Low
**Created**: 2026-02-26
**Generate Tutorial**: No

---

## Summary

Break the root `CMakeLists.txt` into modular `.cmake` files to improve maintainability as the number of custom targets grows.

---

## Problem

The root `CMakeLists.txt` has grown to include custom targets for multiple concerns:
- Doxygen documentation (`doxygen`, `doxygen-db`, `codebase-db`)
- Traceability indexing (`trace-git`, `trace-symbols`, `trace-decisions`, `trace-record-mappings`, `traceability`)
- Guidelines database (`guidelines-seed`)
- Core library builds (`msd_utils`, `msd_assets`, `msd_sim`, `msd_gui`, `msd_exe`, etc.)

This makes the file long and harder to navigate. Each concern's targets are logically independent and could live in separate files.

---

## Solution

Extract custom target groups into separate `.cmake` files included from the root `CMakeLists.txt`:

```
cmake/
├── Doxygen.cmake          # doxygen, doxygen-db, codebase-db targets
├── Traceability.cmake     # trace-git, trace-symbols, trace-decisions, trace-record-mappings, traceability
└── Guidelines.cmake       # guidelines-seed
```

The root `CMakeLists.txt` would include them with:
```cmake
include(cmake/Doxygen.cmake)
include(cmake/Traceability.cmake)
include(cmake/Guidelines.cmake)
```

---

## Acceptance Criteria

- [ ] Custom targets extracted into separate `.cmake` files under `cmake/`
- [ ] Root `CMakeLists.txt` includes them via `include()`
- [ ] All existing build presets continue to work unchanged
- [ ] `cmake --build --preset debug-all` builds everything as before

---

## Origin

PR #98 review comment: CMakeLists.txt has gotten large with different targets — break them out into separate `.cmake` files.

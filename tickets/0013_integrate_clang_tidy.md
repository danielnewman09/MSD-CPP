# Feature Ticket: Integrate clang-tidy Static Analysis

## Status
- [X] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-08
- **Author**: Daniel M Newman
- **Priority**: Medium
- **Estimated Complexity**: Small
- **Target Component(s)**: build-system, developer-tooling

---

## Summary
Enhance the existing clang-tidy integration with comprehensive static analysis checks covering readability, bug detection, modernization, performance, and thread safety. Add standalone execution scripts, VSCode task integration, and auto-fix capabilities. All enabled warnings will be treated as errors when clang-tidy is run on-demand.

## Motivation
Static analysis catches bugs earlier in the development cycle and enforces consistent code quality:
1. **Bug prevention**: Detect use-after-move, null dereference, and memory issues at compile time
2. **Code modernization**: Automatically suggest C++20 idioms and best practices
3. **Performance**: Identify unnecessary copies, inefficient algorithms, and missed optimizations
4. **Thread safety**: Detect concurrency issues before they become runtime bugs
5. **Consistency**: Enforce naming conventions and coding standards project-wide

## Requirements

### Functional Requirements
1. The `.clang-tidy` configuration shall enable comprehensive checks across all major categories
2. All enabled clang-tidy warnings shall be treated as errors (WarningsAsErrors: '*')
3. A standalone script shall allow running clang-tidy on specific files, directories, or changed files
4. The script shall support auto-fixing via a `--fix` flag
5. VSCode tasks shall provide convenient access to clang-tidy operations
6. CMake presets shall allow running clang-tidy during compilation (on-demand)

### Non-Functional Requirements
- **Performance**: clang-tidy runs on-demand only, not during normal builds
- **Memory**: No additional memory overhead (clang-tidy runs as external process)
- **Thread Safety**: Not applicable (clang-tidy is a build-time tool)
- **Backward Compatibility**: Must work with existing Conan/CMake build system

## Constraints
- Must work with existing Conan/CMake build system
- Must not slow down normal development builds (on-demand only)
- Must be compatible with macOS (primary development platform)

## Acceptance Criteria
- [ ] `.clang-tidy` updated with `WarningsAsErrors: '*'` and `concurrency-*` checks
- [ ] `HeaderFilterRegex` updated to `'msd/.*\.(hpp|h)$'`
- [ ] `scripts/run-clang-tidy.sh` created with `--fix` and `--diff` support
- [ ] 5 VSCode tasks added for clang-tidy operations
- [ ] CMake presets added for clang-tidy builds (`clang-tidy-debug`, `clang-tidy-release`)
- [ ] CLAUDE.md updated with clang-tidy documentation

---

## Design Decisions (Human Input)

### Preferred Approaches
- All warnings as errors when clang-tidy is enabled
- On-demand execution only (not during normal builds)
- Auto-fix capability via scripts and VSCode tasks
- No CI integration (local-only tooling for now)

### Things to Avoid
- Don't run clang-tidy automatically during normal builds (too slow)
- Don't add CI integration yet (can be added later if desired)

### Open Questions
- None

---

## References

### Related Code
- `.clang-tidy` — Existing configuration to enhance
- `CMakeLists.txt` lines 18, 70-78 — Existing ENABLE_CLANG_TIDY option
- `conanfile.py` lines 24, 29, 46 — Existing enable_clang_tidy Conan option
- `.vscode/tasks.json` — Add clang-tidy tasks
- `CMakeUserPresets.json` — Add clang-tidy presets

### Related Documentation
- clang-tidy documentation: https://clang.llvm.org/extra/clang-tidy/
- Available checks: https://clang.llvm.org/extra/clang-tidy/checks/list.html

### Related Tickets
- None

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0013_integrate_clang_tidy/design.md`
  - `docs/designs/0013_integrate_clang_tidy/0013_integrate_clang_tidy.puml`
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
  - P1: {name} — {result}
- **Artifacts**:
  - `docs/designs/0013_integrate_clang_tidy/prototype-results.md`
- **Notes**: Prototype not required — clang-tidy integration is well-established pattern

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
  - `scripts/run-clang-tidy.sh`
- **Files Modified**:
  - `.clang-tidy`
  - `.vscode/tasks.json`
  - `CMakeUserPresets.json`
  - `CLAUDE.md`
- **Artifacts**:
  - `docs/designs/0013_integrate_clang_tidy/implementation-notes.md`
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

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}

# Feature Ticket: Add macOS Profiling Support

## Status
- [ ] Draft
- [X] Ready for Design
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
- **Target Component(s)**: build-system

---

## Summary
Add profiling infrastructure for macOS using Xcode Instruments. This enables developers to perform deep CPU profiling, memory analysis, and call graph visualization on Apple Silicon and Intel Macs. The implementation includes build configuration for profiling-friendly binaries and helper scripts for streamlined profiling workflows.

## Motivation
Performance profiling on macOS requires different tooling than Linux (where Valgrind/Callgrind is standard):
1. Valgrind does not support Apple Silicon (ARM64 macOS)
2. Xcode Instruments provides comprehensive native profiling capabilities
3. Profiling-optimized builds require specific compiler flags (debug symbols + optimizations)
4. Streamlined workflows reduce friction for developers to profile regularly

## Requirements

### Functional Requirements
1. The build system shall support an `enable_profiling` option via Conan
2. Profiling builds shall include debug symbols with Release optimizations (`-g -O2`)
3. A helper script shall automate Instruments Time Profiler invocation via `xctrace`
4. Documentation shall explain profiling workflows for both GUI and CLI usage

### Non-Functional Requirements
- **Performance**: Profiling builds shall maintain near-Release performance (O2 optimization)
- **Compatibility**: Shall work on both Apple Silicon and Intel Macs
- **Usability**: Single-command profiling via helper script
- **Backward Compatibility**: No impact on existing builds when profiling disabled

## Constraints
- macOS-only feature (Instruments is not available on Linux/Windows)
- Requires Xcode Command Line Tools installed
- xctrace requires macOS 12.0+ for full functionality

## Acceptance Criteria
- [ ] `enable_profiling` option added to conanfile.py
- [ ] `ENABLE_PROFILING` CMake option added with appropriate compiler flags for Apple
- [ ] `profiling-release` build preset added to CMakeUserPresets.json
- [ ] `scripts/profile-instruments.sh` helper script created
- [ ] CLAUDE.md updated with macOS profiling documentation
- [ ] Profiling workflow tested and produces valid .trace files

---

## Design Decisions (Human Input)

### Preferred Approaches
- Use Xcode Instruments as the primary profiling tool (native, full-featured, no SIP issues)
- Time Profiler template for CPU profiling (most common use case)
- Keep profiling infrastructure simple (build flags + helper script)
- Document both GUI and CLI workflows

### Things to Avoid
- Don't require disabling System Integrity Protection (SIP)
- Don't add complex profiling frameworks or dependencies
- Don't create profiling-specific executables initially (use existing test/benchmark executables)

### Open Questions
- Should we add Allocations template support in addition to Time Profiler?
- Should profiling output be organized into a specific directory?

---

## References

### Related Code
- `conanfile.py` — Add enable_profiling option
- `CMakeLists.txt` — Add ENABLE_PROFILING option and compiler flags
- `CMakeUserPresets.json` — Add profiling-release preset

### Related Documentation
- Apple Instruments documentation: https://developer.apple.com/documentation/xcode/instruments
- xctrace man page: `man xctrace`

### Related Tickets
- `0011_add_google_benchmark.md` — Complementary performance tooling (benchmarks measure time, profiling finds hotspots)

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0012_add_macos_profiling_support/design.md`
  - `docs/designs/0012_add_macos_profiling_support/0012_add_macos_profiling_support.puml`
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
  - `docs/designs/0012_add_macos_profiling_support/prototype-results.md`
- **Notes**:

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
- **Artifacts**:
  - `docs/designs/0012_add_macos_profiling_support/implementation-notes.md`
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

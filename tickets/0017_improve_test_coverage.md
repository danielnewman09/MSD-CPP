# Feature Ticket: Improve Unit Test Coverage

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
- **Created**: 2026-01-18
- **Author**: Daniel M Newman
- **Priority**: Medium
- **Estimated Complexity**: Large
- **Target Component(s)**: msd-sim, msd-assets, msd-gui, msd-utils

---

## Summary
The codebase currently has low test coverage. Only ~11% of source code (910 lines out of ~8200 lines) is exercised by tests. This ticket tracks the effort to add unit tests for untested modules, prioritized by testability and importance.

## Motivation
1. **Current State**: 75.8% line coverage of *tested* code, but only 25 of 66 source files have any test coverage
2. **Risk**: Untested code in critical paths (collision detection, physics calculations, camera transforms) could contain latent bugs
3. **Maintainability**: Tests document expected behavior and catch regressions during refactoring
4. **Quality**: Higher test coverage enables confident code changes and faster development velocity

## Requirements

### Functional Requirements
1. Add unit tests for HIGH PRIORITY files (pure logic, no external dependencies)
2. Add unit tests for MEDIUM PRIORITY files (testable with mocking or fixtures)
3. Achieve meaningful coverage of critical algorithms (GJK, inertia calculations, camera transforms)

### Non-Functional Requirements
- **Performance**: Tests should complete quickly (< 30 seconds total)
- **Maintainability**: Tests should follow existing patterns (GTest, consistent naming)
- **Independence**: Tests should not require external resources (GPU, network, real files where possible)

## Constraints
- SDL/GPU-dependent code (SDLApp, SDLGPUManager) will remain integration-tested only
- Some platform-specific code (PathUtils) may have limited cross-platform test coverage

## Acceptance Criteria
- [ ] GJK collision detection has comprehensive test coverage
- [ ] InertialCalculations has tests with known reference values
- [ ] Camera3D matrix computations have verification tests
- [ ] InputControlAgent command translation is tested
- [ ] STLLoader file parsing is tested with fixture files
- [ ] Overall source file coverage increases from 25/66 to at least 40/66

---

## Coverage Analysis (2026-01-18)

### Current State
- **Lines in coverage report**: 910 (75.8% hit rate)
- **Total source lines (excluding tests)**: ~8,200
- **Files with any coverage**: 25 of 66 source files

### HIGH PRIORITY — Pure logic, easily testable, high impact

| File | LOC | Complexity | Importance | Notes |
|------|-----|------------|------------|-------|
| **GJK.cpp/hpp** | ~350 | High | Critical | Core collision detection, 0% coverage currently |
| **InertialCalculations.cpp/hpp** | ~210 | Medium | Critical | Inertia tensor math, verifiable with reference values |
| **Camera3D.cpp/hpp** | ~225 | High | Critical | View/Projection matrices, pure Eigen math |
| **InputControlAgent.cpp/hpp** | ~220 | Medium | High | Command translation, pure state machine |
| **Asset.cpp/hpp** | ~195 | Low | Medium-High | Asset container, simple accessors |
| **STLLoader.cpp/hpp** | ~440 | Medium | Medium-High | File parsing, testable with fixture files |

### MEDIUM PRIORITY — Testable with some setup

| File | LOC | Complexity | Importance | Notes |
|------|-----|------------|------------|-------|
| **InputState.cpp/hpp** | ~190 | Low-Medium | High | Key state tracking, mock SDL keycodes |
| **InputHandler.cpp/hpp** | ~305 | Medium | High | Binding modes, mock InputState |
| **PathUtils.cpp/hpp** | ~105 | Low | Medium | Platform paths, platform-specific testing |
| **AssetRegistry.cpp/hpp** | ~295 | Medium | Critical | Cache logic, needs test database |
| **WorldModel.cpp/hpp** | ~365 | Medium | Critical | Object lifecycle, needs mock objects |
| **Engine.cpp/hpp** | ~200 | Medium | Critical | Simulation loop, integration test |

### LOW PRIORITY — Heavy external dependencies

| File | LOC | Complexity | Importance | Notes |
|------|-----|------------|------------|-------|
| **SDLApp.cpp/hpp** | ~600 | High | High | Full application, integration/manual testing |
| **SDLGPUManager.cpp/hpp** | ~430 | High | Critical | GPU pipeline, requires real GPU |
| **SDLUtils.cpp/hpp** | ~200 | Low | Low-Medium | SDL utilities, mostly visual |

### Files Already Tested (for reference)
- msd-sim: Angle, Coordinate, ReferenceFrame, ConvexHull, MotionController, Platform
- msd-assets: Geometry, GeometryFactory (partial)
- msd-gui: ShaderPolicy, GPUInstanceManager, ShaderTransforms
- msd-transfer: MeshRecord

---

## Implementation Plan

### Phase 1: Core Physics (HIGH PRIORITY)
1. **GJK Tests** — Collision detection algorithm
   - Known intersecting/non-intersecting geometry pairs
   - Edge cases: touching, overlapping, separated
   - Simplex progression validation

2. **InertialCalculations Tests** — Physics calculations
   - Known shapes (cube, sphere, tetrahedron) with reference tensors
   - Parallel axis theorem validation
   - Edge cases: degenerate hulls

### Phase 2: Rendering Math (HIGH PRIORITY)
3. **Camera3D Tests** — View/Projection matrices
   - View matrix computation (inverse transform)
   - Projection matrix (frustum validation)
   - MVP composition
   - Aspect ratio changes

### Phase 3: Input System (HIGH PRIORITY)
4. **InputControlAgent Tests** — Command translation
   - Individual command activation
   - Combined commands
   - Max speed limiting
   - Direction conversion

### Phase 4: Asset Pipeline (HIGH PRIORITY)
5. **STLLoader Tests** — File parsing
   - Binary STL parsing (valid/malformed)
   - ASCII STL parsing (valid/malformed)
   - Format auto-detection
   - Test fixture files

6. **Asset Tests** — Container logic
   - Asset creation from records
   - Geometry presence/absence
   - Optional reference access

### Phase 5: Input Handling (MEDIUM PRIORITY)
7. **InputState Tests** — Key state tracking
8. **InputHandler Tests** — Binding evaluation

### Phase 6: Integration (MEDIUM PRIORITY)
9. **AssetRegistry Tests** — With test database
10. **WorldModel Tests** — Object lifecycle
11. **PathUtils Tests** — Platform paths

---

## Design Decisions (Human Input)

### Preferred Approaches
- Follow existing test patterns (GTest, `*Test.cpp` naming)
- Use test fixtures for file-based tests (STLLoader)
- Prefer pure unit tests over integration tests where possible

### Things to Avoid
- Don't try to unit test SDL/GPU code (use integration tests)
- Don't add complex mocking infrastructure unless necessary

### Open Questions
- Should STL test fixtures be checked into the repo or generated?
- What reference values should be used for inertia tensor validation?

---

## References

### Related Code
- `msd/msd-sim/test/Physics/ConvexHullTest.cpp` — Example physics tests
- `msd/msd-gui/test/ShaderTransformTest.cpp` — Example matrix tests
- `msd/msd-assets/test/GeometryDatabaseTest.cpp` — Example asset tests

### Related Documentation
- Coverage report: `build/Debug/coverage/index.html`
- GTest documentation: https://google.github.io/googletest/

### Related Tickets
- #0011 — Google Benchmark (similar infrastructure pattern)

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Analysis Phase
- **Completed**: 2026-01-18
- **Notes**:
  - Identified 66 source files, only 25 have test coverage
  - Prioritized files by testability (pure logic vs external dependencies)
  - GJK has 0% coverage despite being critical collision detection code
  - Camera3D, InputControlAgent, InertialCalculations are high-value targets

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Implementation
{Your comments on the implementation}

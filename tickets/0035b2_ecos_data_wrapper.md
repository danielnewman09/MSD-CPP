# Ticket 0035b2: ECOSData RAII Wrapper

## Status
- [x] Draft
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Ready for Implementation
**Assignee**: N/A
**Created**: 2026-01-31
**Generate Tutorial**: No
**Parent Ticket**: [0035b_box_constrained_asm_solver](0035b_box_constrained_asm_solver.md)

---

## Summary

Implement the `ECOSData` RAII wrapper that manages the ECOS solver workspace lifecycle. This struct owns all memory passed to ECOS (sparse matrices, vectors, cone sizes) and ensures `ECOS_cleanup()` is called on destruction, even if exceptions occur. This is the bridge between our Eigen-based data structures and the ECOS C API.

---

## Motivation

ECOS is a C library with manual memory management. The `ECOS_setup()` function allocates a `pwork*` workspace, and `ECOS_cleanup()` must be called to free it. Without RAII, any exception between setup and cleanup leaks memory. ECOSData encapsulates this lifecycle.

---

## Technical Approach

### Component: ECOSData

**Location**: `msd-sim/src/Physics/Constraints/ECOS/ECOSData.hpp` / `ECOSData.cpp`

**Responsibilities**:
1. Own all data arrays passed to `ECOS_setup()` (G sparse matrix, h vector, c vector, cone sizes)
2. Call `ECOS_setup()` to create the workspace
3. Call `ECOS_cleanup()` in destructor
4. Provide move semantics (disable copy — ECOS workspace is not copyable)
5. Expose workspace pointer for `ECOS_solve()` calls

**Interface** (from design.md):
```cpp
struct ECOSData {
    pwork* workspace{nullptr};
    idxint num_variables{0};    // Decision variables (3C)
    idxint num_cones{0};        // Number of second-order cones (C)

    // Owned sparse matrix storage (CSC format for G)
    ECOSSparseMatrix G;

    // Owned constraint vectors
    std::vector<pfloat> h;      // RHS for inequality constraints
    std::vector<pfloat> c;      // Linear objective

    // Owned cone dimensions
    std::vector<idxint> cone_sizes;

    // Construction
    explicit ECOSData(idxint numVariables, idxint numCones);
    ~ECOSData();

    // Move-only (ECOS workspace is not copyable)
    ECOSData(const ECOSData&) = delete;
    ECOSData& operator=(const ECOSData&) = delete;
    ECOSData(ECOSData&&) noexcept;
    ECOSData& operator=(ECOSData&&) noexcept;

    // Setup ECOS workspace (call after populating G, h, c, cone_sizes)
    void setup();

    // Check if workspace is active
    bool isSetup() const { return workspace != nullptr; }

    // Cleanup (called automatically by destructor)
    void cleanup();
};
```

**Key design decisions**:
- ECOSData **owns** all arrays (G, h, c, cone_sizes) so they remain valid for the lifetime of the ECOS workspace
- `setup()` is separate from construction to allow populating data before calling `ECOS_setup()`
- `cleanup()` is idempotent (safe to call multiple times)
- Move constructor nullifies source workspace to prevent double-free

---

## Requirements

### Functional Requirements

1. **FR-1**: Constructor initializes dimensions, allocates vector storage
2. **FR-2**: `setup()` calls `ECOS_setup()` with owned data arrays
3. **FR-3**: Destructor calls `ECOS_cleanup()` if workspace is active
4. **FR-4**: Move constructor transfers ownership, nullifies source
5. **FR-5**: Move assignment cleans up current workspace before transfer
6. **FR-6**: `cleanup()` is idempotent (no-op if workspace is null)
7. **FR-7**: Copy construction/assignment is deleted (compile error)

### Non-Functional Requirements

1. **NFR-1**: No memory leaks (ECOS_cleanup called on all code paths)
2. **NFR-2**: Exception-safe (destructor handles cleanup even after exceptions)

---

## Acceptance Criteria

- [ ] **AC1**: ECOSData constructor initializes dimensions and reserves vector storage
- [ ] **AC2**: `setup()` successfully calls ECOS_setup() with valid data
- [ ] **AC3**: Destructor calls ECOS_cleanup() (verified via workspace becoming null)
- [ ] **AC4**: Move constructor transfers workspace ownership correctly
- [ ] **AC5**: Move assignment cleans up existing workspace before transfer
- [ ] **AC6**: `cleanup()` is safe to call when workspace is null (idempotent)
- [ ] **AC7**: Copy construction is a compile error (= delete)
- [ ] **AC8**: All existing tests pass (zero regressions)

---

## Test Plan

| Test Case | What It Validates |
|-----------|-------------------|
| Default construction | Dimensions set, workspace null |
| Setup with valid data | ECOS_setup() succeeds, workspace non-null |
| Destructor cleanup | After destruction, no memory leak (ASAN) |
| Move constructor | Source workspace nullified, target owns workspace |
| Move assignment | Old workspace cleaned, new workspace transferred |
| Double cleanup | cleanup() is idempotent |
| Setup without data | Handles error from ECOS_setup() gracefully |

---

## Files

### New Files

| File | Purpose |
|------|---------|
| `msd-sim/src/Physics/Constraints/ECOS/ECOSData.hpp` | RAII wrapper header |
| `msd-sim/src/Physics/Constraints/ECOS/ECOSData.cpp` | RAII wrapper implementation |
| `msd-sim/test/Physics/Constraints/ECOS/ECOSDataTest.cpp` | Unit tests |

### Modified Files

| File | Change |
|------|--------|
| `msd-sim/src/Physics/Constraints/ECOS/CMakeLists.txt` | Add ECOSData.cpp |
| `msd-sim/test/Physics/Constraints/ECOS/CMakeLists.txt` | Add ECOSDataTest.cpp |

---

## Dependencies

- **Requires**: [0035b1](0035b1_ecos_utilities.md) (ECOSSparseMatrix for G matrix storage)
- **Blocks**: [0035b3](0035b3_ecos_problem_construction.md) (problem construction uses ECOSData)

---

## References

- **Design document**: [design.md](../docs/designs/0035b_box_constrained_asm_solver/design.md) — ECOSData section
- **ECOS API**: `ECOS_setup()`, `ECOS_solve()`, `ECOS_cleanup()` in `ecos/ecos.h`
- **ECOS exit codes**: ECOS_OPTIMAL (0), ECOS_PINF (1), ECOS_DINF (2), ECOS_MAXIT (-1), ECOS_NUMERICS (-2)

# Design: ECOSData RAII Wrapper

## Summary

Implement `ECOSData`, a RAII wrapper that manages the ECOS solver workspace lifecycle. This struct owns all memory passed to ECOS (sparse matrices, vectors, cone sizes) and ensures `ECOS_cleanup()` is called on destruction, even if exceptions occur. This is the bridge between our Eigen-based data structures and the ECOS C API.

**Parent Ticket**: [0035b_box_constrained_asm_solver](../0035b_box_constrained_asm_solver/design.md)

**Dependencies**: [0035b1_ecos_utilities](../../tickets/0035b1_ecos_utilities.md) (ECOSSparseMatrix)

---

## Architecture Changes

### PlantUML Diagram
See: [`./0035b2_ecos_data_wrapper.puml`](./0035b2_ecos_data_wrapper.puml)

---

## Motivation

ECOS is a C library with manual memory management. The `ECOS_setup()` function allocates a `pwork*` workspace, and `ECOS_cleanup()` must be called to free it. Without RAII, any exception between setup and cleanup leaks memory. ECOSData encapsulates this lifecycle.

**Key requirements**:
1. **Ownership**: ECOSData owns all data arrays (G matrix, h vector, c vector, cone sizes) so they remain valid for the lifetime of the ECOS workspace
2. **Exception safety**: Destructor guarantees cleanup even if exceptions occur
3. **Move semantics**: ECOS workspace is not copyable, so ECOSData must support move-only semantics
4. **Idempotent cleanup**: `cleanup()` must be safe to call multiple times

---

## Component Design

### ECOSWorkspaceDeleter (struct)

**Purpose**: Custom deleter for `std::unique_ptr<pwork>` that calls `ECOS_cleanup()`.

**Location**: Defined in `ECOSData.hpp`, before ECOSData.

```cpp
struct ECOSWorkspaceDeleter {
    void operator()(pwork* w) const noexcept {
        if (w != nullptr) {
            ECOS_cleanup(w, 0);
        }
    }
};
```

**Rationale**: Wrapping the ECOS C API cleanup in a deleter struct enables `std::unique_ptr` to manage the workspace lifetime automatically. This follows the project standard of using `std::unique_ptr` for exclusive ownership (see CLAUDE.md Memory Management section). The deleter is `noexcept` because `ECOS_cleanup()` is a C function that does not throw.

---

### ECOSData (struct)

**Purpose**: RAII wrapper for ECOS solver workspace and problem data.

**Header location**: `msd/msd-sim/src/Physics/Constraints/ECOS/ECOSData.hpp`

**Source location**: `msd/msd-sim/src/Physics/Constraints/ECOS/ECOSData.cpp`

**Key interfaces**:
```cpp
/// Custom deleter for ECOS workspace
struct ECOSWorkspaceDeleter {
    void operator()(pwork* w) const noexcept;
};

/// Type alias for managed ECOS workspace pointer
using ECOSWorkspacePtr = std::unique_ptr<pwork, ECOSWorkspaceDeleter>;

struct ECOSData {
    // Problem dimensions
    idxint num_variables{0};    // Decision variables (3C)
    idxint num_cones{0};        // Number of second-order cones (C)

    // Owned sparse matrix storage (CSC format for G)
    ECOSSparseMatrix G;

    // Owned constraint vectors
    std::vector<pfloat> h;      // RHS for inequality constraints
    std::vector<pfloat> c;      // Linear objective

    // Owned cone dimensions
    std::vector<idxint> cone_sizes;

    // ECOS workspace (owned via unique_ptr with custom deleter)
    // IMPORTANT: Declared last — see "ECOS Equilibration Constraint" section
    ECOSWorkspacePtr workspace{nullptr};

    // Construction
    explicit ECOSData(idxint numVariables, idxint numCones);

    // Destructor: default (workspace_ destroyed first due to reverse declaration order)
    ~ECOSData() = default;

    // Move-only semantics
    ECOSData(const ECOSData&) = delete;
    ECOSData& operator=(const ECOSData&) = delete;
    ECOSData(ECOSData&&) noexcept;              // Custom: moves data then workspace
    ECOSData& operator=(ECOSData&&) noexcept;   // Custom: cleanup() first, then move

    // Setup ECOS workspace (call after populating G, h, c, cone_sizes)
    void setup();

    // Check if workspace is active
    bool isSetup() const { return workspace != nullptr; }

    // Release workspace (reset to null, triggering ECOS_cleanup via deleter)
    void cleanup();
};
```

**Key design change**: The `pwork*` raw pointer is wrapped in `std::unique_ptr<pwork, ECOSWorkspaceDeleter>`. This follows the project's coding standard: "Use `std::unique_ptr` for exclusive ownership and transfer" and "Never use raw pointers in public interfaces." The custom deleter calls `ECOS_cleanup()` automatically when the `unique_ptr` is destroyed or reset.

**Implications**:
- **Destructor**: `= default` — `workspace` is declared last and therefore destroyed first (reverse declaration order), ensuring data arrays are still alive during `ECOS_cleanup()` (see [ECOS Equilibration Constraint](#ecos-equilibration-constraint))
- **Move constructor**: Custom implementation — moves data arrays first, then workspace, maintaining pointer validity
- **Move assignment**: Custom implementation — calls `cleanup()` first (while data arrays are still valid), then moves all members
- **Copy**: Deleted because `std::unique_ptr` is non-copyable; explicit `= delete` for documentation
- **cleanup()**: Implemented as `workspace.reset()`, which triggers the deleter if non-null
- **isSetup()**: Checks `workspace != nullptr` (unique_ptr supports contextual bool conversion)
- **Accessing workspace**: Use `workspace.get()` to obtain the raw `pwork*` for ECOS API calls

---

## Interface Specification

### Constructor

```cpp
explicit ECOSData(idxint numVariables, idxint numCones);
```

**Purpose**: Initialize dimensions and allocate vector storage.

**Behavior**:
- Set `num_variables` and `num_cones`
- Reserve storage for `h` (size: `numVariables`), `c` (size: `numVariables`), `cone_sizes` (size: `numCones`)
- `workspace` is default-initialized to `nullptr` (empty unique_ptr)
- Does NOT call `ECOS_setup()` (that's done in `setup()`)

**Preconditions**: None

**Postconditions**:
- `num_variables == numVariables`
- `num_cones == numCones`
- `workspace == nullptr`
- `h`, `c`, `cone_sizes` have reserved capacity

**Exception safety**: Strong guarantee (no resources allocated yet)

---

### Destructor

```cpp
~ECOSData() = default;
```

**Purpose**: Free ECOS workspace if active.

**Behavior**:
- Compiler-generated destructor destroys members in reverse declaration order
- `workspace_` is declared last, so it is destroyed **first**
- `unique_ptr` destructor invokes `ECOSWorkspaceDeleter`, which calls `ECOS_cleanup()` if non-null
- `ECOS_cleanup()` calls `unset_equilibration()`, which writes to `G_`, `h_`, `c_` data arrays — these are still alive because they are destroyed after `workspace_`
- Then `G_`, `h_`, `c_`, etc. are destroyed normally

**Preconditions**: None

**Postconditions**: All ECOS-allocated memory freed, all owned vectors freed

**Exception safety**: No-throw guarantee (ECOSWorkspaceDeleter is noexcept, C API does not throw)

**Note**: The `= default` destructor is safe here **only because** `workspace_` is declared last. See [ECOS Equilibration Constraint](#ecos-equilibration-constraint).

---

### setup()

```cpp
void setup();
```

**Purpose**: Call `ECOS_setup()` to create the ECOS workspace.

**Behavior**:
1. Verify that `G`, `h`, `c`, `cone_sizes` have been populated
2. Call `ECOS_setup()` with owned data arrays, wrapping the result in the unique_ptr:
   ```cpp
   pwork* raw = ECOS_setup(
       num_variables,           // n (number of variables)
       num_variables,           // m (number of inequality constraints)
       0,                       // p (number of equality constraints, 0 for friction)
       0,                       // l (dimension of positive orthant, 0 for SOC-only)
       num_cones,               // ncones (number of second-order cones)
       cone_sizes.data(),       // q (array of cone dimensions)
       0,                       // e (exponent cone dimensions, 0)
       G.data.data(),           // G_data (CSC sparse matrix)
       G.col_ptrs.data(),       // G_col_ptrs
       G.row_indices.data(),    // G_row_indices
       nullptr,                 // A_data (no equality constraints)
       nullptr,                 // A_col_ptrs
       nullptr,                 // A_row_indices
       c.data(),                // c (linear objective)
       h.data(),                // h (RHS for cone constraints)
       nullptr                  // b (RHS for equality constraints, NULL)
   );
   if (raw == nullptr) {
       throw std::runtime_error("ECOS_setup failed: invalid problem data");
   }
   workspace.reset(raw);
   ```
3. The unique_ptr now owns the workspace; ECOS_cleanup will be called automatically

**Preconditions**:
- `workspace == nullptr` (not already set up)
- `G` has been populated (nnz > 0)
- `h.size() == num_variables`
- `c.size() == num_variables`
- `cone_sizes.size() == num_cones`

**Postconditions**:
- `workspace != nullptr` (if successful)
- ECOS workspace ready for `ECOS_solve()`

**Exception safety**: Strong guarantee (if ECOS_setup fails, throws before workspace is assigned; if workspace was previously non-null, `reset()` cleans up the old one first via the deleter)

**Error handling**:
- If `ECOS_setup()` returns `nullptr`, throw `std::runtime_error("ECOS_setup failed: invalid problem data")`

---

### cleanup()

```cpp
void cleanup();
```

**Purpose**: Release ECOS workspace, triggering cleanup via the custom deleter.

**Behavior**:
1. Call `workspace.reset()`:
   - If `workspace` was non-null, `ECOSWorkspaceDeleter` calls `ECOS_cleanup(w, 0)`
   - If `workspace` was null, `reset()` is a no-op

**Preconditions**: None

**Postconditions**: `workspace == nullptr`

**Exception safety**: No-throw guarantee (`reset()` invokes the noexcept deleter)

**Idempotency**: Yes (safe to call multiple times — `unique_ptr::reset()` on a null pointer is a no-op)

---

### Move Constructor

```cpp
ECOSData(ECOSData&& other) noexcept;
```

**Purpose**: Transfer ownership of ECOS workspace and data arrays.

**Behavior**:
1. Move-construct all data members from `other` (dimensions, G, h, c, cone_sizes)
2. Move-construct `workspace_` from `other.workspace_` (nullifies source)

This is safe because the target object has no pre-existing workspace to clean up. The data arrays are moved before the workspace in the member initializer list, but since both operations are moves (no destruction of old state), the order is not critical here.

**Preconditions**: None

**Postconditions**:
- `this->workspace` owns the workspace that `other` previously owned
- `other.workspace == nullptr`
- `other`'s vectors are in a valid but unspecified (moved-from) state

**Exception safety**: No-throw guarantee

**Note**: A `= default` move constructor would also be correct here (no cleanup needed during construction), but we use a custom implementation for symmetry with move assignment and to ensure the member initializer order matches declaration order explicitly.

---

### Move Assignment

```cpp
ECOSData& operator=(ECOSData&& other) noexcept;
```

**Purpose**: Transfer ownership, cleaning up existing workspace first.

**Behavior**:
1. Self-assignment check (`this != &other`)
2. Call `cleanup()` to destroy the current workspace **while our data arrays are still valid** — this is critical because `ECOS_cleanup()` accesses `G_`, `h_`, `c_` during `unset_equilibration()`
3. Move-assign all data members from `other`
4. Move-assign `workspace_` from `other` (nullifies source)

**Why not `= default`?** The default move assignment would move-assign members in declaration order. Since data arrays are declared before `workspace_`, they would be overwritten first. When `workspace_`'s move-assign then destroys the old workspace (via the deleter), `ECOS_cleanup()` would write to the now-invalid old data arrays. See [ECOS Equilibration Constraint](#ecos-equilibration-constraint).

**Preconditions**: None

**Postconditions**:
- Old workspace cleaned up (via `cleanup()` before data is moved)
- `this->workspace` owns the workspace that `other` previously owned
- `other.workspace == nullptr`

**Exception safety**: No-throw guarantee

---

### isSetup()

```cpp
bool isSetup() const { return workspace != nullptr; }
```

**Purpose**: Check if ECOS workspace is active.

**Returns**: `true` if `setup()` has been called successfully, `false` otherwise. Uses `unique_ptr`'s contextual conversion to `bool`.

**Exception safety**: No-throw guarantee

---

## ECOS API Integration

### ECOS Lifecycle

The ECOS C API has three phases:

1. **Setup**: Allocate workspace with `ECOS_setup()`
2. **Solve**: Call `ECOS_solve(workspace)` repeatedly (can reuse workspace)
3. **Cleanup**: Free workspace with `ECOS_cleanup(workspace, 0)`

### ECOSData Lifecycle Mapping

| ECOS Phase | ECOSData Method | When Called |
|------------|-----------------|-------------|
| Setup | `setup()` | After populating G, h, c, cone_sizes |
| Solve | (External) | Caller uses `workspace` pointer directly |
| Cleanup | `cleanup()` or `~ECOSData()` | Automatic on destruction or explicit call |

### Example Usage

```cpp
// 1. Construct ECOSData
ECOSData data{3 * numContacts, numContacts};

// 2. Populate problem data
data.G = ECOSSparseMatrix::fromDense(G_matrix);
data.h = convertToVector(h_vector);
data.c = std::vector<pfloat>(data.num_variables, 0.0);  // Zero objective
data.cone_sizes = std::vector<idxint>(numContacts, 3);  // All cones size 3

// 3. Setup ECOS workspace
data.setup();

// 4. Configure solver settings (access raw pointer via .get())
data.workspace.get()->stgs->maxit = 100;
data.workspace.get()->stgs->abstol = 1e-6;
data.workspace.get()->stgs->reltol = 1e-6;
data.workspace.get()->stgs->feastol = 1e-6;

// 5. Solve (pass raw pointer to ECOS C API)
idxint exit_flag = ECOS_solve(data.workspace.get());

// 6. Extract solution
if (exit_flag == ECOS_OPTIMAL) {
    Eigen::VectorXd lambda = Eigen::Map<Eigen::VectorXd>(
        data.workspace.get()->x, data.num_variables);
}

// 7. Cleanup automatic when data goes out of scope
//    (unique_ptr destructor calls ECOSWorkspaceDeleter → ECOS_cleanup)
```

---

## ECOS Equilibration Constraint

### Discovery

ECOS v2.0.10 is compiled with `EQUILIBRATE = 1` by default. When enabled, `ECOS_setup()` calls `set_equilibration()` which **mutates the caller's data arrays in-place** — specifically `G->pr` (sparse matrix values), `h` (constraint RHS), and `c` (objective vector). During `ECOS_cleanup()`, `unset_equilibration()` **writes back** to these same arrays to restore original values.

This means `ECOS_cleanup()` accesses the raw pointers `w->G->pr`, `w->h`, and `w->c` that were passed to `ECOS_setup()`. These pointers point directly into our `std::vector` storage (via `.data()`). If the vectors have been destroyed or moved-from before `ECOS_cleanup()` runs, the write-back causes a use-after-free.

### Relevant ECOS Source (v2.0.10)

```c
// ecos/src/preproc.c — ECOS_setup() stores raw pointers:
mywork->c = c;      // Points to our c_.data()
mywork->h = h;      // Points to our h_.data()
mywork->G = ecoscreateSparseMatrix(m, n, Gjc[n], Gjc, Gir, Gpr);
                     // G->pr points to our G_.data.data()

// ecos/src/equil.c — unset_equilibration() writes to those pointers:
restore(w->Gequil, w->xequil, w->G);  // Writes to G->pr
for(i = 0; i < num_G_rows; i++) {
    w->h[i] *= w->Gequil[i];          // Writes to h
}

// ecos/src/preproc.c — ECOS_cleanup() calls unset_equilibration first:
void ECOS_cleanup(pwork* w, idxint keepvars) {
    unset_equilibration(w);  // <-- accesses G, h, c data arrays
    // ... then frees internal ECOS memory
}
```

### Impact on ECOSData Design

This imposes two **hard constraints** on member destruction and move ordering:

**Constraint 1: Member declaration order**

`workspace_` must be declared **after** `G_`, `h_`, and `c_` so that it is destroyed **before** them (C++ destroys members in reverse declaration order). This ensures `ECOS_cleanup()` can safely access the data arrays during destruction.

```cpp
struct ECOSData {
    // Data arrays declared FIRST (destroyed last)
    idxint num_variables_{0};
    idxint num_cones_{0};
    ECOSSparseMatrix G_{};
    std::vector<pfloat> h_{};
    std::vector<pfloat> c_{};
    std::vector<idxint> cone_sizes_{};

    // Workspace declared LAST (destroyed first)
    ECOSWorkspacePtr workspace_{nullptr};
};
```

**Constraint 2: Move assignment must cleanup before moving data**

The default `operator=(ECOSData&&)` would move-assign members in declaration order: first the data vectors (invalidating the old storage), then the workspace (which triggers `ECOS_cleanup()` on the **old** workspace — but the old data arrays are already gone). This is a use-after-free.

The fix is a custom move assignment that explicitly calls `cleanup()` **before** moving any data:

```cpp
ECOSData& operator=(ECOSData&& other) noexcept {
    if (this != &other) {
        cleanup();  // Destroy workspace while our data arrays are still valid
        // Now safe to overwrite data arrays
        num_variables_ = other.num_variables_;
        num_cones_ = other.num_cones_;
        G_ = std::move(other.G_);
        h_ = std::move(other.h_);
        c_ = std::move(other.c_);
        cone_sizes_ = std::move(other.cone_sizes_);
        workspace_ = std::move(other.workspace_);
    }
    return *this;
}
```

**The move constructor does not have this problem** because there is no pre-existing workspace to clean up — the target object is being constructed fresh.

### Why `= default` Is Not Safe

| Operation | `= default` safe? | Reason |
|-----------|-------------------|--------|
| Destructor | Yes | `workspace_` is declared last, so destroyed first |
| Move constructor | Yes | No pre-existing workspace to clean up in target |
| Move assignment | **No** | Default move-assigns members in declaration order; data vectors are overwritten before the old workspace is destroyed |
| Copy | N/A (deleted) | — |

---

## ECOS Exit Codes

ECOSData users should check the exit code returned by `ECOS_solve()`:

| Exit Code | Name | Meaning |
|-----------|------|---------|
| 0 | `ECOS_OPTIMAL` | Solved to optimality |
| 1 | `ECOS_PINF` | Primal infeasible (no solution exists) |
| 2 | `ECOS_DINF` | Dual infeasible (unbounded) |
| -1 | `ECOS_MAXIT` | Maximum iterations reached |
| -2 | `ECOS_NUMERICS` | Numerical issues (ill-conditioned) |
| -3 | `ECOS_OUTCONE` | Slack variables exited cone (line search failed) |

**Handling strategy**:
- `ECOS_OPTIMAL`: Success, extract solution
- `ECOS_MAXIT`: Max iterations reached, return last iterate with `converged=false`
- `ECOS_NUMERICS`, `ECOS_OUTCONE`: Log warning, return `converged=false`
- `ECOS_PINF`, `ECOS_DINF`: Problem is malformed, throw exception

---

## Memory Management Strategy

### Ownership Model

ECOSData owns:
1. **ECOSSparseMatrix G**: Stored by value, manages CSC storage
2. **std::vector<pfloat> h, c**: Owned vectors
3. **std::vector<idxint> cone_sizes**: Owned vector
4. **ECOSWorkspacePtr workspace**: `std::unique_ptr<pwork, ECOSWorkspaceDeleter>` — exclusive ownership of ECOS-allocated workspace

### Lifetime Guarantees

1. **G, h, c, cone_sizes remain valid** for the lifetime of ECOSData
2. **ECOS workspace remains valid** until `cleanup()`, `workspace.reset()`, or destructor is called
3. **No dangling pointers** — `unique_ptr` guarantees single ownership and automatic cleanup
4. **No double-free** — `unique_ptr` is move-only; moved-from state is null

### Copy vs Move Semantics

| Operation | Implementation | Rationale |
|-----------|---------------|-----------|
| Copy construction | `= delete` | `unique_ptr` is non-copyable (ECOS workspace cannot be deep-copied) |
| Copy assignment | `= delete` | `unique_ptr` is non-copyable |
| Move construction | Custom | Moves data arrays then workspace (safe — no pre-existing workspace to clean up) |
| Move assignment | Custom | Calls `cleanup()` first while data arrays are valid, then moves all members (see [ECOS Equilibration Constraint](#ecos-equilibration-constraint)) |
| Destructor | `= default` | `workspace_` declared last, destroyed first — safe due to member ordering |

**Why `std::unique_ptr` instead of raw `pwork*`?**
- **Project standard**: CLAUDE.md mandates "Use `std::unique_ptr` for exclusive ownership" and "Never use raw pointers in public interfaces"
- **Exception safety**: If an exception occurs after `setup()`, the unique_ptr destructor still calls `ECOS_cleanup()`
- **Self-documenting**: The type `std::unique_ptr<pwork, ECOSWorkspaceDeleter>` makes ownership semantics explicit
- **Destructor for free**: `= default` destructor is correct thanks to member declaration ordering

**Why custom move operations instead of `= default`?**
- ECOS's equilibration feature mutates the caller's data arrays (`G->pr`, `h`, `c`) in-place
- `ECOS_cleanup()` writes back to these arrays during `unset_equilibration()`
- Default move assignment would overwrite data arrays before the old workspace is destroyed, causing use-after-free
- Custom move assignment calls `cleanup()` first, ensuring the old workspace is destroyed while data arrays are still valid
- See [ECOS Equilibration Constraint](#ecos-equilibration-constraint) for full analysis

**Why a custom deleter instead of `std::default_delete`?**
- ECOS workspace is allocated by `ECOS_setup()` (not `new`), so `delete` would be incorrect
- `ECOS_cleanup()` frees the workspace and all its internal allocations
- The deleter struct is stateless (zero overhead — same size as raw pointer)

---

## Error Handling

### Setup Errors

**Scenario**: `ECOS_setup()` returns `nullptr`

**Cause**: Invalid problem data (negative dimensions, null pointers, etc.)

**Handling**:
```cpp
void ECOSData::setup() {
    pwork* raw = ECOS_setup(...);
    if (raw == nullptr) {
        throw std::runtime_error("ECOS_setup failed: invalid problem data");
    }
    workspace.reset(raw);  // unique_ptr takes ownership
}
```

Note: If `workspace` was already non-null when `setup()` is called, `workspace.reset(raw)` will first clean up the old workspace via the deleter before taking ownership of the new one.

### Cleanup Errors

**Scenario**: Destructor called during stack unwinding (exception in progress)

**Handling**:
- `unique_ptr` destructor invokes `ECOSWorkspaceDeleter`, which is `noexcept`
- `ECOS_cleanup()` does not throw (C API)
- Safe to call from destructor even during exception handling
- No manual cleanup code needed — compiler-generated destructor handles everything

### Move Errors

**Scenario**: Move constructor/assignment called

**Handling**:
- Custom move operations are `noexcept` (cleanup is noexcept, unique_ptr move is noexcept, vector move is noexcept)
- Required for container operations (e.g., `std::vector<ECOSData>`)
- Move assignment calls `cleanup()` first to safely destroy old workspace before overwriting data arrays
- No possibility of failure

---

## Thread Safety

**ECOSData is not thread-safe.**

**Rationale**:
- ECOS `pwork*` workspace is stateful (stores solver state between iterations)
- `ECOS_solve()` modifies workspace in-place
- No internal mutex (C API has no thread safety guarantees)

**Usage requirements**:
- Each thread must have its own ECOSData instance
- Do not share ECOSData across threads
- Do not call `setup()`, `cleanup()`, or access `workspace` concurrently

**Multi-threading strategy**:
- Create one ECOSData per solve
- Solves can run in parallel on different ECOSData instances

---

## Dependencies

### Required Components (from 0035b1)

| Component | Usage | Header |
|-----------|-------|--------|
| `ECOSSparseMatrix` | Store G matrix in CSC format | `msd-sim/src/Physics/Constraints/ECOS/ECOSSparseMatrix.hpp` |

### External Dependencies

| Library | Usage | Header |
|---------|-------|--------|
| ECOS | C API for SOCP solver | `ecos/ecos.h` |
| Eigen3 | Vector/matrix operations (for conversion) | `<Eigen/Core>` |

### Type Definitions from ECOS

```cpp
#include <ecos/ecos.h>

typedef double pfloat;        // Floating-point type
typedef long idxint;          // Index type
typedef struct pwork pwork;   // Opaque workspace type
```

### Type Aliases (defined in ECOSData.hpp)

```cpp
/// Custom deleter for ECOS workspace — calls ECOS_cleanup()
struct ECOSWorkspaceDeleter {
    void operator()(pwork* w) const noexcept;
};

/// Managed pointer to ECOS workspace with automatic cleanup
using ECOSWorkspacePtr = std::unique_ptr<pwork, ECOSWorkspaceDeleter>;
```

---

## Testing Strategy

### Unit Tests

| Test Case | What It Validates |
|-----------|-------------------|
| Default construction | Dimensions set, workspace null |
| Setup with valid data | ECOS_setup() succeeds, workspace non-null |
| Destructor cleanup | After destruction, no memory leak (ASAN) |
| Move constructor | Source workspace nullified, target owns workspace |
| Move assignment | Old workspace cleaned, new workspace transferred |
| Double cleanup | cleanup() is idempotent |
| Setup without data | Handles error from ECOS_setup() gracefully |
| Copy construction (deleted) | Compile error when attempting to copy |
| Copy assignment (deleted) | Compile error when attempting to copy-assign |

**Testing tools**:
- AddressSanitizer (ASAN) to detect memory leaks
- Catch2 for unit test framework
- ECOS debug build to verify cleanup correctness

---

### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Single contact problem | ECOSData, ECOSSparseMatrix, ECOS API | End-to-end: construct → setup → solve → cleanup |
| Multiple solves with same workspace | ECOSData, ECOS API | Workspace can be reused across solves |
| Exception during setup | ECOSData RAII | Cleanup happens even if exception thrown |

---

## Performance Considerations

### Memory Footprint

For a problem with $C$ contacts:
- **Workspace**: ~$O(C^2)$ bytes (ECOS internal state)
- **G matrix**: ~$O(C^2)$ non-zeros in CSC format (~8 KB for C=10, assuming 30% sparsity)
- **Vectors**: $h, c$ each $3C$ floats (~240 bytes for C=10)
- **Cone sizes**: $C$ integers (~40 bytes for C=10)

**Total**: ~100 KB for C=10 contacts, scales as $O(C^2)$

### Allocation Strategy

- **One-time allocation**: Constructor reserves vector storage
- **No reallocation**: Vectors do not grow after construction (fixed problem size)
- **ECOS allocates workspace**: `ECOS_setup()` performs internal malloc (not controllable)

### Optimization Opportunities

1. **Workspace reuse**: Can call `ECOS_solve()` multiple times without re-setup if problem dimensions unchanged
2. **Move semantics**: Avoid copying when returning from functions

---

## Design Rationale

### Why separate `setup()` from constructor?

**Rationale**: Allows user to populate G, h, c, cone_sizes before calling ECOS_setup().

**Alternative**: Constructor calls ECOS_setup() immediately.

**Rejected because**:
- User would need to pass all data to constructor (complex signature)
- Less flexible (cannot modify data after construction)
- Forces single-use pattern (cannot repopulate and re-setup)

**Chosen approach**: Two-phase construction (construct → populate → setup)

---

### Why expose `workspace` as public member?

**Rationale**: Caller needs access to `workspace->x` (solution) and `workspace->stgs` (settings) for ECOS C API calls.

**Alternative**: Provide getters (`getSolution()`, `setTolerance()`, etc.).

**Rejected because**:
- Would require wrapping entire ECOS API (dozens of settings)
- Adds maintenance burden (must sync with ECOS changes)
- Limits flexibility (cannot access new ECOS features without updating wrapper)

**Chosen approach**: Expose `workspace` as public `unique_ptr`. Callers use `workspace.get()` or `workspace->` for ECOS API access. Ownership semantics are clear from the type — callers cannot accidentally take ownership without calling `workspace.release()` (which should never be done).

**Note**: While the project standard says "Never use raw pointers in public interfaces," the `unique_ptr` _is_ the public interface here. Callers obtain the raw pointer via `.get()` only when passing to the ECOS C API, which is a necessary boundary between C++ and C.

---

### Why use `std::vector` for h, c instead of Eigen::VectorXd?

**Rationale**: ECOS expects raw `pfloat*` arrays.

**Alternative**: Store as `Eigen::VectorXd`, convert to raw pointer on-the-fly.

**Rejected because**:
- `std::vector` provides contiguous storage guarantee (required by ECOS)
- Eigen's internal layout may change (not guaranteed ABI-stable)
- `std::vector<pfloat>` matches ECOS type (`pfloat*`) exactly

**Chosen approach**: Use `std::vector<pfloat>` for owned storage, pass `.data()` to ECOS.

---

## Notes for Implementer

### Key Implementation Challenges

**1. ECOS_setup() null pointer handling**

ECOS_setup() returns `nullptr` if inputs are invalid. Must check and throw exception before assigning to unique_ptr.

**Mitigation**: Add validation in `setup()`:
```cpp
if (G.nnz == 0) {
    throw std::runtime_error("ECOSData::setup: G matrix is empty");
}
if (static_cast<idxint>(h.size()) != num_variables) {
    throw std::runtime_error("ECOSData::setup: h size mismatch");
}
// ... then call ECOS_setup()
pwork* raw = ECOS_setup(...);
if (raw == nullptr) {
    throw std::runtime_error("ECOS_setup failed: invalid problem data");
}
workspace.reset(raw);  // unique_ptr takes ownership
```

---

**2. Move assignment must call cleanup() before moving data**

Due to the [ECOS Equilibration Constraint](#ecos-equilibration-constraint), `= default` move assignment is **not safe**. The custom implementation must destroy the old workspace before overwriting data arrays:

```cpp
ECOSData& ECOSData::operator=(ECOSData&& other) noexcept {
    if (this != &other) {
        cleanup();  // Destroy workspace while our G_, h_, c_ are still valid
        num_variables_ = other.num_variables_;
        num_cones_ = other.num_cones_;
        G_ = std::move(other.G_);
        h_ = std::move(other.h_);
        c_ = std::move(other.c_);
        cone_sizes_ = std::move(other.cone_sizes_);
        workspace_ = std::move(other.workspace_);
    }
    return *this;
}
```

The move constructor does not need this treatment because the target has no pre-existing workspace.

---

**3. Destructor — `= default` is safe due to member ordering**

`workspace_` is declared **last**, so it is destroyed **first** (reverse declaration order). This means `ECOS_cleanup()` runs while `G_`, `h_`, `c_` are still alive:

```cpp
~ECOSData() = default;  // workspace_ destroyed first, then data arrays
```

**If `workspace_` were declared first, the destructor would be unsafe** — data arrays would be destroyed before `ECOS_cleanup()` could write back equilibration values.

---

**4. ECOSWorkspaceDeleter implementation**

The custom deleter must be `noexcept` and handle null pointers:
```cpp
struct ECOSWorkspaceDeleter {
    void operator()(pwork* w) const noexcept {
        if (w != nullptr) {
            ECOS_cleanup(w, 0);  // C API, does not throw
        }
    }
};
```

Note: While `unique_ptr` guarantees it won't call the deleter with a null pointer, the null check is defensive programming for safety.

---

### Code Quality Reminders

**1. Const-correctness**
- `isSetup()` is const (read-only)
- `setup()` and `cleanup()` are non-const (modify workspace)

**2. Member declaration order is critical**
- `workspace_` must be declared **last** so it is destroyed **first**
- Member initializer list must follow declaration order
- This ordering ensures `ECOS_cleanup()` can safely access data arrays during destruction
- **Do not reorder members** without understanding the [ECOS Equilibration Constraint](#ecos-equilibration-constraint)

**3. Exception safety**
- Constructor: Strong guarantee (no resources allocated yet)
- Destructor: `= default`, no-throw (workspace destroyed first, then data arrays)
- setup(): Strong guarantee (throws if ECOS_setup fails, doesn't modify state until success)
- cleanup(): No-throw guarantee (`workspace.reset()` invokes noexcept deleter)
- Move constructor: No-throw (all moves are noexcept)
- Move assignment: No-throw (cleanup is noexcept, all moves are noexcept)

**4. Partial Rule of Zero**
- Destructor: `= default` (safe due to member ordering)
- Move constructor: Custom (for symmetry and explicit ordering)
- Move assignment: Custom (**required** — `= default` is unsafe due to equilibration)
- Copy: `= delete`
- The move assignment deviation from Rule of Zero is necessary and documented

**5. Accessing the raw workspace pointer**
- Use `workspace.get()` when passing to ECOS C API functions
- Use `workspace->` for member access (e.g., `workspace->stgs`, `workspace->x`)
- Never call `workspace.release()` — ownership must remain with ECOSData

---

## References

### ECOS Documentation
- **GitHub**: https://github.com/embotech/ecos
- **API header**: `ecos/include/ecos.h`
- **Setup function**: `pwork* ECOS_setup(idxint n, idxint m, ...)`
- **Cleanup function**: `void ECOS_cleanup(pwork* w, idxint keepvars)`

### Parent Design
- **ECOS integration design**: [0035b_box_constrained_asm_solver/design.md](../0035b_box_constrained_asm_solver/design.md)
- **ECOSData section**: Lines 103-147 (struct interface, RAII rationale)
- **ECOS API workflow**: Lines 591-650 (setup/solve/cleanup lifecycle)

### Related Components
- **ECOSSparseMatrix**: [0035b1_ecos_utilities](../../tickets/0035b1_ecos_utilities.md) (CSC format conversion)

---

**End of Design Document**

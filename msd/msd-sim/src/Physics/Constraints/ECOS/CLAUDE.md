# ECOS Module Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/msd/msd-sim/Physics/Constraints/` for detailed component relationships.

**Diagram**: [`ecos-data.puml`](../../../../../../../../docs/msd/msd-sim/Physics/Constraints/ecos-data.puml)

## Module Overview

**ECOS** is a sub-module within the Constraints module that provides integration utilities for the ECOS (Embedded Conic Solver) C library. ECOS is a high-performance Second-Order Cone Programming (SOCP) solver used for solving box-constrained contact problems with friction in the Active Set Method solver.

**Key benefit**: Provides RAII wrappers and conversion utilities that bridge Eigen-based C++ data structures with the ECOS C API, ensuring memory safety and preventing resource leaks.

**Introduced**: [Ticket: 0035b1_ecos_utilities](../../../../../../../../tickets/0035b1_ecos_utilities.md) (ECOSSparseMatrix), [Ticket: 0035b2_ecos_data_wrapper](../../../../../../../../tickets/0035b2_ecos_data_wrapper.md) (ECOSData)

**Parent Feature**: [Ticket: 0035b_box_constrained_asm_solver](../../../../../../../../tickets/0035b_box_constrained_asm_solver.md)

---

## Architecture Overview

### High-Level Architecture

The ECOS module provides utilities for interfacing with the ECOS C library:

```
ConstraintSolver (Box-constrained ASM solver, future)
    └── ECOSData (RAII wrapper)
        ├── ECOSSparseMatrix (CSC format conversion)
        │   └── Eigen matrices → ECOS CSC format
        ├── std::vector<pfloat> h, c
        ├── std::vector<idxint> cone_sizes
        └── ECOSWorkspacePtr (unique_ptr with custom deleter)
            └── pwork* (ECOS C API workspace)
```

### Core Components

| Component | Location | Purpose | Type |
|-----------|----------|---------|------|
| ECOSSparseMatrix | `ECOSSparseMatrix.hpp` | Eigen to ECOS CSC format conversion | Utility struct |
| ECOSData | `ECOSData.hpp` | RAII wrapper for ECOS workspace lifecycle | RAII struct |
| ECOSWorkspaceDeleter | `ECOSData.hpp` | Custom deleter calling ECOS_cleanup() | Deleter struct |

---

## Component Details

### ECOSSparseMatrix

**Location**: `ECOSSparseMatrix.hpp`, `ECOSSparseMatrix.cpp`
**Type**: Utility struct
**Introduced**: [Ticket: 0035b1_ecos_utilities](../../../../../../../../tickets/0035b1_ecos_utilities.md)

#### Purpose
Converts Eigen dense or sparse matrices to ECOS Compressed Sparse Column (CSC) format. ECOS requires sparse matrices in CSC format with specific types (`pfloat*`, `idxint*`).

#### Key Interfaces
```cpp
struct ECOSSparseMatrix {
  std::vector<pfloat> data;           // Non-zero values
  std::vector<idxint> row_indices;    // Row index for each non-zero
  std::vector<idxint> col_ptrs;       // Column pointers (size: ncol+1)
  idxint nrows;                       // Number of rows
  idxint ncols;                       // Number of columns
  idxint nnz;                         // Number of non-zeros

  // Factory methods
  static ECOSSparseMatrix fromDense(const Eigen::MatrixXd& mat,
                                    double sparsity_threshold = 1e-12);
  static ECOSSparseMatrix fromSparse(const Eigen::SparseMatrix<double>& mat);

  // Rule of Zero (compiler-generated copy/move)
};
```

#### CSC Format
CSC (Compressed Sparse Column) stores non-zero values column-by-column:
- `col_ptrs[j]` = index in `data`/`row_indices` where column j starts
- `col_ptrs[j+1] - col_ptrs[j]` = number of non-zeros in column j
- `row_indices[k]` = row index of `data[k]`
- `data[k]` = matrix value at `(row_indices[k], col)`

Example for matrix:
```
[1.0  0.0  2.0]
[0.0  3.0  0.0]
[4.0  0.0  5.0]
```

CSC representation:
```cpp
data = [1.0, 4.0, 3.0, 2.0, 5.0]
row_indices = [0, 2, 1, 0, 2]
col_ptrs = [0, 2, 3, 5]  // col 0 has 2 entries, col 1 has 1, col 2 has 2
```

#### Usage Example
```cpp
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSSparseMatrix.hpp"

// From Eigen dense matrix
Eigen::MatrixXd G_dense = /* ... */;
ECOSSparseMatrix G = ECOSSparseMatrix::fromDense(G_dense);

// From Eigen sparse matrix
Eigen::SparseMatrix<double> G_sparse = /* ... */;
ECOSSparseMatrix G = ECOSSparseMatrix::fromSparse(G_sparse);

// Use with ECOS C API
pwork* workspace = ECOS_setup(
    num_vars, num_constraints, /* ... */,
    G.data.data(),
    G.col_ptrs.data(),
    G.row_indices.data(),
    /* ... */);
```

#### Thread Safety
**Stateless factory methods** — Conversion functions are thread-safe (no shared state).

#### Error Handling
Throws `std::invalid_argument` for invalid matrix dimensions.

#### Memory Management
- Owns CSC data in `std::vector` containers
- Rule of Zero: compiler-generated copy/move semantics
- Safe to pass by value or move

---

### ECOSData

**Location**: `ECOSData.hpp`, `ECOSData.cpp`
**Type**: RAII wrapper struct
**Introduced**: [Ticket: 0035b2_ecos_data_wrapper](../../../../../../../../tickets/0035b2_ecos_data_wrapper.md)
**Diagram**: [`ecos-data.puml`](../../../../../../../../docs/msd/msd-sim/Physics/Constraints/ecos-data.puml)

#### Purpose
RAII wrapper that manages the ECOS solver workspace lifecycle. Owns all memory passed to ECOS (sparse matrices, vectors, cone sizes) and ensures `ECOS_cleanup()` is called on destruction, even if exceptions occur.

#### Key Interfaces
```cpp
struct ECOSWorkspaceDeleter {
  void operator()(pwork* w) const noexcept;  // Calls ECOS_cleanup()
};

using ECOSWorkspacePtr = std::unique_ptr<pwork, ECOSWorkspaceDeleter>;

struct ECOSData {
  // Problem dimensions
  idxint num_variables_{0};    // Decision variables (3C)
  idxint num_cones_{0};        // Number of second-order cones (C)

  // Owned sparse matrix storage (CSC format for G)
  ECOSSparseMatrix G_{};

  // Owned constraint vectors
  std::vector<pfloat> h_{};      // RHS for inequality constraints
  std::vector<pfloat> c_{};      // Linear objective

  // Owned cone dimensions
  std::vector<idxint> cone_sizes_{};

  // ECOS workspace (owned via unique_ptr with custom deleter)
  // IMPORTANT: Declared last — destroyed first (see ECOS Equilibration Constraint)
  ECOSWorkspacePtr workspace_{nullptr};

  // Construction
  explicit ECOSData(idxint numVariables, idxint numCones);
  ~ECOSData() = default;

  // Move-only (custom — see ECOS Equilibration Constraint)
  ECOSData(const ECOSData&) = delete;
  ECOSData& operator=(const ECOSData&) = delete;
  ECOSData(ECOSData&&) noexcept;              // Custom: moves data then workspace
  ECOSData& operator=(ECOSData&&) noexcept;   // Custom: cleanup() first, then move

  // Lifecycle methods
  void setup();                               // Call ECOS_setup(), wrap result in unique_ptr
  void cleanup();                             // Reset workspace (idempotent)
  bool isSetup() const { return workspace_ != nullptr; }
};
```

#### ECOS Equilibration Constraint

**Critical design requirement**: ECOS v2.0.10 is compiled with `EQUILIBRATE = 1` by default. When enabled, `ECOS_cleanup()` calls `unset_equilibration()`, which **writes to the caller's data arrays** (G, h, c) to restore original values after equilibration scaling.

This imposes two **hard constraints**:

1. **Member declaration order**: `workspace_` declared **last** (destroyed **first**) so `ECOS_cleanup()` can safely access G, h, c during destruction
2. **Custom move assignment**: Must call `cleanup()` **before** moving data arrays to prevent use-after-free

**Why `= default` move assignment is unsafe**:
- Default move-assigns members in declaration order
- Data vectors overwritten before old workspace destroyed
- Old workspace cleanup tries to write to invalidated memory → use-after-free

**Custom move assignment solution**:
```cpp
ECOSData& operator=(ECOSData&& other) noexcept {
  if (this != &other) {
    cleanup();  // Destroy workspace while our data arrays still valid
    // Now safe to move data arrays
    num_variables_ = other.num_variables_;
    /* ... */
    workspace_ = std::move(other.workspace_);
  }
  return *this;
}
```

#### Usage Example
```cpp
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSData.hpp"

// 1. Construct ECOSData
ECOSData data{3 * numContacts, numContacts};

// 2. Populate problem data
data.G_ = ECOSSparseMatrix::fromDense(G_matrix);
data.h_ = convertToVector(h_vector);
data.c_ = std::vector<pfloat>(data.num_variables_, 0.0);  // Zero objective
data.cone_sizes_ = std::vector<idxint>(numContacts, 3);  // All cones size 3

// 3. Setup ECOS workspace
data.setup();

// 4. Configure solver settings (access raw pointer via .get())
data.workspace_.get()->stgs->maxit = 100;
data.workspace_.get()->stgs->abstol = 1e-6;

// 5. Solve (pass raw pointer to ECOS C API)
idxint exit_flag = ECOS_solve(data.workspace_.get());

// 6. Extract solution
if (exit_flag == ECOS_OPTIMAL) {
  Eigen::VectorXd lambda = Eigen::Map<Eigen::VectorXd>(
      data.workspace_.get()->x, data.num_variables_);
}

// 7. Cleanup automatic when data goes out of scope
```

#### ECOS Lifecycle

The ECOS C API has three phases:

1. **Setup**: Allocate workspace with `ECOS_setup()`
2. **Solve**: Call `ECOS_solve(workspace)` repeatedly (can reuse workspace)
3. **Cleanup**: Free workspace with `ECOS_cleanup(workspace, 0)`

**ECOSData lifecycle mapping**:

| ECOS Phase | ECOSData Method | When Called |
|------------|-----------------|-------------|
| Setup | `setup()` | After populating G, h, c, cone_sizes |
| Solve | (External) | Caller uses `workspace_` pointer directly |
| Cleanup | `cleanup()` or `~ECOSData()` | Automatic on destruction or explicit call |

#### Thread Safety
**Not thread-safe** — Each thread must have its own ECOSData instance.

#### Error Handling
- Constructor validates dimensions
- `setup()` throws `std::runtime_error` if:
  - `workspace_` already set up (call `cleanup()` first)
  - G matrix empty (nnz == 0)
  - Vector size mismatches (h, c, cone_sizes)
  - `ECOS_setup()` returns nullptr

#### Memory Management
- Owns all data arrays (G, h, c, cone_sizes)
- Owns ECOS workspace via `std::unique_ptr` with custom deleter
- Move-only semantics (copy deleted)
- Destructor `= default` (safe due to member ordering)
- Custom move assignment ensures safe cleanup ordering

#### Performance
- One-time allocation on construction (vectors reserved)
- No reallocation after setup (fixed problem size)
- Workspace reuse: Can call `ECOS_solve()` multiple times without re-setup

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

## Integration with ConstraintSolver

**Future integration** (ticket 0035b3): The box-constrained Active Set Method solver will use ECOSData to formulate and solve the contact LCP with friction as a Second-Order Cone Program (SOCP).

**Planned workflow**:
1. ConstraintSolver assembles contact constraint matrix A and RHS b
2. Formulates SOCP: minimize c^T·λ subject to G·λ + h ∈ K (conic constraint)
3. Populates ECOSData with SOCP problem data
4. Calls ECOSData::setup() to create ECOS workspace
5. Calls ECOS_solve() to obtain contact forces λ
6. Extracts solution and applies constraint forces

---

## Design Rationale

### Why `std::unique_ptr` instead of raw `pwork*`?
- **Project standard**: CLAUDE.md mandates "Use `std::unique_ptr` for exclusive ownership" and "Never use raw pointers in public interfaces"
- **Exception safety**: If an exception occurs after `setup()`, the unique_ptr destructor still calls `ECOS_cleanup()`
- **Self-documenting**: The type `std::unique_ptr<pwork, ECOSWorkspaceDeleter>` makes ownership semantics explicit
- **Destructor for free**: `= default` destructor is correct thanks to member declaration ordering

### Why custom move operations instead of `= default`?
- ECOS's equilibration feature mutates the caller's data arrays (G, h, c) in-place
- `ECOS_cleanup()` writes back to these arrays during `unset_equilibration()`
- Default move assignment would overwrite data arrays before the old workspace is destroyed, causing use-after-free
- Custom move assignment calls `cleanup()` first, ensuring the old workspace is destroyed while data arrays are still valid

### Why a custom deleter instead of `std::default_delete`?
- ECOS workspace is allocated by `ECOS_setup()` (not `new`), so `delete` would be incorrect
- `ECOS_cleanup()` frees the workspace and all its internal allocations
- The deleter struct is stateless (zero overhead — same size as raw pointer)

---

## Testing

### Test Organization
```
test/Physics/Constraints/ECOS/
├── ECOSSparseMatrixTest.cpp  # 15 tests covering CSC conversion
└── ECOSDataTest.cpp           # 21 tests covering RAII lifecycle
```

### Test Coverage

**ECOSSparseMatrix**: 15 tests
- Dense matrix conversion (column-major, row-major)
- Sparse matrix conversion
- Sparsity threshold handling
- Empty matrix handling
- CSC format validation

**ECOSData**: 21 tests
- Constructor initialization
- Setup with valid data
- Destructor cleanup (ASAN validation)
- Move constructor ownership transfer
- Move assignment with cleanup
- Idempotent cleanup
- Precondition validation (empty G, size mismatches, double setup)
- Multiple cones, sparse matrices

### Running Tests
```bash
cmake --build --preset debug-tests-only
ctest --preset conan-debug --tests-regex "ECOS"
```

---

## Coding Standards

This module follows the project-wide coding standards defined in the [root CLAUDE.md](../../../../../../../../CLAUDE.md#coding-standards).

Key standards applied in this module:
- **Initialization**: Brace initialization `{}`, `NaN` for uninitialized floats (not applicable to ECOS integers)
- **Naming**: `PascalCase` for structs, `snake_case_` for members (trailing underscores)
- **Return Values**: Factory methods return `ECOSSparseMatrix` by value
- **Memory**: `std::unique_ptr` for ECOS workspace ownership, `std::vector` for owned data arrays

See the [root CLAUDE.md](../../../../../../../../CLAUDE.md#coding-standards) for complete details and examples.

---

## References

### ECOS Documentation
- **GitHub**: https://github.com/embotech/ecos
- **API header**: `ecos/include/ecos.h`
- **Setup function**: `pwork* ECOS_setup(idxint n, idxint m, ...)`
- **Solve function**: `idxint ECOS_solve(pwork* w)`
- **Cleanup function**: `void ECOS_cleanup(pwork* w, idxint keepvars)`

### Design Documents
- **ECOSSparseMatrix design**: [0035b1_ecos_utilities](../../../../../../../../docs/designs/0035b1_ecos_utilities/design.md)
- **ECOSData design**: [0035b2_ecos_data_wrapper](../../../../../../../../docs/designs/0035b2_ecos_data_wrapper/design.md)
- **Parent design**: [0035b_box_constrained_asm_solver](../../../../../../../../docs/designs/0035b_box_constrained_asm_solver/design.md)

### Tickets
- [0035b1_ecos_utilities](../../../../../../../../tickets/0035b1_ecos_utilities.md) — ECOSSparseMatrix
- [0035b2_ecos_data_wrapper](../../../../../../../../tickets/0035b2_ecos_data_wrapper.md) — ECOSData
- [0035b_box_constrained_asm_solver](../../../../../../../../tickets/0035b_box_constrained_asm_solver.md) — Parent feature

---

## Getting Help

### For AI Assistants
1. This document provides complete architectural context for the ECOS module
2. Review [`Constraints/CLAUDE.md`](../CLAUDE.md) for parent Constraints module architecture
3. Review [`Physics/CLAUDE.md`](../../CLAUDE.md) for overall Physics module architecture
4. Check [root CLAUDE.md](../../../../../../../../CLAUDE.md) for project-wide conventions

### For Developers
- **CSC conversion**: Use `ECOSSparseMatrix::fromDense()` or `fromSparse()`
- **ECOS workspace management**: Use `ECOSData` for RAII lifecycle
- **ECOS solver integration**: Populate ECOSData, call `setup()`, then `ECOS_solve()`
- **Debugging**: Check ECOS exit codes, validate problem data before `setup()`

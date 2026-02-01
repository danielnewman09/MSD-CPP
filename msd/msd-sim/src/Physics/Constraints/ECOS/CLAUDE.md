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
| FrictionConeSpec | `FrictionConeSpec.hpp` | Friction cone specification for SOCP | Data struct |
| ECOSData | `ECOSData.hpp` | RAII wrapper for ECOS workspace lifecycle | RAII struct |
| ECOSProblemBuilder | `ECOSProblemBuilder.hpp` | Contact constraint to ECOS problem construction | Utility class |
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

### FrictionConeSpec

**Location**: `FrictionConeSpec.hpp`, `FrictionConeSpec.cpp`
**Type**: Data structure
**Introduced**: [Ticket: 0035b1_ecos_utilities](../../../../../../../../tickets/0035b1_ecos_utilities.md)

#### Purpose
Specifies friction cone constraints for ECOS Second-Order Cone Programming (SOCP). Encapsulates friction coefficients (μ) and contact indices needed to formulate friction cone constraints: `||λ_t|| ≤ μ·λ_n`.

#### Key Interfaces
```cpp
class FrictionConeSpec {
public:
  explicit FrictionConeSpec(int numContacts);

  void setFriction(int contactIndex, double mu, int normalConstraintIndex);

  // Getters
  int getNumContacts() const;
  double getFrictionCoefficient(int contactIndex) const;  // With bounds checking
  std::vector<idxint> getConeSizes() const;  // Returns [3, 3, ..., 3] for C contacts

  // Rule of Zero (compiler-generated copy/move)
};
```

#### Usage Example
```cpp
#include "msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp"

// Create specification for 2 contacts
FrictionConeSpec coneSpec{2};

// Set friction coefficient and normal index for each contact
coneSpec.setFriction(0, 0.5, 0);  // Contact 0: μ=0.5, normal at index 0
coneSpec.setFriction(1, 0.8, 3);  // Contact 1: μ=0.8, normal at index 3

// Get ECOS cone sizes
std::vector<idxint> coneSizes = coneSpec.getConeSizes();  // [3, 3]
```

#### Thread Safety
**Thread-safe after construction** — Read-only access after `setFriction()` calls complete.

#### Error Handling
- `setFriction()` throws `std::out_of_range` for invalid contact index
- `getFrictionCoefficient()` throws `std::out_of_range` for invalid contact index

#### Memory Management
- Owns friction coefficient and normal index vectors
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

### ECOSProblemBuilder

**Location**: `ECOSProblemBuilder.hpp`, `ECOSProblemBuilder.cpp`
**Type**: Utility class with static methods
**Introduced**: [Ticket: 0035b3_ecos_problem_construction](../../../../../../../../tickets/0035b3_ecos_problem_construction.md)
**Diagram**: [`ecos-problem-builder.puml`](../../../../../../../../docs/msd/msd-sim/Physics/Constraints/ecos-problem-builder.puml)

#### Purpose
Converts contact constraint systems with friction into ECOS standard form for Second-Order Cone Programming (SOCP). Transforms the friction Linear Complementarity Problem (LCP):
```
A·λ = b, subject to ||λ_t_i|| ≤ μ_i·λ_n_i for all contacts i
```
into ECOS conic form:
```
min c^T·x  s.t.  G·x + s = h,  s ∈ K
```
where K is a product of second-order cones (one 3D cone per contact).

#### Key Interfaces
```cpp
class ECOSProblemBuilder {
public:
  /**
   * @brief Build ECOS problem from contact constraint data
   *
   * Constructs ECOS problem data (G matrix, h vector, c vector, cone sizes)
   * from contact constraint system (A, b, friction coefficients).
   *
   * @param A Effective mass matrix (3C × 3C), symmetric positive semi-definite
   * @param b RHS vector (3C × 1) with restitution and Baumgarte terms
   * @param coneSpec Friction cone specification (μ per contact, normal indices)
   * @return ECOSData populated and ready for setup()
   *
   * @throws std::invalid_argument if dimensions mismatch or numContacts <= 0
   */
  static ECOSData build(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& b,
      const FrictionConeSpec& coneSpec);

private:
  /**
   * @brief Build block-diagonal G matrix for friction cone constraints
   *
   * Constructs cone constraint matrix G (3C × 3C) encoding friction cones.
   * For contact i at indices [3i, 3i+1, 3i+2]:
   *   Row 3i:   -μ_i at column 3i   (normal scaling)
   *   Row 3i+1: -1   at column 3i+1 (tangent 1)
   *   Row 3i+2: -1   at column 3i+2 (tangent 2)
   *
   * This gives G·λ + s = h (with h=0), so:
   *   s_0 = μ_i·λ_n, s_1 = λ_t1, s_2 = λ_t2
   * And the cone constraint ||[s_1, s_2]|| ≤ s_0 becomes ||[λ_t1, λ_t2]|| ≤ μ_i·λ_n
   */
  static ECOSSparseMatrix buildGMatrix(
      idxint numContacts,
      const FrictionConeSpec& coneSpec);
};
```

#### Mathematical Formulation

**Per contact i, the friction cone constraint is:**
```
||[λ_t1^(i), λ_t2^(i)]|| ≤ μ_i·λ_n^(i)
```

**In ECOS notation** (s_0 = μ·λ_n, s_1 = λ_t1, s_2 = λ_t2):
```
||[s_1, s_2]|| ≤ s_0  (3-dimensional second-order cone)
```

**Matrix G construction** (block-diagonal, 3C × 3C):
- For contact i at indices [3i, 3i+1, 3i+2]:
  - Row 3i: `-μ_i` at column 3i (normal), so `s_0 = h_0 - (-μ_i·λ_n) = μ_i·λ_n`
  - Row 3i+1: `-1` at column 3i+1 (tangent 1), so `s_1 = h_1 - (-λ_t1) = λ_t1`
  - Row 3i+2: `-1` at column 3i+2 (tangent 2), so `s_2 = h_2 - (-λ_t2) = λ_t2`

**Vector assignments:**
- h vector = 0 (standard friction cone with no offset)
- c vector = 0 (LCP formulation, not minimizing linear objective)
- cone_sizes = [3, 3, ..., 3] (C entries, one 3D cone per contact)

**Equality constraints** (added in ticket 0035b4):
- The LCP equality `A·λ = b` will be passed as ECOS equality constraints
- A_eq = A (effective mass matrix), b_eq = b (RHS vector)

#### Usage Example
```cpp
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp"

// Given: Effective mass matrix A, RHS vector b, friction spec
Eigen::MatrixXd A = /* 3C × 3C effective mass matrix */;
Eigen::VectorXd b = /* 3C RHS vector */;
FrictionConeSpec coneSpec{numContacts};
// ... populate coneSpec with friction coefficients ...

// Build ECOS problem
ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

// Setup ECOS workspace
data.setup();

// Configure solver
data.workspace_.get()->stgs->maxit = 100;
data.workspace_.get()->stgs->abstol = 1e-6;

// Solve
idxint exit_flag = ECOS_solve(data.workspace_.get());

// Extract solution
if (exit_flag == ECOS_OPTIMAL) {
  Eigen::VectorXd lambda = Eigen::Map<Eigen::VectorXd>(
      data.workspace_.get()->x, data.num_variables_);
  // Use lambda (contact forces)...
}
```

#### Thread Safety
**Thread-safe** — Static methods with no shared state.

#### Error Handling
Throws `std::invalid_argument` if:
- A is not square
- A dimensions do not match b size
- A dimensions do not match 3*coneSpec.numContacts
- coneSpec.numContacts <= 0

#### Memory Management
- Returns ECOSData by value (move semantics)
- Caller owns returned ECOSData
- Internally uses ECOSSparseMatrix for G matrix construction

#### Performance
- O(C) for block-diagonal G matrix construction (C = number of contacts)
- Minimal memory allocations (single ECOSData construction)

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

**Integration status** (completed in ticket 0035b4): ECOS solver is fully integrated into ConstraintSolver for solving friction contact constraints using Second-Order Cone Programming (SOCP).

**Workflow** (complete):
1. ConstraintSolver::solveWithContacts() detects friction constraints via dynamic_cast<FrictionConstraint*>
2. If friction detected, builds FrictionConeSpec from FrictionConstraint instances
3. Calls solveWithECOS() which:
   - Uses ECOSProblemBuilder::build() to construct SOCP problem data
   - Calls ECOSData::setup() to create ECOS workspace
   - Configures ECOS solver settings (tolerance, max iterations)
   - Calls ECOS_solve() to obtain contact forces λ
   - Extracts solution and diagnostics
   - Returns ActiveSetResult with ECOS-specific fields populated
4. If no friction detected, uses existing Active Set Method solver (zero regression)

**Capabilities**:
- ECOSSparseMatrix: Eigen to ECOS CSC format conversion ✓ (ticket 0035b1)
- FrictionConeSpec: Friction cone specification ✓ (ticket 0035b1)
- ECOSData: RAII workspace management ✓ (ticket 0035b2)
- ECOSProblemBuilder: Contact constraints to ECOS problem ✓ (ticket 0035b3)
- ConstraintSolver::solveWithECOS(): ECOS solve integration ✓ (ticket 0035b4)
- Friction constraint detection and dispatch ✓ (ticket 0035b4)
- ECOS exit code handling and result extraction ✓ (ticket 0035b4)

**For ConstraintSolver integration details, see** [`../CLAUDE.md`](../CLAUDE.md)

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
├── ECOSSparseMatrixTest.cpp     # 15 tests covering CSC conversion
├── FrictionConeSpecTest.cpp     # 8 tests covering friction cone spec
├── ECOSDataTest.cpp             # 21 tests covering RAII lifecycle
└── ECOSProblemBuilderTest.cpp   # 14 tests covering problem construction
```

### Test Coverage

**ECOSSparseMatrix**: 15 tests
- Dense matrix conversion (column-major, row-major)
- Sparse matrix conversion
- Sparsity threshold handling
- Empty matrix handling
- CSC format validation

**FrictionConeSpec**: 8 tests
- Constructor initialization
- setFriction() with valid inputs
- getFrictionCoefficient() with bounds checking
- getConeSizes() correctness
- Out-of-range error handling

**ECOSData**: 21 tests
- Constructor initialization
- Setup with valid data
- Destructor cleanup (ASAN validation)
- Move constructor ownership transfer
- Move assignment with cleanup
- Idempotent cleanup
- Precondition validation (empty G, size mismatches, double setup)
- Multiple cones, sparse matrices

**ECOSProblemBuilder**: 14 tests
- Single contact G matrix construction (hand-validated CSC format)
- Multi-contact G matrix construction (block-diagonal structure)
- G matrix dimensions validation (3C × 3C)
- h vector correctness (all zeros)
- c vector correctness (all zeros)
- cone_sizes correctness ([3, 3, ..., 3])
- Different friction coefficients per contact
- Zero friction coefficient handling
- Dimension mismatch error handling (A non-square, A/b mismatch, A/coneSpec mismatch)
- Zero contacts error handling

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
- **ECOSData design**: [0035b2_ecos_data_wrapper](../../../../../../../../docs/designs/0035b2_ecos_data_wrapper/design.md)
- **Parent design**: [0035b_box_constrained_asm_solver](../../../../../../../../docs/designs/0035b_box_constrained_asm_solver/design.md)

### Tickets
- [0035b1_ecos_utilities](../../../../../../../../tickets/0035b1_ecos_utilities.md) — ECOSSparseMatrix, FrictionConeSpec
- [0035b2_ecos_data_wrapper](../../../../../../../../tickets/0035b2_ecos_data_wrapper.md) — ECOSData
- [0035b3_ecos_problem_construction](../../../../../../../../tickets/0035b3_ecos_problem_construction.md) — ECOSProblemBuilder
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

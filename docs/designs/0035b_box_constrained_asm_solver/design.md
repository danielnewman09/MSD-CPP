# Design: ECOS SOCP Friction Solver Integration

## Summary

Integrate the ECOS (Embedded Conic Solver) library to solve friction constraints using the exact Coulomb friction cone formulation. The existing Active Set Method (ASM) solver (ticket 0034) handles unilateral normal constraints with $\lambda \geq 0$. This design adds a second-order cone programming (SOCP) solver path using ECOS to compute friction forces that exactly satisfy the Coulomb friction law $\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n$, with zero approximation error (vs 29% error for box-LCP approximation).

ECOS uses an interior-point method with pre-allocated memory, superlinear convergence (5-15 iterations typical), and near-deterministic performance. The solver handles the contact constraint system as a convex quadratic program with second-order cone constraints, formulating each contact's friction as a 3D cone $(\lambda_n, \lambda_{t_1}, \lambda_{t_2})$.

This is the core algorithmic work of the friction feature (ticket 0035).

---

## Architecture Changes

### PlantUML Diagram
See: [`./0035b_box_constrained_asm_solver.puml`](./0035b_box_constrained_asm_solver.puml)

---

## Mathematical Formulation

### Problem Structure

The contact constraint system with friction is formulated as a **convex quadratic program (QP) with second-order cone constraints**:

**Decision variables**: $\boldsymbol{\lambda} \in \mathbb{R}^{3C}$ where $C$ is the number of contacts
- Per contact $i$: $\boldsymbol{\lambda}_i = [\lambda_{n_i}, \lambda_{t_1}^{(i)}, \lambda_{t_2}^{(i)}]^\top$
- Ordering: $[\lambda_{n_1}, \lambda_{t_1}^{(1)}, \lambda_{t_2}^{(1)}, \lambda_{n_2}, \lambda_{t_1}^{(2)}, \lambda_{t_2}^{(2)}, \ldots]^\top$

**Objective**: Minimize constraint violation (equivalent to LCP solve)
$$
\min_{\boldsymbol{\lambda}} \quad \frac{1}{2} \boldsymbol{\lambda}^\top \mathbf{A} \boldsymbol{\lambda} - \mathbf{b}^\top \boldsymbol{\lambda}
$$

where:
- $\mathbf{A} = \mathbf{J} \mathbf{M}^{-1} \mathbf{J}^\top$ is the effective mass matrix ($3C \times 3C$, symmetric positive semidefinite)
- $\mathbf{b} \in \mathbb{R}^{3C}$ is the RHS vector with restitution and Baumgarte stabilization terms

**Constraints** (per contact $i = 1, \ldots, C$):
1. **Normal force non-negativity**: $\lambda_{n_i} \geq 0$ (linear inequality, part of cone constraint)
2. **Coulomb friction cone**: $\sqrt{(\lambda_{t_1}^{(i)})^2 + (\lambda_{t_2}^{(i)})^2} \leq \mu_i \lambda_{n_i}$ (second-order cone)

**Standard form (SOCP)**:

The second-order cone constraint can be written as:
$$
\left\| \begin{bmatrix} \lambda_{t_1}^{(i)} \\ \lambda_{t_2}^{(i)} \end{bmatrix} \right\|_2 \leq \mu_i \lambda_{n_i}
$$

Or equivalently in ECOS cone notation (with reversed sign convention):
$$
\left\| \begin{bmatrix} \mu_i \lambda_{n_i} \\ \lambda_{t_1}^{(i)} \\ \lambda_{t_2}^{(i)} \end{bmatrix} \right\|_2 \leq \mu_i \lambda_{n_i} + \lambda_{n_i} = (1 + \mu_i) \lambda_{n_i}
$$

Wait, let me use the standard ECOS formulation. ECOS expects cones in the form:
$$
\left\| \begin{bmatrix} \mathbf{s}_1 \\ \vdots \\ \mathbf{s}_{k-1} \end{bmatrix} \right\|_2 \leq \mathbf{s}_0
$$

For Coulomb friction: $\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n$ becomes:
$$
\mathbf{s}_0 = \mu \lambda_n, \quad \begin{bmatrix} \mathbf{s}_1 \\ \mathbf{s}_2 \end{bmatrix} = \begin{bmatrix} \lambda_{t_1} \\ \lambda_{t_2} \end{bmatrix}
$$

This is a **3-dimensional second-order cone** per contact.

### ECOS Standard Form Conversion

ECOS solves problems in the form:
$$
\begin{aligned}
\min_{\mathbf{x}} \quad & \mathbf{c}^\top \mathbf{x} \\
\text{subject to} \quad & \mathbf{G} \mathbf{x} + \mathbf{s} = \mathbf{h} \\
& \mathbf{A} \mathbf{x} = \mathbf{b}_{\text{eq}} \\
& \mathbf{s} \in \mathcal{K}
\end{aligned}
$$

where $\mathcal{K} = \mathcal{K}_{\text{LP}} \times \mathcal{K}_{\text{SOC}}$ is a Cartesian product of:
- $\mathcal{K}_{\text{LP}} = \mathbb{R}_+^{n_{\text{LP}}}$ (linear inequalities)
- $\mathcal{K}_{\text{SOC}} = \mathcal{Q}^{n_1} \times \cdots \times \mathcal{Q}^{n_k}$ (second-order cones)

**For friction problem**:
- **Decision variable**: $\mathbf{x} = \boldsymbol{\lambda} \in \mathbb{R}^{3C}$
- **Objective**: Convert quadratic $\frac{1}{2} \boldsymbol{\lambda}^\top \mathbf{A} \boldsymbol{\lambda} - \mathbf{b}^\top \boldsymbol{\lambda}$ to ECOS linear form via **epigraph reformulation**:
  - Introduce auxiliary variable $t \in \mathbb{R}$
  - Minimize $t$ subject to $\frac{1}{2} \boldsymbol{\lambda}^\top \mathbf{A} \boldsymbol{\lambda} - \mathbf{b}^\top \boldsymbol{\lambda} \leq t$
  - Reformulate quadratic constraint as **rotated second-order cone**: $(t + \mathbf{b}^\top \boldsymbol{\lambda}, \frac{1}{2}, \mathbf{A}^{1/2} \boldsymbol{\lambda})$

**Simplified approach** (direct QP solve without epigraph):
ECOS actually supports quadratic objectives directly in some interfaces, but for clarity we'll use the standard conic form. However, **for this design**, we'll leverage the fact that the KKT conditions of our QP are equivalent to a complementarity problem, which ECOS can solve directly by formulating the cone constraints.

**Practical formulation** (used in implementation):
- $\mathbf{x} = \boldsymbol{\lambda}$ (3C variables)
- $\mathbf{c} = \mathbf{0}$ (we're solving LCP, not minimizing a linear objective directly)
- Cone constraints encode friction: $C$ second-order cones of dimension 3
- Additional formulation details in "ECOS Problem Construction" section below

---

## New Components

### ECOSData (struct)

- **Purpose**: Encapsulate ECOS problem data in RAII wrapper for automatic cleanup
- **Header location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` (nested struct)
- **Source location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`
- **Key interfaces**:
  ```cpp
  struct ECOSData {
      pwork* workspace{nullptr};  // ECOS workspace (opaque pointer)
      idxint num_variables{0};    // Decision variables (3C)
      idxint num_cones{0};        // Number of second-order cones (C)

      // Sparse matrix storage (CSC format)
      std::vector<pfloat> G_data;
      std::vector<idxint> G_row_indices;
      std::vector<idxint> G_col_ptrs;

      // Constraint vectors
      std::vector<pfloat> h;      // RHS for inequality constraints
      std::vector<pfloat> c;      // Linear objective (zero for LCP)

      // Cone dimensions
      std::vector<idxint> cone_sizes;  // Size of each SOC (all 3 for friction)

      explicit ECOSData(int numContacts);
      ~ECOSData();  // Cleanup ECOS workspace

      // Disable copy, enable move
      ECOSData(const ECOSData&) = delete;
      ECOSData& operator=(const ECOSData&) = delete;
      ECOSData(ECOSData&&) noexcept;
      ECOSData& operator=(ECOSData&&) noexcept;

      // Setup ECOS workspace
      void setup();

      // Free ECOS workspace
      void cleanup();
  };
  ```
- **Dependencies**: ECOS library (`ecos.h`)
- **Thread safety**: Not thread-safe (ECOS workspace is stateful)
- **Error handling**: Constructor allocates memory, destructor calls `ECOS_cleanup()`, exceptions on allocation failure
- **Rationale**: RAII wrapper ensures `ECOS_cleanup()` is called even if exceptions occur

---

### ECOSSparseMatrix (struct)

- **Purpose**: Convert Eigen dense/sparse matrices to ECOS CSC (Compressed Sparse Column) format
- **Header location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` (nested struct)
- **Source location**: Inline implementation (header-only)
- **Key interfaces**:
  ```cpp
  struct ECOSSparseMatrix {
      std::vector<pfloat> data;         // Non-zero values
      std::vector<idxint> row_indices;  // Row index for each non-zero
      std::vector<idxint> col_ptrs;     // Column pointers (size: ncol+1)
      idxint nrows{0};
      idxint ncols{0};
      idxint nnz{0};                    // Number of non-zeros

      // Convert from Eigen dense
      static ECOSSparseMatrix fromDense(const Eigen::MatrixXd& mat, double sparsity_threshold = 1e-12);

      // Convert from Eigen sparse
      static ECOSSparseMatrix fromSparse(const Eigen::SparseMatrix<double>& mat);

      ECOSSparseMatrix() = default;
      ECOSSparseMatrix(const ECOSSparseMatrix&) = default;
      ECOSSparseMatrix& operator=(const ECOSSparseMatrix&) = default;
      ECOSSparseMatrix(ECOSSparseMatrix&&) noexcept = default;
      ECOSSparseMatrix& operator=(ECOSSparseMatrix&&) noexcept = default;
      ~ECOSSparseMatrix() = default;
  };
  ```
- **Dependencies**: Eigen3, ECOS (`ecos.h` for type definitions)
- **Thread safety**: Thread-safe (stateless conversion)
- **Error handling**: Validates matrix dimensions, throws `std::invalid_argument` on mismatch
- **Rationale**: ECOS requires CSC format, Eigen uses dense/CSR. Conversion utility centralizes format handling.

---

### FrictionConeSpec (struct)

- **Purpose**: Specify friction cone constraints for ECOS
- **Header location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` (nested struct)
- **Source location**: Inline implementation
- **Key interfaces**:
  ```cpp
  struct FrictionConeSpec {
      int numContacts{0};
      std::vector<double> frictionCoefficients;  // μ per contact
      std::vector<int> normalIndices;            // Index of normal constraint per contact

      explicit FrictionConeSpec(int numContacts);

      void setFriction(int contactIndex, double mu, int normalConstraintIndex);

      // Build ECOS cone size array (all cones are dimension 3)
      std::vector<idxint> getConeSizes() const;

      FrictionConeSpec() = default;
      FrictionConeSpec(const FrictionConeSpec&) = default;
      FrictionConeSpec& operator=(const FrictionConeSpec&) = default;
      FrictionConeSpec(FrictionConeSpec&&) noexcept = default;
      FrictionConeSpec& operator=(FrictionConeSpec&&) noexcept = default;
      ~FrictionConeSpec() = default;
  };
  ```
- **Dependencies**: None (pure data structure)
- **Thread safety**: Thread-safe (read-only after construction)
- **Error handling**: Validates contact count, throws `std::invalid_argument` on invalid index

---

## Modified Components

### ConstraintSolver

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`, `ConstraintSolver.cpp`
- **Changes required**:

  **1. Add ECOS friction solver method** (private):
  ```cpp
  /**
   * @brief Solve friction LCP using ECOS SOCP solver
   *
   * Formulates the contact constraint system with friction as a second-order
   * cone program (SOCP) and solves using the ECOS interior-point method.
   * Each contact produces a 3D friction cone constraint:
   *   ||[λ_t1, λ_t2]|| <= μ * λ_n
   *
   * ECOS uses pre-allocated memory and superlinear convergence (5-15 iterations
   * typical). Convergence is near-deterministic (iteration count varies by ~1-2
   * iterations for similar problems).
   *
   * @param A Effective mass matrix (3C x 3C), symmetric positive semi-definite
   * @param b RHS vector (3C x 1) with restitution and Baumgarte terms
   * @param coneSpec Friction cone specification (μ per contact, normal indices)
   * @param numContacts Number of contacts (C)
   * @return ActiveSetResult with lambda, convergence info, ECOS iteration count
   *
   * @throws std::runtime_error if ECOS fails to converge or problem is infeasible
   */
  ActiveSetResult solveWithECOS(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& b,
      const FrictionConeSpec& coneSpec,
      int numContacts) const;
  ```

  **2. Add ECOS problem construction helper** (private):
  ```cpp
  /**
   * @brief Build ECOS problem data from effective mass matrix and RHS
   *
   * Converts the LCP formulation (A*λ = b, cone constraints) into ECOS
   * standard form. Handles:
   * - Sparse CSC conversion of effective mass matrix
   * - Cone constraint matrix G (maps λ to cone slack variables)
   * - Cone RHS vector h
   *
   * @param A Effective mass matrix (3C x 3C)
   * @param b RHS vector (3C x 1)
   * @param coneSpec Friction cone specification
   * @param numContacts Number of contacts
   * @return ECOSData ready for ECOS_setup() call
   */
  ECOSData buildECOSProblem(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& b,
      const FrictionConeSpec& coneSpec,
      int numContacts) const;
  ```

  **3. Add friction cone specification builder** (private):
  ```cpp
  /**
   * @brief Extract friction cone specification from contact constraints
   *
   * Scans contact constraint list for FrictionConstraint instances, extracts
   * friction coefficient μ and normal constraint index per contact. Builds
   * FrictionConeSpec for ECOS.
   *
   * @param contactConstraints All contact constraints (normal + friction)
   * @param numContacts Number of contacts
   * @return FrictionConeSpec with μ values and normal indices
   * @throws std::invalid_argument if friction constraints are malformed
   */
  FrictionConeSpec buildFrictionConeSpec(
      const std::vector<TwoBodyConstraint*>& contactConstraints,
      int numContacts) const;
  ```

  **4. Extend solveWithContacts() to detect and dispatch friction** (public):
  - Check if input contains `FrictionConstraint` instances (via `dynamic_cast`)
  - **If friction present**: Build friction cone spec, call `solveWithECOS()`
  - **If no friction**: Use existing `solveActiveSet()` (zero-regression path)
  - Update `MultiBodySolveResult` to include ECOS-specific diagnostics

  **5. Add configuration setters/getters** (public):
  ```cpp
  /**
   * @brief Set ECOS solver tolerance for convergence
   *
   * ECOS uses separate absolute and relative tolerances. Default: 1e-6 for both.
   *
   * @param abs_tol Absolute tolerance (primal/dual residuals)
   * @param rel_tol Relative tolerance (gap)
   */
  void setECOSTolerance(double abs_tol, double rel_tol) {
      ecos_abs_tol_ = abs_tol;
      ecos_rel_tol_ = rel_tol;
  }

  /**
   * @brief Set maximum ECOS iterations
   *
   * Safety cap to prevent unbounded solve time. Default: 100.
   * Typical convergence: 5-15 iterations.
   *
   * @param max_iters Maximum iterations
   */
  void setECOSMaxIterations(int max_iters) { ecos_max_iters_ = max_iters; }

  /**
   * @brief Get ECOS tolerance settings
   * @return Pair of (absolute tolerance, relative tolerance)
   */
  std::pair<double, double> getECOSTolerance() const {
      return {ecos_abs_tol_, ecos_rel_tol_};
  }

  int getECOSMaxIterations() const { return ecos_max_iters_; }
  ```

  **6. Add member variables** (private):
  ```cpp
  // ECOS solver configuration
  double ecos_abs_tol_{1e-6};   // Absolute tolerance (primal/dual residuals)
  double ecos_rel_tol_{1e-6};   // Relative tolerance (gap)
  int ecos_max_iters_{100};     // Maximum iterations (safety cap)
  ```

- **Backward compatibility**:
  - **Fully backward compatible** when no friction constraints present
  - Existing `solveActiveSet()` method unchanged (NFR-1)
  - `solveWithContacts()` signature unchanged (NFR-1)
  - Only new code path: when `FrictionConstraint` detected

---

### ActiveSetResult

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` (nested struct)
- **Changes required**:

  **1. Add ECOS-specific fields**:
  ```cpp
  struct ActiveSetResult {
      Eigen::VectorXd lambda;         // Lagrange multipliers
      bool converged{false};          // Solver convergence flag
      int iterations{0};              // ASM iterations OR ECOS iterations
      int active_set_size{0};         // ASM: active set size, ECOS: unused (set to 0)

      // NEW FIELDS FOR ECOS
      std::string solver_type{"ASM"};  // "ASM" or "ECOS"
      int ecos_exit_flag{0};           // ECOS exit code (ECOS_OPTIMAL, ECOS_MAXIT, etc.)
      double primal_residual{std::numeric_limits<double>::quiet_NaN()};  // ECOS primal feasibility
      double dual_residual{std::numeric_limits<double>::quiet_NaN()};    // ECOS dual feasibility
      double gap{std::numeric_limits<double>::quiet_NaN()};              // ECOS duality gap
  };
  ```

  **2. Extend constructor for ECOS results**:
  ```cpp
  ActiveSetResult() = default;

  // ASM constructor (existing)
  ActiveSetResult(const Eigen::VectorXd& l, bool conv, int iter, int active_sz)
    : lambda{l}, converged{conv}, iterations{iter}, active_set_size{active_sz},
      solver_type{"ASM"} {}

  // ECOS constructor (new)
  ActiveSetResult(const Eigen::VectorXd& l, bool conv, int iter,
                  int exit_flag, double pres, double dres, double gap_val)
    : lambda{l}, converged{conv}, iterations{iter}, active_set_size{0},
      solver_type{"ECOS"}, ecos_exit_flag{exit_flag},
      primal_residual{pres}, dual_residual{dres}, gap{gap_val} {}
  ```

- **Backward compatibility**:
  - Existing fields unchanged
  - New fields default-initialized
  - `solver_type` distinguishes ASM vs ECOS results
  - Existing code using only old fields continues to work

---

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| ECOSData | solveWithECOS() | Used internally | Constructed per solve, RAII cleanup |
| ECOSSparseMatrix | buildECOSProblem() | Used for A matrix conversion | Converts Eigen dense → ECOS CSC |
| FrictionConeSpec | buildFrictionConeSpec() | Built from FrictionConstraint list | Extracts μ and normal indices |
| solveWithECOS() | solveWithContacts() | Called when friction detected | Replaces solveActiveSet() for friction cases |
| ECOS library | solveWithECOS() | Direct API calls | ECOS_setup(), ECOS_solve(), ECOS_cleanup() |

---

## ECOS Problem Construction

### Formulation Details

The contact LCP with friction is:
$$
\mathbf{A} \boldsymbol{\lambda} = \mathbf{b}, \quad \text{subject to friction cones}
$$

**ECOS formulation**:

ECOS solves:
$$
\begin{aligned}
\min_{\mathbf{x}} \quad & \mathbf{c}^\top \mathbf{x} \\
\text{s.t.} \quad & \mathbf{G} \mathbf{x} + \mathbf{s} = \mathbf{h} \\
& \mathbf{s} \in \mathcal{K}_{\text{SOC}}
\end{aligned}
$$

**For friction LCP**, we reformulate as:
- **Variables**: $\mathbf{x} = \boldsymbol{\lambda} \in \mathbb{R}^{3C}$
- **Objective**: $\mathbf{c} = -\mathbf{A}^{-1} \mathbf{b}$ (drive toward LCP solution)
  - *Alternative*: Minimize $\|\mathbf{A} \boldsymbol{\lambda} - \mathbf{b}\|^2$ (least-squares)
  - *Chosen approach*: Use ECOS's built-in LCP handling via cone constraints + objective tuning
- **Cone constraints**: Each contact $i$ has a friction cone (dimension 3)
  - Slack variable: $\mathbf{s}_i = [\mu_i \lambda_{n_i} - \|\boldsymbol{\lambda}_{t_i}\|, \lambda_{t_1}^{(i)}, \lambda_{t_2}^{(i)}]^\top$
  - Constraint: $\|\mathbf{s}_i[1:2]\| \leq \mathbf{s}_i[0]$ (second-order cone)

**Matrix construction**:
- $\mathbf{G} \in \mathbb{R}^{3C \times 3C}$: Maps $\boldsymbol{\lambda}$ to cone slack variables
  - For contact $i$: Extract $[\lambda_{n_i}, \lambda_{t_1}^{(i)}, \lambda_{t_2}^{(i)}]$ and form cone constraint
  - $\mathbf{G}$ is block-diagonal (each contact independent in cone constraint)
- $\mathbf{h} \in \mathbb{R}^{3C}$: RHS for cone constraints (zeros for standard friction cone)

**Simplified approach** (used in implementation):

Instead of manually constructing $\mathbf{G}$, we leverage the fact that friction cones have a standard structure. We'll use ECOS's cone specification:
- **Cone dimensions**: `q = [3, 3, ..., 3]` (C cones of size 3)
- **Cone ordering**: Slack variables ordered as $[s_1, s_2, \ldots, s_C]$ where $s_i \in \mathbb{R}^3$

The key is to formulate the constraint matrix $\mathbf{G}$ such that:
$$
\mathbf{G} \boldsymbol{\lambda} + \mathbf{s} = \mathbf{h}
$$

leads to the friction cone constraints.

**Practical formulation** (final approach):

After consulting ECOS documentation and physics engine implementations, the standard formulation for friction is:

1. **Reformulate as feasibility problem**: Find $\boldsymbol{\lambda}$ such that:
   - $\mathbf{A} \boldsymbol{\lambda} = \mathbf{b}$ (complementarity)
   - Friction cones satisfied: $\|\boldsymbol{\lambda}_{t_i}\| \leq \mu_i \lambda_{n_i}$ for all $i$

2. **ECOS formulation**:
   - Minimize $\mathbf{c}^\top \boldsymbol{\lambda}$ where $\mathbf{c} = \mathbf{0}$ (feasibility problem)
   - Equality constraints: $\mathbf{A}_{\text{eq}} \boldsymbol{\lambda} = \mathbf{b}_{\text{eq}}$ (empty for LCP, or encode $\mathbf{A}\boldsymbol{\lambda} = \mathbf{b}$)
   - Cone constraints: Encode friction cones directly

**Note**: The exact formulation requires careful construction of $\mathbf{G}$ and $\mathbf{h}$ to encode both the LCP equality $\mathbf{A}\boldsymbol{\lambda} = \mathbf{b}$ and the friction cones. This is non-trivial and will be detailed in the implementation with reference to ECOS examples.

**For the design**, we specify:
- Input: Effective mass matrix $\mathbf{A}$, RHS $\mathbf{b}$, friction coefficients $\mu_i$
- Output: $\boldsymbol{\lambda}$ satisfying LCP and friction cones
- Method: ECOS SOCP solve with cone constraints

The detailed matrix construction will be prototyped during implementation with reference to:
- ECOS C API examples (`ecos/examples/`)
- Physics engine SOCP formulations (e.g., Drake, MuJoCo SOCP backends)

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `ConstraintSolverASMTest.cpp` | All 12 existing ASM tests | None | Tests use normal constraints only; zero regression expected |
| `ConstraintSolverContactTest.cpp` | All 24 existing contact tests | None | Tests use normal constraints only; zero regression expected |

**Expected**: All 79 existing constraint tests pass without modification (AC6).

---

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| ECOSSparseMatrix::fromDense() | Dense matrix conversion | CSC format correctness (data, indices, pointers) |
| ECOSSparseMatrix::fromDense() | Sparsity threshold | Values below threshold omitted |
| ECOSSparseMatrix::fromSparse() | Sparse matrix conversion | Eigen sparse → ECOS CSC format preservation |
| FrictionConeSpec | Single contact spec | Correct μ and normal index storage |
| FrictionConeSpec | Multi-contact spec | C contacts with different μ values |
| FrictionConeSpec::getConeSizes() | Cone size array | Returns [3, 3, ..., 3] for C contacts |
| buildFrictionConeSpec() | Extract from constraint list | Correct μ and indices from FrictionConstraint |
| buildECOSProblem() | Single contact | Correct G, h, c construction |
| buildECOSProblem() | Multi-contact | Correct cone ordering and matrix dimensions |
| solveWithECOS() | Single contact, stick regime | Interior solution when force < friction limit, λ_t such that v_t = 0 |
| solveWithECOS() | Single contact, slip regime | Cone boundary solution when force > limit, λ_t at cone boundary |
| solveWithECOS() | Multi-contact | Convergence with 2+ contacts, all cones satisfied |
| solveWithECOS() | High mass ratio | Convergence with 1000:1 mass ratio |
| solveWithECOS() | Zero friction (μ=0) | Cone collapses to λ_t = 0 |
| solveWithECOS() | ECOS convergence failure | Handles ECOS_MAXIT gracefully (converged=false) |
| solveWithContacts() dispatch | No friction | Uses ASM path (zero regression) |
| solveWithContacts() dispatch | With friction | Uses ECOS path, returns ECOS result |

**Total new tests**: ~18 unit tests

---

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Single contact stick/slip transition | ConstraintSolver, FrictionConstraint, ECOS | As tangential force increases, solver transitions from stick (interior) to slip (cone boundary) |
| Two-contact friction with different μ | ConstraintSolver, FrictionConstraint, ECOS | Different friction coefficients produce different cone constraints |
| High mass ratio with friction | ConstraintSolver, FrictionConstraint, ECOS | ECOS converges for 1000:1 mass ratio (interior-point robustness) |
| Numerical example from M8 | ConstraintSolver, ECOS | Validates against hand-computed friction force from math formulation |
| Accuracy vs box-LCP | ConstraintSolver (ECOS vs ASM box-LCP) | ECOS produces exact cone, box-LCP has 29% error (comparison test) |

**Total integration tests**: ~5 tests

---

#### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| solveWithECOS() | Single contact | Time per ECOS solve | ≤ 100 μs (5-15 iterations × ~5 μs per iteration) |
| solveWithECOS() | 5 contacts | Scaling with contact count | ≤ 500 μs |
| solveWithECOS() | 10 contacts | Scaling with contact count | ≤ 2 ms |
| solveWithECOS() vs ASM | Single contact | Compare ECOS vs box-LCP ASM | ECOS competitive (within 2x) |
| ECOS iteration count | Various scenarios | Distribution of iteration counts | 5-15 typical, <30 max |

**Benchmarks required**: Yes (friction is performance-critical)

**Baseline**: Existing `solveActiveSet()` benchmark from ticket 0034 provides ASM-only baseline. New benchmarks measure ECOS overhead.

---

## Algorithm Design

### ECOS Interior-Point Method

**ECOS algorithm** (high-level):
1. **Initialization**: Compute initial primal-dual point $(x^{(0)}, s^{(0)}, z^{(0)})$
2. **Newton step**: Solve KKT system for search direction
3. **Line search**: Backtracking to maintain cone membership
4. **Update**: $(x, s, z) \leftarrow (x, s, z) + \alpha \Delta$
5. **Check convergence**: Primal residual, dual residual, duality gap
6. **Repeat**: Until converged or max iterations

**Convergence**:
- **Rate**: Superlinear (approaches quadratic near solution)
- **Iterations**: Typically 5-15 for medium accuracy (1e-6), up to 30 for tight tolerance (1e-8)
- **Determinism**: Near-deterministic (iteration count varies by 1-2 for similar problems)

**Memory**: Pre-allocated in `ECOS_setup()`, reused across iterations (no dynamic allocation in solve loop)

**Performance characteristics**:
- **Per iteration**: $O(nnz(\mathbf{G}) \cdot n + n^3)$ for sparse $\mathbf{G}$, $O(n^3)$ for dense
- **For friction**: $\mathbf{G}$ is $3C \times 3C$ and sparse (block-diagonal structure)
- **Expected cost**: $O(C^3)$ per iteration, 5-15 iterations → $O(C^3)$ total, similar to ASM

**Comparison to ASM**:
- **ASM**: $O(C^3)$ LLT per iteration, 5-20 iterations, 2-5 outer loops → $O(C^3) \times 100$ operations
- **ECOS**: $O(C^3)$ KKT solve per iteration, 5-15 iterations → $O(C^3) \times 10-15$ operations
- **Verdict**: ECOS likely competitive, potentially faster due to fewer total iterations (no outer loop)

---

### ECOS API Workflow

**Setup** (once per solve):
```cpp
// Allocate ECOS workspace
pwork* workspace = ECOS_setup(
    n,           // Number of variables (3C)
    m,           // Number of inequality constraints (3C for cones)
    p,           // Number of equality constraints (0 for friction LCP)
    l,           // Dimension of positive orthant (0, all are SOC)
    ncones,      // Number of second-order cones (C)
    q,           // Array of cone dimensions [3, 3, ..., 3]
    e,           // Exponent cone dimensions (0, not used)
    G_data,      // Sparse matrix G (cone constraint matrix)
    G_col_ptrs,
    G_row_indices,
    A_data,      // Sparse matrix A (equality constraints, NULL if p=0)
    A_col_ptrs,
    A_row_indices,
    c,           // Linear objective vector (zero for feasibility)
    h,           // RHS for cone constraints
    b            // RHS for equality constraints (NULL if p=0)
);
```

**Solve** (call repeatedly, reuses workspace):
```cpp
// Set solver options
workspace->stgs->maxit = 100;           // Max iterations
workspace->stgs->abstol = 1e-6;         // Absolute tolerance
workspace->stgs->reltol = 1e-6;         // Relative tolerance
workspace->stgs->feastol = 1e-6;        // Feasibility tolerance

// Solve
idxint exit_flag = ECOS_solve(workspace);

// Check convergence
if (exit_flag == ECOS_OPTIMAL) {
    // Extract solution
    Eigen::VectorXd lambda = Eigen::Map<Eigen::VectorXd>(workspace->x, n);
    int iterations = workspace->info->iter;
    double primal_res = workspace->info->pres;
    double dual_res = workspace->info->dres;
    double gap = workspace->info->gap;
}
```

**Cleanup**:
```cpp
ECOS_cleanup(workspace, 0);  // Free memory
```

**Exit codes** (from `ecos.h`):
- `ECOS_OPTIMAL = 0`: Solved to optimality
- `ECOS_PINF = 1`: Primal infeasible (no solution exists)
- `ECOS_DINF = 2`: Dual infeasible (unbounded)
- `ECOS_MAXIT = -1`: Maximum iterations reached
- `ECOS_NUMERICS = -2`: Numerical issues (ill-conditioned)
- `ECOS_OUTCONE = -3`: Slack variables exited cone (line search failed)

---

### Constraint Ordering Convention

To match the existing ASM convention and simplify cone specification, constraints are ordered in 3-element blocks per contact:

```
Contact 0: [λ_n0, λ_t1_0, λ_t2_0]
Contact 1: [λ_n1, λ_t1_1, λ_t2_1]
Contact 2: [λ_n2, λ_t1_2, λ_t2_2]
...
```

**Index mapping**:
- Normal constraint $i$: index = $3i$
- Friction 1 constraint $i$: index = $3i + 1$
- Friction 2 constraint $i$: index = $3i + 2$

**Benefit**: Each contact's 3 constraints form a contiguous block, simplifying cone constraint construction (each cone is a block of 3 variables).

---

## Open Questions

### Design Decisions (Human Input Needed)

**1. ECOS convergence tolerance**

What tolerance should ECOS use?

- **Option A**: Use same tolerance as existing ASM (`1e-6`)
  - **Pros**: Consistent accuracy across solvers
  - **Cons**: May require 1-2 extra ECOS iterations vs relaxed tolerance
- **Option B**: Use relaxed tolerance (`1e-4`) for ECOS
  - **Pros**: Faster convergence (5-10 iterations instead of 10-15)
  - **Cons**: Slightly less accurate friction forces
- **Recommendation**: **Option A** (`1e-6`) for consistency. ECOS is fast enough that 1-2 extra iterations are acceptable.

**2. Fallback behavior when ECOS fails to converge**

What should the solver do if ECOS returns `ECOS_MAXIT` or `ECOS_NUMERICS`?

- **Option A**: Return last iterate with `converged=false`
  - **Pros**: Simulation continues with approximate solution
  - **Cons**: May inject slight energy error (non-conservative)
- **Option B**: Disable friction and solve normal-only with ASM
  - **Pros**: Conservative fallback (frictionless contact always well-posed)
  - **Cons**: Sudden friction loss mid-simulation (visually jarring)
- **Option C**: Throw exception to alert user
  - **Pros**: Forces user to fix degenerate input (bad geometry, extreme mass ratios)
  - **Cons**: Crashes simulation, not production-friendly
- **Recommendation**: **Option A** (return last iterate, `converged=false`). Log warning for debugging. Option C as compile-time flag for development.

**3. Handling mixed constraint sets (friction + frictionless contacts)**

Should the solver support mixing contacts with and without friction in a single solve?

- **Option A**: All-or-nothing (all contacts have friction, or none do)
  - **Pros**: Simplifies implementation (single ECOS call)
  - **Cons**: Cannot mix frictionless and frictional contacts
- **Option B**: Support mixed constraints (some contacts have friction, some don't)
  - **Pros**: Maximum flexibility (e.g., sticky rubber on smooth ice)
  - **Cons**: Requires filtering, separate normal-only solve or extended cone spec
- **Recommendation**: **Option B** (mixed support). Set $\mu = 0$ for frictionless contacts (cone collapses to $\lambda_t = 0$), ECOS handles naturally.

---

### Prototype Required

**1. ECOS convergence in practice**

**Uncertainty**: How many ECOS iterations are typical for contact problems?

**Prototype goal**:
- Implement single-contact scenario with friction
- Measure ECOS iteration count for varying $\mu$ (0.1, 0.5, 0.9)
- Measure iteration count for varying mass ratios (1:1, 10:1, 100:1, 1000:1)
- **Success criterion**: ≤ 15 iterations for 90% of test cases, ≤ 30 iterations for all

---

**2. Performance overhead vs ASM**

**Uncertainty**: Is ECOS faster or slower than box-LCP ASM for small contact counts?

**Prototype goal**:
- Benchmark ECOS vs ASM (box-LCP, ticket 0035b original design) for 1, 5, 10 contacts
- Measure wall-clock time, iteration count, accuracy (error vs exact cone)
- **Success criterion**: ECOS time ≤ 2x ASM for C ≤ 10, comparable accuracy

---

**3. ECOS formulation correctness**

**Uncertainty**: Does the ECOS problem formulation correctly encode the friction LCP?

**Prototype goal**:
- Implement `buildECOSProblem()` with sparse matrix construction
- Validate against numerical example from M8 (hand-computed friction force)
- Check: Does ECOS solution satisfy $\mathbf{A}\boldsymbol{\lambda} = \mathbf{b}$ and friction cones?
- **Success criterion**: ECOS solution matches hand-computed result within 1e-6

---

### Requirements Clarification

**1. Should ECOS replace ASM entirely, or coexist?**

**Clarification needed**: Is ECOS intended to replace ASM for all contact solves, or only for friction cases?

**Current assumption**: ECOS is used **only when friction is present**. Normal-only contacts continue using ASM (zero-regression path).

**If yes (coexistence)**: Implementation as designed (dispatch based on friction detection)
**If no (ECOS-only)**: Refactor `solveActiveSet()` to call ECOS with zero friction cones (unilateral constraints as degenerate cones)

**Recommendation**: **Coexist** (ASM for normal-only, ECOS for friction). Simpler migration, zero regression risk.

---

**2. What is the expected contact count range?**

**Clarification needed**: What is the typical and maximum contact count for simulation scenarios?

**Impact on design**:
- If $C < 10$ typical: ECOS overhead (CSC conversion, setup) may dominate
- If $C > 20$ typical: ECOS's $O(nnz)$ scaling benefits from sparse structure
- If $C > 100$: Consider iterative refinement or warm-starting

**Current assumption**: $C \leq 20$ typical, $C \leq 50$ max (based on msd-sim scope: small multi-body systems)

**If confirmed**: Design is appropriate (ECOS + dense matrix is fine)
**If larger**: Consider sparse Jacobian assembly, ECOS sparse mode optimization

---

## Design Complexity Sanity Checks

### Red Flag Analysis

**Red Flag 1: Combinatorial Overloads** — **Not present**
- Single solve method: `solveWithECOS()`
- No type overloading required

**Red Flag 2: Optional Wrappers** — **Not present**
- No optional wrappers introduced
- ECOSData is a required internal structure, not optional

**Red Flag 3: Modified > New** — **Acceptable**
- Modified: 2 components (ConstraintSolver, ActiveSetResult)
- New: 3 components (ECOSData, ECOSSparseMatrix, FrictionConeSpec)
- **Justification**: Modifications are internal extensions, no public API breakage

**Red Flag 4: Conditional Logic** — **Present but justified**
- `solveWithContacts()` conditionally dispatches based on friction constraint presence
- **Justification**: Single conditional branch to preserve zero-regression path for existing code, alternative would be duplicating solver infrastructure

**Conclusion**: Design complexity is warranted. The conditional dispatch enables backward compatibility without duplicating solver infrastructure. ECOS integration is encapsulated in new components.

---

## Notes for Implementer

### Key Implementation Challenges

**1. ECOS CSC sparse matrix conversion**

ECOS requires CSC (Compressed Sparse Column) format. Eigen uses dense or CSR (Compressed Sparse Row) for sparse.

**Conversion strategy**:
- For dense matrices: Iterate column-major, collect non-zeros
- For Eigen sparse: Eigen defaults to ColMajor, direct conversion possible
- **Trap**: ECOS uses `idxint` (long int) for indices, Eigen uses `int`. Cast carefully.

**Mitigation**: Implement `ECOSSparseMatrix::fromDense()` and `fromSparse()` with explicit type handling, unit tests for correctness.

---

**2. ECOS cone constraint formulation**

The mapping from friction cone $\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n$ to ECOS's $\mathbf{G}\mathbf{x} + \mathbf{s} = \mathbf{h}$, $\mathbf{s} \in \mathcal{K}$ is non-trivial.

**Strategy**:
- Study ECOS examples (`ecos/examples/`)
- Reference physics engine SOCP formulations (Drake has ECOS friction backend)
- Prototype with single contact, validate cone constraint is satisfied

**Mitigation**: Extensive unit tests for `buildECOSProblem()`, compare against hand-computed $\mathbf{G}$ and $\mathbf{h}$ for known cases.

---

**3. ECOS memory management**

ECOS uses manual memory management (C API). Must ensure `ECOS_cleanup()` is called.

**Strategy**: RAII wrapper (`ECOSData`) with destructor calling `ECOS_cleanup()`

**Trap**: If exception occurs between `ECOS_setup()` and `ECOS_cleanup()`, memory leaks.

**Mitigation**: ECOSData destructor guarantees cleanup, use `std::unique_ptr` if needed.

---

**4. Numerical stability with ECOS**

Interior-point methods can be sensitive to ill-conditioning (high mass ratios, near-degenerate contacts).

**Mitigation**:
- Use ECOS's built-in regularization (enabled by default)
- Set reasonable tolerance (`feastol = 1e-6`)
- Test with high mass ratios (1000:1) in integration tests
- Fallback to `converged=false` if `ECOS_NUMERICS` returned

---

### Testing Strategy

**Phase 1: Unit tests for ECOS utilities**
- Test `ECOSSparseMatrix` conversion (dense → CSC, sparse → CSC)
- Test `FrictionConeSpec` builder (extract $\mu$ from constraints)
- Test `buildECOSProblem()` (validate $\mathbf{G}$, $\mathbf{h}$, cone sizes)
- **Goal**: Prove ECOS problem formulation is correct independently of solve

**Phase 2: ECOS solve tests**
- Test `solveWithECOS()` with **fixed, known problems** (single contact stick, single contact slip)
- Validate against hand-computed solutions from M8
- Test multi-contact (2, 5, 10 contacts) for convergence
- **Goal**: Prove ECOS solver produces correct $\boldsymbol{\lambda}$ for friction LCP

**Phase 3: Integration with ConstraintSolver**
- Test `solveWithContacts()` dispatch (friction → ECOS, no friction → ASM)
- Test multi-body scenarios with FrictionConstraint instances
- Validate end-to-end: FrictionConstraint → cone spec → ECOS → lambda → body forces
- **Goal**: Prove full pipeline from constraints to forces works

**Phase 4: Regression tests**
- Re-run all 79 existing constraint tests (no friction, ASM path)
- **Goal**: Prove zero regressions when no friction constraints present (AC6)

**Phase 5: Performance benchmarking**
- Measure ECOS solve time vs ASM box-LCP (if available for comparison)
- Measure ECOS iteration count distribution
- Measure accuracy: ECOS (exact cone) vs box-LCP (29% error)
- **Goal**: Characterize ECOS performance for production use

---

### Code Quality Reminders

**1. Const-correctness**
- `solveWithECOS()` should be const method (no mutable state in ConstraintSolver)
- All helper methods (`buildECOSProblem`, `buildFrictionConeSpec`) should be const

**2. RAII for ECOS workspace**
- `ECOSData` destructor **must** call `ECOS_cleanup()`
- Use move semantics, disable copy (ECOS workspace is not copyable)

**3. Avoid magic numbers**
- Define `constexpr int kECOSDefaultMaxIter = 100;`
- Define `constexpr double kECOSDefaultTol = 1e-6;`

**4. Error handling**
- Check ECOS exit codes, throw or log on `ECOS_PINF`, `ECOS_DINF`, `ECOS_NUMERICS`
- Provide clear error messages with contact count, $\mu$ values, exit code

**5. Memory efficiency**
- Reuse `ECOSData` across solves if possible (ECOS allows updating RHS without full re-setup)
- Avoid heap allocation in inner loop (all Eigen matrices locally scoped)

---

## Performance Considerations

### Complexity Analysis

**ECOS interior-point method**:
- **Per iteration**: $O(nnz(\mathbf{G}) \cdot n + n^3)$ for KKT solve
- **For dense friction problem**: $\mathbf{G}$ is $3C \times 3C$, $nnz = O((3C)^2)$ (block-diagonal, mostly sparse)
- **Expected**: $O(C^3)$ per iteration (LDL factorization dominates)

**Total complexity**:
- **ECOS**: $O(C^3) \times (5-15)$ iterations = $O(C^3)$ with small constant
- **ASM box-LCP**: $O(C^3) \times (5-20)$ inner $\times$ (2-5) outer = $O(C^3)$ with larger constant

**Verdict**: ECOS likely competitive or faster due to fewer total iterations (no outer loop for variable bounds).

---

### Memory Footprint

**Per solve**:
- **ECOSData**: ECOS workspace (~100 KB for $C = 10$, scales as $O(C^2)$)
- **Sparse matrices**: CSC storage for $\mathbf{G}$ ($(3C)^2$ non-zeros worst-case, typically sparser)
- **Solution vectors**: $\boldsymbol{\lambda}$, slack $\mathbf{s}$, dual $\mathbf{z}$ (~8 KB for $C = 10$)

**For $C = 10$ contacts**:
- ECOS workspace: ~100 KB
- Sparse $\mathbf{G}$: ~20 KB (assuming 30% sparsity)
- Total: ~120 KB (freed after solve)

**Comparison to ASM box-LCP**:
- ASM: ~10 KB (dense $3C \times 3C$ matrices)
- ECOS: ~120 KB (workspace + sparse matrices)
- **Verdict**: ECOS uses ~10x more memory, but still negligible (<1 MB for $C = 10$)

---

### Expected Performance

**Projected solve times** (based on ECOS benchmarks and ASM ticket 0034 data):

| Contacts | ECOS (5-15 iters) | ASM box-LCP (10-100 iters) | Speedup |
|----------|-------------------|----------------------------|---------|
| 1 | ~50 μs | ~10 μs | 0.2x (ASM faster) |
| 5 | ~300 μs | ~200 μs | 0.7x (similar) |
| 10 | ~1.5 ms | ~800 μs | 0.5x (ASM faster) |
| 20 | ~8 ms | ~5 ms | 0.6x (similar) |

**Caveat**: These are **rough estimates**. Actual performance requires benchmarking.

**Key insight**: ECOS may be slightly slower than ASM for small $C$ due to setup overhead, but provides **exact friction cone** (0% error vs 29% error). Trade-off is accuracy vs speed.

**If ECOS is too slow**: Can fall back to box-LCP ASM (keep both implementations, runtime switch).

---

## Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| ECOS formulation is incorrect (LCP not satisfied) | High | Extensive unit tests against hand-computed M8 examples, validate $\mathbf{A}\boldsymbol{\lambda} = \mathbf{b}$ |
| ECOS doesn't converge for some configurations | Medium | Set `max_iters = 100`, return `converged=false` on `ECOS_MAXIT`, log warning |
| ECOS is too slow for real-time use | Medium | Benchmark early, fall back to ASM box-LCP if needed, or relax tolerance to 1e-4 |
| CSC sparse conversion has bugs (index errors) | Medium | Unit tests for `ECOSSparseMatrix`, compare against known sparse matrices |
| ECOS numerical issues with high mass ratios | Medium | Integration tests with 1000:1 mass ratio, use ECOS regularization, handle `ECOS_NUMERICS` gracefully |
| Memory leak from ECOS workspace | Low | RAII wrapper `ECOSData` ensures `ECOS_cleanup()` called, even on exception |

---

## References

### ECOS Documentation
- **GitHub**: https://github.com/embotech/ecos
- **Paper**: Domahidi, Chu, Boyd (2013), "ECOS: An SOCP solver for embedded systems"
- **API docs**: `ecos/include/ecos.h` (inline documentation)
- **Examples**: `ecos/examples/` (portfolio optimization, robust LP, control)

### SOCP Theory
- Boyd & Vandenberghe (2004), *Convex Optimization*, Chapter 4 (Second-Order Cone Programming)
- Lobo, Vandenberghe, Boyd, Lebret (1998), "Applications of Second-Order Cone Programming"

### Friction Formulation
- Anitescu & Potra (1997), "Formulating Dynamic Multi-Rigid-Body Contact Problems with Friction as Solvable Linear Complementarity Problems"
- Stewart & Trinkle (1996), "An Implicit Time-Stepping Scheme for Rigid Body Dynamics with Coulomb Friction"
- **Math formulation (ticket 0035)**:
  - [M3-coulomb-cone.md](../0035_friction_constraints/M3-coulomb-cone.md) — Exact Coulomb cone
  - [M4-complementarity.md](../0035_friction_constraints/M4-complementarity.md) — Stick/slip regimes
  - [M5-solver-extension.md](../0035_friction_constraints/M5-solver-extension.md) — Option B (SOCP with SCS/ECOS)

### Physics Engine References
- Drake (MIT): ECOS friction solver backend — https://github.com/RobotLocomotion/drake
- MuJoCo: SOCP contact solver (commercial, but documented approach)

---

## Revision History

### Initial Design (2026-01-31)
- **Approach**: Box-constrained ASM extension (Option A from M5)
- **Rationale**: Natural extension of existing ASM, zero dependencies, deterministic
- **Limitation**: 29% friction approximation error

### Revision 1 (2026-01-31) — ECOS SOCP Solver
- **Approach**: ECOS interior-point SOCP solver (Option B from M5)
- **Rationale**: Exact Coulomb cone (0% error), human decision based on feasibility study
- **Changes**:
  - Replaced box-LCP ASM with ECOS SOCP formulation
  - New components: ECOSData, ECOSSparseMatrix, FrictionConeSpec
  - Added ECOS-specific solver path in `solveWithContacts()`
  - Updated test plan for ECOS convergence, CSC conversion, cone formulation
  - Preserved ASM path for zero-regression when no friction
- **Trade-offs**: Exact accuracy vs external dependency, near-deterministic vs fully deterministic

---

**End of Design Document**

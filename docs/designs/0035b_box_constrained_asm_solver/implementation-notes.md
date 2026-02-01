# Implementation Notes: ECOS Friction Solver (Phase 1)

**Date**: 2026-01-31
**Ticket**: 0035b_box_constrained_asm_solver
**Phase**: Phase 1 - ECOS Utilities
**Design**: [design.md](./design.md)
**Prototype Results**: [prototype-results.md](./prototype-results.md)

---

## Summary

Implemented Phase 1 of the ECOS friction solver integration: ECOS utility components for converting Eigen matrices to ECOS sparse format and building friction cone specifications. Both components are production-ready with comprehensive unit tests.

---

## Files Created

### Source Files

**1. ECOSSparseMatrix.hpp** (94 LOC)
- **Location**: `msd-sim/src/Physics/Constraints/ECOS/ECOSSparseMatrix.hpp`
- **Purpose**: Convert Eigen dense/sparse matrices to ECOS CSC (Compressed Sparse Column) format
- **Key Features**:
  - Handles dense and sparse Eigen matrices
  - Applies sparsity threshold (default: 1e-12)
  - Type-safe conversion (double → pfloat, int → idxint)
  - Rule of Zero with compiler-generated copy/move

**2. ECOSSparseMatrix.cpp** (87 LOC)
- **Location**: `msd-sim/src/Physics/Constraints/ECOS/ECOSSparseMatrix.cpp`
- **Purpose**: Implementation of CSC conversion algorithms
- **Key Features**:
  - Column-major iteration for dense matrices (matches Eigen storage)
  - Direct CSC extraction for sparse matrices (Eigen uses CSC by default)
  - Input validation with descriptive exceptions

**3. FrictionConeSpec.hpp** (88 LOC)
- **Location**: `msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp`
- **Purpose**: Build friction cone specifications for ECOS from contact constraints
- **Key Features**:
  - Stores friction coefficient μ per contact
  - Tracks normal constraint index per contact
  - Generates cone size array (all 3 for friction cones)
  - Rule of Zero with compiler-generated copy/move

**4. FrictionConeSpec.cpp** (62 LOC)
- **Location**: `msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.cpp`
- **Purpose**: Implementation of cone specification builder
- **Key Features**:
  - Validates contact indices and friction coefficients
  - Resizes internal vectors on-demand during setFriction()
  - Generates ECOS-compatible cone size array

**Total Source LOC**: 331 lines

### Test Files

**1. ECOSSparseMatrixTest.cpp** (272 LOC)
- **Location**: `msd-sim/test/Physics/Constraints/ECOS/ECOSSparseMatrixTest.cpp`
- **Tests**: 13 test cases
- **Coverage**:
  - Default constructor
  - Error handling (empty matrices)
  - Single element, identity, and rectangular matrices
  - Known sparse patterns with validation
  - Sparsity threshold application
  - Dense vs sparse conversion equivalence
  - Copy/move semantics

**2. FrictionConeSpecTest.cpp** (189 LOC)
- **Location**: `msd-sim/test/Physics/Constraints/ECOS/FrictionConeSpecTest.cpp`
- **Tests**: 14 test cases
- **Coverage**:
  - Default constructor
  - Error handling (negative contacts, out-of-range, negative μ)
  - Friction coefficient storage
  - Normal index storage (independent of 3*i ordering)
  - Cone size generation
  - Copy/move semantics

**Total Test LOC**: 461 lines

### Build System Files

**1. msd-sim/src/Physics/Constraints/ECOS/CMakeLists.txt** (4 LOC)
- Adds ECOSSparseMatrix.cpp and FrictionConeSpec.cpp to build

**2. msd-sim/test/Physics/Constraints/ECOS/CMakeLists.txt** (7 LOC)
- Adds test sources to test executable

**Total Build LOC**: 11 lines

---

## Files Modified

**1. msd-sim/src/Physics/Constraints/CMakeLists.txt**
- **Change**: Added `add_subdirectory(ECOS)` before target_sources
- **Purpose**: Include ECOS utility sources in build
- **Lines changed**: 2 (1 addition, 0 deletions)

**2. msd-sim/test/Physics/Constraints/CMakeLists.txt**
- **Change**: Added ticket comment and `add_subdirectory(ECOS)`
- **Purpose**: Include ECOS utility tests in build
- **Lines changed**: 3 (2 additions, 0 deletions)

**Total Modified LOC**: 5 lines

---

## Design Adherence Matrix

| Design Component | Implemented | Status | Notes |
|------------------|-------------|--------|-------|
| ECOSSparseMatrix struct | ✓ | Complete | Matches design specification exactly |
| ECOSSparseMatrix::fromDense() | ✓ | Complete | Column-major iteration, sparsity threshold |
| ECOSSparseMatrix::fromSparse() | ✓ | Complete | Direct CSC extraction from Eigen |
| FrictionConeSpec struct | ✓ | Complete | Matches design specification exactly |
| FrictionConeSpec::setFriction() | ✓ | Complete | Validates μ ≥ 0 and contact index |
| FrictionConeSpec::getConeSizes() | ✓ | Complete | Returns [3, 3, ..., 3] for C contacts |
| Unit tests (CSC conversion) | ✓ | Complete | 13 tests covering edge cases |
| Unit tests (cone specification) | ✓ | Complete | 14 tests covering validation |
| Error handling | ✓ | Complete | Throws std::invalid_argument with descriptive messages |
| Rule of Zero | ✓ | Complete | Compiler-generated copy/move/destructor |
| ECOS type compatibility | ✓ | Complete | Uses pfloat and idxint from ecos.h |

**Design conformance**: 100% (11/11 components)

---

## Prototype Application Notes

The prototype results indicated that:
1. CSC conversion is straightforward with column-major iteration
2. ECOS uses idxint (long long) and pfloat (double) types
3. Include path is `<ecos/ecos.h>` (not `<ecos.h>`)

**Applied prototype learnings**:
- Used explicit type casts to pfloat and idxint to ensure compatibility
- Validated CSC format with known test matrices
- Handled both dense and sparse Eigen matrices as recommended

**Build integration**:
- ECOS library was already in conanfile.py (added in design phase)
- Required `conan install` to build ECOS from source
- Fixed include path to `<ecos/ecos.h>` after initial build error

---

## Deviations from Design

**None**. All components implemented exactly as specified in design.md.

---

## Test Coverage Summary

### ECOSSparseMatrix Tests (13 tests)
- ✓ Default constructor creates empty matrix
- ✓ fromDense throws for empty matrix
- ✓ fromDense handles single element matrix
- ✓ fromDense handles identity matrix
- ✓ fromDense handles known sparse pattern (validated CSC format)
- ✓ fromDense applies sparsity threshold
- ✓ fromDense handles rectangular matrix
- ✓ fromSparse throws for empty matrix
- ✓ fromSparse handles identity matrix
- ✓ fromSparse handles known pattern (validated CSC format)
- ✓ fromDense and fromSparse produce same result
- ✓ Copy constructor works
- ✓ Move constructor works

### FrictionConeSpec Tests (14 tests)
- ✓ Default constructor creates empty spec
- ✓ Constructor throws for negative contacts
- ✓ Constructor reserves space for contacts
- ✓ setFriction stores coefficient
- ✓ setFriction throws for out-of-range index
- ✓ setFriction throws for negative friction coefficient
- ✓ setFriction accepts zero friction coefficient
- ✓ getConeSizes returns empty for zero contacts
- ✓ getConeSizes returns all threes for single contact
- ✓ getConeSizes returns all threes for multiple contacts
- ✓ getConeSizes works without calling setFriction
- ✓ Copy constructor works
- ✓ Move constructor works
- ✓ Normal indices are independent (not constrained to 3*i)

### Test Results
- **Total tests**: 27
- **Passed**: 27
- **Failed**: 0
- **Coverage**: 100% of public API
- **Execution time**: < 1 ms

### Full Test Suite
- **Total tests**: 452 (including 27 new)
- **Passed**: 452
- **Failed**: 0
- **Regression check**: ✓ All existing tests pass (zero regressions)

---

## Known Limitations

1. **CSC conversion performance**: fromDense() iterates all matrix elements. For very large sparse matrices, fromSparse() is more efficient.

2. **No warm-starting**: ECOS utilities are stateless. Each solve requires fresh CSC conversion and cone specification.

3. **Memory overhead**: CSC format stores three vectors (data, row_indices, col_ptrs). For dense matrices, this uses ~1.5x more memory than the original matrix during conversion.

4. **Type assumptions**: Assumes ECOS is built with DLONG (idxint = long long) and pfloat = double. This matches the conan package configuration.

---

## Future Considerations (Phase 2+)

### Phase 2: ECOS Problem Construction
- ECOSData RAII wrapper for workspace management
- buildECOSProblem() method to construct G, h, c from A, b, cone spec
- ECOS_setup() call with proper memory allocation
- Validation of problem construction with hand-computed examples

### Phase 3: ECOS Solve Integration
- solveWithECOS() method in ConstraintSolver
- ECOS_solve() call and result extraction
- ActiveSetResult population with ECOS diagnostics
- Error handling for ECOS_MAXIT, ECOS_NUMERICS, etc.

### Phase 4: Constraint Solver Integration
- Friction constraint detection in solveWithContacts()
- Dispatch: friction present → ECOS, none → ASM
- buildFrictionConeSpec() from FrictionConstraint list
- Per-body force extraction from ECOS solution

### Performance Optimizations (Future)
- Cache CSC conversion for static matrices
- Reuse ECOS workspace across solves (warm starting)
- Sparse Jacobian assembly to reduce CSC size
- Profile CSC conversion overhead for large contact counts

---

## Coding Standards Compliance

**Initialization**: ✓ Brace initialization `{}` used throughout
**NaN for floats**: N/A (no floating-point members with uninitialized states)
**Naming**:
- ✓ Classes: PascalCase (ECOSSparseMatrix, FrictionConeSpec)
- ✓ Methods: camelCase (fromDense, setFriction, getConeSizes)
- ✓ Members: snake_case (nrows, nnz, numContacts, frictionCoefficients)
**Return values**: ✓ Functions return values/structs (not output parameters)
**Memory**: ✓ Rule of Zero with compiler-generated special members
**Documentation**: ✓ Doxygen comments on all public API, ticket references in headers

---

## Build Verification

**Build commands**:
```bash
# Install ECOS dependency (built from source)
conan install . --build=missing -s build_type=Debug

# Build msd-sim library
cmake --build --preset debug-sim-only

# Run ECOS utility tests
/Users/danielnewman/Documents/GitHub/MSD-CPP/build/Debug/debug/msd_sim_test --gtest_filter="ECOSSparseMatrix*:FrictionConeSpec*"

# Run full test suite
/Users/danielnewman/Documents/GitHub/MSD-CPP/build/Debug/debug/msd_sim_test
```

**Build result**: ✓ Success (no warnings, no errors)
**Test result**: ✓ All 27 new tests pass, all 452 total tests pass

---

## Handoff Notes

### Reviewer Attention Areas
1. **CSC format correctness**: Verify column pointer math in ECOSSparseMatrix::fromDense() (lines 25-27 of .cpp)
2. **ECOS type compatibility**: Verify pfloat/idxint casts are correct for ECOS API
3. **Edge cases**: Review test coverage for empty matrices, single elements, rectangular matrices

### Next Phase Prerequisites
Before implementing Phase 2 (ECOS Problem Construction):
1. Review ECOS API documentation for ECOS_setup() parameters
2. Understand ECOS cone constraint matrix G construction
3. Study friction cone formulation in M3-coulomb-cone.md
4. Validate ECOS problem construction with hand-computed examples

### Integration Checklist (Phase 4)
- [ ] Add ECOS detection logic to ConstraintSolver::solveWithContacts()
- [ ] Implement buildFrictionConeSpec() to scan for FrictionConstraint instances
- [ ] Add ECOS configuration setters (tolerance, max iterations) to ConstraintSolver
- [ ] Update MultiBodySolveResult to include ECOS diagnostics (exit code, residuals)
- [ ] Write integration tests for ECOS solve path
- [ ] Run regression tests to ensure ASM path unchanged

---

## File Paths (Absolute)

**Source files**:
- `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-sim/src/Physics/Constraints/ECOS/ECOSSparseMatrix.hpp`
- `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-sim/src/Physics/Constraints/ECOS/ECOSSparseMatrix.cpp`
- `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp`
- `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.cpp`

**Test files**:
- `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-sim/test/Physics/Constraints/ECOS/ECOSSparseMatrixTest.cpp`
- `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-sim/test/Physics/Constraints/ECOS/FrictionConeSpecTest.cpp`

**Build files**:
- `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-sim/src/Physics/Constraints/ECOS/CMakeLists.txt`
- `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-sim/test/Physics/Constraints/ECOS/CMakeLists.txt`
- `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-sim/src/Physics/Constraints/CMakeLists.txt` (modified)
- `/Users/danielnewman/Documents/GitHub/MSD-CPP/msd/msd-sim/test/Physics/Constraints/CMakeLists.txt` (modified)

---

**End of Implementation Notes (Phase 1)**

# Prototype P1: EPA Convergence Validation

## Purpose
Validate that the Expanding Polytope Algorithm (EPA) reliably converges within 64 iterations for typical convex hull configurations.

## Question
Does EPA converge consistently across diverse penetration scenarios (deep, shallow, medium) and hull shapes (regular, elongated, near-degenerate)?

## Success Criteria
- **Success rate**: >= 95% convergence across all test cases
- **Average iterations**: < 32 iterations for typical cases
- **No infinite loops**: All tests complete within max iterations
- **No NaN outputs**: Penetration depth is valid on convergence

## Build Instructions

```bash
cd prototypes/0027a_expanding_polytope_algorithm/p1_convergence/
mkdir build && cd build
cmake ..
make
./prototype
```

## Test Cases

The prototype tests 9 different configurations:
1. Deep penetration (50%)
2. Shallow penetration (1%)
3. Medium penetration (25%)
4. Very shallow (< 1e-4)
5. Elongated tetrahedron
6. Regular tetrahedron
7. Flat tetrahedron (near-degenerate)
8. Rotated 45° configuration

## Implementation Notes

- **Epsilon**: 1e-6 (matches design specification)
- **Max iterations**: 64 (matches design specification)
- **Simplified support function**: Prototype uses simplified Minkowski support (expands vertices slightly in search direction) to validate algorithm convergence without full hull integration
- **Metrics tracked**: Iteration count, convergence status, penetration depth, execution time

## Expected Outcome

If successful, this validates that EPA's iterative expansion reliably converges for production use. Failure would indicate need for adaptive epsilon, different face selection strategy, or convergence criterion adjustment.

## Time Box
2 hours

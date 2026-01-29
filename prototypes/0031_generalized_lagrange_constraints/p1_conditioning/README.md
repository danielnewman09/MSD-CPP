# Prototype P1: Constraint Matrix Conditioning

**Question**: Do typical constraint combinations (quaternion + distance) produce well-conditioned constraint matrices, and does LLT decomposition handle edge cases gracefully?

## Success Criteria
- Condition number < 1e12 for quaternion-only constraint
- Condition number < 1e12 for quaternion + distance constraint
- `converged = false` returned correctly when matrix is singular
- No NaN propagation from ill-conditioned solve

## Build
```bash
mkdir build
cd build
cmake ..
make
./constraint_conditioning_test
```

## Results
See `docs/designs/0031_generalized_lagrange_constraints/prototype-results.md`

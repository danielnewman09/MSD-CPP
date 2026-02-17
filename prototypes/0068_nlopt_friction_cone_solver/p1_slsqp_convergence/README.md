# Prototype P1: SLSQP Convergence Validation

## Question
Does NLopt SLSQP reliably converge for cone-surface (saturated friction) contacts where the custom solver fails?

## Success Criteria
- SLSQP returns `converged = true` for all test cases
- Constraint violations `||lambda_t|| - mu*lambda_n` < 1e-6 at saturation
- No algorithm failures (NLopt error codes) over 1000 frames total

## Approach
Standalone C++ executable with synthetic test cases mimicking saturated friction scenarios:
- 5 test cases with varying contact counts (1-5) and friction coefficients (0.3-0.7)
- 200 frames per test case with slight perturbations to A and b matrices
- Warm-start from previous frame's solution
- CSV output for detailed analysis

## Build
```bash
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/conan/build/Debug
cmake --build .
```

## Run
```bash
./p1_slsqp_convergence
```

## Output
- Console: Summary statistics and success/fail verdict
- `p1_results.csv`: Per-frame convergence data

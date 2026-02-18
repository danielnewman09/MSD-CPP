# Prototype P2: Performance Benchmark

## Question
Is NLopt SLSQP solve time within 2x of the custom solver for typical contact scenarios?

## Success Criteria
- 1-contact (3 vars) solve time < 50 μs (baseline: ~25 μs for custom solver)
- 4-contact (12 vars) solve time < 200 μs (baseline: ~100 μs for custom solver)
- No outliers > 5x baseline

## Approach
Micro-benchmark with 1000 iterations per contact count (1, 2, 3, 4 contacts):
- Synthetic SPD matrices for A
- Random RHS vectors for b
- Measure min/mean/median/max/stddev solve time
- Compare against assumed baseline from custom solver

## Build
```bash
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/conan/build/Debug -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

## Run
```bash
./p2_performance
```

## Output
- Console: Statistics table with pass/fail per contact count
- `p2_results.csv`: Per-iteration timing data

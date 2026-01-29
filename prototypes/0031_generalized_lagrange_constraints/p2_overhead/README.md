# Prototype P2: Virtual Function Overhead

**Question**: What is the performance overhead of the generalized constraint framework compared to the hard-coded QuaternionConstraint implementation?

## Success Criteria
- Less than 10% performance regression for single quaternion constraint
- Less than 5 microseconds overhead per constraint solve (release build)

## Build
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
./constraint_overhead_bench
```

## Results
See `docs/designs/0031_generalized_lagrange_constraints/prototype-results.md`

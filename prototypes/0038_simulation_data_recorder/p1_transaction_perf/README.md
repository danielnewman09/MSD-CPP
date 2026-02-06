# Prototype P1: Transaction Performance Assessment

## Question
Does `withTransaction()` provide meaningful speedup over auto-commit for batch sizes 10-1000?

## Success Criteria
Transaction mode at least 2x faster for batch size >= 100

## Approach
Standalone executable that benchmarks insert throughput with and without explicit transactions using cpp_sqlite DAOs and buffered writes.

## Build and Run

```bash
# From prototype directory
cd prototypes/0038_simulation_data_recorder/p1_transaction_perf

# Configure with Conan dependencies (use project's Conan setup)
# NOTE: Run from project root first:
cd ../../../
conan install . --build=missing -s build_type=Release

# Then build the prototype
cd prototypes/0038_simulation_data_recorder/p1_transaction_perf
mkdir -p build
cd build
cmake .. -DCMAKE_PREFIX_PATH=../../../../build/Release/generators \
         -DCMAKE_BUILD_TYPE=Release
cmake --build .

# Run
./p1_transaction_perf
```

## Expected Output
- Timing measurements for auto-commit vs transaction modes
- Summary table with speedup ratios
- Pass/fail evaluation against success criteria

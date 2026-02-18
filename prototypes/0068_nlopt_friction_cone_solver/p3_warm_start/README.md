# Prototype P3: Warm-Start Effectiveness

## Question
Does warm-starting from previous frame's lambda reduce iteration count by > 30% for steady-state contact?

## Success Criteria
- Cold start (lambda0 = 0): average 15-25 iterations per solve
- Warm start (lambda0 = previous frame): average < 10 iterations per solve
- Iteration reduction > 30%

## Approach
Simulate 100 frames of steady-state contact (2 contacts, mu=0.5):
- Minimal frame-to-frame variation (0.1% perturbation to A/b)
- Solve each frame twice: cold start vs warm start
- Track iteration counts
- Compute reduction percentage

## Build
```bash
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/conan/build/Debug
cmake --build .
```

## Run
```bash
./p3_warm_start
```

## Output
- Console: Statistics comparing cold vs warm start
- `p3_results.csv`: Per-frame iteration data

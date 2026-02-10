# Prototype 0053: Collision Pipeline Performance Optimization

This directory contains three prototypes validating design assumptions from the Design Review:

## Prototypes

| ID | Name | Question | Time Box | Status |
|----|------|----------|----------|--------|
| P1 | Friction Warm-Start | Does warm-starting reduce iterations by ≥40%? | 1 hour | Complete |
| P2 | SAT Gating Threshold | What threshold minimizes SAT invocations? | 1 hour | Complete |
| P3 | Fixed-Size Matrices | Do fixed-size matrices maintain precision? | 1 hour | Complete |

## Directory Structure

```
p1_friction_warm_start/
├── main.cpp              # Instrumented test harness
└── README.md             # Build and run instructions

p2_sat_gating/
├── main.cpp              # Threshold sweep test
└── README.md             # Build and run instructions

p3_fixed_size_precision/
├── main.cpp              # Precision validation
└── README.md             # Build and run instructions
```

## Build All Prototypes

Each prototype is standalone:

```bash
cd p1_friction_warm_start && clang++ -std=c++20 -O2 -I$HOME/eigen main.cpp -o prototype && ./prototype
cd ../p2_sat_gating && clang++ -std=c++20 -O2 -I/usr/local/include main.cpp -o prototype && ./prototype
cd ../p3_fixed_size_precision && clang++ -std=c++20 -O2 -I$HOME/eigen main.cpp -o prototype && ./prototype
```

## Results Summary

See `docs/designs/0053_collision_pipeline_performance/prototype-results.md` for full analysis.

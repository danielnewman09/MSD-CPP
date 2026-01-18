# Benchmarking Guide

> This document provides comprehensive documentation for the Google Benchmark infrastructure in MSD-CPP.

**Related Tickets**:
- [0011_add_google_benchmark](../tickets/0011_add_google_benchmark.md) — Initial infrastructure
- [0014_benchmark_metrics_tracker](../tickets/0014_benchmark_metrics_tracker.md) — Regression detection

**Design Documents**:
- [`docs/designs/0011_add_google_benchmark/design.md`](designs/0011_add_google_benchmark/design.md)
- [`docs/designs/0014_benchmark_metrics_tracker/design.md`](designs/0014_benchmark_metrics_tracker/design.md)

---

## Overview

The project uses Google Benchmark for micro-benchmarking performance-critical code paths. Benchmarks are optional and disabled by default to avoid extending build times.

---

## Building Benchmarks

### Prerequisites

Install dependencies with benchmarks enabled:
```bash
# Release build recommended for accurate measurements
conan install . --build=missing -s build_type=Release -o "&:enable_benchmarks=True"
```

### Configure and Build

```bash
# Configure with benchmarks enabled
cmake --preset conan-release -DENABLE_BENCHMARKS=ON

# Build benchmark executable(s)
cmake --build --preset conan-release --target msd_sim_bench
```

---

## Running Benchmarks

### Basic Execution

```bash
# Run all benchmarks
./build/Release/release/msd_sim_bench

# Run with specific filters
./build/Release/release/msd_sim_bench --benchmark_filter=ConvexHull_Construction

# Run with repetitions for statistical significance
./build/Release/release/msd_sim_bench --benchmark_repetitions=10
```

### Output Formats

```bash
# JSON output for analysis
./build/Release/release/msd_sim_bench --benchmark_out=results.json --benchmark_out_format=json

# CSV output for spreadsheets
./build/Release/release/msd_sim_bench --benchmark_out=results.csv --benchmark_out_format=csv
```

### Performance Options

```bash
# Control minimum benchmark time (default 0.5s per benchmark)
./build/Release/release/msd_sim_bench --benchmark_min_time=1.0s

# Set CPU affinity to reduce variance (Linux/macOS)
./build/Release/release/msd_sim_bench --benchmark_enable_random_interleaving=true
```

---

## Generating Benchmark Reports

Use the `run_benchmarks.sh` script to generate JSON reports for tracking performance over time:

```bash
# Generate JSON report (default: benchmark_results/ directory)
./scripts/run_benchmarks.sh

# Custom output directory with 5 repetitions
./scripts/run_benchmarks.sh -o reports -r 5

# Console output only (no file)
./scripts/run_benchmarks.sh -f console

# Show all options
./scripts/run_benchmarks.sh --help
```

### Script Options

| Option | Default | Description |
|--------|---------|-------------|
| `-o, --output DIR` | `benchmark_results` | Output directory for JSON reports |
| `-f, --format FMT` | `json` | Output format: `json` or `console` |
| `-b, --build-type` | `Release` | Build type: `Debug` or `Release` |
| `-r, --repetitions N` | `3` | Number of repetitions per benchmark |

### Output Organization

The script organizes results by executable name:
```
benchmark_results/
└── msd_sim_bench/
    ├── benchmark_20260108_143000.json
    ├── benchmark_20260108_150000.json
    └── benchmark_latest.json -> benchmark_20260108_150000.json
```

Each suite folder contains timestamped JSON files and a `benchmark_latest.json` symlink for convenience.

---

## Available Benchmark Suites

| Benchmark Suite | Executable | Location | Purpose |
|-----------------|------------|----------|---------|
| **ConvexHull** | `msd_sim_bench` | `msd/msd-sim/bench/` | Convex hull construction, containment, distance, and GJK collision |

### ConvexHull Benchmarks

- `BM_ConvexHull_Construction` — Hull construction from point clouds (8, 64, 512, 4096 points)
- `BM_ConvexHull_Contains` — Point containment queries (collision detection hot path)
- `BM_ConvexHull_SignedDistance` — Signed distance calculations (proximity queries)
- `BM_ConvexHull_Intersects` — GJK intersection tests (collision detection)

---

## Interpreting Results

### Example Output

```
--------------------------------------------------------------------------
Benchmark                                Time             CPU   Iterations
--------------------------------------------------------------------------
BM_ConvexHull_Construction/8         36974 ns        36833 ns         4174
BM_ConvexHull_Construction/64        94272 ns        93823 ns         1341
BM_ConvexHull_Construction/512      254729 ns       253761 ns          566
BM_ConvexHull_Construction/4096    1055573 ns      1048969 ns          127
BM_ConvexHull_Construction_BigO     261.71 N        260.09 N
BM_ConvexHull_Construction_RMS          21 %            21 %
BM_ConvexHull_Contains                2263 ns         2248 ns        64469
```

### Key Metrics

- **Time**: Wall-clock time per iteration
- **CPU**: CPU time per iteration (excludes I/O wait)
- **Iterations**: Number of times benchmark ran (auto-adjusted for min_time)
- **BigO**: Algorithmic complexity estimate (for parameterized benchmarks)
- **RMS**: Root-mean-square deviation (measure of consistency)

---

## Benchmark Organization

Benchmarks follow the same directory structure as tests:
```
msd/msd-sim/
├── src/            # Source code
├── test/           # Unit/integration tests
└── bench/          # Performance benchmarks
    ├── CMakeLists.txt
    └── ConvexHullBench.cpp
```

---

## Writing New Benchmarks

### Benchmark Template

```cpp
// Ticket: {ticket-name}
// Design: docs/designs/{ticket-name}/design.md

#include <benchmark/benchmark.h>
#include "msd-sim/src/YourComponent.hpp"

static void BM_YourComponent_Operation(benchmark::State& state) {
  // Setup (outside timing loop)
  YourComponent component{/* ... */};

  // Benchmark loop
  for (auto _ : state) {
    auto result = component.operation();
    benchmark::DoNotOptimize(result);  // Prevent optimization
  }
}
BENCHMARK(BM_YourComponent_Operation);

// Parameterized benchmark
static void BM_YourComponent_Scaled(benchmark::State& state) {
  auto data = generateData(state.range(0));
  for (auto _ : state) {
    auto result = component.process(data);
    benchmark::DoNotOptimize(result);
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_YourComponent_Scaled)
    ->Args({10})
    ->Args({100})
    ->Args({1000})
    ->Complexity();
```

### Best Practices

- Use `benchmark::DoNotOptimize()` to prevent dead code elimination
- Place setup code outside the timing loop
- Use fixed seeds for random data to ensure reproducibility
- Add complexity analysis for parameterized benchmarks
- Include ticket references in file header and function documentation

---

## Benchmark Regression Detection

The project uses `compare_benchmarks.py` to detect performance regressions by comparing results against golden baseline files.

### Basic Workflow

```bash
# Run benchmarks
./scripts/run_benchmarks.sh

# Compare against baseline
./scripts/compare_benchmarks.py

# Update baseline (when performance changes are intentional)
./scripts/compare_benchmarks.py --set-baseline
```

### Interpreting Results

- **GREEN**: Performance within threshold or improved
- **YELLOW**: New/missing benchmarks (review if expected)
- **RED**: Regression detected (exceeds threshold)

**Default threshold**: 10% slower than baseline triggers regression

### Advanced Options

```bash
# Use custom threshold (5% instead of default 10%)
./scripts/compare_benchmarks.py --threshold 5.0

# Strict mode: exit code 1 on regression (for CI)
./scripts/compare_benchmarks.py --strict

# Compare specific result file
./scripts/compare_benchmarks.py --current benchmark_results/msd_sim_bench/benchmark_20260108.json

# Disable colors (for CI logs)
./scripts/compare_benchmarks.py --no-color

# Output JSON report only (no console output)
./scripts/compare_benchmarks.py --output-json-only
```

### Baseline Files

- **Location**: `benchmark_baselines/{suite}/baseline.json`
- Committed to git for team-wide consistency
- Update when intentional performance changes occur

### Comparison Reports

- **Location**: `benchmark_results/{suite}/comparison_{timestamp}.json`
- Format: JSON with per-benchmark diff, summary statistics
- Useful for: Design review, pull request analysis

### When to Update Baselines

1. After performance optimizations that improve metrics
2. When algorithmic changes intentionally trade performance for correctness
3. When refactoring changes performance characteristics
4. Always commit baseline updates with code changes that affect them

### Example Workflow for Optimization

```bash
# Verify current performance
./scripts/run_benchmarks.sh
./scripts/compare_benchmarks.py

# Make optimization changes
# ... edit code ...

# Run benchmarks again
./scripts/run_benchmarks.sh
./scripts/compare_benchmarks.py

# If improved, update baseline
./scripts/compare_benchmarks.py --set-baseline

# Commit code and baseline together
git add src/optimized_code.cpp
git add benchmark_baselines/msd_sim_bench/baseline.json
git commit -m "Optimize ConvexHull construction

Performance improvement:
- BM_ConvexHull_Construction/512: 266ms -> 180ms (-32%)

Updated benchmark baseline."
```

---

## CI Integration

**Status**: Local execution only (no CI integration yet)

**Future enhancement**: Automated benchmark execution on pull requests with baseline comparison for performance regression detection using the `--strict` flag.

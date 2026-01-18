# Analysis Infrastructure

> This document provides context for the performance analysis tools in MSD-CPP.

## Overview

The `analysis/` directory contains all performance analysis infrastructure including benchmarking scripts, profiling tools, and regression detection systems.

---

## Directory Structure

```
analysis/
├── scripts/                      # Analysis scripts
│   ├── run_benchmarks.sh         # Run Google Benchmark suites
│   ├── compare_benchmarks.py     # Benchmark regression detection
│   ├── profile-instruments.sh    # macOS profiling with Instruments
│   ├── parse-profile.py          # Parse profiling traces to JSON
│   └── compare-profiles.py       # Profiling regression detection
│
├── benchmark_baselines/          # Golden baselines for benchmark comparison
│   └── msd_sim_bench/
│       └── baseline.json         # Committed baseline for team consistency
│
├── benchmark_results/            # Generated benchmark output (gitignored)
│
├── profile_baselines/            # Golden baselines for profiling comparison
│
└── profile_results/              # Generated profiling output (gitignored)
```

---

## Benchmarking

### Running Benchmarks

```bash
# Build with benchmarks enabled
conan install . --build=missing -s build_type=Release -o "&:enable_benchmarks=True"
cmake --preset conan-release -DENABLE_BENCHMARKS=ON
cmake --build --preset conan-release --target msd_sim_bench

# Run benchmarks directly
./build/Release/release/msd_sim_bench

# Or use the helper script for JSON output
./analysis/scripts/run_benchmarks.sh
```

### Benchmark Regression Detection

```bash
# Compare against baseline
./analysis/scripts/compare_benchmarks.py

# Update baseline (when performance changes are intentional)
./analysis/scripts/compare_benchmarks.py --set-baseline

# Strict mode for CI (exit code 1 on regression)
./analysis/scripts/compare_benchmarks.py --strict

# Custom threshold (default is 10%)
./analysis/scripts/compare_benchmarks.py --threshold 5.0
```

### Script: `run_benchmarks.sh`

Runs Google Benchmark executables and generates JSON reports.

| Option | Default | Description |
|--------|---------|-------------|
| `-o, --output DIR` | `benchmark_results` | Output directory for JSON reports |
| `-f, --format FMT` | `json` | Output format: `json` or `console` |
| `-b, --build-type` | `Release` | Build type: `Debug` or `Release` |
| `-r, --repetitions N` | `3` | Number of repetitions per benchmark |

### Script: `compare_benchmarks.py`

Compares benchmark results against golden baselines to detect regressions.

| Option | Default | Description |
|--------|---------|-------------|
| `--current FILE` | auto-detect | Specific result file to compare |
| `--threshold N` | `10.0` | Regression threshold percentage |
| `--set-baseline` | — | Update baseline with current results |
| `--strict` | — | Exit code 1 on regression (for CI) |
| `--no-color` | — | Disable colored output |
| `--output-json-only` | — | Output JSON report only |

---

## Profiling (macOS)

### Running Profiler

```bash
# Build with profiling enabled
conan install . --build=missing -s build_type=Release -o "&:enable_profiling=True"
cmake --preset profiling-release
cmake --build --preset conan-release

# Profile with automatic XML export
./analysis/scripts/profile-instruments.sh ./build/Release/release/msd_sim_bench -x

# Parse the trace to JSON
./analysis/scripts/parse-profile.py analysis/profile_results/*.trace
```

### Profiling Regression Detection

```bash
# Compare against baseline
./analysis/scripts/compare-profiles.py

# Update baseline (when performance changes are intentional)
./analysis/scripts/compare-profiles.py --set-baseline

# Strict mode for CI
./analysis/scripts/compare-profiles.py --strict

# Custom threshold (default is 50%)
./analysis/scripts/compare-profiles.py --threshold 75.0
```

### Script: `profile-instruments.sh`

Profiles executables using Xcode Instruments (Time Profiler or Allocations).

| Option | Default | Description |
|--------|---------|-------------|
| `-x, --export-xml` | — | Export XML for parsing |
| `-d, --output-dir DIR` | `profile_results` | Output directory for traces |
| `TEMPLATE` (arg 2) | `Time Profiler` | Instruments template to use |

### Script: `parse-profile.py`

Extracts Time Profiler data from `.trace` files into structured JSON.

| Option | Default | Description |
|--------|---------|-------------|
| `-o, --output FILE` | auto-generated | Output JSON file path |
| `--top N` | `20` | Limit to top N functions |
| `--json-only` | — | Output JSON only, no console summary |
| `--no-color` | — | Disable colored output |

### Script: `compare-profiles.py`

Compares profiling results against golden baselines to detect regressions.

| Option | Default | Description |
|--------|---------|-------------|
| `--threshold N` | `50.0` | Regression threshold percentage |
| `--runs N` | `5` | Number of runs to average |
| `--top N` | `10` | Number of top functions to track |
| `--set-baseline` | — | Update baseline with current results |
| `--strict` | — | Exit code 1 on regression (for CI) |
| `--no-color` | — | Disable colored output |

---

## Baseline Management

### When to Update Baselines

1. After performance optimizations that improve metrics
2. When algorithmic changes intentionally trade performance for correctness
3. When refactoring changes performance characteristics
4. Always commit baseline updates with code changes that affect them

### Example Workflow

```bash
# Verify current performance
./analysis/scripts/run_benchmarks.sh
./analysis/scripts/compare_benchmarks.py

# Make optimization changes
# ... edit code ...

# Run benchmarks again
./analysis/scripts/run_benchmarks.sh
./analysis/scripts/compare_benchmarks.py

# If improved, update baseline
./analysis/scripts/compare_benchmarks.py --set-baseline

# Commit code and baseline together
git add src/optimized_code.cpp
git add analysis/benchmark_baselines/msd_sim_bench/baseline.json
git commit -m "Optimize ConvexHull construction"
```

---

## Related Documentation

- **Benchmarking guide**: [`docs/benchmarking.md`](../docs/benchmarking.md)
- **Profiling guide**: [`docs/profiling.md`](../docs/profiling.md)
- **Design documents**:
  - [`docs/designs/0011_add_google_benchmark/`](../docs/designs/0011_add_google_benchmark/)
  - [`docs/designs/0014_benchmark_metrics_tracker/`](../docs/designs/0014_benchmark_metrics_tracker/)
  - [`docs/designs/0012_add_macos_profiling_support/`](../docs/designs/0012_add_macos_profiling_support/)
  - [`docs/designs/0015_profiling_trace_parser/`](../docs/designs/0015_profiling_trace_parser/)
  - [`docs/designs/0016_profiling_regression_tracker/`](../docs/designs/0016_profiling_regression_tracker/)

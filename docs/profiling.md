# Profiling Guide (macOS)

> This document provides comprehensive documentation for the macOS profiling infrastructure in MSD-CPP using Xcode Instruments.

**Related Tickets**:
- [0012_add_macos_profiling_support](../tickets/0012_add_macos_profiling_support.md) — Initial infrastructure
- [0015_profiling_trace_parser](../tickets/0015_profiling_trace_parser.md) — JSON parsing
- [0016_profiling_regression_tracker](../tickets/0016_profiling_regression_tracker.md) — Regression detection

**Design Documents**:
- [`docs/designs/0012_add_macos_profiling_support/design.md`](designs/0012_add_macos_profiling_support/design.md)
- [`docs/designs/0015_profiling_trace_parser/design.md`](designs/0015_profiling_trace_parser/design.md)
- [`docs/designs/0016_profiling_regression_tracker/design.md`](designs/0016_profiling_regression_tracker/design.md)

---

## Overview

The project provides profiling infrastructure for macOS using Xcode Instruments. This enables deep CPU profiling, memory analysis, and call graph visualization on Apple Silicon and Intel Macs. Profiling builds use debug symbols with Release optimizations for realistic performance measurements.

---

## Prerequisites

### System Requirements

- macOS 12.0+ (Monterey or later)
- Xcode Command Line Tools installed
- Apple Clang 13.0+

### Install Xcode Command Line Tools

```bash
xcode-select --install
```

---

## Building with Profiling Support

### Prerequisites

Install dependencies with profiling enabled:
```bash
# Release build recommended for realistic performance
conan install . --build=missing -s build_type=Release -o "&:enable_profiling=True"
```

### Configure and Build

```bash
# Configure with profiling preset
cmake --preset profiling-release

# Build all targets
cmake --build --preset conan-release

# Build specific executable
cmake --build --preset conan-release --target msd_sim_bench
```

### Compiler Flags Applied

- `-g` — Generate debug symbols for Instruments
- `-O2` — Release-level optimizations for realistic performance
- No `-DNDEBUG` — Keeps assertions enabled for issue detection

---

## Profiling Workflows

### Option 1: Helper Script (Recommended)

Use the `profile-instruments.sh` script for streamlined profiling:

```bash
# Profile a benchmark (Time Profiler)
./analysis/scripts/profile-instruments.sh ./build/Release/release/msd_sim_bench

# Profile with Allocations template
./analysis/scripts/profile-instruments.sh ./build/Release/release/msd_sim_bench "Allocations"

# Open generated trace file
open profile_20260108_143000.trace
```

**Available templates**:
- `Time Profiler` (default) — CPU profiling, hotspot identification
- `Allocations` — Memory profiling, leak detection

### Option 2: Direct xctrace Usage

Run `xctrace` manually for more control:

```bash
# Time Profiler (CPU profiling)
xctrace record --template "Time Profiler" \
    --output profile.trace \
    --launch -- ./build/Release/release/msd_sim_bench

# Allocations (Memory profiling)
xctrace record --template "Allocations" \
    --output allocations.trace \
    --launch -- ./build/Release/release/msd_sim_bench

# Open trace file in Instruments GUI
open profile.trace
```

**Common xctrace options**:
```bash
# Record for specific duration (30 seconds)
xctrace record --template "Time Profiler" \
    --time-limit 30s \
    --output profile.trace \
    --launch -- ./build/Release/release/msd_sim_bench

# Attach to running process
xctrace record --template "Time Profiler" \
    --output profile.trace \
    --attach <pid>
```

### Option 3: Instruments GUI

Launch Instruments directly for interactive profiling:

```bash
# Launch Instruments with executable
open -a Instruments ./build/Release/release/msd_sim_bench

# Then manually:
# 1. Choose "Time Profiler" or "Allocations" template
# 2. Click record button
# 3. Analyze results in GUI
```

---

## Parsing Profiling Data

The `parse-profile.py` script extracts Time Profiler data from `.trace` files into structured JSON format, enabling programmatic analysis of profiling results without requiring manual inspection in the Instruments GUI.

### Basic Usage

```bash
# Parse and display top 20 functions
./analysis/scripts/parse-profile.py profile_results/profile_20260108_183915.trace

# Limit to top 10 functions
./analysis/scripts/parse-profile.py profile_results/profile_20260108_183915.trace --top 10

# Save to specific JSON file
./analysis/scripts/parse-profile.py profile_results/profile_20260108_183915.trace -o report.json

# JSON output only (no console summary)
./analysis/scripts/parse-profile.py profile_results/profile_20260108_183915.trace --json-only
```

### Integrated Workflow

**Profile with automatic XML export**:
```bash
# Profile and export XML in one step
./analysis/scripts/profile-instruments.sh ./build/Release/release/msd_sim_bench -x

# Then parse the trace
./analysis/scripts/parse-profile.py profile_results/profile_20260108_183915.trace
```

**Custom output directory**:
```bash
# Profile to custom directory
./analysis/scripts/profile-instruments.sh ./build/Release/release/msd_sim_bench -d my_profiles -x

# Parse from custom directory
./analysis/scripts/parse-profile.py my_profiles/profile_20260108_183915.trace
```

### JSON Output Schema

The parser generates JSON reports with the following structure:

```json
{
  "metadata": {
    "trace_file": "profile_20260108_183915.trace",
    "template": "Time Profiler",
    "export_timestamp": "2026-01-08T18:20:00Z",
    "executable": "unknown"
  },
  "summary": {
    "total_samples": 7821,
    "total_time_ms": 7821.0
  },
  "top_functions": [
    {
      "rank": 1,
      "name": "void msd_sim::ConvexHull::computeHull<msd_sim::Coordinate>(...)",
      "samples": 28,
      "percentage": 0.4,
      "source_file": "ConvexHull.hpp",
      "line": 263
    }
  ]
}
```

**Field descriptions**:
- `total_samples` — Number of time samples collected (~1ms per sample)
- `total_time_ms` — Approximate profiling duration in milliseconds
- `rank` — Position in top-N list (1-indexed)
- `name` — Demangled function name (C++ symbols decoded)
- `samples` — Number of samples where this function was on the stack
- `percentage` — Percent of total samples (rounded to 1 decimal)
- `source_file` — Source filename (if available in debug symbols, otherwise null)
- `line` — Line number (if available, otherwise null)

### Console Output

When not using `--json-only`, the parser displays a color-coded summary:

```
Profiling Summary: profile_20260108_183915.trace
Total Samples: 7,821 (~7,821 ms)

┌─────┬──────────────────────────────────────────────────────┬─────────┬──────────┬──────────────────┐
│ Rank│ Function                                             │ Samples │ Percent  │ Source           │
├─────┼──────────────────────────────────────────────────────┼─────────┼──────────┼──────────────────┤
│   1 │ msd_sim::ConvexHull::computeHull<Coordinate>(...)    │      28 │    0.4%  │ ConvexHull.hpp   │
│   2 │ msd_sim::ConvexHull::extractHullData(qhT*)           │      25 │    0.3%  │ ConvexHull.cpp   │
│   3 │ msd_sim::ConvexHull::computeCentroid()               │      25 │    0.3%  │ N/A              │
└─────┴──────────────────────────────────────────────────────┴─────────┴──────────┴──────────────────┘

Top hotspot: msd_sim::ConvexHull::computeHull<Coordinate>(...) (0.4% of samples)

JSON report: profile_results/profile_20260108_184520.json
```

### Script Options

| Option | Default | Description |
|--------|---------|-------------|
| `-o, --output FILE` | `profile_results/profile_<timestamp>.json` | Output JSON file path |
| `--top N` | `20` | Limit to top N functions |
| `--json-only` | `false` | Output JSON only, no console summary |
| `--no-color` | `false` | Disable colored output |
| `-h, --help` | — | Show help message |

### Output Directory

Profile traces and JSON reports are stored in `profile_results/` by default:

```
profile_results/
├── profile_20260108_183915.trace       # Instruments trace file
├── profile_20260108_183915.xml         # XML export (if --export-xml used)
└── profile_20260108_184520.json        # JSON report from parser
```

### Use Cases

**Quick hotspot identification**:
```bash
# Profile and immediately see top functions
./analysis/scripts/profile-instruments.sh ./build/Release/release/msd_sim_bench -x
./analysis/scripts/parse-profile.py profile_results/profile_*.trace --top 5
```

**Machine-readable output for CI**:
```bash
# Generate JSON report for automated analysis
./analysis/scripts/parse-profile.py profile_results/profile_*.trace --json-only > hotspots.json
```

**Profiling test executables**:
```bash
# Profile unit tests
./analysis/scripts/profile-instruments.sh ./build/Release/release/msd_sim_test -x
./analysis/scripts/parse-profile.py profile_results/profile_*.trace
```

### Limitations

- **Time Profiler only**: Parser currently supports Time Profiler template only (Allocations and other templates not supported in v1)
- **Flat function list**: Call tree hierarchy not included in JSON output (shows flat list of functions sorted by sample count)
- **Source locations**: Source file and line number may be null if debug symbols are incomplete
- **Backtrace references**: Parser processes inline backtraces only; backtrace references in XML are skipped (may undercount samples in some cases)

---

## Profiling Regression Detection

The project uses `compare-profiles.py` to detect profiling regressions by comparing results against golden baseline files. The system tracks when functions consume meaningfully more CPU time (sample percentage) after code changes.

### Basic Workflow

```bash
# Run profiling and parse results
./analysis/scripts/profile-instruments.sh ./build/Release/release/msd_sim_test -x
./analysis/scripts/parse-profile.py profile_results/*.trace --project-only

# Compare against baseline (averages top 5 runs)
./analysis/scripts/compare-profiles.py

# Update baseline (when performance changes are intentional)
./analysis/scripts/compare-profiles.py --set-baseline
```

### Interpreting Results

- **GREEN**: Performance within threshold or improved
- **YELLOW**: New/disappeared hotspots (review if expected)
- **RED**: Regression detected (exceeds threshold)

**Default threshold**: 50% increase in sample percentage triggers regression

**Example regression**:
```
Baseline: 10.0% samples (function consumed 10% of CPU time)
Current:  15.0% samples (function now consumes 15% of CPU time)
diff_percent = ((15 - 10) / 10) * 100 = 50% increase
status = REGRESSION (if threshold <= 50%)
```

### Advanced Options

```bash
# Use custom threshold (75% instead of default 50%)
./analysis/scripts/compare-profiles.py --threshold 75.0

# Average more runs for stability (10 instead of default 5)
./analysis/scripts/compare-profiles.py --runs 10

# Track more functions (20 instead of default 10)
./analysis/scripts/compare-profiles.py --top 20

# Strict mode: exit code 1 on regression (for CI)
./analysis/scripts/compare-profiles.py --strict

# Disable colors (for CI logs)
./analysis/scripts/compare-profiles.py --no-color

# Compare specific executable (auto-detected by default)
./analysis/scripts/compare-profiles.py --executable msd_sim_bench
```

### Baseline Files

- **Location**: `profile_baselines/{executable}/baseline.json`
- Committed to git for team-wide consistency
- Update when intentional performance changes occur
- Contains averaged data from top 5 runs (configurable)

### Comparison Reports

- **Location**: `profile_results/{executable}/comparison_{timestamp}.json`
- Format: JSON with per-function diff, summary statistics
- Useful for: Design review, pull request analysis

### When to Update Baselines

1. After performance optimizations that change function percentages
2. When refactoring redistributes CPU usage across functions
3. When adding new features that become new hotspots
4. Always commit baseline updates with code changes that affect them

### Example Workflow for Optimization

```bash
# Verify current performance
./analysis/scripts/profile-instruments.sh ./build/Release/release/msd_sim_test -x
./analysis/scripts/parse-profile.py profile_results/*.trace --project-only
./analysis/scripts/compare-profiles.py

# Make optimization changes
# ... edit code ...

# Run profiling again (collect 5 runs for averaging)
for i in {1..5}; do
  ./analysis/scripts/profile-instruments.sh ./build/Release/release/msd_sim_test -x
  ./analysis/scripts/parse-profile.py profile_results/*.trace --project-only
done

# Compare
./analysis/scripts/compare-profiles.py

# If improved, update baseline
./analysis/scripts/compare-profiles.py --set-baseline

# Commit code and baseline together
git add src/optimized_code.cpp
git add profile_baselines/msd_sim_test/baseline.json
git commit -m "Optimize ConvexHull extraction

Performance improvement:
- extractHullData(): 10.5% -> 7.2% samples (-31%)

Updated profiling baseline."
```

### Multi-Run Averaging

The comparison tool averages the most recent M profiling runs (default 5) to handle variance in profiling data:

- Profiles can vary between runs due to system load, GC pauses, etc.
- Averaging smooths out noise and provides stable baselines
- Only functions appearing in top N (default 10) of each run are tracked
- Functions are matched by exact demangled name across runs

### Comparison Metrics

- **Primary metric**: Sample percentage (not absolute sample count)
- **Threshold type**: Relative percentage increase (not absolute percentage point difference)
- **Why percentage**: Normalizes for different profiling durations
- **Why relative**: A function going from 1% → 2% (+1pp) is a 100% increase (significant), but 50% → 51% (+1pp) is only 2% increase (noise)

---

## Interpreting Results in Instruments

### Call Tree Navigation

- Function names appear instead of raw addresses (thanks to `-g` flag)
- Template instantiations visible (e.g., `ConvexHull::computeHull<Coordinate>`)
- Source line attribution enables jumping to code

### Time Profiler Metrics

- **Self Time** — Time spent in function excluding callees
- **Total Time** — Time spent in function including callees
- **Call Count** — Number of times function was called

### Allocations Metrics

- **Persistent Bytes** — Memory still allocated at end of profiling
- **Transient Bytes** — Memory allocated and freed during profiling
- **Allocation Count** — Number of allocations per function

---

## Profiling Best Practices

### What to Profile

- Benchmark executables (`msd_sim_bench`) — Repeatable, isolated performance tests
- Test executables (`msd_sim_test`) — Real-world code paths with known inputs
- Main application (`msd_exe`) — Overall performance characteristics

### When to Profile

- After benchmarks identify slow operations
- When investigating performance regressions
- Before and after optimizations
- When memory usage grows unexpectedly

### Profiling Workflow

1. Run benchmarks to identify slow operations
2. Profile with Time Profiler to find hotspots
3. Optimize identified bottlenecks
4. Re-run benchmarks to verify improvement
5. Compare baseline to detect regressions

---

## Troubleshooting

**Problem**: `xctrace: command not found`
**Solution**: Install Xcode Command Line Tools: `xcode-select --install`

**Problem**: No function names in call tree (only addresses)
**Solution**: Rebuild with profiling flags: `conan install . -o "&:enable_profiling=True"`

**Problem**: Trace file won't open in Instruments
**Solution**: Ensure macOS 12.0+ and Xcode Command Line Tools are up to date

**Problem**: Profiling overhead too high
**Solution**: Use Time Profiler (5% overhead) instead of System Trace (30% overhead)

---

## Platform Limitations

**macOS-only feature**: Profiling infrastructure uses Xcode Instruments and is only available on macOS. For cross-platform profiling:
- **Linux**: Use `perf` or Valgrind Callgrind
- **Windows**: Use Visual Studio Profiler
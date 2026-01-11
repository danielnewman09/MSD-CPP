# Design: Profiling Regression Tracker

## Summary

This design document describes a Python-based profiling regression tracker that compares Time Profiler results against golden baselines to detect when code changes cause specific functions to consume meaningfully more CPU time. The tool follows the same architectural patterns established in ticket 0014 (benchmark metrics tracker) but applies them to profiling data from parse-profile.py (ticket 0015), enabling function-level performance regression detection across test executables.

## Architecture Changes

### PlantUML Diagram
See: `./0016_profiling_regression_tracker.puml`

### Script Architecture

#### compare-profiles.py
**Location**: `scripts/compare-profiles.py`
**Purpose**: Standalone Python script for comparing profiling results against baselines

This script follows the architecture of `compare_benchmarks.py` but adapts it for profiling data:

**Key differences from benchmark comparison**:
1. **Multiple run averaging**: Profiles can vary between runs, so we average the top M (default 5) most recent profile runs for stability
2. **Function name matching**: Match functions by exact demangled name (not benchmark name patterns)
3. **Sample percentage comparison**: Compare percentage of total samples (not absolute time)
4. **Top N filtering**: Only track top N functions by sample percentage (default 10)
5. **Executable-specific baselines**: Profile baselines organized by executable name (e.g., `msd_sim_test`, `msd_sim_bench`)

### Core Functions

#### main()
**Purpose**: Entry point, CLI argument parsing, workflow orchestration
**Returns**: Exit code (0 = success, 1 = error or regression in strict mode)

```python
def main() -> int:
    """
    Main entry point.

    Workflow:
    1. Parse CLI arguments
    2. Auto-detect executable name from profile_results directory structure
    3. If --set-baseline: copy averaged runs to baseline and exit
    4. Load and average recent M profile runs
    5. Load baseline profile
    6. Match functions between current and baseline
    7. Compare and generate results
    8. Write JSON comparison report
    9. Print console output (unless --output-json-only)
    10. Return exit code (1 if --strict and regressions detected)
    """
```

**CLI Arguments**:
```
--current PATH         Path to current profile (default: auto-detect latest M runs)
--baseline PATH        Path to baseline (default: profile_baselines/{exec}/baseline.json)
--executable NAME      Executable name (default: auto-detect from directory structure)
--threshold FLOAT      Regression threshold as percentage increase (default: 50.0)
--top N                Number of top functions to track (default: 10)
--runs N               Number of recent runs to average (default: 5)
--set-baseline         Copy averaged current runs to baseline file
--strict               Exit code 1 on regression (for CI)
--output-json-only     Suppress console output, write JSON only
--no-color             Disable ANSI color codes
```

#### load_multiple_profiles()
**Purpose**: Load the most recent M profile JSON files for averaging
**Signature**: `load_multiple_profiles(results_dir: Path, executable: str, count: int) -> list[dict]`
**Returns**: List of profile dictionaries (up to `count` most recent)

```python
def load_multiple_profiles(
    results_dir: Path,
    executable: str,
    count: int
) -> list[dict]:
    """
    Load most recent profile runs from profile_results/{executable}/.

    Args:
        results_dir: Path to profile_results directory
        executable: Executable name (e.g., "msd_sim_test")
        count: Number of recent profiles to load

    Returns:
        List of profile dictionaries sorted by timestamp (newest first)

    Raises:
        FileNotFoundError: If executable directory doesn't exist
        ValueError: If fewer than 1 profile found

    Implementation:
    1. Scan profile_results/{executable}/profile_*.json files
    2. Sort by timestamp in filename (YYYYMMDD_HHMMSS format)
    3. Return top N most recent files as loaded JSON dicts
    """
```

#### average_profile_runs()
**Purpose**: Average function sample percentages across multiple profile runs
**Signature**: `average_profile_runs(profiles: list[dict], top_n: int) -> dict`
**Returns**: Averaged profile dictionary with same schema as input

```python
def average_profile_runs(profiles: list[dict], top_n: int) -> dict:
    """
    Average sample percentages across multiple profile runs.

    Args:
        profiles: List of profile dictionaries from load_multiple_profiles()
        top_n: Number of top functions to include in averaged result

    Returns:
        Averaged profile dictionary with schema:
        {
          "metadata": {
            "runs_averaged": int,
            "executable": str,
            "average_timestamp": ISO-8601
          },
          "summary": {
            "avg_total_samples": float
          },
          "top_functions": [
            {
              "name": str,
              "avg_percentage": float,
              "avg_samples": float,
              "occurrences": int  # How many runs included this function
            }
          ]
        }

    Algorithm:
    1. Accumulate sample percentages per function name across all runs
    2. Calculate average percentage for each function
    3. Sort functions by average percentage descending
    4. Filter to top N functions
    5. Note: A function may not appear in all runs - track occurrence count
    """
```

**Design decision**: We average percentages (not absolute sample counts) because:
- Profile runs may have different durations
- Percentage is the normalized metric for comparison
- Matches how parse-profile.py reports results

#### match_functions()
**Purpose**: Match functions between current and baseline by exact name
**Signature**: `match_functions(current: dict, baseline: dict) -> tuple[list[tuple[dict, dict]], list[str], list[str]]`
**Returns**: Tuple of (matched_pairs, new_hotspots, disappeared_functions)

```python
def match_functions(
    current: dict,
    baseline: dict
) -> tuple[list[tuple[dict, dict]], list[str], list[str]]:
    """
    Match functions by exact name between current and baseline.

    Args:
        current: Averaged current profile
        baseline: Baseline profile

    Returns:
        Tuple of:
        - matched_pairs: [(current_func, baseline_func), ...]
        - new_hotspots: ["function::name()", ...] (in current, not in baseline)
        - disappeared: ["function::name()", ...] (in baseline, not in current)

    Implementation:
    1. Build name-indexed dicts for current and baseline top_functions
    2. Match functions by exact name
    3. Identify functions only in current (new hotspots)
    4. Identify functions only in baseline (disappeared)

    Note: Both current and baseline are already filtered to top N functions,
    so this is comparing top N vs top N (not all functions).
    """
```

#### compare_profiles()
**Purpose**: Compare matched functions and detect regressions
**Signature**: `compare_profiles(current: dict, baseline: dict, threshold_percent: float) -> dict`
**Returns**: Comparison result dictionary with per-function results and summary

```python
def compare_profiles(
    current: dict,
    baseline: dict,
    threshold_percent: float = 50.0
) -> dict:
    """
    Compare current profile against baseline.

    Args:
        current: Averaged current profile
        baseline: Baseline profile
        threshold_percent: Regression threshold (default 50%)

    Returns:
        Comparison dictionary:
        {
          "metadata": {
            "generated_at": ISO-8601,
            "executable": str,
            "runs_averaged": int,
            "threshold_percent": float,
            "current_timestamp": ISO-8601,
            "baseline_timestamp": ISO-8601
          },
          "functions": [
            {
              "name": str,
              "current_percentage": float,
              "baseline_percentage": float,
              "diff_percent": float,  # Relative increase percentage
              "status": "PASS" | "REGRESSION"
            }
          ],
          "summary": {
            "total_compared": int,
            "passed": int,
            "regressed": int,
            "new_hotspots": [str],
            "disappeared": [str]
          }
        }

    Regression calculation:
        diff_percent = ((current - baseline) / baseline) * 100
        status = "REGRESSION" if diff_percent > threshold_percent else "PASS"

    Example:
        Baseline: 10.0% samples
        Current:  15.0% samples
        diff_percent = ((15 - 10) / 10) * 100 = 50% increase
        status = "REGRESSION" (if threshold <= 50%)
    """
```

#### print_comparison_results()
**Purpose**: Print color-coded comparison table to console
**Signature**: `print_comparison_results(comparison: dict, color: bool = True) -> None`

```python
def print_comparison_results(comparison: dict, color: bool = True) -> None:
    """
    Print color-coded comparison results to console.

    Uses same color scheme as compare_benchmarks.py:
    - GREEN: PASS (no regression)
    - RED: REGRESSION (exceeds threshold)
    - YELLOW: New hotspots / disappeared functions
    - BLUE: Headers and metadata

    Output format:
    ┌─────────────────────────────────────────┬──────────┬──────────┬───────────┬──────────┐
    │ Function                                │ Current  │ Baseline │ Diff      │ Status   │
    ├─────────────────────────────────────────┼──────────┼──────────┼───────────┼──────────┤
    │ msd_sim::ConvexHull::extractHullData... │  10.5%   │   7.0%   │  +50.0%   │   REGR   │
    │ msd_sim::GJK::intersects(int)           │   5.2%   │   5.0%   │   +4.0%   │   PASS   │
    └─────────────────────────────────────────┴──────────┴──────────┴───────────┴──────────┘

    Summary:
      Total: 10
      Passed: 8
      Regressed: 2

    New hotspots (not in baseline):
      - msd_sim::NewFunction()

    Disappeared (in baseline, not in current):
      - msd_sim::OldFunction()
    """
```

#### set_baseline()
**Purpose**: Copy current averaged runs to baseline file
**Signature**: `set_baseline(results_dir: Path, baseline_file: Path, executable: str, runs: int) -> None`

```python
def set_baseline(
    results_dir: Path,
    baseline_file: Path,
    executable: str,
    runs: int
) -> None:
    """
    Average recent runs and copy to baseline file.

    Args:
        results_dir: Path to profile_results directory
        baseline_file: Path to baseline.json destination
        executable: Executable name
        runs: Number of recent runs to average

    Raises:
        FileNotFoundError: If no profiles found
        ValueError: If invalid JSON

    Implementation:
    1. Load recent M runs using load_multiple_profiles()
    2. Average using average_profile_runs()
    3. Create baseline directory if needed
    4. Write averaged profile to baseline.json
    5. Print confirmation with green color
    """
```

### Directory Structure

```
project/
├── scripts/
│   ├── compare-profiles.py          # New script (this design)
│   ├── compare_benchmarks.py        # Reference implementation
│   └── parse-profile.py             # Produces input JSON
├── profile_results/
│   ├── msd_sim_test/
│   │   ├── profile_20260110_140000.json
│   │   ├── profile_20260110_141500.json
│   │   └── comparison_20260110_143000.json  # Generated by compare-profiles.py
│   └── msd_sim_bench/
│       ├── profile_20260110_150000.json
│       └── comparison_20260110_153000.json
└── profile_baselines/
    ├── msd_sim_test/
    │   └── baseline.json            # Committed to git
    └── msd_sim_bench/
        └── baseline.json
```

**Key points**:
- `profile_baselines/` is committed to git (contains golden baselines)
- `profile_results/` is gitignored (contains run-specific data)
- Executables are auto-detected from subdirectory names
- Comparison reports written to same directory as profile results

### Integration Points

| Script Component | External Component | Integration Type | Notes |
|------------------|-------------------|------------------|-------|
| `load_json()` | `profile_results/*.json` | File I/O | Reads JSON produced by parse-profile.py |
| `set_baseline()` | `profile_baselines/*.json` | File I/O | Writes averaged baseline (committed to git) |
| `compare_profiles()` | `profile_baselines/*.json` | File I/O | Reads committed baseline |
| `main()` | CI pipeline | Process exit code | `--strict` mode for CI integration |

### JSON Schema Compatibility

#### Input: Profile JSON (from parse-profile.py)
```json
{
  "metadata": {
    "trace_file": "profile_20260108_185430.trace",
    "template": "Time Profiler",
    "export_timestamp": "2026-01-10T14:58:21.385744",
    "executable": "unknown"
  },
  "summary": {
    "total_samples": 6668,
    "total_time_ms": 6668.0
  },
  "top_functions": [
    {
      "name": "msd_sim::ConvexHull::extractHullData(qhT*)",
      "samples": 25,
      "source_file": "ConvexHull.cpp",
      "line": 149,
      "percentage": 0.4,
      "rank": 1
    }
  ]
}
```

**Fields used by compare-profiles.py**:
- `top_functions[].name` — Function matching key
- `top_functions[].percentage` — Primary comparison metric
- `top_functions[].samples` — Used for averaging calculation
- `metadata.export_timestamp` — Comparison metadata
- `summary.total_samples` — Averaging calculation

#### Output: Comparison JSON
```json
{
  "metadata": {
    "generated_at": "2026-01-10T15:00:00.123456",
    "executable": "msd_sim_test",
    "runs_averaged": 5,
    "threshold_percent": 50.0,
    "current_timestamp": "2026-01-10T14:58:21.385744",
    "baseline_timestamp": "2026-01-08T12:00:00.000000"
  },
  "functions": [
    {
      "name": "msd_sim::ConvexHull::extractHullData(qhT*)",
      "current_percentage": 10.5,
      "baseline_percentage": 7.0,
      "diff_percent": 50.0,
      "status": "REGRESSION",
      "current_samples": 700,
      "baseline_samples": 467
    }
  ],
  "summary": {
    "total_compared": 10,
    "passed": 8,
    "regressed": 2,
    "new_hotspots": ["msd_sim::NewFunction()"],
    "disappeared": ["msd_sim::OldFunction()"]
  }
}
```

### Comparison Algorithm Details

#### Relative Percentage Increase Calculation

The script uses **relative percentage increase** (not absolute percentage point difference):

```python
baseline_pct = 7.0   # 7.0% of samples
current_pct = 10.5   # 10.5% of samples

diff_percent = ((current_pct - baseline_pct) / baseline_pct) * 100
             = ((10.5 - 7.0) / 7.0) * 100
             = (3.5 / 7.0) * 100
             = 50.0%  # This is a 50% increase

status = "REGRESSION" if diff_percent > threshold_percent else "PASS"
# With default threshold of 50%, this would be a regression
```

**Why relative increase instead of absolute difference**:
- A function going from 1% → 2% (+1pp) is a 100% increase (significant)
- A function going from 50% → 51% (+1pp) is a 2% increase (noise)
- Relative increase normalizes for baseline magnitude

**Edge case**: If baseline percentage is 0.0%, skip comparison (avoid division by zero)

#### Averaging Algorithm

Functions are averaged by **percentage**, not absolute sample counts:

```python
# Example: Averaging 3 runs for a function
Run 1: 25 samples / 6668 total = 0.375% (percentage field)
Run 2: 30 samples / 8000 total = 0.375%
Run 3: 20 samples / 5000 total = 0.400%

avg_percentage = (0.375 + 0.375 + 0.400) / 3 = 0.383%
```

**Why percentage averaging**:
- Profile durations vary (different total_samples)
- Percentage normalizes for duration
- Matches how humans interpret profiling results

**Handling missing functions**:
```python
# Function appears in 2 of 3 runs
Run 1: 0.4%
Run 2: 0.5%
Run 3: Not in top N (0.0%)

# Option A: Average over runs where it appeared (2 runs)
avg_percentage = (0.4 + 0.5) / 2 = 0.45%

# Option B: Average over all runs (treat missing as 0.0%)
avg_percentage = (0.4 + 0.5 + 0.0) / 3 = 0.30%

# DECISION: Use Option A (only runs where it appeared)
# Rationale: If a function wasn't in top N, it's noise (below threshold)
```

Implementation:
```python
def average_profile_runs(profiles: list[dict], top_n: int) -> dict:
    # Accumulate percentages per function
    function_data: dict[str, list[float]] = {}

    for profile in profiles:
        for func in profile['top_functions']:
            name = func['name']
            if name not in function_data:
                function_data[name] = []
            function_data[name].append(func['percentage'])

    # Average percentages (only over runs where function appeared)
    averaged_functions = []
    for name, percentages in function_data.items():
        avg_pct = sum(percentages) / len(percentages)
        averaged_functions.append({
            'name': name,
            'avg_percentage': avg_pct,
            'occurrences': len(percentages)
        })

    # Sort by average percentage descending
    averaged_functions.sort(key=lambda f: f['avg_percentage'], reverse=True)

    # Filter to top N
    return {
        'metadata': {...},
        'top_functions': averaged_functions[:top_n]
    }
```

### Error Handling Strategy

| Error Condition | Detection | Response | Exit Code |
|-----------------|-----------|----------|-----------|
| No profile results found | `load_multiple_profiles()` checks directory | Print error with hint to run profiling | 1 |
| No baseline found | `load_json()` raises FileNotFoundError | Print error with hint to use `--set-baseline` | 1 |
| Invalid JSON in profile | `json.load()` raises JSONDecodeError | Print error with file path | 1 |
| Fewer than 1 profile for averaging | `load_multiple_profiles()` validates count | Print error (need at least 1 run) | 1 |
| Regression detected (--strict) | `compare_profiles()` sets regressed > 0 | Print error with count and threshold | 1 |
| Missing required fields in JSON | Dict access raises KeyError | Print error with missing field name | 1 |

**Error message format** (matching compare_benchmarks.py):
```python
print(f"{RED}ERROR: No profile results found in {results_dir}/{executable}{NC}",
      file=sys.stderr)
print(f"Run profiling first:", file=sys.stderr)
print(f"  ./scripts/profile-instruments.sh <executable>", file=sys.stderr)
print(f"  ./scripts/parse-profile.py profile_results/*.trace", file=sys.stderr)
```

### Color Coding Conventions

Using ANSI codes matching project standards:

```python
GREEN = '\033[0;32m'   # PASS, success messages
YELLOW = '\033[1;33m'  # New hotspots, disappeared functions
RED = '\033[0;31m'     # REGRESSION, errors
BLUE = '\033[0;34m'    # Headers, metadata
NC = '\033[0m'         # No Color (reset)
```

**Usage**:
- `PASS` status: Green
- `REGRESSION` status: Red
- New hotspots: Yellow
- Disappeared functions: Yellow
- Section headers: Blue
- Error messages: Red to stderr

## Test Impact

### Manual Testing Required

Since this is a Python script (not C++ code), testing will be manual:

#### Test Case 1: Basic Comparison
**Setup**:
1. Run profiling to generate 5+ profile results
2. Set baseline: `./scripts/compare-profiles.py --set-baseline`
3. Make code change that increases function CPU usage
4. Run profiling again

**Expected**:
- Script detects regression
- JSON report written
- Console output shows RED status for regressed function

#### Test Case 2: Multiple Run Averaging
**Setup**:
1. Generate 5 profile runs
2. Verify function percentages vary slightly between runs
3. Run comparison

**Expected**:
- Script averages top 5 runs
- Averaged percentages smooth out variance
- Comparison uses averaged values

#### Test Case 3: Set Baseline
**Setup**:
1. Delete existing baseline
2. Generate 5 profile runs
3. Run `--set-baseline`

**Expected**:
- Baseline created at `profile_baselines/{exec}/baseline.json`
- Baseline contains averaged data from 5 runs
- Green success message printed

#### Test Case 4: Strict Mode (CI)
**Setup**:
1. Create baseline
2. Introduce regression
3. Run with `--strict`

**Expected**:
- Exit code 1
- Error message to stderr
- Comparison report still generated

#### Test Case 5: New Hotspots
**Setup**:
1. Baseline with top 10 functions
2. Add new code path that enters top 10

**Expected**:
- New function appears in "new_hotspots" list
- Yellow console output for new function
- Does not trigger regression (status informational)

#### Test Case 6: Multiple Executables
**Setup**:
1. Profile both `msd_sim_test` and `msd_sim_bench`
2. Set baselines for each
3. Run comparison for each

**Expected**:
- Separate baselines in `profile_baselines/msd_sim_test/` and `profile_baselines/msd_sim_bench/`
- Auto-detection works for both
- No cross-contamination

### Integration with CI (Future)

**Planned workflow**:
```bash
# In CI pipeline
./scripts/profile-instruments.sh ./build/Release/msd_sim_test
./scripts/parse-profile.py profile_results/*.trace --project-only
./scripts/compare-profiles.py --strict --no-color

# Exit code 1 if regression detected
```

**Not in scope for this design**: Actual CI integration (defer to future ticket)

## Open Questions

### Design Decisions (Human Input Needed)

None — All design decisions have been resolved via ticket feedback:
- ✓ Threshold type: 50% relative increase (confirmed)
- ✓ Top N functions: 10 (configurable via --top)
- ✓ Run averaging: 5 runs (configurable via --runs)
- ✓ Missing function handling: Average over runs where function appeared

### Prototype Required

None — The algorithm is straightforward and well-established from compare_benchmarks.py reference.

### Requirements Clarification

None — Requirements are clear and complete.

## Implementation Notes

### Similarities to compare_benchmarks.py

The implementation should closely mirror compare_benchmarks.py structure:

| compare_benchmarks.py | compare-profiles.py | Notes |
|-----------------------|---------------------|-------|
| `load_json()` | `load_json()` | Identical function |
| `match_benchmarks()` | `match_functions()` | Same pattern, different field names |
| `compare_benchmarks()` | `compare_profiles()` | Same algorithm (relative increase) |
| `print_comparison_results()` | `print_comparison_results()` | Same table format, different columns |
| `set_baseline()` | `set_baseline()` | Enhanced with averaging |
| N/A | `load_multiple_profiles()` | New (profiles need averaging) |
| N/A | `average_profile_runs()` | New (profiles need averaging) |

### Key Differences from Benchmarks

1. **Multiple run averaging**: Benchmarks use single run, profiles average M runs
2. **Executable detection**: Benchmarks use suite name, profiles use executable name
3. **Comparison metric**: Benchmarks use cpu_time (ns), profiles use percentage (%)
4. **Filtering**: Benchmarks filter aggregates, profiles filter to top N functions

### Python Version and Dependencies

- **Python**: 3.9+ (for built-in generics: `list`, `dict` not `List`, `Dict`)
- **Dependencies**: Standard library only (json, argparse, pathlib, datetime)
- **Type hints**: Use modern syntax (`list[dict]` not `List[Dict]`)

### File Organization

```
scripts/compare-profiles.py
├── Imports (json, argparse, pathlib, datetime, sys)
├── ANSI color constants
├── Helper functions:
│   ├── load_json()
│   ├── load_multiple_profiles()
│   ├── average_profile_runs()
│   ├── match_functions()
│   ├── compare_profiles()
│   ├── print_comparison_results()
│   └── set_baseline()
└── main()
    ├── Argument parsing
    ├── Auto-detection logic
    ├── --set-baseline path
    ├── Comparison path
    └── Exit code logic
```

**Line count estimate**: ~500-600 lines (similar to compare_benchmarks.py but with averaging logic)

## CLAUDE.md Updates Required

Add new section under "Profiling" (after profiling trace parser documentation):

```markdown
### Profiling Regression Detection

**Ticket**: [0016_profiling_regression_tracker](tickets/0016_profiling_regression_tracker.md)
**Design**: [`docs/designs/0016_profiling_regression_tracker/design.md`](docs/designs/0016_profiling_regression_tracker/design.md)

The project uses `compare-profiles.py` to detect profiling regressions by comparing results against golden baseline files.

**Basic workflow**:
```bash
# Run profiling and parse results
./scripts/profile-instruments.sh ./build/Release/msd_sim_test
./scripts/parse-profile.py profile_results/*.trace --project-only

# Compare against baseline (averages top 5 runs)
./scripts/compare-profiles.py

# Update baseline (when performance changes are intentional)
./scripts/compare-profiles.py --set-baseline
```

**Interpreting results**:
- **GREEN**: Performance within threshold or improved
- **YELLOW**: New/disappeared hotspots (review if expected)
- **RED**: Regression detected (exceeds threshold)

**Default threshold**: 50% increase in sample percentage triggers regression

**Advanced options**:
```bash
# Use custom threshold (75% instead of default 50%)
./scripts/compare-profiles.py --threshold 75.0

# Average more runs for stability (10 instead of default 5)
./scripts/compare-profiles.py --runs 10

# Track more functions (20 instead of default 10)
./scripts/compare-profiles.py --top 20

# Strict mode: exit code 1 on regression (for CI)
./scripts/compare-profiles.py --strict

# Disable colors (for CI logs)
./scripts/compare-profiles.py --no-color
```

**Baseline files**:
- Location: `profile_baselines/{executable}/baseline.json`
- Committed to git for team-wide consistency
- Update when intentional performance changes occur
- Contains averaged data from top 5 runs (configurable)

**When to update baselines**:
1. After performance optimizations that change function percentages
2. When refactoring redistributes CPU usage across functions
3. When adding new features that become new hotspots
4. Always commit baseline updates with code changes that affect them
```

## Next Steps

1. **Human Review**: Confirm design aligns with expectations
2. **Prototype** (if requested): Create minimal implementation to validate averaging algorithm
3. **Implementation**: Build full script following this design
4. **Testing**: Manual testing per test cases above
5. **Documentation**: Update CLAUDE.md per specification above

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-01-10
**Status**: APPROVED
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | Python script follows snake_case convention matching compare_benchmarks.py. Function names are descriptive and consistent. |
| File structure | ✓ | Script location `scripts/compare-profiles.py` matches established pattern. Directory structure for baselines (`profile_baselines/`) mirrors `benchmark_baselines/`. |
| Consistency with existing tools | ✓ | Excellent alignment with compare_benchmarks.py architecture. Design reuses proven patterns (color codes, CLI structure, exit codes, JSON reports). |
| Dependency direction | ✓ | Script consumes parse-profile.py output as input. No circular dependencies. Clear data flow: profile → parse → compare. |

**Overall**: The design demonstrates strong architectural consistency with the existing benchmark comparison infrastructure. The parallels between compare-profiles.py and compare_benchmarks.py are well-documented and appropriate.

#### Python Design Quality (adapting C++ criteria)
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Type hints | ✓ | Modern Python 3.9+ syntax used (`list[dict]` not `List[Dict]`). All function signatures include type hints. |
| Error handling | ✓ | Comprehensive error cases identified with appropriate exceptions (FileNotFoundError, JSONDecodeError, ValueError). |
| Separation of concerns | ✓ | Functions have single responsibilities: load_multiple_profiles (I/O), average_profile_runs (computation), match_functions (pairing), compare_profiles (analysis). |
| Constants | ✓ | ANSI color codes defined at module level matching project conventions. |
| Dependencies | ✓ | Stdlib only (json, argparse, pathlib, datetime). No external dependencies. |
| Input validation | ✓ | Edge cases handled: zero baseline percentage, missing functions, fewer than minimum runs. |

**Overall**: Python code quality standards are well-defined and follow best practices. The design correctly adapts C++ principles to Python idioms.

#### Feasibility Assessment
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Algorithm complexity | ✓ | Averaging algorithm is straightforward O(n×m) where n=functions, m=runs. Performance well within 2-second target. |
| JSON schema compatibility | ✓ | Design correctly uses parse-profile.py output schema. Fields `name`, `percentage`, `samples` are all present in actual profile JSON (verified against profile_20260110_145821.json). |
| File I/O patterns | ✓ | Uses pathlib.Path consistently. Handles missing directories with mkdir(parents=True, exist_ok=True). |
| Integration points | ✓ | Clear handoff from parse-profile.py (produces input) to compare-profiles.py (consumes input). CI integration via --strict flag well-defined. |
| Multi-run averaging | ✓ | Algorithm handles missing functions gracefully (Option A: average only over runs where function appeared). Timestamp-based sorting for "most recent N runs" is straightforward. |

**Overall**: The design is highly feasible. All components are straightforward to implement with no complex dependencies or algorithmic challenges.

#### Testability Assessment
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Manual testing | ✓ | Six comprehensive test cases defined covering: basic comparison, multi-run averaging, baseline setting, strict mode, new hotspots, multiple executables. |
| Test data generation | ✓ | Real profile data exists (profile_results/*.json). Test cases can use actual profiling runs. |
| Deterministic behavior | ✓ | Algorithm is deterministic for given inputs. Averaging uses arithmetic mean (repeatable). |
| Observable outputs | ✓ | Multiple verification points: JSON reports, console output, exit codes, baseline files. |

**Overall**: Strong testability. Manual testing approach is appropriate for Python scripts. Test cases are concrete and measurable.

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Profile variance across runs may be too high for 5-run average to be stable | Performance | Medium | Medium | --runs parameter allows increasing sample size. Design documents that functions not in top N are treated as noise (excluded from average). | No |
| R2 | 50% default threshold may be too sensitive or too lenient for profiling data | Technical | Medium | Low | --threshold parameter allows tuning. Can adjust default after initial usage. Documented as configurable. | No |
| R3 | Auto-detection of executable name may fail if directory structure changes | Integration | Low | Low | --executable parameter provides manual override. Error messages guide user to available options. | No |
| R4 | Timestamps in filenames must parse correctly for "most recent N" sorting | Technical | Low | Medium | Design specifies YYYYMMDD_HHMMSS format matching parse-profile.py. Simple string sort works correctly for this format. | No |

**Risk Summary**: All identified risks are low-to-medium likelihood with clear mitigations. No high-impact risks without mitigation. No prototyping required.

### Design Strengths

1. **Excellent precedent reuse**: The design closely follows compare_benchmarks.py, which is a proven, working implementation. This significantly reduces implementation risk.

2. **Comprehensive edge case handling**: The design explicitly addresses:
   - Functions appearing in some runs but not others (averaging algorithm Option A)
   - Division by zero when baseline percentage is 0.0%
   - Multiple executables via subdirectory organization
   - Missing baselines with helpful error messages

3. **Clear algorithm specification**: The relative percentage increase calculation is fully specified with examples (baseline 7% → current 10.5% = 50% increase). The averaging algorithm includes pseudocode.

4. **Thoughtful metric choice**: Using sample percentage (not absolute counts) correctly normalizes for varying profile durations. This is the right metric for comparison.

5. **Integration clarity**: JSON schema compatibility is documented with actual fields from parse-profile.py. Integration points are explicit.

6. **Testability**: Six concrete test cases with setup steps and expected outcomes provide a clear testing roadmap.

### Minor Observations (not requiring changes)

1. **Threshold semantics**: The 50% default threshold for profiling is higher than the 10% threshold for benchmarks. This is appropriate because profiling data has more variance than benchmarks. The design correctly documents this as configurable.

2. **Top N filtering**: Tracking only top 10 functions (not all functions) is pragmatic. This avoids noise from functions with <0.1% samples. The design allows override via --top parameter.

3. **Run averaging**: The choice to average over 5 runs is reasonable for balancing stability vs. recency. This is configurable via --runs parameter.

4. **Missing function handling**: Option A (average only over runs where function appeared) is the correct choice. If a function wasn't in top N in a run, treating it as 0% would incorrectly drag down the average.

### Prototype Guidance

No prototypes required. The algorithm is straightforward, well-specified, and follows a proven reference implementation.

### Summary

This design is **APPROVED** for implementation. The profiling regression tracker design demonstrates excellent architectural fit with existing infrastructure (compare_benchmarks.py), clear specification of algorithms (multi-run averaging, relative percentage comparison), and comprehensive error handling. All feasibility criteria pass, with no high-impact risks lacking mitigation.

The design correctly adapts the benchmark comparison pattern to profiling data, making appropriate adjustments:
- Multi-run averaging for stability (profiles vary more than benchmarks)
- Sample percentage comparison (normalizes for duration)
- Top N function filtering (reduces noise)
- Higher default threshold (50% vs 10%, appropriate for noisier profiling data)

The JSON schema compatibility is verified against actual parse-profile.py output. The six test cases provide a clear validation roadmap. No prototyping is required—proceed directly to implementation.

**Estimated implementation time**: 4-6 hours (script development, manual testing per 6 test cases, CLAUDE.md documentation update)

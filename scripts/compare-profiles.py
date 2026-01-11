#!/usr/bin/env python3
"""
Profiling Regression Tracker - Compare Time Profiler results against golden baselines.

Ticket: 0016_profiling_regression_tracker
Design: docs/designs/0016_profiling_regression_tracker/design.md
"""

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

# ANSI color codes matching compare_benchmarks.py
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
RED = '\033[0;31m'
BLUE = '\033[0;34m'
NC = '\033[0m'  # No color


def load_json(file_path: Path) -> dict:
    """Load and validate JSON file."""
    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"{RED}ERROR: File not found: {file_path}{NC}", file=sys.stderr)
        raise
    except json.JSONDecodeError as e:
        print(f"{RED}ERROR: Invalid JSON in {file_path}: {e}{NC}", file=sys.stderr)
        raise


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
    """
    exec_dir = results_dir / executable
    if not exec_dir.exists():
        raise FileNotFoundError(f"Executable directory not found: {exec_dir}")

    # Find all profile_*.json files
    profile_files = sorted(exec_dir.glob('profile_*.json'), reverse=True)

    if len(profile_files) < 1:
        raise ValueError(f"No profile results found in {exec_dir}")

    # Load top N most recent files
    profiles = []
    for profile_file in profile_files[:count]:
        try:
            profiles.append(load_json(profile_file))
        except (FileNotFoundError, json.JSONDecodeError):
            # Skip invalid files
            continue

    if len(profiles) < 1:
        raise ValueError(f"No valid profile JSON files found in {exec_dir}")

    return profiles


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
    # Accumulate percentages and samples per function
    function_data: dict[str, list[tuple[float, int]]] = {}

    total_samples_sum = 0
    for profile in profiles:
        total_samples_sum += profile.get('summary', {}).get('total_samples', 0)

        for func in profile.get('top_functions', []):
            name = func['name']
            if name not in function_data:
                function_data[name] = []
            function_data[name].append((func['percentage'], func['samples']))

    # Average percentages (only over runs where function appeared)
    averaged_functions = []
    for name, data_list in function_data.items():
        percentages = [pct for pct, _ in data_list]
        samples = [smp for _, smp in data_list]

        avg_pct = sum(percentages) / len(percentages)
        avg_smp = sum(samples) / len(samples)

        averaged_functions.append({
            'name': name,
            'avg_percentage': avg_pct,
            'avg_samples': avg_smp,
            'occurrences': len(percentages)
        })

    # Sort by average percentage descending
    averaged_functions.sort(key=lambda f: f['avg_percentage'], reverse=True)

    # Filter to top N
    top_functions = averaged_functions[:top_n]

    # Extract executable name from first profile
    executable = profiles[0].get('metadata', {}).get('executable', 'unknown')

    return {
        'metadata': {
            'runs_averaged': len(profiles),
            'executable': executable,
            'average_timestamp': datetime.now().isoformat()
        },
        'summary': {
            'avg_total_samples': total_samples_sum / len(profiles)
        },
        'top_functions': top_functions
    }


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
    # Build name-indexed dictionaries
    current_by_name = {f['name']: f for f in current.get('top_functions', [])}
    baseline_by_name = {f['name']: f for f in baseline.get('top_functions', [])}

    # Find matches and differences
    matched_pairs = []
    for name, current_func in current_by_name.items():
        if name in baseline_by_name:
            matched_pairs.append((current_func, baseline_by_name[name]))

    new_hotspots = [name for name in current_by_name if name not in baseline_by_name]
    disappeared = [name for name in baseline_by_name if name not in current_by_name]

    return matched_pairs, new_hotspots, disappeared


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
    matched_pairs, new_hotspots, disappeared = match_functions(current, baseline)

    # Compare matched functions
    function_results = []
    passed = 0
    regressed = 0

    for current_func, baseline_func in matched_pairs:
        # Use avg_percentage for averaged profiles, percentage for baseline
        current_pct = current_func.get('avg_percentage', current_func.get('percentage', 0.0))
        baseline_pct = baseline_func.get('avg_percentage', baseline_func.get('percentage', 0.0))

        current_samples = current_func.get('avg_samples', current_func.get('samples', 0))
        baseline_samples = baseline_func.get('avg_samples', baseline_func.get('samples', 0))

        # Calculate relative percentage increase
        if baseline_pct == 0.0:
            # Avoid division by zero - skip comparison
            diff_percent = 0.0
            status = 'PASS'
        else:
            diff_percent = ((current_pct - baseline_pct) / baseline_pct) * 100.0
            status = 'REGRESSION' if diff_percent > threshold_percent else 'PASS'

        if status == 'REGRESSION':
            regressed += 1
        else:
            passed += 1

        function_results.append({
            'name': current_func['name'],
            'current_percentage': current_pct,
            'baseline_percentage': baseline_pct,
            'current_samples': current_samples,
            'baseline_samples': baseline_samples,
            'diff_percent': diff_percent,
            'status': status
        })

    # Extract timestamps
    current_timestamp = current.get('metadata', {}).get('average_timestamp', 'unknown')
    baseline_timestamp = baseline.get('metadata', {}).get('average_timestamp',
                                                           baseline.get('metadata', {}).get('export_timestamp', 'unknown'))
    executable = current.get('metadata', {}).get('executable', 'unknown')
    runs_averaged = current.get('metadata', {}).get('runs_averaged', 1)

    return {
        'metadata': {
            'generated_at': datetime.now().isoformat(),
            'executable': executable,
            'runs_averaged': runs_averaged,
            'threshold_percent': threshold_percent,
            'current_timestamp': current_timestamp,
            'baseline_timestamp': baseline_timestamp
        },
        'functions': function_results,
        'summary': {
            'total_compared': len(matched_pairs),
            'passed': passed,
            'regressed': regressed,
            'new_hotspots': sorted(new_hotspots),
            'disappeared': sorted(disappeared)
        }
    }


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
    def colorize(text: str, color_code: str) -> str:
        return f"{color_code}{text}{NC}" if color else text

    print(f"\n{colorize('Comparing profiles against baseline...', BLUE)}")
    print(f"Threshold: {comparison['metadata']['threshold_percent']}%")
    print(f"Runs averaged: {comparison['metadata']['runs_averaged']}\n")

    # Print table header
    print("┌─────────────────────────────────────────┬──────────┬──────────┬───────────┬──────────┐")
    print("│ Function                                │ Current  │ Baseline │ Diff      │ Status   │")
    print("├─────────────────────────────────────────┼──────────┼──────────┼───────────┼──────────┤")

    # Print function rows
    for func in comparison['functions']:
        name = func['name'][:39]  # Truncate if too long
        current_pct = f"{func['current_percentage']:.2f}%"
        baseline_pct = f"{func['baseline_percentage']:.2f}%"
        diff = f"{func['diff_percent']:+.1f}%"
        status = func['status']

        # Color code the status
        if status == 'PASS':
            status_colored = colorize('PASS', GREEN)
            diff_colored = colorize(diff, GREEN)
        else:
            status_colored = colorize('REGR', RED)
            diff_colored = colorize(diff, RED)

        print(f"│ {name:<39} │ {current_pct:>8} │ {baseline_pct:>8} │ {diff_colored:>9} │ {status_colored:>8} │")

    print("└─────────────────────────────────────────┴──────────┴──────────┴───────────┴──────────┘")

    # Print summary
    summary = comparison['summary']
    print(f"\n{colorize('Summary:', BLUE)}")
    print(f"  Total: {summary['total_compared']}")
    print(f"  Passed: {colorize(str(summary['passed']), GREEN)}")
    print(f"  Regressed: {colorize(str(summary['regressed']), RED if summary['regressed'] > 0 else GREEN)}")

    # Print new hotspots
    if summary['new_hotspots']:
        print(f"\n{colorize('New hotspots (not in baseline):', YELLOW)}")
        for name in summary['new_hotspots']:
            print(f"  - {name}")

    # Print disappeared functions
    if summary['disappeared']:
        print(f"\n{colorize('Disappeared (in baseline, not in current):', YELLOW)}")
        for name in summary['disappeared']:
            print(f"  - {name}")


def set_baseline(
    results_dir: Path,
    baseline_file: Path,
    executable: str,
    runs: int,
    top_n: int
) -> None:
    """
    Average recent runs and copy to baseline file.

    Args:
        results_dir: Path to profile_results directory
        baseline_file: Path to baseline.json destination
        executable: Executable name
        runs: Number of recent runs to average
        top_n: Number of top functions to include

    Raises:
        FileNotFoundError: If no profiles found
        ValueError: If invalid JSON
    """
    # Load and average recent runs
    profiles = load_multiple_profiles(results_dir, executable, runs)
    averaged = average_profile_runs(profiles, top_n)

    # Create baseline directory if needed
    baseline_file.parent.mkdir(parents=True, exist_ok=True)

    # Write averaged profile to baseline
    with open(baseline_file, 'w') as f:
        json.dump(averaged, f, indent=2)

    print(f"{GREEN}Baseline updated: {baseline_file}{NC}")
    print(f"  Runs averaged: {len(profiles)}")
    print(f"  Top functions: {len(averaged['top_functions'])}")


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
    parser = argparse.ArgumentParser(
        description='Compare Time Profiler results against golden baselines.'
    )

    parser.add_argument(
        '--current',
        type=Path,
        help='Path to current profile (default: auto-detect latest M runs)'
    )
    parser.add_argument(
        '--baseline',
        type=Path,
        help='Path to baseline (default: profile_baselines/{exec}/baseline.json)'
    )
    parser.add_argument(
        '--executable',
        type=str,
        help='Executable name (default: auto-detect from directory structure)'
    )
    parser.add_argument(
        '--threshold',
        type=float,
        default=50.0,
        help='Regression threshold as percentage increase (default: 50.0)'
    )
    parser.add_argument(
        '--top',
        type=int,
        default=10,
        help='Number of top functions to track (default: 10)'
    )
    parser.add_argument(
        '--runs',
        type=int,
        default=5,
        help='Number of recent runs to average (default: 5)'
    )
    parser.add_argument(
        '--set-baseline',
        action='store_true',
        help='Copy averaged current runs to baseline file'
    )
    parser.add_argument(
        '--strict',
        action='store_true',
        help='Exit code 1 on regression (for CI)'
    )
    parser.add_argument(
        '--output-json-only',
        action='store_true',
        help='Suppress console output, write JSON only'
    )
    parser.add_argument(
        '--no-color',
        action='store_true',
        help='Disable ANSI color codes'
    )

    args = parser.parse_args()

    # Determine project root (parent of scripts/ directory)
    script_dir = Path(__file__).parent
    project_root = script_dir.parent

    # Auto-detect executable name if not provided
    executable_name = args.executable
    if not executable_name:
        # Look for profile_results directory and find executables
        results_dir = project_root / 'profile_results'
        if results_dir.exists():
            executables = [d.name for d in results_dir.iterdir() if d.is_dir()]
            if len(executables) == 1:
                executable_name = executables[0]
            elif len(executables) == 0:
                print(f"{RED}ERROR: No executable directories found in {results_dir}{NC}", file=sys.stderr)
                return 1
            else:
                print(f"{RED}ERROR: Multiple executables found. Please specify --executable{NC}", file=sys.stderr)
                print(f"Available executables: {', '.join(executables)}", file=sys.stderr)
                return 1
        else:
            print(f"{RED}ERROR: profile_results/ directory not found{NC}", file=sys.stderr)
            return 1

    # Determine baseline file
    if args.baseline:
        baseline_file = args.baseline
    else:
        baseline_file = project_root / 'profile_baselines' / executable_name / 'baseline.json'

    # Handle --set-baseline
    if args.set_baseline:
        try:
            results_dir = project_root / 'profile_results'
            set_baseline(results_dir, baseline_file, executable_name, args.runs, args.top)
            return 0
        except FileNotFoundError as e:
            print(f"{RED}ERROR: {e}{NC}", file=sys.stderr)
            print(f"Run profiling first:", file=sys.stderr)
            print(f"  ./scripts/profile-instruments.sh <executable>", file=sys.stderr)
            print(f"  ./scripts/parse-profile.py profile_results/*.trace --project-only", file=sys.stderr)
            return 1
        except ValueError as e:
            print(f"{RED}ERROR: {e}{NC}", file=sys.stderr)
            return 1

    # Load and average current runs
    try:
        results_dir = project_root / 'profile_results'
        current_profiles = load_multiple_profiles(results_dir, executable_name, args.runs)
        current_data = average_profile_runs(current_profiles, args.top)
    except FileNotFoundError:
        print(f"{RED}ERROR: No profile results found for {executable_name}{NC}", file=sys.stderr)
        print(f"Run profiling first:", file=sys.stderr)
        print(f"  ./scripts/profile-instruments.sh <executable>", file=sys.stderr)
        print(f"  ./scripts/parse-profile.py profile_results/*.trace --project-only", file=sys.stderr)
        return 1
    except ValueError as e:
        print(f"{RED}ERROR: {e}{NC}", file=sys.stderr)
        return 1

    # Load baseline
    try:
        baseline_data = load_json(baseline_file)
    except FileNotFoundError:
        print(f"{RED}ERROR: Baseline file not found: {baseline_file}{NC}", file=sys.stderr)
        print(f"To create a baseline, run:", file=sys.stderr)
        print(f"  ./scripts/compare-profiles.py --set-baseline", file=sys.stderr)
        return 1
    except json.JSONDecodeError:
        print(f"{RED}ERROR: Invalid JSON in baseline file{NC}", file=sys.stderr)
        print(f"Check git history or regenerate baseline.", file=sys.stderr)
        return 1

    # Compare profiles
    comparison = compare_profiles(current_data, baseline_data, args.threshold)

    # Write comparison report
    comparison_file = (
        project_root / 'profile_results' / executable_name /
        f"comparison_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )
    comparison_file.parent.mkdir(parents=True, exist_ok=True)

    with open(comparison_file, 'w') as f:
        json.dump(comparison, f, indent=2)

    # Print results to console (unless --output-json-only)
    if not args.output_json_only:
        print_comparison_results(comparison, color=not args.no_color)
        print(f"\n{BLUE if not args.no_color else ''}Comparison report: {comparison_file}{NC if not args.no_color else ''}")

    # Handle exit code
    if args.strict and comparison['summary']['regressed'] > 0:
        print(f"\n{RED}ERROR: Profiling regression detected (--strict mode){NC}", file=sys.stderr)
        print(f"{comparison['summary']['regressed']} function(s) exceeded threshold of {args.threshold}%", file=sys.stderr)
        print(f"See comparison report for details.", file=sys.stderr)
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())

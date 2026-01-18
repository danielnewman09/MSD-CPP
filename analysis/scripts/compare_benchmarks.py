#!/usr/bin/env python3
"""
Benchmark Metrics Tracker - Compare Google Benchmark results against golden baselines.

Ticket: 0014_benchmark_metrics_tracker
Design: docs/designs/0014_benchmark_metrics_tracker/design.md
"""

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

# ANSI color codes matching run_benchmarks.sh
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


def is_aggregate_benchmark(benchmark: dict) -> bool:
    """Check if benchmark is an aggregate (BigO, RMS)."""
    return benchmark.get('run_type') == 'aggregate' or 'aggregate_name' in benchmark


def match_benchmarks(
    current_benchmarks: list[dict],
    baseline_benchmarks: list[dict]
) -> tuple[list[tuple[dict, dict]], list[str], list[str]]:
    """
    Match benchmarks by exact name, filtering out aggregates.

    Returns:
        Tuple of:
        - matched_pairs: [(current_bench, baseline_bench), ...]
        - missing_in_baseline: ["BM_NewBench", ...]
        - missing_in_current: ["BM_OldBench", ...]
    """
    # Filter out aggregates
    current_filtered = [b for b in current_benchmarks if not is_aggregate_benchmark(b)]
    baseline_filtered = [b for b in baseline_benchmarks if not is_aggregate_benchmark(b)]

    # Build name-indexed dictionaries
    current_by_name = {b['name']: b for b in current_filtered}
    baseline_by_name = {b['name']: b for b in baseline_filtered}

    # Find matches and missing benchmarks
    matched_pairs = []
    for name, current_bench in current_by_name.items():
        if name in baseline_by_name:
            matched_pairs.append((current_bench, baseline_by_name[name]))

    missing_in_baseline = [name for name in current_by_name if name not in baseline_by_name]
    missing_in_current = [name for name in baseline_by_name if name not in current_by_name]

    return matched_pairs, missing_in_baseline, missing_in_current


def compare_benchmarks(
    current: dict,
    baseline: dict,
    threshold_percent: float = 10.0
) -> dict:
    """
    Compare current benchmark results against baseline.

    Args:
        current: Google Benchmark JSON output (current run)
        baseline: Google Benchmark JSON output (baseline)
        threshold_percent: Regression threshold (default 10%)

    Returns:
        Comparison result dictionary with metadata, per-benchmark results, and summary.
    """
    current_benchmarks = current['benchmarks']
    baseline_benchmarks = baseline['benchmarks']

    matched_pairs, missing_in_baseline, missing_in_current = match_benchmarks(
        current_benchmarks, baseline_benchmarks
    )

    # Compare matched benchmarks
    benchmark_results = []
    passed = 0
    regressed = 0

    for current_bench, baseline_bench in matched_pairs:
        current_cpu_time = current_bench['cpu_time']
        baseline_cpu_time = baseline_bench['cpu_time']

        diff_ns = current_cpu_time - baseline_cpu_time
        diff_percent = (diff_ns / baseline_cpu_time) * 100.0 if baseline_cpu_time != 0 else 0.0

        # Determine status based on threshold
        status = 'REGRESSION' if diff_percent > threshold_percent else 'PASS'
        if status == 'REGRESSION':
            regressed += 1
        else:
            passed += 1

        benchmark_results.append({
            'name': current_bench['name'],
            'current_cpu_time': current_cpu_time,
            'baseline_cpu_time': baseline_cpu_time,
            'diff_ns': diff_ns,
            'diff_percent': diff_percent,
            'status': status,
            'current_iterations': current_bench.get('iterations', 0),
            'baseline_iterations': baseline_bench.get('iterations', 0)
        })

    # Extract dates from context
    current_date = current.get('context', {}).get('date', 'unknown')
    baseline_date = baseline.get('context', {}).get('date', 'unknown')

    return {
        'metadata': {
            'generated_at': datetime.now().isoformat(),
            'current_date': current_date,
            'baseline_date': baseline_date,
            'threshold_percent': threshold_percent
        },
        'benchmarks': benchmark_results,
        'summary': {
            'total_compared': len(matched_pairs),
            'passed': passed,
            'regressed': regressed,
            'missing_in_current': sorted(missing_in_current),
            'missing_in_baseline': sorted(missing_in_baseline)
        }
    }


def print_comparison_results(comparison: dict, color: bool = True) -> None:
    """Print color-coded comparison results to console."""
    def colorize(text: str, color_code: str) -> str:
        return f"{color_code}{text}{NC}" if color else text

    print(f"\n{colorize('Comparing benchmarks against baseline...', BLUE)}")
    print(f"Threshold: {comparison['metadata']['threshold_percent']}%\n")

    # Print table header
    print("┌─────────────────────────────────────────┬──────────────┬──────────────┬───────────┬──────────┐")
    print("│ Benchmark                               │ Current      │ Baseline     │ Diff      │ Status   │")
    print("├─────────────────────────────────────────┼──────────────┼──────────────┼───────────┼──────────┤")

    # Print benchmark rows
    for bench in comparison['benchmarks']:
        name = bench['name'][:39]  # Truncate if too long
        current_time = f"{bench['current_cpu_time']:.0f} ns"
        baseline_time = f"{bench['baseline_cpu_time']:.0f} ns"
        diff = f"{bench['diff_percent']:+.2f}%"
        status = bench['status']

        # Color code the status
        if status == 'PASS':
            status_colored = colorize('PASS', GREEN)
            diff_colored = colorize(diff, GREEN)
        else:
            status_colored = colorize('REGR', RED)
            diff_colored = colorize(diff, RED)

        print(f"│ {name:<39} │ {current_time:>12} │ {baseline_time:>12} │ {diff_colored:>9} │ {status_colored:>8} │")

    print("└─────────────────────────────────────────┴──────────────┴──────────────┴───────────┴──────────┘")

    # Print summary
    summary = comparison['summary']
    print(f"\n{colorize('Summary:', BLUE)}")
    print(f"  Total: {summary['total_compared']}")
    print(f"  Passed: {colorize(str(summary['passed']), GREEN)}")
    print(f"  Regressed: {colorize(str(summary['regressed']), RED if summary['regressed'] > 0 else GREEN)}")

    # Print new benchmarks
    if summary['missing_in_baseline']:
        print(f"\n{colorize('New benchmarks (not in baseline):', YELLOW)}")
        for name in summary['missing_in_baseline']:
            print(f"  - {name}")

    # Print missing benchmarks
    if summary['missing_in_current']:
        print(f"\n{colorize('Missing benchmarks (in baseline, not in current):', YELLOW)}")
        for name in summary['missing_in_current']:
            print(f"  - {name}")


def set_baseline(results_file: Path, baseline_file: Path) -> None:
    """
    Copy current results to baseline file.

    Args:
        results_file: Path to benchmark_latest.json
        baseline_file: Path to baseline.json destination

    Raises:
        FileNotFoundError: If results_file doesn't exist
        ValueError: If results_file contains invalid JSON
    """
    if not results_file.exists():
        raise FileNotFoundError(f"Results file not found: {results_file}")

    # Validate JSON
    try:
        data = load_json(results_file)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in results file: {e}")

    # Create baseline directory if needed
    baseline_file.parent.mkdir(parents=True, exist_ok=True)

    # Copy results to baseline
    with open(baseline_file, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"{GREEN}Baseline updated: {baseline_file}{NC}")


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Compare Google Benchmark results against golden baselines.'
    )

    parser.add_argument(
        '--current',
        type=Path,
        help='Path to current benchmark results (default: auto-detect benchmark_latest.json)'
    )
    parser.add_argument(
        '--baseline',
        type=Path,
        help='Path to baseline file (default: benchmark_baselines/{suite}/baseline.json)'
    )
    parser.add_argument(
        '--suite',
        type=str,
        help='Benchmark suite name (default: auto-detect from current file path)'
    )
    parser.add_argument(
        '--threshold',
        type=float,
        default=10.0,
        help='Regression threshold percentage (default: 10.0)'
    )
    parser.add_argument(
        '--set-baseline',
        action='store_true',
        help='Copy current results to baseline file'
    )
    parser.add_argument(
        '--strict',
        action='store_true',
        help='Exit with code 1 on regression (for CI)'
    )
    parser.add_argument(
        '--output-json-only',
        action='store_true',
        help='Only write JSON report, no console output'
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

    # Auto-detect suite name if not provided
    suite_name = args.suite
    if not suite_name:
        # Look for benchmark_results directory and find suites
        results_dir = project_root / 'benchmark_results'
        if results_dir.exists():
            suites = [d.name for d in results_dir.iterdir() if d.is_dir()]
            if len(suites) == 1:
                suite_name = suites[0]
            elif len(suites) == 0:
                print(f"{RED}ERROR: No benchmark suites found in {results_dir}{NC}", file=sys.stderr)
                return 1
            else:
                print(f"{RED}ERROR: Multiple benchmark suites found. Please specify --suite{NC}", file=sys.stderr)
                print(f"Available suites: {', '.join(suites)}", file=sys.stderr)
                return 1
        else:
            print(f"{RED}ERROR: benchmark_results/ directory not found{NC}", file=sys.stderr)
            return 1

    # Determine current results file
    if args.current:
        current_file = args.current
    else:
        current_file = project_root / 'benchmark_results' / suite_name / 'benchmark_latest.json'

    # Determine baseline file
    if args.baseline:
        baseline_file = args.baseline
    else:
        baseline_file = project_root / 'benchmark_baselines' / suite_name / 'baseline.json'

    # Handle --set-baseline
    if args.set_baseline:
        try:
            set_baseline(current_file, baseline_file)
            return 0
        except FileNotFoundError as e:
            print(f"{RED}ERROR: {e}{NC}", file=sys.stderr)
            print(f"Run benchmarks first:", file=sys.stderr)
            print(f"  ./scripts/run_benchmarks.sh", file=sys.stderr)
            return 1
        except ValueError as e:
            print(f"{RED}ERROR: {e}{NC}", file=sys.stderr)
            return 1

    # Load current and baseline results
    try:
        current_data = load_json(current_file)
    except FileNotFoundError:
        print(f"{RED}ERROR: Current results file not found: {current_file}{NC}", file=sys.stderr)
        print(f"Run benchmarks first:", file=sys.stderr)
        print(f"  ./scripts/run_benchmarks.sh", file=sys.stderr)
        return 1
    except json.JSONDecodeError:
        print(f"{RED}ERROR: Invalid JSON in current results file{NC}", file=sys.stderr)
        return 1

    try:
        baseline_data = load_json(baseline_file)
    except FileNotFoundError:
        print(f"{RED}ERROR: Baseline file not found: {baseline_file}{NC}", file=sys.stderr)
        print(f"To create a baseline, run:", file=sys.stderr)
        print(f"  ./scripts/compare_benchmarks.py --set-baseline", file=sys.stderr)
        return 1
    except json.JSONDecodeError:
        print(f"{RED}ERROR: Invalid JSON in baseline file{NC}", file=sys.stderr)
        print(f"Check git history or regenerate baseline.", file=sys.stderr)
        return 1

    # Compare benchmarks
    comparison = compare_benchmarks(current_data, baseline_data, args.threshold)

    # Write comparison report
    comparison_file = (
        project_root / 'benchmark_results' / suite_name /
        f"comparison_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )
    comparison_file.parent.mkdir(parents=True, exist_ok=True)

    # Add suite name to metadata
    comparison['metadata']['suite'] = suite_name

    with open(comparison_file, 'w') as f:
        json.dump(comparison, f, indent=2)

    # Print results to console (unless --output-json-only)
    if not args.output_json_only:
        print_comparison_results(comparison, color=not args.no_color)
        print(f"\n{BLUE if not args.no_color else ''}Comparison report: {comparison_file}{NC if not args.no_color else ''}")

    # Handle exit code
    if args.strict and comparison['summary']['regressed'] > 0:
        print(f"\n{RED}ERROR: Performance regression detected (--strict mode){NC}", file=sys.stderr)
        print(f"{comparison['summary']['regressed']} benchmark(s) exceeded threshold of {args.threshold}%", file=sys.stderr)
        print(f"See comparison report for details.", file=sys.stderr)
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())

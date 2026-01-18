#!/usr/bin/env python3
"""
Profile Trace Parser - Extract Time Profiler data from Instruments traces.

Ticket: 0015_profiling_trace_parser
Design: docs/designs/0015_profiling_trace_parser/design.md

This script parses Instruments .trace files, exports Time Profiler data to XML
using xctrace, and extracts function sample counts into structured JSON format.
It enables programmatic analysis of profiling results without requiring manual
inspection in the Instruments GUI.

Usage:
    ./parse-profile.py <trace_file> [options]

Arguments:
    trace_file          - Path to .trace file (required)

Options:
    -o, --output FILE   - Output JSON file (default: profile_results/profile_<timestamp>.json)
    --top N             - Limit to top N functions (default: 20)
    --project-only      - Only include project functions (msd_sim::, msd_utils::, etc.)
    --json-only         - Output JSON only, no console summary
    --no-color          - Disable colored output
    -h, --help          - Show this help message

Example:
    ./parse-profile.py profile_results/profile_20260108_183915.trace
    ./parse-profile.py profile_results/profile_20260108_183915.trace --top 10
    ./parse-profile.py profile_results/profile_20260108_183915.trace -o report.json
"""

import argparse
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from typing import Optional

# ANSI color codes (matching project convention)
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
RED = '\033[0;31m'
BLUE = '\033[0;34m'
NC = '\033[0m'  # No Color

# Global flag for color output
USE_COLOR = True

# Project namespace prefixes for filtering
PROJECT_NAMESPACES = (
    'msd_sim::',
    'msd_utils::',
    'msd_assets::',
    'msd_transfer::',
    'msd_gui::',
)


def color_text(text: str, color: str) -> str:
    """Apply color to text if colors are enabled."""
    if USE_COLOR:
        return f"{color}{text}{NC}"
    return text


def error(message: str) -> None:
    """Print error message to stderr."""
    print(color_text(f"ERROR: {message}", RED), file=sys.stderr)


def warning(message: str) -> None:
    """Print warning message to stderr."""
    print(color_text(f"WARNING: {message}", YELLOW), file=sys.stderr)


def info(message: str) -> None:
    """Print info message to stdout."""
    print(color_text(message, GREEN))


def check_xctrace_available() -> bool:
    """Check if xctrace is available on the system."""
    try:
        subprocess.run(
            ['xctrace', 'version'],
            capture_output=True,
            check=True,
            timeout=5
        )
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError):
        return False


def export_trace_to_xml(trace_file: Path) -> Optional[Path]:
    """
    Export .trace to XML using xctrace.

    Args:
        trace_file: Path to .trace file

    Returns:
        Path to XML file if export succeeded, None otherwise
    """
    xml_file = trace_file.parent / f"{trace_file.stem}.xml"

    info(f"Exporting trace to XML: {xml_file.name}")

    try:
        # Export Time Profiler data using xpath
        result = subprocess.run(
            [
                'xctrace', 'export',
                '--input', str(trace_file),
                '--xpath', '/trace-toc/run[@number="1"]/data/table[@schema="time-profile"]',
                '--output', str(xml_file)
            ],
            capture_output=True,
            timeout=60,
            text=True
        )

        if result.returncode != 0:
            error(f"xctrace export failed: {result.stderr}")
            return None

        if not xml_file.exists():
            error("XML file was not created")
            return None

        return xml_file

    except subprocess.TimeoutExpired:
        error("XML export timed out (>60 seconds)")
        return None
    except Exception as e:
        error(f"Failed to export XML: {e}")
        return None


def is_project_function(func_name: str) -> bool:
    """Check if function name belongs to project namespaces."""
    return func_name.startswith(PROJECT_NAMESPACES)


def parse_time_profiler_xml(xml_file: Path, project_only: bool = False) -> dict:
    """
    Parse Time Profiler XML schema to extract function samples.

    Based on prototype findings (P1), the XML uses a time-sample based format
    with <row> elements containing backtraces. This function accumulates sample
    counts per function across all time samples.

    Args:
        xml_file: Path to XML file
        project_only: If True, only include functions from project namespaces

    Returns:
        Dictionary with metadata, summary, and top_functions
    """
    try:
        tree = ET.parse(xml_file)
        root = tree.getroot()
    except ET.ParseError as e:
        error(f"Failed to parse XML: {e}")
        return None
    except Exception as e:
        error(f"Failed to read XML file: {e}")
        return None

    # Dictionary to accumulate sample counts per function
    function_samples: dict[str, dict] = {}
    total_samples = 0

    # Find all row elements (time samples)
    rows = root.findall('.//row')

    if not rows:
        warning("No time samples found in XML")
        return None

    # Process each time sample
    for row in rows:
        total_samples += 1

        # Extract backtrace (call stack)
        backtrace = row.find('backtrace')
        if backtrace is None:
            continue

        # If backtrace is a reference, we can't resolve it without building a reference map
        # For simplicity, we only process inline backtraces in this version
        if backtrace.get('ref') is not None:
            continue

        # Extract all frames in the backtrace
        frames = backtrace.findall('frame')

        for frame in frames:
            func_name = frame.get('name', '<unknown>')

            # Skip empty or unknown functions
            if not func_name or func_name == '<unknown>':
                continue

            # Filter to project namespaces if requested
            if project_only and not is_project_function(func_name):
                continue

            # Extract source location if available
            source_elem = frame.find('source')
            source_file = None
            line = None

            if source_elem is not None:
                path_elem = source_elem.find('path')
                if path_elem is not None and path_elem.text:
                    # Extract just the filename (not full path)
                    source_file = Path(path_elem.text).name
                line_str = source_elem.get('line')
                if line_str:
                    try:
                        line = int(line_str)
                    except ValueError:
                        pass

            # Accumulate sample count for this function
            if func_name not in function_samples:
                function_samples[func_name] = {
                    'name': func_name,
                    'samples': 1,  # First sample for this function
                    'source_file': source_file,
                    'line': line
                }
            else:
                # Increment sample count
                function_samples[func_name]['samples'] += 1
                # Update source location if this occurrence has it and previous didn't
                if source_file and not function_samples[func_name]['source_file']:
                    function_samples[func_name]['source_file'] = source_file
                if line and not function_samples[func_name]['line']:
                    function_samples[func_name]['line'] = line

    # Sort functions by sample count descending
    sorted_functions = sorted(
        function_samples.values(),
        key=lambda x: x['samples'],
        reverse=True
    )

    # Calculate percentages
    for func in sorted_functions:
        func['percentage'] = round((func['samples'] / total_samples) * 100, 1)

    return {
        'total_samples': total_samples,
        'functions': sorted_functions
    }


def build_json_report(
    trace_file: Path,
    parsed_data: dict,
    top_n: int
) -> dict:
    """
    Build JSON report in the project's standard format.

    Args:
        trace_file: Original trace file path
        parsed_data: Parsed profiling data
        top_n: Number of top functions to include

    Returns:
        Dictionary representing the JSON report
    """
    top_functions = parsed_data['functions'][:top_n]

    # Add rank to each function
    for i, func in enumerate(top_functions, 1):
        func['rank'] = i

    # Build metadata
    metadata = {
        'trace_file': trace_file.name,
        'template': 'Time Profiler',
        'export_timestamp': datetime.now().isoformat(),
        'executable': 'unknown'  # Could extract from trace metadata if needed
    }

    # Build summary
    summary = {
        'total_samples': parsed_data['total_samples'],
        'total_time_ms': round(parsed_data['total_samples'] * 1.0, 1)  # ~1ms per sample
    }

    return {
        'metadata': metadata,
        'summary': summary,
        'top_functions': top_functions
    }


def format_console_output(report: dict, top_n: int) -> None:
    """
    Print color-coded table of top functions to console.

    Args:
        report: JSON report dictionary
        top_n: Number of functions to display
    """
    metadata = report['metadata']
    summary = report['summary']
    functions = report['top_functions']

    # Header
    print()
    print(color_text(f"Profiling Summary: {metadata['trace_file']}", BLUE))
    print(f"Total Samples: {summary['total_samples']:,} (~{summary['total_time_ms']:,.0f} ms)")
    print()

    # Table header
    print("┌─────┬──────────────────────────────────────────────────────┬─────────┬──────────┬──────────────────┐")
    print("│ Rank│ Function                                             │ Samples │ Percent  │ Source           │")
    print("├─────┼──────────────────────────────────────────────────────┼─────────┼──────────┼──────────────────┤")

    # Table rows
    for func in functions[:top_n]:
        rank = func['rank']
        name = func['name'][:52]  # Truncate long names
        samples = func['samples']
        percentage = func['percentage']
        source = func.get('source_file', 'N/A')
        if source and func.get('line'):
            source = f"{source}:{func['line']}"
        elif not source:
            source = 'N/A'

        print(f"│ {rank:3d} │ {name:52s} │ {samples:7,d} │ {percentage:6.1f}%  │ {source:16s} │")

    # Table footer
    print("└─────┴──────────────────────────────────────────────────────┴─────────┴──────────┴──────────────────┘")

    # Top hotspot highlight
    if functions:
        top_func = functions[0]
        print()
        print(color_text(
            f"Top hotspot: {top_func['name']} ({top_func['percentage']}% of samples)",
            GREEN
        ))


def main() -> int:
    """Main entry point."""
    global USE_COLOR

    parser = argparse.ArgumentParser(
        description='Extract Time Profiler data from Instruments traces.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    %(prog)s profile_results/profile_20260108_183915.trace
    %(prog)s profile_results/profile_20260108_183915.trace --top 10
    %(prog)s profile_results/profile_20260108_183915.trace -o report.json
        """
    )

    parser.add_argument(
        'trace_file',
        type=Path,
        help='Path to .trace file'
    )
    parser.add_argument(
        '-o', '--output',
        type=Path,
        help='Output JSON file (default: profile_results/profile_<timestamp>.json)'
    )
    parser.add_argument(
        '--top',
        type=int,
        default=20,
        help='Limit to top N functions (default: 20)'
    )
    parser.add_argument(
        '--project-only',
        action='store_true',
        help='Only include project functions (msd_sim::, msd_utils::, etc.)'
    )
    parser.add_argument(
        '--json-only',
        action='store_true',
        help='Output JSON only, no console summary'
    )
    parser.add_argument(
        '--no-color',
        action='store_true',
        help='Disable colored output'
    )

    args = parser.parse_args()

    # Disable colors if requested
    if args.no_color:
        USE_COLOR = False

    # Validate trace file
    if not args.trace_file.exists():
        error(f"Trace file not found: {args.trace_file}")
        print("Run profiling first:", file=sys.stderr)
        print("  ./scripts/profile-instruments.sh <executable> --export-xml", file=sys.stderr)
        return 1

    if not args.trace_file.suffix == '.trace':
        error(f"File is not a .trace file: {args.trace_file}")
        return 1

    # Check xctrace availability
    if not check_xctrace_available():
        error("xctrace not found")
        print("Please install Xcode Command Line Tools:", file=sys.stderr)
        print("  xcode-select --install", file=sys.stderr)
        return 1

    # Export trace to XML
    xml_file = export_trace_to_xml(args.trace_file)
    if xml_file is None:
        return 1

    # Parse XML
    info(f"Parsing XML: {xml_file.name}")
    parsed_data = parse_time_profiler_xml(xml_file, project_only=args.project_only)
    if parsed_data is None:
        return 1

    if parsed_data['total_samples'] == 0:
        warning("No samples collected in trace (profiling session may have been too short)")
        return 1

    # Build JSON report
    report = build_json_report(args.trace_file, parsed_data, args.top)

    # Determine output file
    if args.output:
        output_file = args.output
    else:
        # Default: profile_results/profile_<timestamp>.json
        output_dir = Path('profile_results')
        output_dir.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_file = output_dir / f"profile_{timestamp}.json"

    # Write JSON report
    try:
        with output_file.open('w') as f:
            json.dump(report, f, indent=2)
        info(f"JSON report: {output_file}")
    except Exception as e:
        error(f"Failed to write JSON report: {e}")
        return 1

    # Print console summary unless --json-only
    if not args.json_only:
        format_console_output(report, args.top)

    return 0


if __name__ == '__main__':
    sys.exit(main())

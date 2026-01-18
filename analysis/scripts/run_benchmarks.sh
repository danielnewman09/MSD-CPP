#!/bin/bash
# Ticket: 0011_add_google_benchmark
# Generate benchmark reports in JSON format
#
# Usage:
#   ./scripts/run_benchmarks.sh [options]
#
# Options:
#   -o, --output DIR    Output directory for reports (default: benchmark_results)
#   -f, --format FMT    Output format: json or console (default: json)
#   -b, --build-type    Build type: Debug or Release (default: Release)
#   -r, --repetitions N Number of repetitions per benchmark (default: 3)
#   -h, --help          Show this help message
#
# Examples:
#   ./scripts/run_benchmarks.sh                          # Default JSON output
#   ./scripts/run_benchmarks.sh -f console               # Console output only
#   ./scripts/run_benchmarks.sh -o reports -r 5          # Custom dir, 5 reps

set -euo pipefail

# Default values
OUTPUT_DIR="benchmark_results"
FORMAT="json"
BUILD_TYPE="Release"
REPETITIONS=3

# Script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_usage() {
    sed -n '3,16p' "$0" | sed 's/^# //' | sed 's/^#//'
}

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -f|--format)
            FORMAT="$2"
            shift 2
            ;;
        -b|--build-type)
            BUILD_TYPE="$2"
            shift 2
            ;;
        -r|--repetitions)
            REPETITIONS="$2"
            shift 2
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

# Validate format
if [[ "$FORMAT" != "json" && "$FORMAT" != "console" ]]; then
    log_error "Invalid format: $FORMAT. Must be 'json' or 'console'."
    exit 1
fi

# Validate build type
if [[ "$BUILD_TYPE" != "Debug" && "$BUILD_TYPE" != "Release" ]]; then
    log_error "Invalid build type: $BUILD_TYPE. Must be 'Debug' or 'Release'."
    exit 1
fi

# Find benchmark executable
BENCH_EXECUTABLE=""
BUILD_TYPE_LOWER=$(echo "$BUILD_TYPE" | tr '[:upper:]' '[:lower:]')
POSSIBLE_PATHS=(
    "${PROJECT_ROOT}/build/${BUILD_TYPE}/${BUILD_TYPE_LOWER}/msd_sim_bench"
    "${PROJECT_ROOT}/build/${BUILD_TYPE}/msd_sim_bench"
    "${PROJECT_ROOT}/build/${BUILD_TYPE_LOWER}/msd_sim_bench"
)

for path in "${POSSIBLE_PATHS[@]}"; do
    if [[ -x "$path" ]]; then
        BENCH_EXECUTABLE="$path"
        break
    fi
done

if [[ -z "$BENCH_EXECUTABLE" ]]; then
    log_error "Benchmark executable not found. Build with ENABLE_BENCHMARKS=ON first."
    log_info "Run: conan install . --build=missing -s build_type=${BUILD_TYPE} -o '&:enable_benchmarks=True'"
    log_info "Then: cmake --preset conan-${BUILD_TYPE_LOWER} && cmake --build --preset conan-${BUILD_TYPE_LOWER}"
    exit 1
fi

log_info "Found benchmark executable: ${BENCH_EXECUTABLE}"

# Derive benchmark suite name from executable
# e.g., msd_sim_bench -> msd_sim_bench, or extract meaningful name
EXECUTABLE_NAME=$(basename "$BENCH_EXECUTABLE")
# Convert executable name to suite folder name (e.g., msd_sim_bench -> MsdSimBench)
# For now, use the executable name directly as folder name
SUITE_NAME="$EXECUTABLE_NAME"

# Create output directory if using JSON format
if [[ "$FORMAT" == "json" ]]; then
    # Organize by executable/suite name
    OUTPUT_PATH="${PROJECT_ROOT}/${OUTPUT_DIR}/${SUITE_NAME}"
    mkdir -p "$OUTPUT_PATH"

    # Generate timestamp for filename
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    OUTPUT_FILE="${OUTPUT_PATH}/benchmark_${TIMESTAMP}.json"

    log_info "Running benchmarks with ${REPETITIONS} repetitions..."
    log_info "Suite: ${SUITE_NAME}"
    log_info "Output file: ${OUTPUT_FILE}"

    # Run benchmarks with JSON output
    "$BENCH_EXECUTABLE" \
        --benchmark_format=json \
        --benchmark_out="$OUTPUT_FILE" \
        --benchmark_repetitions="$REPETITIONS" \
        --benchmark_report_aggregates_only=true

    log_info "Benchmark report saved to: ${OUTPUT_FILE}"

    # Also create a 'latest' symlink for convenience within the suite folder
    LATEST_LINK="${OUTPUT_PATH}/benchmark_latest.json"
    ln -sf "$(basename "$OUTPUT_FILE")" "$LATEST_LINK"
    log_info "Latest symlink updated: ${LATEST_LINK}"

    # Print summary from JSON
    echo ""
    log_info "Benchmark Summary:"
    if command -v jq &> /dev/null; then
        jq -r '.benchmarks[] | select(.run_type == "aggregate" and .aggregate_name == "mean") | "\(.name): \(.real_time | floor)ns"' "$OUTPUT_FILE" 2>/dev/null || \
        jq -r '.benchmarks[] | "\(.name): \(.real_time | floor)ns"' "$OUTPUT_FILE"
    else
        log_warn "Install 'jq' for formatted JSON summary"
        echo "Raw results saved to: ${OUTPUT_FILE}"
    fi
else
    # Console output only
    log_info "Running benchmarks with ${REPETITIONS} repetitions (console output)..."
    log_info "Suite: ${SUITE_NAME}"
    "$BENCH_EXECUTABLE" \
        --benchmark_repetitions="$REPETITIONS" \
        --benchmark_report_aggregates_only=true
fi

echo ""
log_info "Benchmarks completed successfully."

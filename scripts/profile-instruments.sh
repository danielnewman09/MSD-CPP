#!/bin/bash
# Ticket: 0012_add_macos_profiling_support, 0015_profiling_trace_parser
# Design: docs/designs/0012_add_macos_profiling_support/design.md
#         docs/designs/0015_profiling_trace_parser/design.md
#
# Helper script to profile executables with Xcode Instruments via xctrace CLI.
# This script automates the common workflow of launching a Time Profiler or
# Allocations session, generating timestamped .trace files, and providing
# instructions for opening results in Instruments GUI or exporting to XML.
#
# Usage:
#   ./profile-instruments.sh <executable> [options] [-- executable_args...]
#
# Arguments:
#   executable       - Path to executable to profile (required)
#
# Options:
#   -t, --template TEMPLATE    - Instruments template (default: "Time Profiler")
#                                Options: "Time Profiler", "Allocations"
#   -d, --output-dir DIR       - Output directory for trace files (default: "profile_results")
#   -x, --export-xml           - Export trace to XML after recording (optional)
#   -h, --help                 - Show this help message
#
# Positional arguments after --:
#   executable_args  - Arguments to pass to executable (after --)
#
# Example:
#   ./profile-instruments.sh ./build/Release/release/msd_sim_bench
#   ./profile-instruments.sh ./build/Release/release/msd_sim_bench -t "Allocations"
#   ./profile-instruments.sh ./build/Release/release/msd_sim_bench -x
#   ./profile-instruments.sh ./build/Release/release/msd_sim_bench -t "Time Profiler" -x -- --benchmark_filter="BM_ConvexHull"
#
# Requirements:
#   - macOS 12.0+ (Monterey or later)
#   - Xcode Command Line Tools installed
#   - Profiling-optimized executable (built with -g -O2)

set -e  # Exit on error
set -u  # Exit on undefined variable

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Validate Xcode Command Line Tools installed
validate_xcode_tools() {
    if ! command -v xctrace &> /dev/null; then
        echo -e "${RED}ERROR: xctrace not found${NC}"
        echo "Please install Xcode Command Line Tools:"
        echo "  xcode-select --install"
        exit 1
    fi
}

# Validate macOS version (requires 12.0+ for full xctrace functionality)
validate_macos_version() {
    local macos_version
    macos_version=$(sw_vers -productVersion)
    local major_version
    major_version=$(echo "$macos_version" | cut -d. -f1)

    if [[ "$major_version" -lt 12 ]]; then
        echo -e "${YELLOW}WARNING: macOS 12.0+ recommended for full xctrace functionality${NC}"
        echo "Current version: $macos_version"
        echo "Some features may not work correctly."
    fi
}

# Validate executable exists and is executable
validate_executable() {
    local executable="$1"

    if [[ ! -f "$executable" ]]; then
        echo -e "${RED}ERROR: Executable not found: $executable${NC}"
        exit 1
    fi

    if [[ ! -x "$executable" ]]; then
        echo -e "${RED}ERROR: File is not executable: $executable${NC}"
        echo "Try: chmod +x $executable"
        exit 1
    fi
}

# Show help message
show_help() {
    echo "Usage: $0 <executable> [options] [-- executable_args...]"
    echo ""
    echo "Arguments:"
    echo "  executable                 - Path to executable to profile (required)"
    echo ""
    echo "Options:"
    echo "  -t, --template TEMPLATE    - Instruments template (default: \"Time Profiler\")"
    echo "                               Options: \"Time Profiler\", \"Allocations\""
    echo "  -d, --output-dir DIR       - Output directory for trace files (default: \"profile_results\")"
    echo "  -x, --export-xml           - Export trace to XML after recording (optional)"
    echo "  -h, --help                 - Show this help message"
    echo ""
    echo "Positional arguments after --:"
    echo "  executable_args            - Arguments to pass to executable (after --)"
    echo ""
    echo "Example:"
    echo "  $0 ./build/Release/release/msd_sim_bench"
    echo "  $0 ./build/Release/release/msd_sim_bench -t \"Allocations\""
    echo "  $0 ./build/Release/release/msd_sim_bench -x"
    echo "  $0 ./build/Release/release/msd_sim_bench -t \"Time Profiler\" -x -- --benchmark_filter=\"BM_ConvexHull\""
}

# Export trace to XML using xctrace
export_trace_to_xml() {
    local trace_file="$1"
    local xml_file="${trace_file%.trace}.xml"

    echo ""
    echo -e "${GREEN}Exporting trace to XML...${NC}"

    # Get table of contents to verify schema availability
    if ! xctrace export --input "$trace_file" --toc > /dev/null 2>&1; then
        echo -e "${YELLOW}WARNING: Failed to read trace table of contents${NC}"
        return 1
    fi

    # Export Time Profiler data using xpath
    if xctrace export --input "$trace_file" \
        --xpath '/trace-toc/run[@number="1"]/data/table[@schema="time-profile"]' \
        --output "$xml_file" 2>&1 | grep -v "^$"; then
        if [[ -f "$xml_file" ]]; then
            echo -e "${GREEN}XML export complete: $xml_file${NC}"
            return 0
        else
            echo -e "${YELLOW}WARNING: XML export failed - file not created${NC}"
            return 1
        fi
    else
        echo -e "${YELLOW}WARNING: XML export failed${NC}"
        return 1
    fi
}

# Main profiling function
main() {
    # Default values
    local template="Time Profiler"
    local output_dir="profile_results"
    local export_xml=false
    local executable=""
    local exe_args=()

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                show_help
                exit 0
                ;;
            -t|--template)
                if [[ -z "${2:-}" ]]; then
                    echo -e "${RED}ERROR: --template requires a value${NC}"
                    exit 1
                fi
                template="$2"
                shift 2
                ;;
            -d|--output-dir)
                if [[ -z "${2:-}" ]]; then
                    echo -e "${RED}ERROR: --output-dir requires a value${NC}"
                    exit 1
                fi
                output_dir="$2"
                shift 2
                ;;
            -x|--export-xml)
                export_xml=true
                shift
                ;;
            --)
                shift
                exe_args=("$@")
                break
                ;;
            -*)
                echo -e "${RED}ERROR: Unknown option: $1${NC}"
                show_help
                exit 1
                ;;
            *)
                if [[ -z "$executable" ]]; then
                    executable="$1"
                    shift
                else
                    echo -e "${RED}ERROR: Unexpected argument: $1${NC}"
                    show_help
                    exit 1
                fi
                ;;
        esac
    done

    # Check if executable was provided
    if [[ -z "$executable" ]]; then
        echo -e "${RED}ERROR: Missing required argument: executable${NC}"
        echo ""
        show_help
        exit 1
    fi

    # Create output directory if it doesn't exist
    mkdir -p "$output_dir"

    local timestamp
    timestamp=$(date +%Y%m%d_%H%M%S)
    local trace_file="$output_dir/profile_${timestamp}.trace"

    # Validate prerequisites
    validate_xcode_tools
    validate_macos_version
    validate_executable "$executable"

    echo -e "${GREEN}Starting profiling session...${NC}"
    echo "Executable: $executable"
    if [[ ${#exe_args[@]} -gt 0 ]]; then
        echo "Arguments: ${exe_args[*]}"
    fi
    echo "Template: $template"
    echo "Output: $trace_file"
    echo ""

    # Run xctrace with specified template
    echo -e "${GREEN}Recording profile data (Ctrl-C when complete)...${NC}"
    if [[ ${#exe_args[@]} -gt 0 ]]; then
        xctrace_result=0
        xctrace record --template "$template" --output "$trace_file" --launch -- "$executable" "${exe_args[@]}" || xctrace_result=$?
    else
        xctrace_result=0
        xctrace record --template "$template" --output "$trace_file" --launch -- "$executable" || xctrace_result=$?
    fi
    if [[ $xctrace_result -eq 0 ]]; then
        echo ""
        echo -e "${GREEN}Profiling complete!${NC}"
        echo "Trace file: $trace_file"

        # Export to XML if requested
        if [[ "$export_xml" == "true" ]]; then
            export_trace_to_xml "$trace_file"
        fi

        echo ""
        echo "To view results:"
        echo "  open $trace_file"
        echo ""
        echo "Or open directly in Instruments:"
        echo "  open -a Instruments $trace_file"

        # Show parse-profile.py suggestion if available
        if [[ -f "$(dirname "$0")/parse-profile.py" ]]; then
            echo ""
            echo "To extract profiling data to JSON:"
            echo "  ./scripts/parse-profile.py $trace_file"
        fi
    else
        echo -e "${RED}ERROR: Profiling failed${NC}"
        exit 1
    fi
}

# Invoke main function
main "$@"

#!/usr/bin/env bash
# run_clang_tidy.sh — Run clang-tidy on all msd/ source files using the Release build.
#
# Uses LLVM's run-clang-tidy for parallel execution across all CPU cores.
# Only analyzes source files under msd/*/src/ — test and bench files are excluded.
#
# Usage:
#   ./analysis/scripts/run_clang_tidy.sh            # Full analysis (stdout + file)
#   ./analysis/scripts/run_clang_tidy.sh --fix       # Apply automatic fixes
#
# Prerequisites:
#   brew install llvm
#   conan install . --build=missing -s build_type=Release

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build/Release"
COMPILE_DB="${BUILD_DIR}/compile_commands.json"

# --- Locate tools (prefer Homebrew LLVM) ---
LLVM_PREFIX="$(brew --prefix llvm 2>/dev/null || true)"

find_tool() {
    local name="$1"
    if [ -n "$LLVM_PREFIX" ] && [ -x "$LLVM_PREFIX/bin/$name" ]; then
        echo "$LLVM_PREFIX/bin/$name"
    elif command -v "$name" &>/dev/null; then
        command -v "$name"
    else
        echo ""
    fi
}

CLANG_TIDY="$(find_tool clang-tidy)"
RUN_CLANG_TIDY="$(find_tool run-clang-tidy)"
CLANG_APPLY_REPLACEMENTS="$(find_tool clang-apply-replacements)"

if [ -z "$CLANG_TIDY" ]; then
    echo "Error: clang-tidy not found. Install with: brew install llvm"
    exit 1
fi

echo "Using: $("$CLANG_TIDY" --version | head -1)"

# --- Parse arguments ---
FIX_FLAG=""
STRICT_MODE=""
for arg in "$@"; do
    case "$arg" in
        --fix) FIX_FLAG="-fix" ;;
        --strict) STRICT_MODE="1" ;;
        --help|-h)
            echo "Usage: $0 [--fix] [--strict]"
            echo "  --fix     Apply clang-tidy automatic fixes"
            echo "  --strict  Exit with error code 1 if any warnings are found"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            exit 1
            ;;
    esac
done

# --- Ensure compile_commands.json exists ---
if [ ! -f "$COMPILE_DB" ]; then
    echo "compile_commands.json not found. Configuring Release build..."
    cmake --preset conan-release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi

if [ ! -f "$COMPILE_DB" ]; then
    echo "Error: Failed to generate compile_commands.json."
    echo "Try: cmake --preset conan-release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    exit 1
fi

# --- Build regex to match only msd/ source files (exclude test/, bench/) ---
# run-clang-tidy uses a regex filter against the file paths in compile_commands.json
SOURCE_REGEX="${PROJECT_ROOT}/msd/.*/src/.*\\.cpp$"

# Count matching files for reporting
FILE_COUNT=$(grep -c '"file"' "$COMPILE_DB" | head -1 || echo 0)
MATCH_COUNT=$(grep '"file"' "$COMPILE_DB" | grep -c '/msd/.*/src/.*\.cpp' || echo 0)

# --- Output file ---
REPORT_DIR="${PROJECT_ROOT}/analysis/clang_tidy_reports"
mkdir -p "$REPORT_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
REPORT_FILE="${REPORT_DIR}/clang_tidy_${TIMESTAMP}.log"

echo "Analyzing $MATCH_COUNT source files (out of $FILE_COUNT total in compile_commands.json)"
[ -n "$FIX_FLAG" ] && echo "Mode: applying fixes"
echo "Report: $REPORT_FILE"
echo "---"

# --- Run clang-tidy (tee output to both terminal and report file) ---
# NOTE: --fix must run serially. Parallel fix causes duplicate edits to shared
# headers when multiple .cpp files include the same header — resulting in
# corrupted fixes (e.g. "kk" prefix instead of "k").
if [ -n "$FIX_FLAG" ]; then
    echo "Running serially (required for --fix to avoid duplicate header edits)..."
    find "$PROJECT_ROOT/msd" -path '*/src/*.cpp' -not -path '*/test/*' -not -path '*/bench/*' -print0 | \
        sort -z | \
        while IFS= read -r -d '' file; do
            REL_PATH="${file#"$PROJECT_ROOT/"}"
            echo "[$REL_PATH]"
            "$CLANG_TIDY" -p "$BUILD_DIR" -fix "$file" 2>&1 || true
        done | tee "$REPORT_FILE"
elif [ -n "$RUN_CLANG_TIDY" ]; then
    # Parallel execution for analysis-only (safe — no file modifications)
    "$RUN_CLANG_TIDY" \
        -p "$BUILD_DIR" \
        -clang-tidy-binary "$CLANG_TIDY" \
        "$SOURCE_REGEX" 2>&1 | tee "$REPORT_FILE"
else
    # Fallback: serial analysis
    echo "run-clang-tidy not found, running serially..." | tee "$REPORT_FILE"
    find "$PROJECT_ROOT/msd" -path '*/src/*.cpp' -not -path '*/test/*' -not -path '*/bench/*' -print0 | \
        sort -z | \
        while IFS= read -r -d '' file; do
            REL_PATH="${file#"$PROJECT_ROOT/"}"
            echo "[$REL_PATH]"
            "$CLANG_TIDY" -p "$BUILD_DIR" "$file" 2>&1 || true
        done | tee -a "$REPORT_FILE"
fi

echo "---"
echo "Report written to: $REPORT_FILE"

# --- Strict mode: fail if any warnings found in user code ---
if [ -n "$STRICT_MODE" ]; then
    # Count warnings in user code (exclude "in non-user code" and "Suppressed" lines)
    # Look for lines matching: "file.cpp:line:col: warning:" pattern
    WARNING_COUNT=$(grep -E '^/.*\.(cpp|hpp):[0-9]+:[0-9]+: warning:' "$REPORT_FILE" 2>/dev/null | wc -l | tr -d ' ')
    ERROR_COUNT=$(grep -E '^/.*\.(cpp|hpp):[0-9]+:[0-9]+: error:' "$REPORT_FILE" 2>/dev/null | wc -l | tr -d ' ')

    echo "Strict mode: $WARNING_COUNT warning(s), $ERROR_COUNT error(s) in user code"

    if [ "$WARNING_COUNT" -gt 0 ] || [ "$ERROR_COUNT" -gt 0 ]; then
        echo "FAILED: clang-tidy found issues in user code"
        exit 1
    else
        echo "PASSED: No clang-tidy issues in user code"
        exit 0
    fi
fi

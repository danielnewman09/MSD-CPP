#!/bin/bash
# Start the MSD Replay FastAPI server
# Ticket: 0056k_example_workflow_test_recording

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$PROJECT_ROOT/.venv"
BUILD_DIR="$PROJECT_ROOT/build/Debug/debug"
RECORDING_DB="$SCRIPT_DIR/recordings/test_cube_drop.db"

# --- 1. Ensure venv exists ---
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating Python venv at $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi
source "$VENV_DIR/bin/activate"

# --- 2. Install replay package ---
echo "Installing replay package..."
pip install -q -e "$SCRIPT_DIR"

# --- 3. Check msd_reader is available ---
export PYTHONPATH="$BUILD_DIR:$PYTHONPATH"
if ! python3 -c "import msd_reader" 2>/dev/null; then
    echo "ERROR: msd_reader not found in $BUILD_DIR"
    echo "Build with pybind enabled first:"
    echo "  conan install . --build=missing -s build_type=Debug -o \"&:enable_pybind=True\""
    echo "  cmake --preset conan-debug"
    echo "  cmake --build --preset conan-debug"
    exit 1
fi

# --- 4. Generate test recording if missing ---
if [ ! -f "$RECORDING_DB" ]; then
    GENERATOR="$BUILD_DIR/generate_test_recording"
    if [ ! -f "$GENERATOR" ]; then
        echo "ERROR: generate_test_recording not found at $GENERATOR"
        echo "Build it first: cmake --build --preset conan-debug --target generate_test_recording"
        exit 1
    fi
    mkdir -p "$SCRIPT_DIR/recordings"
    echo "Generating test recording..."
    "$GENERATOR" "$RECORDING_DB"
fi

# --- 5. Start FastAPI server ---
echo ""
echo "Starting replay server..."
echo "  API:  http://localhost:8000/api/v1"
echo "  Docs: http://localhost:8000/docs"
echo ""
exec uvicorn replay.app:app --reload --app-dir "$SCRIPT_DIR"
